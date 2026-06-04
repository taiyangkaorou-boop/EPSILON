/*
 * @file onlane_ai_agent.cc
 * @brief AI驱动的交通智能体节点 —— 仿真周围车辆的自主动态行为
 *
 * [架构定位]
 * 本文件实现了一个独立的 ROS2 节点, 用于模拟 EPSILON 仿真环境中的周围交通车辆
 * (surrounding vehicles / AI agents)。每个 AI 智能体运行自己的 MPDM 行为规划器,
 * 通过前向仿真器 (OnLaneForwardSimulation) 生成物理上可行的控制指令,
 * 并将控制信号发布到仿真器以更新自身状态。
 *
 * [系统上下文]
 * 在 EPSILON 系统中, 真实自车 (ego vehicle) 由 planning_integrated 模块控制,
 * 而周围的社会车辆 (social vehicles) 由本模块的多个节点实例控制。
 * 典型部署方式:
 *   - onlane_ego_agent_launch.py: 启动 1 个 AI 智能体 (agent_0), 作为自车
 *   - onlane_ai_agent_launch.py: 启动 10 个 AI 智能体 (agent_1..10), 作为周围车辆
 *
 * [数据流]
 *   ROS话题 (arena_info_static / arena_info_dynamic)
 *       |
 *       v
 *   RosAdapter -> SemanticMapManager (维护全局场景模型)
 *       |
 *       |-- SemanticMapUpdateCallback --> MPDM 行为规划器 (多策略行为决策)
 *       |                                   |
 *       |                                   v
 *       |                              BehaviorUpdateCallback --> 无锁队列 (p_ctrl_input_smm_buff_)
 *       |                                                           |
 *       |                                                           v
 *       |                                                      PublishControl() 主循环:
 *       |                                                      1) 从队列取出最新语义地图
 *       |                                                      2) 调用 OnLaneForwardSimulation::PropagateOnce()
 *       |                                                         基于 IDM 跟车模型 + 当前行为策略,
 *       |                                                         前向仿真一步得到新状态
 *       |                                                      3) 发布 vehicle_msgs::msg::ControlSignal 到 ctrl 话题
 *       |                                                      4) 更新可视化数据
 *       v
 *   phy_simulator 订阅 ctrl 话题, 更新车辆状态后发布 arena_info_dynamic → 循环
 *
 * [核心机制]
 * - RandomBehavior(): 周期性随机改变期望速度 (±2 到 +5 m/s 噪声), 模拟人类驾驶员速度偏好变化
 * - OnLaneForwardSimulation::PropagateOnce(): 基于 IDM (Intelligent Driver Model) 进行单车道上的一步前向仿真
 * - 无锁队列 (ReaderWriterQueue): 高性能线程间通信, 避免加锁开销
 * - 多线程架构: MPDM 行为规划器在独立线程中运行, 主线程负责仿真推进和控制指令发布
 */

#include <chrono>
#include <iostream>
#include <memory>
#include <random>

#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"

#include "behavior_planner/behavior_server_ros.h"
#include "common/basics/tic_toc.h"
#include "forward_simulator/multimodal_forward.h"
#include "forward_simulator/onlane_forward_simulation.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
// Joy 消息: 用于 HMI 远程控制接口 (与本文件核心逻辑间接相关)
#include "sensor_msgs/msg/joy.hpp"
// ControlSignal: 本文件输出的主要消息类型 —— 方向盘转角+油门/刹车
#include "vehicle_msgs/msg/control_signal.hpp"
#include "vehicle_msgs/encoder.h"

// 启用 Backward-cpp 堆栈跟踪库
DECLARE_BACKWARD;

// fs_work_rate: 前向仿真器工作频率 (Hz), 50Hz 保证足够的控制精度
double fs_work_rate = 50.0;
// visualization_msg_rate: 可视化消息发布频率 (Hz), 20Hz 足以让人眼感知平滑
double visualization_msg_rate = 20.0;
// bp_work_rate: MPDM 行为规划器工作频率 (Hz), 20Hz 平衡计算效率和决策响应
double bp_work_rate = 20.0;
// ego_id: 当前智能体的唯一标识符 (在 launch 文件中指定)
int ego_id;

// ctrl_signal_pub_: 控制信号发布者 —— 向物理仿真器发布本车的油门/刹车/转向指令
rclcpp::Publisher<vehicle_msgs::msg::ControlSignal>::SharedPtr ctrl_signal_pub_;
// p_bp_server_: MPDM 行为规划器服务端 —— 为当前 AI 智能体提供行为决策
std::shared_ptr<planning::BehaviorPlannerServer> p_bp_server_{nullptr};
/* p_ctrl_input_smm_buff_: 无锁队列 (ReaderWriterQueue) —— 作为行为规划线程与主线程之间的桥梁
 * 行为规划器更新后, 语义地图被 push 到队列中; 主线程的 PublishControl() 从中取出最新数据 */
std::shared_ptr<moodycamel::ReaderWriterQueue<semantic_map_manager::SemanticMapManager>> p_ctrl_input_smm_buff_{nullptr};
// p_smm_vis_: 语义地图可视化器 —— 负责发布 RViz 可视化和 TF 坐标变换
std::shared_ptr<semantic_map_manager::Visualizer> p_smm_vis_{nullptr};

// desired_state: 当前期望状态 (位置、速度、朝向), 由前向仿真器迭代更新
common::State desired_state;
// has_init_state: 标志位 —— 是否已从语义地图中获取到初始状态
bool has_init_state = false;
// desired_vel: 基准期望速度 (m/s), 实际速度会在此基础上有随机噪声
double desired_vel;

// next_vis_pub_time: 下次可视化发布的时间戳, 用于控制发布频率
rclcpp::Time next_vis_pub_time;
// last_smm: 最近一次从队列中取出的语义地图副本
semantic_map_manager::SemanticMapManager last_smm;
// sim_param: 前向仿真参数 (包含 IDM 参数、策略参数等)
planning::OnLaneForwardSimulation::Param sim_param;

// ---------- 随机行为噪声相关 ----------
// rng: Mersenne Twister 随机数生成器, 用于产生速度噪声
std::mt19937 rng;
// vel_noise: 速度噪声的当前值 (m/s), 叠加在 desired_vel 上
double vel_noise = 0.0;
// cnt: 计数器 —— 控制 RandomBehavior() 的调用频率 (每 2000 次迭代 ~ 40 秒更新一次)
int cnt = 0;
// aggressiveness_level: 激进程度等级 (1-5), 影响 IDM 跟车参数 (跟车距离、加/减速度)
int aggressiveness_level = 3;


/*
 * SemanticMapUpdateCallback —— 语义地图更新回调
 *
 * [调用链] RosAdapter 刷新语义地图 → 调用此回调
 * [功能]
 *   1. 将最新语义地图推送给 MPDM 行为规划器, 触发行为决策重新计算
 *   2. 首次调用时, 从语义地图中提取自车的初始状态作为 desired_state 的初值
 *
 * [数据流] ROS话题 → RosAdapter → 语义地图 → MPDM 行为规划器
 */
int SemanticMapUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) {
  if (p_bp_server_) {
    p_bp_server_->PushSemanticMap(smm);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("onlane_ai_agent"), "p_bp_server_ is nullptr in SemanticMapUpdateCallback");
  }

  // 首次接收到语义地图时, 用当前自车状态初始化 desired_state
  if (!has_init_state) {
    desired_state = smm.ego_vehicle().state();
    has_init_state = true;
    desired_state.print();
  }
  return 0;
}

/*
 * BehaviorUpdateCallback —— 行为更新回调
 *
 * [调用链] MPDM 行为规划器完成行为决策 → 调用此回调
 * [功能]   将带有新行为指令的语义地图放入无锁队列 (p_ctrl_input_smm_buff_),
 *          使主线程的 PublishControl() 可以取用最新的语义地图来推进仿真。
 *
 * [线程安全] try_enqueue 使用无锁队列, 确保多线程安全的同时避免传统的 mutex 开销
 *
 * [数据流] MPDM 行为规划器 → 无锁队列 → PublishControl() 主循环
 */
int BehaviorUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) {
  if (p_ctrl_input_smm_buff_) {
    p_ctrl_input_smm_buff_->try_enqueue(smm);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("onlane_ai_agent"), "p_ctrl_input_smm_buff_ is nullptr in BehaviorUpdateCallback");
  }
  return 0;
}

/*
 * RandomBehavior —— 随机速度行为生成器
 *
 * [功能]   周期性 (每 2000 次迭代约 40 秒, 取决于 fs_work_rate) 生成随机速度偏置,
 *          叠加在基准期望速度 (desired_vel) 上, 模拟人类驾驶员的速度偏好变化。
 * [噪声范围] -2 到 +5 m/s (即允许略微减速但更倾向于加速)
 * [更新频率] cnt == 0 时更新 (即每 2000 次迭代更新一次)
 */
void RandomBehavior() {
  if (cnt == 0) {
    // 在 [-2, 5] m/s 范围内均匀采样速度噪声
    std::uniform_real_distribution<double> dist_vel(-2, 5);
    vel_noise = dist_vel(rng);
    if (p_bp_server_) {
      // 将带噪声的期望速度设置到行为规划器中
      p_bp_server_->set_user_desired_velocity(desired_vel + vel_noise);
      RCLCPP_INFO(rclcpp::get_logger("onlane_ai_agent"), "[OnlaneAi]%d - desired velocity: %lf", ego_id, desired_vel + vel_noise);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("onlane_ai_agent"), "p_bp_server_ is nullptr in RandomBehavior");
    }
  }
  cnt++;
  if (cnt >= 2000) cnt = 0;  // 每 2000 次迭代重置计数器
}

/*
 * PublishControl —— 控制指令发布核心函数
 *
 * [功能] 在每个仿真步中执行以下操作:
 *   1. 从无锁队列中取出最新的语义地图 (非阻塞, 取最近一份)
 *   2. 获取行为规划器的参考期望速度, 并施加车道速度限制
 *   3. 查找当前车道上的前车 (Leading Vehicle)
 *   4. 调用 OnLaneForwardSimulation::PropagateOnce() 基于 IDM 模型进行一步前向仿真
 *   5. 将仿真结果封装为 vehicle_msgs::msg::ControlSignal 发布到仿真器
 *   6. 按固定频率 (visualization_msg_rate) 发布可视化数据
 *
 * [前向仿真原理]
 * OnLaneForwardSimulation::PropagateOnce() 基于 IDM (Intelligent Driver Model) 实现:
 *   - 输入: 自车状态、前车状态、车道参考线、仿真步长、IDM 参数
 *   - 输出: 下一时刻的车辆状态 (位置、速度、加速度)
 *   - IDM 公式: a = a_max * (1 - (v/v0)^delta - (s*/s)^2)
 *     其中 s* 为期望跟车距离, 取决于相对速度和激进程度参数
 *
 * [线程安全] 本函数在 ROS2 spin_some 回调的执行上下文中运行, 行为规划器在独立线程中运行,
 *            通过无锁队列实现无等待通信
 */
void PublishControl() {
  // 尚未获取初始状态, 无法推进仿真
  if (!has_init_state) return;
  // 行为规划器或队列未初始化时安全返回
  if (p_bp_server_ == nullptr) return;
  if (p_ctrl_input_smm_buff_ == nullptr) return;

  // 从无锁队列中取出最新的语义地图 (非阻塞, 取最近一份)
  bool is_map_updated = false;
  decimal_t previous_stamp = last_smm.time_stamp();
  while (p_ctrl_input_smm_buff_->try_dequeue(last_smm)) {
    is_map_updated = true;
  }
  // 无新数据时直接返回, 跳过本次控制发布
  if (!is_map_updated) return;

  // 计算仿真步长 delta_t, 并施加上限防止异常跳帧
  decimal_t delta_t = last_smm.time_stamp() - previous_stamp;
  if (delta_t > 100.0 / fs_work_rate) delta_t = 1.0 / fs_work_rate;

  // 获取行为规划器输出的参考期望速度
  decimal_t command_vel = p_bp_server_->reference_desired_velocity();

  // 施加车道速度限制: 取期望速度和车道限速中的较小值
  decimal_t speed_limit;
  if (last_smm.GetSpeedLimit(last_smm.ego_vehicle().state(),
                             last_smm.ego_behavior().ref_lane,
                             &speed_limit) == kSuccess) {
    command_vel = std::min(speed_limit, command_vel);
  }

  // 获取当前自车对象, 并设置其状态为上一时刻的 desired_state
  common::Vehicle ego_vehicle = last_smm.ego_vehicle();
  ego_vehicle.set_state(desired_state);
  // 更新 IDM 参数中的期望速度
  sim_param.idm_param.kDesiredVelocity = command_vel;

  // 查找当前车道上的前车 (Leading Vehicle)
  common::Vehicle leading_vehicle;
  common::State state;
  decimal_t distance_residual_ratio = 0.0;
  const decimal_t lat_range = 2.2;  // 横向检测范围 (m), 用于判断是否在同一车道
  last_smm.GetLeadingVehicleOnLane(last_smm.ego_behavior().ref_lane,
                                   desired_state,
                                   last_smm.surrounding_vehicles(), lat_range,
                                   &leading_vehicle, &distance_residual_ratio);

  // IDM 前向仿真: 基于当前状态、前车信息、IDM 参数推进一个仿真步
  // StateTransformer 将全局坐标转换为 Frenet 坐标便于沿车道仿真
  if (planning::OnLaneForwardSimulation::PropagateOnce(
          common::StateTransformer(last_smm.ego_behavior().ref_lane),
          ego_vehicle, leading_vehicle, delta_t, sim_param,
          &state) != kSuccess) {
    RCLCPP_ERROR(rclcpp::get_logger("onlane_ai_agent"), "[AiAgent]Err-Simulation error (with leading vehicle).");
    return;
  }

  // 将仿真结果封装为 ControlSignal 消息并发布
  common::VehicleControlSignal ctrl(state);
  {
    vehicle_msgs::msg::ControlSignal ctrl_msg;
    // 通过 Encoder 将内部 ControlSignal 结构转换为 ROS2 消息格式
    vehicle_msgs::Encoder::GetRosControlSignalFromControlSignal(
        ctrl, rclcpp::Clock(RCL_ROS_TIME).now(), std::string("map"), &ctrl_msg);
    ctrl_signal_pub_->publish(ctrl_msg);
  }
  // 更新期望状态, 为下一轮仿真做准备
  desired_state = ctrl.state;

  // ---------- 可视化数据发布 ----------
  // 按照固定频率 (visualization_msg_rate) 发布, 避免过高的带宽占用
  {
    rclcpp::Time tnow = rclcpp::Clock(RCL_ROS_TIME).now();
    if (tnow >= next_vis_pub_time) {
      next_vis_pub_time += rclcpp::Duration::from_seconds(1.0 / visualization_msg_rate);
      if (p_smm_vis_) {
        // VisualizeDataWithStamp: 发布用于 RViz 可视化的标记数据 (车辆位置、车道等)
        p_smm_vis_->VisualizeDataWithStamp(tnow, last_smm);
        // SendTfWithStamp: 发布 TF 坐标变换 (各车辆在全局坐标系中的位姿)
        p_smm_vis_->SendTfWithStamp(tnow, last_smm);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("onlane_ai_agent"), "p_smm_vis_ is nullptr in PublishControl");
      }
    }
  }
}

/*
 * main —— AI 智能体主函数
 *
 * [执行流程]
 *   1. 初始化 ROS2 节点, 读取启动参数 (ego_id, desired_vel, agent_config_path 等)
 *   2. 创建 ctrl 话题发布者
 *   3. 初始化随机数生成器种子
 *   4. 根据 aggressiveness_level 查表获取前向仿真的参数 (IDM 参数等)
 *   5. 创建 SemanticMapManager → RosAdapter → Visualizer
 *   6. 创建并配置 MPDM 行为规划器 (设置自主等级、激进程度、启用 HMI 接口)
 *   7. 绑定回调函数 (地图更新 → 行为规划器, 行为更新 → 无锁队列)
 *   8. 创建无锁队列
 *   9. 启动行为规划器, 进入主循环 (spin + PublishControl)
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("onlane_ai_agent");

  // 初始化下次可视化发布时间为当前时间
  next_vis_pub_time = node->get_clock()->now();

  // ---------- 参数声明与读取 ----------
  double desired_vel = 6.0;
  int autonomous_level = 3;
  int aggressiveness_level = 3;
  std::string agent_config_path = "";

  node->declare_parameter<int>("ego_id", ego_id);
  node->declare_parameter<std::string>("agent_config_path", agent_config_path);
  node->declare_parameter<double>("desired_vel", desired_vel);
  node->declare_parameter<int>("autonomous_level", aggressiveness_level);
  node->declare_parameter<int>("aggressiveness_level", aggressiveness_level);

  node->get_parameter("ego_id", ego_id);
  node->get_parameter("desired_vel", desired_vel);
  node->get_parameter("agent_config_path", agent_config_path);
  node->get_parameter("autonomous_level", autonomous_level);
  node->get_parameter("aggressiveness_level", aggressiveness_level);

  // 读取并打印 ego_id
  if (!node->get_parameter("ego_id", ego_id)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: ego_id");
  } else {
    RCLCPP_INFO(node->get_logger(), "ego_id: %d", ego_id);
  }

  // 读取并打印 desired_vel
  if (!node->get_parameter("desired_vel", desired_vel)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: desired_vel");
  } else {
    RCLCPP_INFO(node->get_logger(), "desired_vel: %f", desired_vel);
  }

  // 读取并打印 agent_config_path
  if (!node->get_parameter("agent_config_path", agent_config_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: agent_config_path");
  } else {
    RCLCPP_INFO(node->get_logger(), "agent_config_path: %s", agent_config_path.c_str());
  }

  // 读取并打印 autonomous_level (自主等级)
  if (!node->get_parameter("autonomous_level", autonomous_level)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: autonomous_level");
  } else {
    RCLCPP_INFO(node->get_logger(), "autonomous_level: %d", autonomous_level);
  }

  // 读取并打印 aggressiveness_level (激进程度)
  if (!node->get_parameter("aggressiveness_level", aggressiveness_level)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: aggressiveness_level");
  } else {
    RCLCPP_INFO(node->get_logger(), "aggressiveness_level: %d", aggressiveness_level);
  }

  // ---------- 构建 AI 智能体规划流水线 ----------
  try {
    // 创建控制信号发布者: 发布到 ctrl 话题, 队列深度为 10
    ctrl_signal_pub_ = node->create_publisher<vehicle_msgs::msg::ControlSignal>("ctrl", 10);

    // 用高精度时钟初始化随机数生成器种子
    rng.seed(std::chrono::high_resolution_clock::now().time_since_epoch().count());

    // 根据激进程度等级查表获取前向仿真参数 (IDM 参数如安全距离、最大加速度等)
    planning::MultiModalForward::ParamLookUp(aggressiveness_level, &sim_param);
    RCLCPP_INFO(node->get_logger(), "[OnlaneAi]%d - aggresive: %d", ego_id, aggressiveness_level);

    /* 第1层: 语义地图管理器 + ROS 适配器 + 可视化器
     * 语义地图管理器维护全局场景模型,
     * RosAdapter 通过订阅 ROS 话题更新语义地图,
     * Visualizer 负责 RViz 可视化 */
    auto semantic_map_manager = std::make_shared<semantic_map_manager::SemanticMapManager>(ego_id, agent_config_path);
    auto smm_ros_adapter = std::make_shared<semantic_map_manager::RosAdapter>(node, semantic_map_manager.get());
    p_smm_vis_ = std::make_shared<semantic_map_manager::Visualizer>(node, ego_id);

    /* 第2层: MPDM 行为规划器
     * 为当前 AI 智能体提供多策略行为决策 (巡航/换道/让行等) */
    p_bp_server_ = std::make_shared<planning::BehaviorPlannerServer>(node, bp_work_rate, ego_id);
    if (p_bp_server_ == nullptr) {
      RCLCPP_ERROR(node->get_logger(), "Failed to create BehaviorPlannerServer");
      return -1;
    } else {
      RCLCPP_INFO(node->get_logger(), "Success to create BehaviorPlannerServer");
    }

    // 配置行为规划器参数
    p_bp_server_->set_user_desired_velocity(desired_vel);
    p_bp_server_->set_autonomous_level(autonomous_level);
    p_bp_server_->set_aggressive_level(aggressiveness_level);
    // 启用 HMI 接口: 允许通过 Joy 消息远程干预 (如 terminal_server.py)
    p_bp_server_->enable_hmi_interface();

    /* 绑定回调函数:
     * 1. RosAdapter 地图更新 → SemanticMapUpdateCallback → 推送给行为规划器
     * 2. 行为规划器决策更新 → BehaviorUpdateCallback → 放入无锁队列供主线程使用 */
    smm_ros_adapter->BindMapUpdateCallback(SemanticMapUpdateCallback);
    p_bp_server_->BindBehaviorUpdateCallback(BehaviorUpdateCallback);

    // 初始化: 创建 ROS 订阅/发布、加载配置
    smm_ros_adapter->Init();
    p_bp_server_->Init();

    /* 第3层: 无锁队列 —— 行为规划线程与主线程之间的通信桥梁
     * 使用 moodycamel::ReaderWriterQueue, 容量为 100,
     * 支持单写单读无锁并发, 适用于生产者-消费者模式 */
    p_ctrl_input_smm_buff_ = std::make_shared<moodycamel::ReaderWriterQueue<semantic_map_manager::SemanticMapManager>>(100);

    // 启动行为规划器 (开始按 bp_work_rate 频率执行决策循环)
    p_bp_server_->Start();

    /* 主循环: 以 fs_work_rate (50Hz) 的频率:
     * 1. spin_some: 处理 ROS2 回调 (地图更新、行为更新)
     * 2. PublishControl: 从队列取数据 → IDM 仿真 → 发布控制指令 */
    rclcpp::Rate rate(fs_work_rate);
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      PublishControl();
      rate.sleep();
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}
