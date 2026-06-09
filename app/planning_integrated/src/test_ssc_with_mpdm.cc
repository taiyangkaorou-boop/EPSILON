/*
 * @file test_ssc_with_mpdm.cc
 * @brief EPSILON自动驾驶系统顶层入口 —— MPDM行为规划器 + SSC运动规划器联合测试节点
 *
 * [架构定位]
 * 本文件是 EPSILON 自动驾驶规划系统的顶层入口之一，负责将语义地图管理器
 * (SemanticMapManager)、MPDM 行为规划器 (BehaviorPlannerServer) 和 SSC 运动规划器
 * (SscPlannerServer) 串联成完整的规划流水线 (planning pipeline)。
 *
 * [数据流]
 *   ROS话题 (arena_info_static / arena_info_dynamic)
 *       |
 *       v
 *   RosAdapter (ROS消息适配层, 负责订阅话题并更新语义地图)
 *       |
 *       v
 *   SemanticMapManager (语义地图管理器, 维护全局场景模型)
 *       |
 *       |-- SemanticMapUpdateCallback --> BehaviorPlannerServer (MPDM 行为规划器, 多策略决策, 生成语义级行为)
 *       |                                   |
 *       |                                   v
 *       |                              BehaviorUpdateCallback --> SscPlannerServer (SSC 运动规划器, 生成轨迹和控制指令)
 *       |                                                           |
 *       |                                                           v
 *       |                                                      发布 ctrl 话题 (vehicle_msgs::msg::ControlSignal)
 *       v
 *   phy_simulator (物理仿真器) 订阅 ctrl 话题, 更新车辆状态后发布 arena_info_dynamic
 *
 * [运行方式]
 * 通过 ROS2 launch 文件 test_ssc_with_mpdm_ros_launch.py 启动,
 * 配置文件路径 (agent_config, ssc_config) 由 launch 文件通过参数传入。
 *
 * [与 test_ssc_with_eudm.cc 的区别]
 * 行为规划器不同: 本文件使用 MPDM (Multi-Policy Decision Making),
 * 而 test_ssc_with_eudm.cc 使用 EUDM (Efficient Uncertainty-aware Decision Making)。
 * MPDM 可以同时处理更多语义级行为策略 (如复杂换道、让行), 适合复杂交通场景,
 * 并支持通过 autonomous_level 控制自动化等级, 以及通过 HMI 接口实现人机交互。
 *
 * [MPDM 特有功能]
 * - autonomous_level: 设置自动化等级 (如 Level 3 表示有条件自动驾驶)
 * - enable_hmi_interface(): 启用 HMI (人机交互) 接口, 允许通过 Joy 消息远程干预
 */

#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>

#include <chrono>
#include <iostream>

#include "behavior_planner/behavior_server_ros.h"
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "ssc_planner/ssc_server_ros.h"

// DECLARE_BACKWARD: 启用 Backward-cpp 堆栈跟踪库, 用于在异常退出时打印调用栈
DECLARE_BACKWARD;

// SSC 运动规划器工作频率 (Hz), 即每秒调用 20 次运动规划
double ssc_planner_work_rate = 20.0;
// MPDM 行为规划器工作频率 (Hz), 即每秒调用 20 次行为规划
double bp_work_rate = 20.0;

// 全局智能指针 —— 分别持有运动规划器和 MPDM 行为规划器的服务端实例
std::shared_ptr<planning::SscPlannerServer> p_ssc_server_{nullptr};
std::shared_ptr<planning::BehaviorPlannerServer> p_bp_server_{nullptr};

/*
 * BehaviorUpdateCallback —— 行为更新回调
 *
 * [调用链] BehaviorPlannerServer 内部定时器触发 → 更新多策略行为决策 → 调用此回调
 * [功能]   将行为规划器更新后的语义地图推送给 SSC 运动规划器,
 *          使运动规划器可以基于最新的行为决策 (如目标车道、期望速度、让行标志) 来生成轨迹。
 * [数据流] 行为规划器 → 语义地图 → 运动规划器
 */
int BehaviorUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) {
  if (p_ssc_server_) p_ssc_server_->PushSemanticMap(smm);
  return 0;
}

/*
 * SemanticMapUpdateCallback —— 语义地图更新回调
 *
 * [调用链] RosAdapter 接收到 ROS 话题更新 → 刷新语义地图 → 调用此回调
 * [功能]   将最新语义地图推送给 MPDM 行为规划器, 触发多策略行为决策的重新计算。
 *         语义地图包含了自车状态、周围车辆信息、车道网络结构等全局场景模型。
 * [数据流] 传感器/仿真器 → ROS 话题 → RosAdapter → 语义地图 → 行为规划器
 */
int SemanticMapUpdateCallback(const semantic_map_manager::SemanticMapManager& smm) {
  if (p_bp_server_) p_bp_server_->PushSemanticMap(smm);
  return 0;
}

/*
 * main —— 主函数, 构建 MPDM + SSC 规划流水线并运行
 *
 * [执行流程]
 *   1. 初始化 ROS2 节点
 *   2. 从参数服务器读取配置 (ego_id, desired_vel, 各模块配置文件路径)
 *   3. 创建 SemanticMapManager → RosAdapter, 绑定地图更新回调
 *   4. 创建 BehaviorPlannerServer (MPDM 行为规划器), 配置自动化等级和 HMI 接口
 *   5. 创建 SscPlannerServer (运动规划器), 初始化并启动两个规划器
 *   6. 进入 ROS2 spin 循环, 以 100Hz 的频率轮询回调
 *   7. 异常发生时打印错误并退出
 *
 * [注意] MPDM 不需要 bp_config_path 参数, 其行为策略内建在代码逻辑中,
 *        这与 EUDM 需要通过 protobuf 配置文件加载参数不同。
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_ssc_with_mpdm");

  // ---------- 参数声明与默认值 ----------
  int ego_id = 0;
  std::string agent_config_path = "";
  std::string ssc_config_path = "";
  double desired_vel = 6.0;
  bool use_sim_state = true;

  // 声明 ROS2 参数, 可由 launch 文件覆盖
  node->declare_parameter<int>("ego_id", ego_id);
  node->declare_parameter<double>("desired_vel", desired_vel);
  node->declare_parameter<std::string>("agent_config_path", agent_config_path);
  node->declare_parameter<std::string>("ssc_config_path", ssc_config_path);
  // 声明仿真状态参数，供 SscPlannerServer::Init() 后续读取。
  node->declare_parameter<bool>("use_sim_state", use_sim_state);

  // 读取 ego_id: 自车在仿真器中的唯一标识符
  if (!node->get_parameter("ego_id", ego_id)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: ego_id");
  } else {
    RCLCPP_INFO(node->get_logger(), "ego_id: %d", ego_id);
  }

  // 读取 desired_vel: 全局期望行驶速度 (m/s)
  if (!node->get_parameter("desired_vel", desired_vel)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: desired_vel");
  } else {
    RCLCPP_INFO(node->get_logger(), "desired_vel: %f", desired_vel);
  }

  // 读取 agent_config_path: 智能体配置 JSON 文件路径 (定义车辆参数、传感器范围等)
  if (!node->get_parameter("agent_config_path", agent_config_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: agent_config_path");
  } else {
    RCLCPP_INFO(node->get_logger(), "agent_config_path: %s", agent_config_path.c_str());
  }

  // 读取 ssc_config_path: SSC 运动规划器 protobuf 配置文件路径
  if (!node->get_parameter("ssc_config_path", ssc_config_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: ssc_config_path");
  } else {
    RCLCPP_INFO(node->get_logger(), "ssc_config_path: %s", ssc_config_path.c_str());
  }

  // 读取 use_sim_state: true 表示用仿真状态驱动执行轨迹采样。
  if (!node->get_parameter("use_sim_state", use_sim_state)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: use_sim_state");
  } else {
    RCLCPP_INFO(node->get_logger(), "use_sim_state: %s",
                use_sim_state ? "true" : "false");
  }

  // ---------- 构建规划流水线 ----------
  try {
    /* 第1层: 语义地图管理器 —— 全局场景模型的核心数据结构
     * 负责维护所有车辆状态、车道网络、速度限制等信息,
     * 由 RosAdapter 通过 ROS 话题实时更新 */
    auto semantic_map_manager = std::make_shared<semantic_map_manager::SemanticMapManager>(ego_id, agent_config_path);
    /* RosAdapter: ROS 消息适配层 —— 订阅 arena_info_static 和 arena_info_dynamic 话题,
     * 将物理仿真器发布的原始数据转换为 SemanticMapManager 可用的结构化语义地图 */
    auto smm_ros_adapter = std::make_shared<semantic_map_manager::RosAdapter>(node, semantic_map_manager.get());
    /* 绑定地图更新回调: 当 RosAdapter 完成一次地图刷新后,
     * 自动调用 SemanticMapUpdateCallback 将最新地图推送给 MPDM 行为规划器 */
    smm_ros_adapter->BindMapUpdateCallback(SemanticMapUpdateCallback);

    /* 第2层: MPDM 行为规划器 (Multi-Policy Decision Making) —— 多策略高层语义决策
     * 与 EUDM 不同, MPDM 维护多个候选策略 (如左换道/右换道/巡航/让行),
     * 通过前向仿真评估每个策略的风险与收益, 选择最优行为 */
    p_bp_server_ = std::make_shared<planning::BehaviorPlannerServer>(node, bp_work_rate, ego_id);
    p_bp_server_->set_user_desired_velocity(desired_vel);
    /* 绑定行为更新回调: 当 MPDM 完成行为决策后,
     * 自动调用 BehaviorUpdateCallback 将带有新行为指令的语义地图推送给 SSC 运动规划器 */
    p_bp_server_->BindBehaviorUpdateCallback(BehaviorUpdateCallback);
    /* MPDM 特有: 设置自动化等级为 Level 3 (有条件自动驾驶)
     * Level 0=手动, Level 1=辅助, Level 2=部分自动化, Level 3=有条件自动化 */
    p_bp_server_->set_autonomous_level(3);
    /* MPDM 特有: 启用 HMI (Human-Machine Interface) 接口,
     * 允许通过 Joy 消息远程发送换道/加速/减速等指令,
     * 与 terminal_server.py (pygame 交互终端) 配合实现人在回路控制 */
    p_bp_server_->enable_hmi_interface();

    /* 第3层: SSC 运动规划器 —— 轨迹生成与控制
     * 接收 MPDM 的行为决策 (目标车道、期望速度等语义指令),
     * 在 Frenet 坐标系下进行时空轨迹优化, 输出 vehicle_msgs::msg::ControlSignal */
    p_ssc_server_ = std::make_shared<planning::SscPlannerServer>(node, ssc_planner_work_rate, ego_id);
    p_ssc_server_->Init(ssc_config_path);
    p_bp_server_->Init();
    smm_ros_adapter->Init();

    // 启动内部定时器, 开始按设定频率执行规划循环
    p_bp_server_->Start();
    p_ssc_server_->Start();

    // 主循环: 以 100Hz 频率轮询 ROS2 回调队列, 确保消息及时处理
    rclcpp::Rate rate(100);
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      rate.sleep();
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}
