/**
 * @file eudm_server_ros.cc
 * @author EPSILON Autonomous Driving Team
 * @brief EUDM规划器ROS2服务器实现
 * @version 0.1
 * @date 2019-07-07
 * @copyright Copyright (c) 2019
 *
 * 本文件实现了EudmPlannerServer类——EUDM行为规划系统与ROS2框架
 * 之间的桥梁。该类管理异步规划循环、HMI交互和ROS2消息通信。
 *
 * ## 并发设计
 *
 * 使用两个线程来解耦数据接收和规划计算：
 * - **ROS回调线程**：PushSemanticMap()向输入缓冲区推送语义地图数据
 * - **规划线程（MainThread）**：以固定频率从缓冲区取出数据进行规划
 *
 * 两个线程通过moodycamel::ReaderWriterQueue（无锁并发队列）通信，
 * 避免了mutex锁竞争带来的延迟不确定性。
 *
 * ## 时间调度
 *
 * 规划循环使用std::this_thread::sleep_until而不是sleep_for，
 * 确保严格的时间对齐，避免累积漂移。
 *
 * ## Joy手柄映射表
 *
 * | 按钮     | 功能              | 行为                       |
 * |----------|-------------------|----------------------------|
 * | button[2]| 左变道            | behavior = -1 (toggle)     |
 * | button[1]| 右变道            | behavior = 1 (toggle)      |
 * | button[3]| 加速              | desired_vel += 1.0 m/s     |
 * | button[0]| 减速              | desired_vel = max(vel-1,0) |
 * | button[4]| 禁止左变道         | forbid_left toggle         |
 * | button[5]| 禁止右变道         | forbid_right toggle        |
 * | button[6]| 自动驾驶开关       | is_under_ctrl toggle       |
 */

#include "eudm_planner/eudm_server_ros.h"

namespace planning {

/// @brief 构造函数（默认20Hz工作频率）
EudmPlannerServer::EudmPlannerServer(std::shared_ptr<rclcpp::Node> node, int ego_id)
    : node_(node), work_rate_(20.0), ego_id_(ego_id) {
  // 创建可视化工具
  p_visualizer_ = std::make_unique<EudmPlannerVisualizer>(node, &bp_manager_, ego_id);
  // 创建无锁输入缓冲区
  p_input_smm_buff_ = std::make_unique<moodycamel::ReaderWriterQueue<SemanticMapManager>>(config_.kInputBufferSize);
  // 初始化用户偏好行为为无偏好
  task_.user_perferred_behavior = 0;
}

/// @brief 构造函数（自定义工作频率）
EudmPlannerServer::EudmPlannerServer(std::shared_ptr<rclcpp::Node> node, double work_rate, int ego_id)
    : node_(node), work_rate_(work_rate), ego_id_(ego_id) {
  p_visualizer_ = std::make_unique<EudmPlannerVisualizer>(node, &bp_manager_, ego_id);
  p_input_smm_buff_ = std::make_unique<moodycamel::ReaderWriterQueue<SemanticMapManager>>(config_.kInputBufferSize);
  task_.user_perferred_behavior = 0;
}

/// @brief 推送语义地图数据到输入缓冲区（非阻塞）
///
/// 由上游模块的ROS回调线程调用。使用try_enqueue确保非阻塞语义，
/// 若队列已满则丢弃数据（以避免内存问题，但不应常态化发生）。
void EudmPlannerServer::PushSemanticMap(const SemanticMapManager &smm) {
  if (p_input_smm_buff_) p_input_smm_buff_->try_enqueue(smm);
}

/// @brief 发布可视化数据
void EudmPlannerServer::PublishData() {
  p_visualizer_->PublishDataWithStamp(node_->get_clock()->now());
}

/// @brief 初始化ROS服务器
///
/// 操作步骤：
/// 1. 初始化EudmManager（读取配置、创建EudmPlanner等）
/// 2. 创建Joy话题订阅，接收用户操作信号
/// 3. 从ROS参数服务器读取"use_sim_state"参数
/// 4. 初始化可视化工具（创建MarkerArray发布者）
void EudmPlannerServer::Init(const std::string &bp_config_path) {
  bp_manager_.Init(bp_config_path, work_rate_);
  // 绑定Joy回调
  auto joy_callback = std::bind(&EudmPlannerServer::JoyCallback, this, std::placeholders::_1);
  joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, joy_callback);
  // 从参数服务器读取仿真状态使用标志
  node_->get_parameter("use_sim_state", use_sim_state_);
  p_visualizer_->Init();
  p_visualizer_->set_use_sim_state(use_sim_state_);
}

/// @brief Joy手柄回调——处理用户交互控制
///
/// 按钮映射：
/// - buttons[2]: 左变道请求（toggle模式：按一下请求，再按一下取消）
/// - buttons[1]: 右变道请求（toggle模式）
/// - buttons[3]: 加速+1m/s
/// - buttons[0]: 减速-1m/s（下限0）
/// - buttons[4]: 切换禁止左变道标志（用于模拟法规限制）
/// - buttons[5]: 切换禁止右变道标志
/// - buttons[6]: 切换自动驾驶控制状态
///
/// 只处理与ego_id匹配的Joy消息（多用户场景）。
void EudmPlannerServer::JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
  int msg_id;
  if (std::string("").compare(msg->header.frame_id) == 0) {
    msg_id = 0;
  } else {
    msg_id = std::stoi(msg->header.frame_id);
  }
  // 只处理匹配自车ID的消息
  if (msg_id != ego_id_) return;

  // buttons[2] --> 1: 左变道请求
  // buttons[1] --> 1: 右变道请求
  // buttons[3] --> 1: +1m/s
  // buttons[0] --> 1: -1m/s

  // 如果所有按钮都没有按下，直接返回（节省处理时间）
  if (msg->buttons[0] == 0 && msg->buttons[1] == 0 && msg->buttons[2] == 0 &&
      msg->buttons[3] == 0 && msg->buttons[4] == 0 && msg->buttons[5] == 0 &&
      msg->buttons[6] == 0)
    return;

  // 左变道：button[2]，toggle模式
  if (msg->buttons[2] == 1) {
    if (task_.user_perferred_behavior != -1) {
      task_.user_perferred_behavior = -1;  // 请求左变道
    } else {
      task_.user_perferred_behavior = 0;   // 取消请求
    }
  } else if (msg->buttons[1] == 1) {
    // 右变道：button[1]，toggle模式
    if (task_.user_perferred_behavior != 1) {
      task_.user_perferred_behavior = 1;   // 请求右变道
    } else {
      task_.user_perferred_behavior = 0;   // 取消请求
    }
  } else if (msg->buttons[3] == 1) {
    // 加速：+1m/s
    task_.user_desired_vel = task_.user_desired_vel + 1.0;
  } else if (msg->buttons[0] == 1) {
    // 减速：-1m/s（不低于0）
    task_.user_desired_vel = std::max(task_.user_desired_vel - 1.0, 0.0);
  } else if (msg->buttons[4] == 1) {
    // 切换禁止左变道标志
    task_.lc_info.forbid_lane_change_left = !task_.lc_info.forbid_lane_change_left;
  } else if (msg->buttons[5] == 1) {
    // 切换禁止右变道标志
    task_.lc_info.forbid_lane_change_right = !task_.lc_info.forbid_lane_change_right;
  } else if (msg->buttons[6] == 1) {
    // 切换自动驾驶控制状态
    task_.is_under_ctrl = !task_.is_under_ctrl;
  }
}

/// @brief 启动异步规划线程
///
/// detach一个独立线程运行MainThread，开始以固定频率
/// 执行规划循环。同时设置is_under_ctrl = true。
void EudmPlannerServer::Start() {
  // 分离规划线程（独立于ROS回调线程）
  std::thread(&EudmPlannerServer::MainThread, this).detach();
  task_.is_under_ctrl = true;
}

/// @brief 主规划线程
///
/// 使用固定的时间步长（1000/work_rate_ ms）循环执行：
/// 1. 记录当前时间
/// 2. 计算下次循环开始时间
/// 3. 执行PlanCycleCallback（单次规划循环）
/// 4. sleep_until下一个周期开始时间
///
/// 使用sleep_until而非sleep_for确保无累积时间漂移。
void EudmPlannerServer::MainThread() {
  using namespace std::chrono;
  system_clock::time_point current_start_time{system_clock::now()};
  system_clock::time_point next_start_time{current_start_time};
  const milliseconds interval{static_cast<int>(1000.0 / work_rate_)};
  while (rclcpp::ok()) {
    current_start_time = system_clock::now();
    next_start_time = current_start_time + interval;
    PlanCycleCallback();
    std::this_thread::sleep_until(next_start_time);
  }
}

/// @brief 单次规划循环回调
///
/// 执行流程：
/// 1. 从无锁输入缓冲区取出所有累积的语义地图数据（取最新）
/// 2. 如果没有新数据，直接返回（空闲跳过）
/// 3. 将时间戳对齐到规划周期边界
/// 4. 调用EudmManager::Run()执行完整规划生命周期
/// 5. 构建语义行为并注入语义地图
/// 6. 调用绑定的外部回调函数（通知上下游模块）
/// 7. 发布可视化数据
void EudmPlannerServer::PlanCycleCallback() {
  if (p_input_smm_buff_ == nullptr) return;

  // 从缓冲区取出所有数据，最后一个有效数据被保留
  bool has_updated_map = false;
  while (p_input_smm_buff_->try_dequeue(smm_)) {
    has_updated_map = true;
  }

  // 如果没有新地图数据，跳过本次规划
  if (!has_updated_map) return;

  // 创建共享指针传递给EudmManager
  auto map_ptr = std::make_shared<semantic_map_manager::SemanticMapManager>(smm_);

  // 时间戳对齐：确保规划在固定时间网格上执行
  decimal_t replan_duration = 1.0 / work_rate_;
  double stamp = std::floor(smm_.time_stamp() / replan_duration) * replan_duration;

  // 运行EUDM规划循环
  if (bp_manager_.Run(stamp, map_ptr, task_) == kSuccess) {
    // 构建语义行为并注入语义地图
    common::SemanticBehavior behavior;
    bp_manager_.ConstructBehavior(&behavior);
    smm_.set_ego_behavior(behavior);
  }

  // 调用绑定的外部回调（通知其他模块）
  if (has_callback_binded_) {
    private_callback_fn_(smm_);
  }

  // 发布可视化数据
  PublishData();
}

/// @brief 绑定行为更新回调函数
void EudmPlannerServer::BindBehaviorUpdateCallback(
    std::function<int(const SemanticMapManager &)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

/// @brief 设置用户期望速度
void EudmPlannerServer::set_user_desired_velocity(const decimal_t desired_vel) {
  task_.user_desired_vel = desired_vel;
}

/// @brief 获取用户期望速度
decimal_t EudmPlannerServer::user_desired_velocity() const {
  return task_.user_desired_vel;
}

}  // namespace planning
