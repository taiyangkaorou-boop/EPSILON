/**
 * @file behavior_server_ros.cc
 * @brief ROS2 行为规划器服务器的实现
 *
 * 实现 BehaviorPlannerServer 类的全部方法，包括：
 *   - 构造函数：创建可视化模块和无锁输入队列
 *   - Init：初始化行为规划器、订阅 HMI 话题、读取 ROS 参数
 *   - Start：绑定地图适配器、启动独立规划线程
 *   - MainThread：以固定频率驱动规划循环
 *   - PlanCycleCallback：取出最新语义地图、调用 MPDM、发布结果
 *   - JoyCallback：解析游戏手柄输入为变道/调速命令
 *   - PushSemanticMap：异步推送地图数据到无锁队列
 *   - 各种 setter/getter：对 BehaviorPlanner 的封装
 */
#include "behavior_planner/behavior_server_ros.h"

namespace planning {

// ============================================================================
// 构造函数
// ============================================================================

/**
 * @brief 构造函数（默认工作频率 20Hz）
 *
 * 创建两个核心组件：
 *   1. BehaviorPlannerVisualizer: 前向轨迹可视化发布器，绑定到 bp_ 和当前 node_
 *   2. ReaderWriterQueue: 无锁并发队列，用于异步接收语义地图数据
 *
 * @param node ROS2 节点共享指针
 * @param ego_id 自车 ID
 */
BehaviorPlannerServer::BehaviorPlannerServer(std::shared_ptr<rclcpp::Node> node, int ego_id)
  : node_(node), work_rate_(20.0), ego_id_(ego_id) {
    p_visualizer_ = std::make_unique<BehaviorPlannerVisualizer>(node, &bp_, ego_id);
    p_input_smm_buff_ = std::make_unique<moodycamel::ReaderWriterQueue<SemanticMapManager>>(config_.kInputBufferSize);
}

/**
 * @brief 构造函数（指定工作频率）
 *
 * @param node ROS2 节点共享指针
 * @param work_rate 规划循环频率 [Hz]
 * @param ego_id 自车 ID
 */
BehaviorPlannerServer::BehaviorPlannerServer(std::shared_ptr<rclcpp::Node> node, double work_rate, int ego_id)
  : node_(node), work_rate_(work_rate), ego_id_(ego_id) {
    p_visualizer_ = std::make_unique<BehaviorPlannerVisualizer>(node, &bp_, ego_id);
    p_input_smm_buff_ = std::make_unique<moodycamel::ReaderWriterQueue<SemanticMapManager>>(config_.kInputBufferSize);
}

// ============================================================================
// 数据输入接口
// ============================================================================

/**
 * @brief 向无锁队列推送语义地图数据
 *
 * 由外部生产者线程（通常是语义地图管理器的更新回调）调用。
 * 使用 try_enqueue 而非阻塞入队，队列满时直接丢弃旧数据帧，
 * 确保系统不会因地图更新积压而产生延迟累积。
 *
 * @param smm 语义地图管理器副本
 */
void BehaviorPlannerServer::PushSemanticMap(const SemanticMapManager &smm) {
  if (p_input_smm_buff_) p_input_smm_buff_->try_enqueue(smm);
}

// ============================================================================
// 初始化
// ============================================================================

/**
 * @brief 初始化 ROS 服务器
 *
 * 执行步骤：
 *   1. 初始化 BehaviorPlanner 核心（创建 RoutePlanner 等内部组件）
 *   2. 若自动驾驶等级 >= L2，订阅 /joy 话题用于 HMI 交互
 *   3. 从 ROS 参数服务器读取 use_sim_state 参数
 *   4. 初始化可视化发布器
 */
void BehaviorPlannerServer::Init() {
  bp_.Init("bp");

  // L2+ 级别启用 HMI 接口（游戏手柄话题订阅）
  if (bp_.autonomous_level() >= 2) {
    joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&BehaviorPlannerServer::JoyCallback, this, std::placeholders::_1));
  }

  // 从 ROS 参数服务器读取是否使用仿真状态
  bool use_sim_state = true;
  // node_->declare_parameter("use_sim_state", use_sim_state);
  node_->get_parameter("use_sim_state", use_sim_state);
  bp_.set_use_sim_state(use_sim_state);

  // 初始化前向轨迹可视化
  p_visualizer_->Init();
}

// ============================================================================
// 启动
// ============================================================================

/**
 * @brief 启动规划服务器
 *
 * 将地图适配器注入到行为规划器（依赖注入），然后启动独立规划线程。
 * 使用 detach 模式使规划线程与 ROS 主线程完全独立运行。
 */
void BehaviorPlannerServer::Start() {
  // 告知行为规划器使用哪个地图接口实现
  bp_.set_map_interface(&map_adapter_);

  // 启动独立规划线程（detach 模式，不阻塞主线程）
  std::thread(&BehaviorPlannerServer::MainThread, this).detach();
}

// ============================================================================
// 主规划循环
// ============================================================================

/**
 * @brief 规划主线程函数
 *
 * 以 work_rate_ 固定频率循环调用 PlanCycleCallback。
 *
 * 实现细节：
 *   - 使用 system_clock 和 sleep_until 实现精确的周期控制
 *   - interval = 1000.0 / work_rate_ [ms]
 *   - 循环条件：rclcpp::ok()（ROS 节点存活时继续）
 */
void BehaviorPlannerServer::MainThread() {
  using namespace std::chrono;
  system_clock::time_point current_start_time{system_clock::now()};
  system_clock::time_point next_start_time{current_start_time};

  // 根据工作频率计算周期间隔（毫秒）
  const milliseconds interval(static_cast<int>(1000.0 / work_rate_));

  while (rclcpp::ok()) {
    current_start_time = system_clock::now();
    next_start_time = current_start_time + interval;

    // 执行单次规划循环
    PlanCycleCallback();

    // 精确休眠到下一个周期开始时刻
    std::this_thread::sleep_until(next_start_time);
  }
}

// ============================================================================
// 规划循环回调
// ============================================================================

/**
 * @brief 单次规划循环
 *
 * 从无锁队列中取出最新语义地图，运行 MPDM 决策，发布结果。
 * 完整的数据流：
 *
 *   1. 从无锁队列持续 dequeue 至空（保留最新一帧，丢弃中间过时帧）
 *   2. 将最新语义地图包装为 shared_ptr 注入到 map_adapter_
 *   3. 调用 bp_.RunOnce() 执行 MPDM 决策全流程
 *   4. 将决策结果（behavior）写回 smm 的 ego_behavior 字段
 *   5. 若已绑定外部回调，将更新后的 smm 传给下游模块
 *   6. 发布前向轨迹可视化数据
 */
void BehaviorPlannerServer::PlanCycleCallback() {
  if (p_input_smm_buff_ == nullptr) return;

  SemanticMapManager smm;
  bool has_updated_map = false;

  // 持续从队列取出数据直到空，保留最后一帧
  // 这样跳过中间的过时数据帧，只处理最新的地图
  while (p_input_smm_buff_->try_dequeue(smm)) {
    has_updated_map = true;
  }

  // 仅当有新地图数据到达时才执行规划
  if (has_updated_map) {
    // 将语义地图注入到适配器（使用 shared_ptr 管理生命周期）
    auto map_ptr = std::make_shared<semantic_map_manager::SemanticMapManager>(smm);
    map_adapter_.set_map(map_ptr);

    TicToc timer;
    // 核心：运行 MPDM 行为决策
    if (bp_.RunOnce() == kSuccess) {
      // 将决策结果写入语义地图，供下游模块（如轨迹规划器）使用
      smm.set_ego_behavior(bp_.behavior());
    }

    // 通知下游模块（如轨迹规划器、控制器等）
    if (has_callback_binded_) {
      private_callback_fn_(smm);
    }

    // 发布前向轨迹可视化（用于 RViz 调试）
    PublishData();
  }
}

// ============================================================================
// 回调绑定
// ============================================================================

/**
 * @brief 绑定行为更新回调
 *
 * 回调在每次 MPDM 决策完成后被调用，用于通知下游模块行为决策结果。
 *
 * @param fn 回调函数，签名为 int(const SemanticMapManager &)
 */
void BehaviorPlannerServer::BindBehaviorUpdateCallback(std::function<int(const SemanticMapManager &)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

// ============================================================================
// 可视化发布
// ============================================================================

/// 发布前向仿真轨迹可视化数据
void BehaviorPlannerServer::PublishData() {
  p_visualizer_->PublishDataWithStamp(node_->get_clock()->now());
}

/// 重新规划（预留接口，暂未实现）
void BehaviorPlannerServer::Replan() {}

// ============================================================================
// HMI 人机接口：游戏手柄回调
// ============================================================================

/**
 * @brief 游戏手柄 Joy 消息回调
 *
 * 按钮映射：
 *   - buttons[2] = 1: 向左变道（LaneChangeLeft）
 *   - buttons[1] = 1: 向右变道（LaneChangeRight）
 *   - buttons[3] = 1: 加速 +1 m/s
 *   - buttons[0] = 1: 减速 -1 m/s
 *
 * 安全限制：
 *   - 仅在 autonomous_level >= 2 且 is_hmi_enabled_ 为 true 时处理
 *   - 仅处理 frame_id 匹配 ego_id_ 的消息（多车场景区分遥控器）
 *
 * @param msg Joy 消息（包含 12 个按键状态）
 */
void BehaviorPlannerServer::JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
  // L1 级别不处理 HMI 输入
  if (bp_.autonomous_level() < 2) return;
  // HMI 接口未启用时不处理
  if (!is_hmi_enabled_) return;

  // 解析 frame_id 作为车辆标识，多车场景下区分遥控器
  int msg_id;
  if (msg->header.frame_id.empty()) {
    msg_id = 0;
  } else {
    msg_id = std::stoi(msg->header.frame_id);
  }

  // 仅处理发送给本车的遥控指令
  if (msg_id != ego_id_) return;

  // 无按键按下时快速返回
  if (msg->buttons[0] == 0 && msg->buttons[1] == 0 && msg->buttons[2] == 0 && msg->buttons[3] == 0)
    return;

  // 按钮 [2]: 向左变道
  if (msg->buttons[2] == 1) {
    bp_.set_hmi_behavior(common::LateralBehavior::kLaneChangeLeft);
  }
  // 按钮 [1]: 向右变道
  else if (msg->buttons[1] == 1) {
    bp_.set_hmi_behavior(common::LateralBehavior::kLaneChangeRight);
  }
  // 按钮 [3]: 加速 +1 m/s
  else if (msg->buttons[3] == 1) {
    bp_.set_user_desired_velocity(bp_.user_desired_velocity() + 1.0);
  }
  // 按钮 [0]: 减速 -1 m/s
  else if (msg->buttons[0] == 1) {
    bp_.set_user_desired_velocity(bp_.user_desired_velocity() - 1.0);
  }
}

// ============================================================================
// 参数设置接口（对 BehaviorPlanner 的封装）
// ============================================================================

/// 设置自动驾驶等级
void BehaviorPlannerServer::set_autonomous_level(int level) {
  bp_.set_autonomous_level(level);
}

/// 设置激进等级
void BehaviorPlannerServer::set_aggressive_level(int level) {
  bp_.set_aggressive_level(level);
}

/// 设置用户期望速度
void BehaviorPlannerServer::set_user_desired_velocity(const decimal_t desired_vel) {
  bp_.set_user_desired_velocity(desired_vel);
}

/// 获取用户设定期望速度
decimal_t BehaviorPlannerServer::user_desired_velocity() const {
  return bp_.user_desired_velocity();
}

/// 获取参考期望速度（受曲率等限制后的安全速度）
decimal_t BehaviorPlannerServer::reference_desired_velocity() const {
  return bp_.reference_desired_velocity();
}

/// 启用 HMI 人机接口
void BehaviorPlannerServer::enable_hmi_interface() {
  is_hmi_enabled_ = true;
}

}  // namespace planning
