/**
 * @file ssc_server_ros.cc
 * @brief SSC 规划器 ROS2 服务器封装层的实现
 *
 * [概述]
 * 本文件实现了 SscPlannerServer 的全部功能，负责将 SSC 规划器
 * 与 ROS2 框架集成，管理规划器的生命周期和运行时行为。
 *
 * [核心功能]
 * 1. 异步地图获取:
 *    PushSemanticMap() 使用 lock-free 队列接收语义地图，
 *    PlanCycleCallback() 每周期从中提取最新数据。
 *
 * 2. 轨迹管理 (双缓冲):
 *    executing_traj_ = 当前车辆正在跟踪的轨迹
 *    next_traj_      = 下一帧将切换到的轨迹
 *    通过这种设计实现平滑的轨迹切换，避免间隙。
 *
 * 3. 控制信号发布 (PublishData):
 *    两种模式:
 *    - use_sim_state_ = true:  根据固定时间步长从轨迹上采样期望状态
 *    - use_sim_state_ = false: 仅当轨迹无效时才重新规划
 *
 * 4. 重新规划 (Replan):
 *    触发条件:
 *    - 当前无有效执行轨迹 (首次规划)
 *    - 当前时间超出执行轨迹范围 (轨迹到期)
 *    - 下一轨迹为空或无效
 *    重规划流程:
 *      a. 根据当前时间计算期望时刻 t
 *      b. 从 executing_traj_ 提取 t 时刻的期望状态
 *      c. 对低速状态进行奇异点滤波
 *      d. 调用 planner_.RunOnce() 生成新轨迹
 *      e. 存入 next_traj_ 供下一帧使用
 *
 * [线程模型]
 *   MainThread: 独立线程, 按 work_rate_ 频率循环
 *   PlanCycleCallback 在 MainThread 中执行 (单生产者单消费者模式)
 *   PushSemanticMap 可由外部线程安全调用
 */
#include "ssc_planner/ssc_server_ros.h"

namespace planning {

/// @brief 构造函数 (默认频率 20Hz)
SscPlannerServer::SscPlannerServer(std::shared_ptr<rclcpp::Node> node, int ego_id)
    : node_(node), work_rate_(20.0), ego_id_(ego_id) {
  // 创建 lock-free 输入缓冲队列
  p_input_smm_buff_ = std::make_unique<moodycamel::ReaderWriterQueue<SemanticMapManager>>(config_.kInputBufferSize);
  // 创建可视化器
  p_smm_vis_ = std::make_unique<semantic_map_manager::Visualizer>(node, ego_id);
  p_ssc_vis_ = std::make_unique<SscVisualizer>(node, ego_id);
}

/// @brief 构造函数 (可指定频率)
SscPlannerServer::SscPlannerServer(std::shared_ptr<rclcpp::Node> node, double work_rate, int ego_id)
    : node_(node), work_rate_(work_rate), ego_id_(ego_id) {
  p_input_smm_buff_ = std::make_unique<moodycamel::ReaderWriterQueue<SemanticMapManager>>(config_.kInputBufferSize);
  p_smm_vis_ = std::make_unique<semantic_map_manager::Visualizer>(node, ego_id);
  p_ssc_vis_ = std::make_unique<SscVisualizer>(node, ego_id);
}

/// @brief 异步压入语义地图 (线程安全)
///
/// 使用 try_enqueue 而非阻塞写入，确保实时性。
/// 如果队列满，当前地图会被丢弃（以保持规划频率）。
void SscPlannerServer::PushSemanticMap(const SemanticMapManager& smm) {
  if (p_input_smm_buff_) p_input_smm_buff_->try_enqueue(smm);
}

/// @brief 发布全部 ROS 数据
///
/// 发布内容:
///   1. 语义地图可视化 (通过 p_smm_vis_)
///   2. SSC 可视化 (通过 p_ssc_vis_)
///   3. 控制信号 (从执行轨迹采样期望状态)
///   4. 轨迹可视化 (显示执行轨迹 + 可能需要接管的警告)
void SscPlannerServer::PublishData() {
  using common::VisualizationUtil;
  auto current_time = node_->now();

  // 1. 语义地图可视化
  {
    p_smm_vis_->VisualizeDataWithStamp(current_time, last_smm_);
    p_smm_vis_->SendTfWithStamp(current_time, last_smm_);
  }

  // 2. SSC 规划器内部数据可视化
  {
    TicToc timer;
    p_ssc_vis_->VisualizeDataWithStamp(current_time, planner_);
    RCLCPP_INFO(node_->get_logger(), "ssc vis all time cost: %lf ms", timer.toc());
  }

  // 3. 轨迹执行反馈 (基于当前执行轨迹发布控制信号)
  {
    if (executing_traj_ == nullptr || !executing_traj_->IsValid()) return;

    if (use_sim_state_) {
      RCLCPP_INFO(node_->get_logger(), "use_sim_state_: true.");
    } else {
      RCLCPP_INFO(node_->get_logger(), "use_sim_state_: false.");
    }

    if (use_sim_state_) {
      // --- 仿真状态模式: 按固定时间步长从轨迹提取期望状态 ---
      decimal_t plan_horizon = 1.0 / work_rate_;  // 控制周期 (秒)
      // 计算从轨迹开始至今经过了多少个控制周期
      int num_cycles =
          std::floor((current_time.seconds() - executing_traj_->begin()) / plan_horizon);
      // 离散化时间: 下一个控制点时刻
      decimal_t ct = executing_traj_->begin() + num_cycles * plan_horizon;
      {
        common::State state;
        if (executing_traj_->GetState(ct, &state) == kSuccess) {
          // 奇异点滤波: 消除低速时的不合理航向角跳变
          FilterSingularityState(ctrl_state_hist_, &state);
          ctrl_state_hist_.push_back(state);
          // 限制历史长度
          if (ctrl_state_hist_.size() > 100)
            ctrl_state_hist_.erase(ctrl_state_hist_.begin());

          // 编码并发布控制信号
          vehicle_msgs::msg::ControlSignal ctrl_msg;
          vehicle_msgs::Encoder::GetRosControlSignalFromControlSignal(
              common::VehicleControlSignal(state), current_time, std::string("map"), &ctrl_msg);
          ctrl_signal_pub_->publish(ctrl_msg);
        } else {
          RCLCPP_WARN(node_->get_logger(), "cannot evaluate state at %lf with begin %lf.",
                      ct, executing_traj_->begin());
        }
      }
    }

    // 4. 轨迹可视化
    {
      auto color = common::cmap["magenta"];  // 默认紫色
      if (require_intervention_signal_) color = common::cmap["yellow"];  // 需要接管时黄色

      visualization_msgs::msg::MarkerArray traj_mk_arr;
      // 将 Frenet 轨迹转换为 MarkerArray (按 0.1s 采样)
      common::VisualizationUtil::GetMarkerArrayByTrajectory(
          *executing_traj_, 0.1, Vecf<3>(0.3, 0.3, 0.3), color, 0.5, &traj_mk_arr);

      // 如果需要人工接管, 在轨迹起点上方显示警告文字
      if (require_intervention_signal_) {
        visualization_msgs::msg::Marker traj_status;
        common::State state_begin;
        executing_traj_->GetState(executing_traj_->begin(), &state_begin);
        Vec3f pos = Vec3f(state_begin.vec_position[0], state_begin.vec_position[1], 5.0);
        common::VisualizationUtil::GetRosMarkerTextUsingPositionAndString(
            pos, std::string("Intervention Needed!"), common::cmap["red"],
            Vec3f(5.0, 5.0, 5.0), 0, &traj_status);
        traj_mk_arr.markers.push_back(traj_status);
      }

      // 处理 Marker ID 递增以避免残留
      int num_traj_mks = static_cast<int>(traj_mk_arr.markers.size());
      common::VisualizationUtil::FillHeaderIdInMarkerArray(
          current_time, std::string("map"), last_trajmk_cnt_, &traj_mk_arr);
      last_trajmk_cnt_ = num_traj_mks;
      executing_traj_vis_pub_->publish(traj_mk_arr);
    }
  }
}

/// @brief 低速奇异状态滤波
///
/// 问题背景:
///   当车辆速度极低 (近乎静止) 时，航向角 (yaw) 的估计可能非常不准确，
///   导致连续的航向角之间出现不合理的跳变。若直接用跳跃后的角度规划，
///   可能产生不可行的急转弯轨迹。
///
/// 检测条件:
///   velocity < kBigEPS (≈0 的速度)
///   && |angle_diff| > max_orientation_change (角度变化超出运动学极限)
///
/// 运动学极限: max_orientation_rate = tan(max_steer) / wheel_base * singular_velocity
///   即: 在近乎零速下，转动方向盘最大角度所产生的最大航向变化率
///
/// 滤波策略: 若检测到奇异，保持上一帧的有效航向角
///
/// @param hist         历史控制状态序列
/// @param filter_state 待检测/滤波的状态 (in/out)
ErrorType SscPlannerServer::FilterSingularityState(
    const vec_E<common::State>& hist, common::State* filter_state) {
  if (hist.empty()) {
    return kWrongStatus;
  }

  decimal_t duration = filter_state->time_stamp - hist.back().time_stamp;
  decimal_t wheel_base = 2.85;        // 轴距 (米)
  decimal_t max_steer = M_PI / 4.0;   // 最大转向角 (45度)
  decimal_t singular_velocity = kBigEPS;  // 判定为"静止"的速度阈值
  // 在奇异速度下允许的最大航向变化率
  decimal_t max_orientation_rate = tan(max_steer) / 2.85 * singular_velocity;
  // 在当前时间间隔内允许的最大航向变化量
  decimal_t max_orientation_change = max_orientation_rate * duration;

  // 检测: 速度极低且角度变化超出运动学允许范围
  if (fabs(filter_state->velocity) < singular_velocity &&
      fabs(normalize_angle(filter_state->angle - hist.back().angle)) >
          max_orientation_change) {
    RCLCPP_WARN(node_->get_logger(), "Detect singularity velocity %lf angle (%lf, %lf).",
                filter_state->velocity, hist.back().angle, filter_state->angle);
    // 将航向角重置为上一帧的值
    filter_state->angle = hist.back().angle;
    RCLCPP_WARN(node_->get_logger(), "Filter angle to %lf.", hist.back().angle);
  }

  return kSuccess;
}

/// @brief 初始化规划服务器
///
/// 步骤:
///   1. 初始化 SSC 规划器 (加载配置)
///   2. 获取 use_sim_state 参数
///   3. 创建 ROS 发布者: ctrl_signal, ssc_map, executing_traj_vis
void SscPlannerServer::Init(const std::string& config_path) {
  planner_.Init(config_path);

  // 构造各 agent 的专属话题名称
  std::string traj_topic = std::string("/vis/agent_") +
                           std::to_string(ego_id_) +
                           std::string("/ssc/exec_traj");

  // 从 ROS 参数服务器获取仿真状态模式
  node_->get_parameter("use_sim_state", use_sim_state_);

  // 创建发布者
  ctrl_signal_pub_ = node_->create_publisher<vehicle_msgs::msg::ControlSignal>("ctrl", 20);
  map_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("ssc_map", 1);
  executing_traj_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(traj_topic, 1);
}

/// @brief 启动规划线程
///
/// 设置地图接口 (适配器) 后将 is_replan_on_ 置为 true,
/// 并创建独立的 MainThread 线程以 work_rate_ 频率运行规划循环。
void SscPlannerServer::Start() {
  if (is_replan_on_) {
    return;  // 防止重复启动
  }
  planner_.set_map_interface(&map_adapter_);
  RCLCPP_INFO(node_->get_logger(), "Planner server started.");
  is_replan_on_ = true;

  // 在独立线程中运行 (detach 以避免阻塞)
  std::thread(&SscPlannerServer::MainThread, this).detach();
}

/// @brief 主循环线程: 按固定频率执行 PlanCycleCallback
///
/// 使用 sleep_until 而非 sleep_for 以保持精确的循环频率,
/// 避免误差累积导致的周期漂移。
void SscPlannerServer::MainThread() {
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

/// @brief 每个规划周期的主回调
///
/// 执行流程:
///   1. 从 lock-free 队列提取最新语义地图 (非阻塞)
///   2. 更新地图适配器
///   3. 发布可视化数据
///   4. 轨迹管理: 判断是否需要重新规划
///
/// 重新规划触发逻辑:
///   - 无有效执行轨迹 → 首次规划
///   - 当前时间超出轨迹结束时间 → 任务完成, 清空轨迹
///   - 下一轨迹为空或无效 → 重新规划
///   - 有有效下一轨迹 → 切换为执行轨迹, 并重新规划下一轨迹
void SscPlannerServer::PlanCycleCallback() {
  if (!is_replan_on_) {
    return;
  }

  // 从缓冲队列中提取最新语义地图 (非阻塞, 只取最新)
  while (p_input_smm_buff_->try_dequeue(last_smm_)) {
    is_map_updated_ = true;
  }

  if (!is_map_updated_) return;  // 无新数据, 跳过本周期

  // 更新地图适配器 (传递语义地图快照的共享指针)
  auto map_ptr = std::make_shared<semantic_map_manager::SemanticMapManager>(last_smm_);
  map_adapter_.set_map(map_ptr);

  // 发布可视化与控制信号
  PublishData();

  auto current_time = node_->now().seconds();
  RCLCPP_INFO(node_->get_logger(), ">>>>>>>current time %lf.", current_time);

  // 情况1: 首次规划 或 非仿真状态模式 → 直接规划
  if (executing_traj_ == nullptr || !executing_traj_->IsValid() || !use_sim_state_) {
    if (planner_.RunOnce() != kSuccess) {
      return;
    }

    // 重置历史状态 (用于后续的奇异点检测)
    desired_state_hist_.clear();
    desired_state_hist_.push_back(last_smm_.ego_vehicle().state());
    ctrl_state_hist_.clear();
    ctrl_state_hist_.push_back(last_smm_.ego_vehicle().state());

    // 获取规划结果作为执行轨迹
    executing_traj_ = std::move(planner_.trajectory());
    global_init_stamp_ = executing_traj_->begin();
    RCLCPP_INFO(node_->get_logger(), "init plan success with stamp: %lf and angle %lf.",
                global_init_stamp_, last_smm_.ego_vehicle().state().angle);
    return;
  }

  // 情况2: 当前时间已超出执行轨迹的有效期 → 任务完成
  if (current_time > executing_traj_->end()) {
    RCLCPP_INFO(node_->get_logger(), "Current time %lf out of [%lf, %lf].",
                current_time, executing_traj_->begin(), executing_traj_->end());
    RCLCPP_INFO(node_->get_logger(), "Mission complete.");
    executing_traj_.reset();
    next_traj_.reset();
    return;
  }

  // 情况3: 下一轨迹为空或无效 → 需要重新规划
  if (next_traj_ == nullptr || !next_traj_->IsValid()) {
    Replan();
    return;
  }

  // 情况4: 下一轨迹有效 → 切换执行轨迹并规划后续
  if (next_traj_->IsValid()) {
    executing_traj_ = std::move(next_traj_);
    Replan();
    return;
  }
}

/// @brief 执行重新规划
///
/// 步骤:
///   1. 计算下一个期望控制时刻 t
///   2. 从 executing_traj_ 提取 t 时刻的期望状态
///   3. 奇异点滤波 (防止低速下的角度跳变)
///   4. 将期望状态设为规划器的初始状态
///   5. 调用 planner_.RunOnce() 生成新轨迹
///   6. 存入 next_traj_
void SscPlannerServer::Replan() {
  if (!is_replan_on_) return;
  if (executing_traj_ == nullptr || !executing_traj_->IsValid()) return;

  decimal_t plan_horizon = 1.0 / work_rate_;
  common::State desired_state;
  decimal_t cur_time = node_->now().seconds();

  // 计算: 从全局初始时刻到当前, 已过去了多少个周期
  int num_cycles_exec = std::floor((executing_traj_->begin() - global_init_stamp_) / plan_horizon);
  // 下一个期望控制周期的索引 (至少比已执行的周期多1)
  int num_cycles_ahead = cur_time > executing_traj_->begin()
                             ? std::floor((cur_time - global_init_stamp_) / plan_horizon) + 1
                             : num_cycles_exec + 1;
  // 下一个期望控制点时刻
  decimal_t t = global_init_stamp_ + plan_horizon * num_cycles_ahead;

  RCLCPP_INFO(node_->get_logger(), "init stamp: %lf, plan horizon: %lf, num cycles %d.",
              global_init_stamp_, plan_horizon, num_cycles_ahead);
  RCLCPP_INFO(node_->get_logger(), "Replan at cur time %lf with executing traj begin time: %lf to rounded t: %lf.",
              cur_time, executing_traj_->begin(), t);

  // 从执行轨迹提取期望状态
  if (executing_traj_->GetState(t, &desired_state) != kSuccess) {
    RCLCPP_WARN(node_->get_logger(), "Cannot get desired state at %lf.", t);
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "t %lf, desired state (x,y,v,a,theta):(%lf, %lf, %lf, %lf, %lf).",
              t, desired_state.vec_position[0], desired_state.vec_position[1],
              desired_state.velocity, desired_state.acceleration, desired_state.angle);

  // 奇异点滤波: 防止低速时的角度跳变
  FilterSingularityState(desired_state_hist_, &desired_state);
  desired_state_hist_.push_back(desired_state);
  if (desired_state_hist_.size() > 100)
    desired_state_hist_.erase(desired_state_hist_.begin());

  // 将期望状态设为规划器的起始状态
  planner_.set_initial_state(desired_state);

  // 执行规划
  time_profile_tool_.tic();
  if (planner_.RunOnce() != kSuccess) {
    RCLCPP_WARN(node_->get_logger(), "Ssc planner core failed in %lf ms.", time_profile_tool_.toc());
    require_intervention_signal_ = true;
    return;
  }

  require_intervention_signal_ = false;
  RCLCPP_INFO(node_->get_logger(), "Ssc planner succeed in %lf ms.", time_profile_tool_.toc());

  // 存储规划结果
  next_traj_ = std::move(planner_.trajectory());
}

}  // namespace planning
