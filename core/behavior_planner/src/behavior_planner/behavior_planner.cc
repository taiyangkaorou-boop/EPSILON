/**
 * @file behavior_planner.cc
 * @brief MPDM（多策略决策）行为规划器的核心实现
 *
 * 本文件实现了 EPSILON 自动驾驶系统行为规划模块的完整 MPDM 算法流程。
 *
 * ============================================================================
 * MPDM 算法总览
 * ============================================================================
 *
 * MPDM（Multi-Policy Decision Making，多策略决策）是一种基于前向仿真与
 * 代价评估的高层行为决策算法。其核心思想是对每个候选的横向驾驶行为
 * （车道保持、左变道、右变道）进行多智能体前向推演，通过多维度代价函数
 * 评估每条仿真轨迹的质量，最终选取综合代价最小的行为作为决策输出。
 *
 * ----- 算法流程图 -----
 *
 *   RunOnce()  [总入口]
 *     |
 *     +---> 获取自车车道 ID (GetEgoLaneIdByPosition)
 *     |
 *     +---> 获取自车状态 (GetEgoVehicle)
 *     |
 *     +---> [可选] 运行路线规划器 (RunRoutePlanner)
 *     |
 *     +---> JudgeBehaviorByLaneId: 通过位置->车道ID->推断当前行为
 *     |
 *     +---> UpdateEgoLaneId: 更新车道ID + 刷新候选车道列表
 *     |
 *     +---> UpdateEgoBehavior: 状态机更新行为状态
 *     |
 *     +---> [L3级] RunMpdm()
 *     |       |
 *     |       +---> MultiBehaviorJudge()
 *     |               |
 *     |               +---> 枚举候选行为 (1~3个)
 *     |               |
 *     |               +---> 为每辆车构建参考车道
 *     |               |
 *     |               +---> FOR EACH 候选行为:
 *     |               |       SimulateEgoBehavior()
 *     |               |         |
 *     |               |         +---> MultiAgentSimForward() [首选]
 *     |               |         |     多智能体交互: 所有车互相影响
 *     |               |         |     对每辆车每步: 找前车 -> IDM -> 更新状态
 *     |               |         |
 *     |               |         +---> OpenloopSimForward() [回退]
 *     |               |               开环仿真: 周围车不反应自车
 *     |               |
 *     |               +---> EvaluateMultiPolicyTrajs()
 *     |               |       FOR EACH 候选轨迹:
 *     |               |         EvaluateSinglePolicyTraj()
 *     |               |           |-- cost_efficiency: 速度效率代价
 *     |               |           |-- cost_safety: 碰撞安全代价
 *     |               |           |-- cost_action: 动作切换代价
 *     |               |           total = action + safety + efficiency
 *     |               |
 *     |               +---> 选择最小代价行为
 *     |               |
 *     |               +---> 期望速度限幅 (max_vel_cmd_gap = 5.0 m/s)
 *     |               |
 *     |               +---> HMI 锁定检查
 *     |
 *     +---> ConstructReferenceLane: 构建最终参考车道
 *
 * ----- 代价函数详解 -----
 *
 * total_cost = cost_action + cost_safety + cost_efficiency
 *
 * 1. cost_action（动作代价）:
 *    - 车道保持: 0.0
 *    - 变道: 0.5（抑制频繁变道）
 *
 * 2. cost_safety（安全代价）:
 *    - 对每辆周围车辆的轨迹进行碰撞检查
 *    - 若在某时间步碰撞：cost += 0.01 * |v_ego - v_other| * 0.5
 *    - 使用膨化车辆（各方向+1m）增加安全裕度
 *
 * 3. cost_efficiency（效率代价）:
 *    - cost_ego_to_desired: |v_ego_terminal - v_desired| / 10.0
 *    - cost_leading_to_desired: 前车限速导致的效率损失
 *      = 1.5 * distance_residual_ratio * |Δv| / max(2.0, d_lead)
 *    - cost_efficiency = 0.5 * (ego + leading)
 *
 * ----- 前向仿真详解 -----
 *
 * 仿真参数：
 *   - sim_horizon_: 仿真时间窗口，默认 4.0s
 *   - sim_resolution_: 仿真步长，默认 0.4s
 *   - 总步数 = sim_horizon_ / sim_resolution_ = 10 步
 *
 * MultiAgentSimForward（多智能体交互仿真）:
 *   对每个仿真步 k（0..9）：
 *     1. 对每辆车 v（包括自车）：
 *        a. 设置期望速度：自车 = reference_desired_velocity_，其他车 = 初始速度
 *        b. 获取限速并修正期望速度
 *        c. 查找 v 参考车道上的前车（GetLeadingVehicleOnLane）
 *        d. 检查与前车是否已碰撞
 *        e. 使用 IDM 跟驰模型推进状态（PropagateOnce）
 *     2. 使用 state_cache 批量更新所有车辆状态
 *     3. 分别记录自车轨迹和周围车辆轨迹
 *
 * IDM 跟驰模型:
 *   a = a_max * [1 - (v/v0)^delta - (s_star / s)^2]
 *   其中 s_star = s0 + v*T + v*Δv/(2*sqrt(a_max*b))
 *
 * OpenloopSimForward（开环仿真，回退方案）:
 *   自车按 IDM 沿参考车道行驶，周围车辆各自独立沿各自车道行驶，
 *   彼此之间不进行交互。
 *
 * ----- 状态机（UpdateEgoBehavior）详解 -----
 *
 * 防止不合逻辑的行为跳变：
 *   - 车道保持 + 观测仍为车道保持 -> OK，保持
 *   - 车道保持 + 观测到变道行为 -> Undefined（异常：车辆不应出现在隔壁车道）
 *   - 车道保持 + 观测到 Undefined -> OK（可能是车道匹配不稳定）
 *   - 左变道中 + 观测为车道保持 -> OK（仍在变道过程中）
 *   - 左变道中 + 观测为左变道 -> 变道完成，切回车道保持，解除 HMI 锁定
 *   - 左变道中 + 观测为 Undefined -> 取消变道，切回车道保持（车道跳变）
 *   - 左变道中 + 观测为右变道 -> Undefined（不应出现反向变道）
 *   - 右变道中同理
 *
 * ----- HMI / 自动驾驶等级交互 -----
 *
 *   L2 级 (autonomous_level_ = 2):
 *     - HMI 直接设置 behavior_.lat_behavior，不运行 MPDM
 *     - 游戏手柄可随时触发变道和调速
 *
 *   L3 级 (autonomous_level_ = 3):
 *     - 运行 MPDM 算法做决策
 *     - HMI 变道命令设置 lock_to_hmi_ = true 和 hmi_behavior_
 *     - MPDM 会优先选择 hmi_behavior_（若在候选集中）
 *     - 变道完成后 lock_to_hmi_ 自动解除
 *
 *   L1 级: 不处理 HMI 输入
 */

#include "behavior_planner/behavior_planner.h"

namespace planning {

// ============================================================================
// 基础接口实现
// ============================================================================

/// 返回规划器名称标识
std::string BehaviorPlanner::Name() {
  return std::string("Generic behavior planner");
}

/**
 * @brief 初始化行为规划器
 *
 * 创建 RoutePlanner 实例用于路径规划，初始化行为状态。
 *
 * @param config 配置标识字符串
 */
ErrorType BehaviorPlanner::Init(const std::string config) {
  p_route_planner_ = new planning::RoutePlanner();
  behavior_.actual_desired_velocity = 0.0;
  return kSuccess;
}

// ============================================================================
// RunMpdm - MPDM 总调度函数
// ============================================================================

/**
 * @brief 运行 MPDM 多策略决策主流程
 *
 * 调用 MultiBehaviorJudge 完成候选枚举、前向仿真、代价评估全流程，
 * 将决策结果写入 behavior_ 结构体（lat_behavior, actual_desired_velocity,
 * forward_trajs, forward_behaviors, surround_trajs）。
 *
 * @return kSuccess 如果 MPDM 成功选出最优行为
 */
ErrorType BehaviorPlanner::RunMpdm() {
  TicToc timer;
  LateralBehavior mpdm_behavior;
  decimal_t mpdm_desired_velocity;

  // 核心：多行为决策（枚举 -> 仿真 -> 评估 -> 选择）
  if (MultiBehaviorJudge(behavior_.actual_desired_velocity, &mpdm_behavior,
                         &mpdm_desired_velocity) == kSuccess) {
    // 将 MPDM 决策结果写入行为状态
    behavior_.lat_behavior = mpdm_behavior;
    behavior_.actual_desired_velocity = mpdm_desired_velocity;
    behavior_.forward_trajs = forward_trajs_;
    behavior_.forward_behaviors = forward_behaviors_;
    behavior_.surround_trajs = surround_trajs_;

    printf("[MPDM]MPDM desired velocity %lf in %lf.\n",
           behavior_.actual_desired_velocity, reference_desired_velocity_);
    printf("[MPDM]Time multi behavior judged in %lf ms.\n", timer.toc());
  } else {
    printf("[MPDM]MPDM failed.\n");
    printf("[MPDM]Time multi behavior judged in %lf ms.\n", timer.toc());
    return kWrongStatus;
  }
  return kSuccess;
}

// ============================================================================
// RunRoutePlanner - 路线规划器
// ============================================================================

/**
 * @brief 运行路线规划器
 *
 * 使用随机扩展模式（kRandomExpansion）生成从当前车道出发的导航路径。
 * 导航路径用于后续参考车道的构建和车道拓扑可达性检查。
 *
 * 数据流：
 *   WholeLaneNet -> RoutePlanner -> navi_path (车道ID序列)
 *
 * @param nearest_lane_id 离自车最近的车道 ID
 */
ErrorType BehaviorPlanner::RunRoutePlanner(const int nearest_lane_id) {
  TicToc timer_rp;

  // 配置路线规划器为随机扩展模式
  p_route_planner_->set_navi_mode(RoutePlanner::NaviMode::kRandomExpansion);

  // 仅首次运行时需要传递车道网络
  if (!p_route_planner_->if_get_lane_net()) {
    common::LaneNet whole_lane_net;
    map_itf_->GetWholeLaneNet(&whole_lane_net);
    p_route_planner_->set_lane_net(whole_lane_net);
  }

  // 设置自车状态和最近车道，用于路线规划起点确定
  State ego_state;
  map_itf_->GetEgoState(&ego_state);
  p_route_planner_->set_ego_state(ego_state);
  p_route_planner_->set_nearest_lane_id(nearest_lane_id);

  if (p_route_planner_->RunOnce() == kSuccess) {
    // 路线规划成功，navi_path 已更新
  }
  return kSuccess;
}

// ============================================================================
// RunOnce - 行为规划器总入口
// ============================================================================

/**
 * @brief 行为规划器单次运行入口
 *
 * 完整行为决策流程（每次规划循环调用一次）：
 *
 *   第1步: 获取自车所在车道 ID（通过位置 + 导航路径匹配）
 *   第2步: 获取自车完整状态信息
 *   第3步: [可选] 运行路线规划器（use_sim_state_为true时）
 *   第4步: 首次运行时初始化 ego_lane_id_
 *   第5步: JudgeBehaviorByLaneId - 通过车道ID推断当前行为
 *   第6步: UpdateEgoLaneId - 更新车道ID，刷新候选车道列表
 *   第7步: UpdateEgoBehavior - 状态机更新（防非法跳变）
 *   第8步: [L3级] RunMpdm - 执行MPDM多策略决策
 *   第9步: ConstructReferenceLane - 构建最终参考车道
 *
 * @return kSuccess 如果整体流程成功完成
 */
ErrorType BehaviorPlanner::RunOnce() {
  // ========================================================================
  // 第1步: 获取自车所在车道 ID
  // 通过自车位置 (x, y, theta) 匹配到语义地图中的车道 ID
  // navi_path 用于约束匹配范围（只匹配导航路径上的车道）
  // ========================================================================
  int ego_lane_id_by_pos = kInvalidLaneId;
  if (map_itf_->GetEgoLaneIdByPosition(p_route_planner_->navi_path(),
                                       &ego_lane_id_by_pos) != kSuccess) {
    printf("[BP RunOnce]Err - Ego not on lane.\n");
    return kWrongStatus;
  }

  // ========================================================================
  // 第2步: 获取自车完整状态信息
  // ========================================================================
  common::Vehicle ego_vehicle;
  if (map_itf_->GetEgoVehicle(&ego_vehicle) != kSuccess) {
    printf("[MPDM]fail to get ego vehicle.\n");
    return kWrongStatus;
  }
  ego_id_ = ego_vehicle.id();

  // ========================================================================
  // 第3步: [可选] 运行路线规划器
  // 仅在启用仿真状态时运行（use_sim_state_ 从 ROS 参数读取）
  // ========================================================================
  if (use_sim_state_) {
    RunRoutePlanner(ego_lane_id_by_pos);
  }

  // ========================================================================
  // 第4步: 首次运行时初始化 ego_lane_id_
  // ========================================================================
  if (ego_lane_id_ == kInvalidAgentId) {
    UpdateEgoLaneId(ego_lane_id_by_pos);
  }

  // ========================================================================
  // 第5步: 通过车道ID推断当前横向行为
  // 原理：自车在某条车道上 -> 查找该车道属于哪个候选集 -> 确定行为
  // ========================================================================
  LateralBehavior behavior_by_lane_id;
  if (JudgeBehaviorByLaneId(ego_lane_id_by_pos, &behavior_by_lane_id) !=
      kSuccess) {
    printf("[RunOnce]fail to judge behavior by lane id!\n");
    return kWrongStatus;
  }

  // ========================================================================
  // 第6步: 更新自车车道ID，刷新三个候选行为对应的车道ID列表
  //   - potential_lk_lane_ids_: 车道保持可达车道
  //   - potential_lcl_lane_ids_: 左变道可达车道
  //   - potential_lcr_lane_ids_: 右变道可达车道
  // ========================================================================
  UpdateEgoLaneId(ego_lane_id_by_pos);
  printf("[MPDM]ego lane id: %d.\n", ego_lane_id_);

  // ========================================================================
  // 第7步: 行为状态机更新
  // 防止不合逻辑的行为跳变（如从车道保持突然变为右变道）
  // ========================================================================
  if (UpdateEgoBehavior(behavior_by_lane_id) != kSuccess) {
    printf("[RunOnce]fail to update ego behavior!\n");
    return kWrongStatus;
  }

  // 兜底：如果行为未定义，默认采用车道保持
  if (behavior_.lat_behavior == common::LateralBehavior::kUndefined) {
    // printf("[RunOnce]Err - Undefined system behavior!.\n");
    // ! temporal solution lane keep at the current lane.
    behavior_.lat_behavior = common::LateralBehavior::kLaneKeeping;
  }

  // ========================================================================
  // 第8步: [L3级] 运行 MPDM 多策略决策
  //
  // L2 级: autonomous_level_ = 2，不运行 MPDM，行为由 HMI 直接控制
  // L3 级: autonomous_level_ = 3，运行 MPDM 算法进行自主决策
  //
  // 执行步骤：
  //   a. 根据 aggressive_level_ 查表获取 IDM 仿真参数
  //   b. 调用 RunMpdm() 执行完整 MPDM 流程
  // ========================================================================
  behavior_.actual_desired_velocity = user_desired_velocity_;
  if (autonomous_level_ >= 3) {
    TicToc timer;
    // 根据激进等级查表获取对应的 IDM 参数集
    planning::MultiModalForward::ParamLookUp(aggressive_level_, &sim_param_);

    if (RunMpdm() != kSuccess) {
      printf("[Summary]Mpdm failed: %lf ms.\n", timer.toc());
      return kWrongStatus;
    }
    printf("[Summary]Mpdm time cost: %lf ms.\n", timer.toc());
  }

  // ========================================================================
  // 第9步: 构建最终参考车道
  // 根据决策出的横向行为，确定目标车道ID，从地图获取采样点，
  // 拟合为平滑的参考车道，同时计算曲率限制下的安全期望速度。
  // ========================================================================
  if (ConstructReferenceLane(behavior_.lat_behavior, &behavior_.ref_lane) !=
      kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

// ============================================================================
// MultiBehaviorJudge - MPDM 多行为决策核心
// ============================================================================

/**
 * @brief MPDM 多行为决策核心函数
 *
 * 完整执行 MPDM 算法的全部步骤：
 *
 *   Step A. 数据采集
 *     - 获取关键语义车辆集合（周围车辆的预测行为和参考车道）
 *     - 获取自车状态
 *
 *   Step B. 候选行为枚举
 *     - 始终包含车道保持（kLaneKeeping）
 *     - 若存在左侧车道 -> 添加左变道（kLaneChangeLeft）
 *     - 若存在右侧车道 -> 添加右变道（kLaneChangeRight）
 *     - 总计 1~3 个候选行为
 *
 *   Step C. 为每辆车构建参考车道
 *     - 获取每辆车的预测横向行为（GetPredictedBehavior）
 *     - 根据预测行为获取对应车道（GetRefLaneForStateByBehavior）
 *     - 构建 <车辆, 参考车道> 配对，用于前向仿真
 *
 *   Step D. 前向仿真
 *     - 对每个候选行为调用 SimulateEgoBehavior
 *     - 内部优先使用 MultiAgentSimForward，失败回退到 OpenloopSimForward
 *     - 缓存结果到 forward_trajs_, forward_behaviors_, surround_trajs_
 *
 *   Step E. 代价评估
 *     - 调用 EvaluateMultiPolicyTrajs 评估所有候选轨迹
 *     - 每个候选轨迹的代价 = 效率 + 安全 + 动作
 *     - 选择最小代价行为
 *
 *   Step F. 速度限幅
 *     - 期望速度变化不超过 max_vel_cmd_gap = 5.0 m/s
 *     - 防止速度指令突变导致的不舒适
 *
 *   Step G. HMI 锁定逻辑
 *     - 若 lock_to_hmi_ 为 true 且 hmi_behavior_ 在候选集中
 *     - 则优先选用 HMI 指定的行为（L3 级下 HMI 作为强建议）
 *
 * @param previous_desired_vel 上一帧的期望速度（用于速度平滑）
 * @param[out] mpdm_behavior MPDM 选出的最优横向行为
 * @param[out] mpdm_desired_velocity MPDM 计算出的最优期望速度
 */
ErrorType BehaviorPlanner::MultiBehaviorJudge(
    const decimal_t previous_desired_vel, LateralBehavior* mpdm_behavior,
    decimal_t* mpdm_desired_velocity) {

  // ========================================================================
  // Step A: 数据采集
  // ========================================================================

  // 获取关键语义车辆集合（含预测行为、参考车道等语义信息）
  common::SemanticVehicleSet semantic_vehicle_set;
  if (map_itf_->GetKeySemanticVehicles(&semantic_vehicle_set) != kSuccess) {
    printf("[MPDM]fail to get key vehicles.\n");
    return kWrongStatus;
  }

  // 获取自车完整状态
  common::Vehicle ego_vehicle;
  if (map_itf_->GetEgoVehicle(&ego_vehicle) != kSuccess) {
    printf("[MPDM]fail to get ego vehicle.\n");
    return kWrongStatus;
  }

  std::cout << "[BehaviorPlanner]" << ego_vehicle.id()
            << "\tsemantic_vehicle_set num:"
            << semantic_vehicle_set.semantic_vehicles.size() << std::endl;

  // ========================================================================
  // 清理上一帧的仿真结果缓存
  // ========================================================================
  forward_trajs_.clear();
  forward_behaviors_.clear();
  surround_trajs_.clear();

  // ========================================================================
  // Step B: 候选行为枚举
  //
  // 基础行为: 车道保持（始终可用）
  // 扩展行为: 左变道（若有左侧车道）/ 右变道（若有右侧车道）
  // ========================================================================
  std::vector<LateralBehavior> potential_behaviors{
      common::LateralBehavior::kLaneKeeping};
  if (!potential_lcl_lane_ids_.empty())
    potential_behaviors.push_back(common::LateralBehavior::kLaneChangeLeft);
  if (!potential_lcr_lane_ids_.empty())
    potential_behaviors.push_back(common::LateralBehavior::kLaneChangeRight);

  // ========================================================================
  // Step C: 为每辆车构建 <车辆, 参考车道> 配对
  //
  // 对每辆周围车辆:
  //   1. 获取其预测的横向行为（意图预测）
  //   2. 根据预测行为构建对应的参考车道
  //   3. 车道长度 = max(vel * 10, 50)m，保证足够长的仿真视野
  //
  // 这样做的目的是让前向仿真中每辆车都有明确的行驶路径。
  // ========================================================================
  const decimal_t max_backward_len = 10.0;  // 向后参考长度（用于车道拟合）
  for (auto it = semantic_vehicle_set.semantic_vehicles.begin();
       it != semantic_vehicle_set.semantic_vehicles.end(); ++it) {
    // 获取该车辆的预测横向行为；若无预测则默认车道保持
    common::LateralBehavior lat_behavior;
    if (map_itf_->GetPredictedBehavior(it->second.vehicle.id(),
                                       &lat_behavior) != kSuccess) {
      lat_behavior = common::LateralBehavior::kLaneKeeping;
    }

    // 前向车道长度：至少50m，速度越快越长
    decimal_t forward_lane_len =
        std::max(it->second.vehicle.state().velocity * 10.0, 50.0);

    // 为车辆构建参考车道
    common::Lane ref_lane;
    if (map_itf_->GetRefLaneForStateByBehavior(
            it->second.vehicle.state(), std::vector<int>(), lat_behavior,
            forward_lane_len, max_backward_len, false, &ref_lane) == kSuccess) {
      it->second.lane = ref_lane;  // 绑定参考车道到车辆
    }
  }

  // ========================================================================
  // Step D: 前向仿真
  //
  // 对每个候选行为执行完整的前向仿真流程:
  //   SimulateEgoBehavior -> MultiAgentSimForward (首选)
  //                       -> OpenloopSimForward (回退)
  // ========================================================================
  std::vector<LateralBehavior> valid_behaviors;
  vec_E<vec_E<common::Vehicle>> valid_forward_trajs;
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> valid_surround_trajs;
  int num_available_behaviors = static_cast<int>(potential_behaviors.size());

  for (int i = 0; i < num_available_behaviors; i++) {
    vec_E<common::Vehicle> traj;
    std::unordered_map<int, vec_E<common::Vehicle>> sur_trajs;
    // 仿真单个候选行为：构建自车参考车道 -> 多智能体仿真/开环仿真
    if (SimulateEgoBehavior(ego_vehicle, potential_behaviors[i],
                            semantic_vehicle_set, &traj,
                            &sur_trajs) != kSuccess) {
      printf("[MPDM]fail to sim %d forward.\n",
             static_cast<int>(potential_behaviors[i]));
      continue;  // 该行为仿真失败，跳过
    }
    // 仅保留仿真成功的行为
    valid_behaviors.push_back(potential_behaviors[i]);
    valid_forward_trajs.push_back(traj);
    valid_surround_trajs.push_back(sur_trajs);
  }

  // 缓存仿真结果到成员变量（供可视化和下游模块使用）
  forward_behaviors_ = valid_behaviors;
  forward_trajs_ = valid_forward_trajs;
  surround_trajs_ = valid_surround_trajs;

  // ========================================================================
  // Step E: 代价评估与最优选择
  // ========================================================================
  int num_valid_behaviors = static_cast<int>(valid_behaviors.size());
  if (num_valid_behaviors < 1) {
    printf("[MPDM]No valid behaviors.\n");
    return kWrongStatus;
  }

  decimal_t winner_score, winner_desired_vel;
  LateralBehavior winner_behavior;
  vec_E<common::Vehicle> winner_forward_traj;

  // 遍历所有有效的候选仿真轨迹，计算代价，选择最小代价行为
  if (EvaluateMultiPolicyTrajs(valid_behaviors, valid_forward_trajs,
                               valid_surround_trajs, &winner_behavior,
                               &winner_forward_traj, &winner_score,
                               &winner_desired_vel) != kSuccess) {
    printf("[MPDM]fail to evaluate multiple policy trajs.\n");
    return kWrongStatus;
  }

  // ========================================================================
  // Step F: 期望速度限幅
  //
  // 限制单帧速度变化不超过 max_vel_cmd_gap = 5.0 m/s，
  // 防止期望速度突变导致下游规划器产生不舒适的轨迹。
  // ========================================================================
  const decimal_t max_vel_cmd_gap = 5.0;
  if (fabs(winner_desired_vel - ego_vehicle.state().velocity) >
      max_vel_cmd_gap) {
    if (winner_desired_vel > ego_vehicle.state().velocity) {
      winner_desired_vel = ego_vehicle.state().velocity + max_vel_cmd_gap;
    } else {
      winner_desired_vel = ego_vehicle.state().velocity - max_vel_cmd_gap;
    }
  }

  // ========================================================================
  // Step G: HMI 锁定逻辑
  //
  // L3 级别下，若 HMI 发起了变道命令：
  //   - lock_to_hmi_ = true
  //   - 如果 hmi_behavior_ 在候选行为集中，优先使用 HMI 行为
  //   - 如果 hmi_behavior_ 不在候选集中（如目标车道不存在），使用 MPDM 结果
  // ========================================================================
  if (lock_to_hmi_) {
    // 检查 HMI 指定的行为是否在可行行为集中
    auto it = std::find(forward_behaviors_.begin(), forward_behaviors_.end(),
                        hmi_behavior_);
    if (it != forward_behaviors_.end()) {
      *mpdm_behavior = hmi_behavior_;  // HMI 行为可用，优先使用
    } else {
      *mpdm_behavior = winner_behavior;  // HMI 行为不可用，使用 MPDM 结果
    }
  } else {
    *mpdm_behavior = winner_behavior;  // 无 HMI 请求，直接使用 MPDM 结果
  }

  *mpdm_desired_velocity = winner_desired_vel;
  return kSuccess;
}

// ============================================================================
// OpenloopSimForward - 开环前向仿真（回退方案）
// ============================================================================

/**
 * @brief 开环前向仿真
 *
 * 当 MultiAgentSimForward（多智能体交互仿真）失败时的回退方案。
 *
 * 与 MultiAgentSimForward 的关键区别：
 *   - 周围车辆各自独立沿自己车道行驶，使用 IDM 跟随（但前车始终为空）
 *   - 自车沿自车参考车道用 IDM 行驶（前车始终为空）
 *   - 车辆之间不进行交互：一辆车的行为不影响其他车
 *
 * 这意味着周围车辆不会对自车的变道行为做出反应，
 * 因此仿真结果比多智能体仿真更保守（碰撞风险可能被低估或高估）。
 *
 * @param ego_semantic_vehicle 自车语义车辆（含参考车道）
 * @param agent_vehicles 周围车辆集合（不含自车）
 * @param[out] traj 自车的仿真轨迹
 * @param[out] surround_trajs 周围车辆的仿真轨迹（按车辆ID索引）
 */
ErrorType BehaviorPlanner::OpenloopSimForward(
    const common::SemanticVehicle& ego_semantic_vehicle,
    const common::SemanticVehicleSet& agent_vehicles,
    vec_E<common::Vehicle>* traj,
    std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs) {
  // 初始化：清空轨迹，压入初始状态
  traj->clear();
  traj->push_back(ego_semantic_vehicle.vehicle);

  // 初始化周围车辆轨迹容器
  surround_trajs->clear();
  for (const auto &v : agent_vehicles.semantic_vehicles) {
    surround_trajs->insert(std::pair<int, vec_E<common::Vehicle>>(
        v.first, vec_E<common::Vehicle>()));
    surround_trajs->at(v.first).push_back(v.second.vehicle);
  }

  // 仿真总步数
  int num_steps_forward = static_cast<int>(sim_horizon_ / sim_resolution_);
  common::Vehicle cur_ego_vehicle = ego_semantic_vehicle.vehicle;
  common::SemanticVehicleSet semantic_vehicle_set_tmp = agent_vehicles;
  common::State ego_state;

  // ---- 主仿真循环 ----
  for (int i = 0; i < num_steps_forward; i++) {
    // ----- 自车前进 -----
    // 期望速度 = 参考期望速度，前车为空（开环假设）
    sim_param_.idm_param.kDesiredVelocity = reference_desired_velocity_;
    if (planning::OnLaneForwardSimulation::PropagateOnce(
            common::StateTransformer(ego_semantic_vehicle.lane),
            cur_ego_vehicle, common::Vehicle(),  // 前车为空
            sim_resolution_, sim_param_,
            &ego_state) != kSuccess) {
      return kWrongStatus;
    }

    // ----- 周围车辆各自独立前进 -----
    // 每辆车使用各自的期望速度（初始速度），前车为空
    std::unordered_map<int, State> state_cache;
    for (auto& v : semantic_vehicle_set_tmp.semantic_vehicles) {
      decimal_t desired_vel =
          agent_vehicles.semantic_vehicles.at(v.first).vehicle.state().velocity;
      sim_param_.idm_param.kDesiredVelocity = desired_vel;
      common::State agent_state;
      if (planning::OnLaneForwardSimulation::PropagateOnce(
              common::StateTransformer(v.second.lane), v.second.vehicle,
              common::Vehicle(),  // 前车为空（开环模式）
              sim_resolution_, sim_param_,
              &agent_state) != kSuccess) {
        return kWrongStatus;
      }
      state_cache.insert(std::make_pair(v.first, agent_state));
    }

    // ----- 碰撞检查 -----
    // 检查自车是否与任何障碍物碰撞
    bool is_collision = false;
    map_itf_->CheckIfCollision(ego_semantic_vehicle.vehicle.param(), ego_state,
                               &is_collision);
    if (is_collision) return kWrongStatus;  // 碰撞 -> 仿真失败

    // ----- 更新状态并记录轨迹 -----
    cur_ego_vehicle.set_state(ego_state);
    for (auto& s : state_cache) {
      semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle.set_state(
          s.second);
      surround_trajs->at(s.first).push_back(
          semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle);
    }
    traj->push_back(cur_ego_vehicle);
  }
  return kSuccess;
}

// ============================================================================
// SimulateEgoBehavior - 单行为仿真调度
// ============================================================================

/**
 * @brief 对单个候选行为执行完整的仿真流程
 *
 * 执行步骤：
 *   1. 根据候选行为构建自车的参考车道
 *   2. 构造自车的语义车辆对象（含参考车道绑定）
 *   3. 将自车加入仿真车辆全集
 *   4. 尝试 MultiAgentSimForward（多智能体交互仿真）
 *   5. 若多智能体仿真失败（如碰撞、数值发散），回退到 OpenloopSimForward
 *
 * @param ego_vehicle 自车当前状态
 * @param ego_behavior 候选的横向行为（车道保持/左变道/右变道）
 * @param semantic_vehicle_set 周围车辆集合
 * @param[out] traj 仿真得到的前向轨迹
 * @param[out] surround_trajs 周围车辆的仿真轨迹
 */
ErrorType BehaviorPlanner::SimulateEgoBehavior(
    const common::Vehicle& ego_vehicle, const LateralBehavior& ego_behavior,
    const common::SemanticVehicleSet& semantic_vehicle_set,
    vec_E<common::Vehicle>* traj,
    std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs) {

  // ----- 步骤1: 构建自车在该行为下的参考车道 -----
  const decimal_t max_backward_len = 10.0;
  decimal_t forward_lane_len =
      std::max(ego_vehicle.state().velocity * 10.0, 50.0);
  common::Lane ego_reflane;
  if (map_itf_->GetRefLaneForStateByBehavior(
          ego_vehicle.state(), p_route_planner_->navi_path(), ego_behavior,
          forward_lane_len, max_backward_len, false,
          &ego_reflane) != kSuccess) {
    printf("[MPDM]fail to get ego reference lane.\n");
    return kWrongStatus;
  }

  // ----- 步骤2: 构造自车的语义车辆对象 -----
  common::SemanticVehicle ego_semantic_vehicle;
  {
    ego_semantic_vehicle.vehicle = ego_vehicle;
    ego_semantic_vehicle.lane = ego_reflane;  // 绑定参考车道
  }

  // ----- 步骤3: 将自车加入仿真车辆全集 -----
  // 这样在 MultiAgentSimForward 中自车也会参与交互
  common::SemanticVehicleSet semantic_vehicle_set_tmp = semantic_vehicle_set;
  semantic_vehicle_set_tmp.semantic_vehicles.insert(
      std::make_pair(ego_vehicle.id(), ego_semantic_vehicle));

  // ----- 步骤4-5: 仿真执行（多智能体 -> 回退开环）-----
  printf("[MPDM]simulating behavior %d.\n", static_cast<int>(ego_behavior));

  if (MultiAgentSimForward(ego_vehicle.id(), semantic_vehicle_set_tmp, traj,
                           surround_trajs) != kSuccess) {
    printf("[MPDM]multi agent forward under %d failed.\n",
           static_cast<int>(ego_behavior));
    // 多智能体仿真失败，回退到开环仿真
    if (OpenloopSimForward(ego_semantic_vehicle, semantic_vehicle_set, traj,
                           surround_trajs) != kSuccess) {
      printf("[MPDM]open loop forward under %d failed.\n",
             static_cast<int>(ego_behavior));
      return kWrongStatus;
    }
  }
  printf("[MPDM]behavior %d traj num of states: %d.\n",
         static_cast<int>(ego_behavior), static_cast<int>(traj->size()));
  return kSuccess;
}

// ============================================================================
// EvaluateMultiPolicyTrajs - 多策略轨迹评估
// ============================================================================

/**
 * @brief 对多个候选策略的仿真轨迹进行代价评估与最优选择
 *
 * 遍历每个候选行为的仿真结果，调用 EvaluateSinglePolicyTraj 计算综合代价，
 * 选取代价最小的行为作为胜出行为。
 *
 * 这是一种典型的"枚举-评估-选优"模式。
 *
 * @param valid_behaviors 有效候选行为列表
 * @param valid_forward_trajs 对应的自车前向轨迹列表
 * @param valid_surround_trajs 对应的周围车辆轨迹列表
 * @param[out] winner_behavior 最小代价的胜出行为
 * @param[out] winner_forward_traj 胜出行为对应的前向轨迹
 * @param[out] winner_score 胜出行为的代价分数
 * @param[out] desired_vel 胜出行为对应的期望速度
 */
ErrorType BehaviorPlanner::EvaluateMultiPolicyTrajs(
    const std::vector<LateralBehavior>& valid_behaviors,
    const vec_E<vec_E<common::Vehicle>>& valid_forward_trajs,
    const vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>&
        valid_surround_trajs,
    LateralBehavior* winner_behavior,
    vec_E<common::Vehicle>* winner_forward_traj, decimal_t* winner_score,
    decimal_t* desired_vel) {

  int num_valid_behaviors = static_cast<int>(valid_behaviors.size());
  if (num_valid_behaviors < 1) return kWrongStatus;

  vec_E<common::Vehicle> traj;
  LateralBehavior behavior = common::LateralBehavior::kUndefined;
  decimal_t min_score = kInf;  // 初始化为无穷大
  decimal_t des_vel = 0.0;

  // 遍历所有候选行为，计算代价，保留最小代价的行为
  for (int i = 0; i < num_valid_behaviors; i++) {
    decimal_t score, vel;
    // 对单条轨迹评估综合代价
    EvaluateSinglePolicyTraj(valid_behaviors[i], valid_forward_trajs[i],
                             valid_surround_trajs[i], &score, &vel);

    // 更新最小代价（越小越好）
    if (score < min_score) {
      min_score = score;
      des_vel = vel;
      behavior = valid_behaviors[i];
      traj = valid_forward_trajs[i];
    }
  }
  printf("[MPDM]choose behavior %d with cost %lf.\n",
         static_cast<int>(behavior), min_score);

  // 输出胜出结果
  *winner_forward_traj = traj;
  *winner_behavior = behavior;
  *winner_score = min_score;
  *desired_vel = des_vel;
  return kSuccess;
}

// ============================================================================
// EvaluateSafetyCost - 轨迹间安全代价评估
// ============================================================================

/**
 * @brief 评估两条轨迹之间的安全代价
 *
 * 这是碰撞检测驱动的安全代价函数。对轨迹的每个时间步进行碰撞检查。
 *
 * 算法：
 *   for 每个时间步 i:
 *     1. 将两辆车各方向膨化 1m（安全距离裕度）
 *     2. 检查膨化矩形是否碰撞
 *     3. 若碰撞，代价 += 0.01 * |速度差| * 0.5
 *        - 速度差越大，潜在碰撞越危险，代价越高
 *
 * 这是 MPDM 中唯一的硬安全约束的"软化"表达：
 *   不直接拒绝会碰撞的轨迹，而是通过代价惩罚使得碰撞轨迹
 *   的综合代价大幅增加，从而在代价排序中被淘汰。
 *
 * @param traj_a 轨迹 A（通常为自车轨迹）
 * @param traj_b 轨迹 B（通常为某辆周围车辆的轨迹）
 * @param[out] cost 累积安全代价
 */
ErrorType BehaviorPlanner::EvaluateSafetyCost(
    const vec_E<common::Vehicle>& traj_a, const vec_E<common::Vehicle>& traj_b,
    decimal_t* cost) {
  // 两条轨迹的长度必须一致（时间步数相同）
  if (traj_a.size() != traj_b.size()) {
    return kWrongStatus;
  }

  int num_states = static_cast<int>(traj_a.size());
  decimal_t cost_tmp = 0.0;

  for (int i = 0; i < num_states; i++) {
    // 将车辆几何膨化（各方向 +1m），增加碰撞检测的安全裕度
    common::Vehicle inflated_a, inflated_b;
    common::SemanticsUtils::InflateVehicleBySize(traj_a[i], 1.0, 1.0,
                                                 &inflated_a);
    common::SemanticsUtils::InflateVehicleBySize(traj_b[i], 1.0, 1.0,
                                                 &inflated_b);

    // 碰撞检测
    bool is_collision = false;
    map_itf_->CheckCollisionUsingState(inflated_a.param(), inflated_a.state(),
                                       inflated_b.param(), inflated_b.state(),
                                       &is_collision);

    // 碰撞惩罚：速度差越大，代价越高
    if (is_collision) {
      cost_tmp +=
          0.01 * fabs(traj_a[i].state().velocity - traj_b[i].state().velocity) *
          0.5;
    }
  }
  *cost = cost_tmp;
  return kSuccess;
}

// ============================================================================
// EvaluateSinglePolicyTraj - 单策略轨迹评估
// ============================================================================

/**
 * @brief 评估单条候选策略轨迹的综合代价
 *
 * 这是 MPDM 代价函数的核心实现。综合代价由三部分组成：
 *
 * ===== 1. cost_efficiency（效率代价）=====
 *
 * 衡量轨迹末端速度与期望速度的偏差，分为两部分：
 *
 *   a) cost_efficiency_ego_to_desired_vel:
 *      自车末端速度与参考期望速度的偏差
 *      = |v_ego_terminal - v_desired| / 10.0
 *      归一化因素：10.0 m/s 为典型城市速度范围
 *
 *   b) cost_efficiency_leading_to_desired_vel:
 *      前车限速导致的效率损失
 *      仅在以下条件同时满足时计算：
 *        - 自车速度 < 期望速度（自车想要加速）
 *        - 前车速度 < 期望速度（前车也在减速）
 *        - 距离 < 100m（前车在可影响范围内）
 *      代价 = 1.5 * distance_residual_ratio * |Δv| / max(2.0, d_lead)
 *      distance_residual_ratio 表示当前距离与 IDM 期望距离的比值，
 *      比值 > 1 说明前车比理想距离远，应降低效率损失权重。
 *
 *   cost_efficiency = 0.5 * (ego_to_desired + leading_to_desired)
 *
 * ===== 2. cost_safety（安全代价）=====
 *
 *   对每辆周围车辆的轨迹与自车轨迹进行碰撞检查，
 *   累积所有碰撞点的惩罚值。
 *
 * ===== 3. cost_action（动作代价）=====
 *
 *   车道保持: 0.0
 *   变道:     0.5
 *   目的：抑制不必要的频繁变道，因为变道有固有的风险和不舒适性
 *
 * total_cost = cost_action + cost_safety + cost_efficiency
 *
 * @param behavior 候选横向行为
 * @param forward_traj 自车的前向仿真轨迹（sim_horizon_/sim_resolution_ + 1 个状态点）
 * @param surround_traj 周围车辆的仿真轨迹（按车辆 ID 索引）
 * @param[out] score 综合代价（越低越好）
 * @param[out] desired_vel 该轨迹对应的期望速度
 */
ErrorType BehaviorPlanner::EvaluateSinglePolicyTraj(
    const LateralBehavior& behavior, const vec_E<common::Vehicle>& forward_traj,
    const std::unordered_map<int, vec_E<common::Vehicle>>& surround_traj,
    decimal_t* score, decimal_t* desired_vel) {

  // ---- 准备：构建末端状态下的车辆集合 ----
  // 用于评估末端时刻的效率（前车影响）
  common::VehicleSet vehicle_set;
  for (auto it = surround_traj.begin(); it != surround_traj.end(); ++it) {
    if (!it->second.empty()) {
      // 取每辆车的末端状态
      vehicle_set.vehicles.insert(std::make_pair(it->first, it->second.back()));
    }
  }

  // ========================================================================
  // 1. 效率代价计算
  // ========================================================================

  // ---- 1a. 自车末端速度与期望速度的偏差 ----
  common::Vehicle ego_vehicle_terminal = forward_traj.back();
  decimal_t cost_efficiency_ego_to_desired_vel =
      fabs(ego_vehicle_terminal.state().velocity -
           reference_desired_velocity_) /
      10.0;

  // ---- 1b. 前车限速导致的效率损失 ----
  // 构建末端时刻的参考车道（车道保持假设）
  common::Vehicle leading_vehicle;
  common::Lane ego_ref_lane;
  const decimal_t max_backward_len = 10.0;
  decimal_t forward_lane_len =
      std::max(ego_vehicle_terminal.state().velocity * 10.0, 50.0);
  if (map_itf_->GetRefLaneForStateByBehavior(
          ego_vehicle_terminal.state(), p_route_planner_->navi_path(),
          common::LateralBehavior::kLaneKeeping, forward_lane_len,
          max_backward_len, false, &ego_ref_lane) != kSuccess) {
    printf("fail to get ego ref lane duration evaluation for behavior %d.\n",
           static_cast<int>(behavior));
  }

  decimal_t cost_efficiency_leading_to_desired_vel = 0.0;
  decimal_t distance_residual_ratio = 0.0;
  const decimal_t lat_range = 2.2;  // 横向搜索范围（约半车道宽）

  // 查找末端时刻的前车
  if (map_itf_->GetLeadingVehicleOnLane(
          ego_ref_lane, ego_vehicle_terminal.state(), vehicle_set, lat_range,
          &leading_vehicle, &distance_residual_ratio) == kSuccess) {
    // 计算到前车的欧式距离
    decimal_t distance_to_leading_vehicle =
        (leading_vehicle.state().vec_position -
         ego_vehicle_terminal.state().vec_position)
            .norm();

    // 仅在自车和前车都低于期望速度且距离较近时，才计算前车导致的效率损失
    if (ego_vehicle_terminal.state().velocity < reference_desired_velocity_ &&
        leading_vehicle.state().velocity < reference_desired_velocity_ &&
        distance_to_leading_vehicle < 100.0) {
      // distance_residual_ratio: 实际距离/IDM期望距离，>1说明前车比理想距离更远
      // 此时效率损失较小（因为自车有足够的空间加速）
      cost_efficiency_leading_to_desired_vel =
          1.5 * distance_residual_ratio *
          fabs(ego_vehicle_terminal.state().velocity -
               reference_desired_velocity_) /
          std::max(2.0, distance_to_leading_vehicle);
    }
  }

  // 综合效率代价（等权平均）
  decimal_t cost_efficiency = 0.5 * (cost_efficiency_ego_to_desired_vel +
                                     cost_efficiency_leading_to_desired_vel);

  // ========================================================================
  // 2. 安全代价计算
  // ========================================================================
  decimal_t cost_safety = 0.0;
  for (auto& traj : surround_traj) {
    decimal_t safety_tmp = 0.0;
    // 对每辆周围车辆，评估与自车轨迹的碰撞安全代价
    EvaluateSafetyCost(forward_traj, traj.second, &safety_tmp);
    cost_safety += safety_tmp;
  }

  // ========================================================================
  // 3. 动作代价计算
  // ========================================================================
  decimal_t cost_action = 0.0;
  if (behavior != common::LateralBehavior::kLaneKeeping) {
    cost_action += 0.5;  // 变道惩罚：每帧 0.5
  }

  // ========================================================================
  // 总代价 = 动作代价 + 安全代价 + 效率代价
  // ========================================================================
  *score = cost_action + cost_safety + cost_efficiency;

  // 调试输出：打印各分项代价
  printf(
      "[CostDebug]behaivor %d: (action %lf, safety %lf, efficiency ego %lf, "
      "leading %lf).\n",
      static_cast<int>(behavior), cost_action, cost_safety,
      cost_efficiency_ego_to_desired_vel,
      cost_efficiency_leading_to_desired_vel);

  // 获取该轨迹对应的期望速度（轨迹中最保守的速度值）
  GetDesiredVelocityOfTrajectory(forward_traj, desired_vel);
  return kSuccess;
}

// ============================================================================
// GetDesiredVelocityOfTrajectory - 轨迹期望速度提取
// ============================================================================

/**
 * @brief 从轨迹中提取期望速度
 *
 * 算法原理：
 *   遍历轨迹上所有状态点，计算每点的法向加速度 a_normal = |curvature| * v^2
 *   （即向心加速度），选取法向加速度最大点对应的速度。
 *
 * 逻辑含义：
 *   法向加速度最大的点是轨迹中最"弯"或最"快"的地方，
 *   使用该点的速度作为期望速度意味着选择轨迹中最受曲率约束的保守速度值，
 *   以此确保车辆能够安全通过轨迹上的所有弯道。
 *
 * @param vehicle_vec 车辆轨迹序列
 * @param[out] vel 期望速度（轨迹中最保守的速度值）
 */
ErrorType BehaviorPlanner::GetDesiredVelocityOfTrajectory(
    const vec_E<common::Vehicle> vehicle_vec, decimal_t* vel) {
  decimal_t min_vel = kInf;
  decimal_t max_acc_normal = 0.0;

  for (auto& v : vehicle_vec) {
    auto state = v.state();
    // 法向加速度 = 曲率绝对值 * 速度平方（向心加速度公式）
    auto acc_normal = fabs(state.curvature) * pow(state.velocity, 2);

    // 记录法向加速度最大点的速度（最保守速度）
    min_vel = acc_normal > max_acc_normal ? state.velocity : min_vel;
  }
  *vel = min_vel;
  return kSuccess;
}

// ============================================================================
// MultiAgentSimForward - 多智能体交互前向仿真
// ============================================================================

/**
 * @brief 多智能体交互前向仿真
 *
 * 这是 MPDM 算法的核心仿真函数。实现了所有交通参与者之间的交互式前向推演。
 *
 * ----- 仿真原理 -----
 *
 * 在每个仿真步 k（0 .. num_steps_forward-1）：
 *
 *   对每辆车 v（包括自车和所有周围车辆）：
 *     a. 设置期望速度：
 *        自车 = reference_desired_velocity_（系统参考速度）
 *        其他车 = 其初始速度（保持当前速度作为期望）
 *     b. 获取限速（speed_limit），修正期望速度为 min(0.9 * speed_limit, desired_vel)
 *     c. 构建"其他所有车辆"的集合（用于查找前车）
 *     d. 查找 v 在其参考车道上的前车（GetLeadingVehicleOnLane）
 *     e. 碰撞预检查：若与前车已碰撞，仿真终止
 *     f. 使用 IDM 跟驰模型计算新状态（PropagateOnce）：
 *        - 若有前车：IDM 跟驰（安全跟车距离 + 速度匹配）
 *        - 若无前车：IDM 自由加速（向期望速度收敛）
 *     g. 新状态存入 state_cache（不立即更新，保证交互的同步性）
 *
 *   同步更新：使用 state_cache 批量更新所有车辆状态
 *   轨迹记录：自车状态追加到 traj，周围车追加到 surround_trajs
 *
 * ----- 与 OpenloopSimForward 的关键区别 -----
 *
 *   1. 交互性：一辆车的动作影响其他车（通过 IDM 前车机制）
 *   2. 前车查找：每步都动态查找前车（开环仿真中前车始终为空）
 *   3. 碰撞检测：碰撞意味着仿真失败（开环仿真也做碰撞检查）
 *   4. 同步更新：使用 state_cache 确保所有车在同一步使用上一步的状态
 *
 * ----- IDM 跟驰模型 -----
 *
 *   a = a_max * [1 - (v/v0)^delta - (s_star / s)^2]
 *   其中：
 *     v:     当前速度
 *     v0:    期望速度
 *     s:     实际跟车距离
 *     s_star: 期望跟车距离 = s0 + v*T + v*Δv/(2*sqrt(a_max*b))
 *     a_max: 最大加速度
 *     b:     舒适减速度
 *     delta: 加速指数
 *     s0:    最小停车距离
 *     T:     安全时距
 *
 * @param ego_id 自车 ID
 * @param semantic_vehicle_set 包含自车和所有周围车辆的完整集合
 * @param[out] traj 自车的仿真轨迹（时间步数 + 1 个状态点）
 * @param[out] surround_trajs 周围车辆的仿真轨迹（按车辆 ID 索引）
 */
ErrorType BehaviorPlanner::MultiAgentSimForward(
    const int ego_id, const common::SemanticVehicleSet& semantic_vehicle_set,
    vec_E<common::Vehicle>* traj,
    std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs) {

  // ---- 初始化 ----
  traj->clear();
  traj->push_back(semantic_vehicle_set.semantic_vehicles.at(ego_id).vehicle);

  // 初始化周围车辆轨迹容器（排除自车）
  surround_trajs->clear();
  for (const auto &v : semantic_vehicle_set.semantic_vehicles) {
    if (v.first == ego_id) continue;  // 跳过自车
    surround_trajs->insert(std::pair<int, vec_E<common::Vehicle>>(
        v.first, vec_E<common::Vehicle>()));
    surround_trajs->at(v.first).push_back(v.second.vehicle);
  }

  // 仿真总步数 = sim_horizon_ / sim_resolution_（如 4.0s / 0.4s = 10 步）
  int num_steps_forward = static_cast<int>(sim_horizon_ / sim_resolution_);

  // 可变的车辆集合副本（每步更新状态）
  common::SemanticVehicleSet semantic_vehicle_set_tmp = semantic_vehicle_set;

  TicToc timer;

  // ========================================================================
  // 主仿真循环：对每个仿真步
  // ========================================================================
  for (int i = 0; i < num_steps_forward; i++) {
    timer.tic();

    // 状态缓存：存储本步计算出的所有车辆新状态
    // 使用缓存而非直接更新是为了保证所有车辆使用上一步的同步状态
    std::unordered_map<int, State> state_cache;

    // ====================================================================
    // 对每辆车进行单步前向推演
    // ====================================================================
    for (auto& v : semantic_vehicle_set_tmp.semantic_vehicles) {
      // ---- a. 设置期望速度 ----
      decimal_t desired_vel = semantic_vehicle_set.semantic_vehicles.at(v.first)
                                  .vehicle.state()
                                  .velocity;
      decimal_t init_stamp = semantic_vehicle_set.semantic_vehicles.at(v.first)
                                 .vehicle.state()
                                 .time_stamp;

      // 自车使用系统参考期望速度；其他车保持初始速度
      if (v.first == ego_id) desired_vel = reference_desired_velocity_;

      // ---- b. 获取限速 ----
      decimal_t speed_limit;
      if (map_itf_->GetSpeedLimit(v.second.vehicle.state(), v.second.lane,
                                  &speed_limit) == kSuccess) {
        // 期望速度受限于 90% 限速值（保留 10% 安全裕度）
        desired_vel = std::min(speed_limit * 0.9, desired_vel);
      }

      // 设置 IDM 模型参数
      sim_param_.idm_param.kDesiredVelocity = desired_vel;

      // ---- c. 构建"其他所有车辆"集合 ----
      // 用于查找前车时排除自己
      common::VehicleSet vehicle_set;
      for (auto& v_other : semantic_vehicle_set_tmp.semantic_vehicles) {
        if (v_other.first != v.first)
          vehicle_set.vehicles.insert(
              std::make_pair(v_other.first, v_other.second.vehicle));
      }

      // ---- d. 查找前车 ----
      common::Vehicle leading_vehicle;
      common::State state;
      decimal_t distance_residual_ratio = 0.0;
      const decimal_t lat_range = 2.2;  // 横向搜索范围 [m]

      if (map_itf_->GetLeadingVehicleOnLane(
              v.second.lane, v.second.vehicle.state(), vehicle_set, lat_range,
              &leading_vehicle, &distance_residual_ratio) == kSuccess) {
        // ---- e. 碰撞预检查 ----
        // 若已与前车碰撞，仿真终止（该行为不应被选择）
        bool is_collision = false;
        map_itf_->CheckCollisionUsingState(
            v.second.vehicle.param(), v.second.vehicle.state(),
            leading_vehicle.param(), leading_vehicle.state(), &is_collision);
        if (is_collision) {
          return kWrongStatus;
        }
      }

      // ---- f. IDM 跟驰推演 ----
      // 使用 IDM 模型计算本步结束时的新状态
      // StateTransformer: 将世界坐标映射到 Frenet 坐标系的变换工具
      // PropagateOnce: IDM 单步推进（若有前车 -> 跟驰；若无前车 -> 自由加速）
      if (planning::OnLaneForwardSimulation::PropagateOnce(
              common::StateTransformer(v.second.lane), v.second.vehicle,
              leading_vehicle, sim_resolution_, sim_param_,
              &state) != kSuccess) {
        printf("[MPDM]fail to forward with leading vehicle.\n");
        return kWrongStatus;
      }

      // ---- g. 新状态存入缓存 ----
      // 时间戳更新：初始时间 + (当前步数 + 1) * 步长
      state.time_stamp = init_stamp + (i + 1) * sim_resolution_;
      state_cache.insert(std::make_pair(v.first, state));
    }

    // ====================================================================
    // 同步更新：使用 state_cache 批量更新所有车辆状态
    // 并记录轨迹
    // ====================================================================
    for (auto& s : state_cache) {
      // 更新车辆状态
      semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle.set_state(
          s.second);

      // 根据车辆身份分别记录轨迹
      if (s.first == ego_id) {
        // 自车轨迹：用于后续代价评估和行为选择
        traj->push_back(
            semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle);
      } else {
        // 周围车辆轨迹：用于安全代价计算
        surround_trajs->at(s.first).push_back(
            semantic_vehicle_set_tmp.semantic_vehicles.at(s.first).vehicle);
      }
    }
  }  // end for num_steps_forward

  return kSuccess;
}

// ============================================================================
// ConstructReferenceLane - 参考车道构建
// ============================================================================

/**
 * @brief 根据横向行为构建最终参考车道
 *
 * 这是行为决策的下游输出接口。根据决策出的横向行为，确定目标车道 ID，
 * 从地图获取车道中心线采样点，拟合成平滑车道曲线，同时计算曲率约束下的
 * 安全参考期望速度。
 *
 * 处理流程：
 *   1. 根据行为确定目标车道 ID
 *   2. 获取自车状态
 *   3. 从地图获取目标车道的局部采样点（向前 150m，向后 20m）
 *   4. 将采样点拟合为 Lane 对象（20 段三次样条）
 *   5. 计算 Frenet 坐标系下的自车位置
 *   6. 扫描前方车道曲率，计算曲率限制的安全速度
 *   7. 设定参考期望速度 = min(安全速度 - 2.0, 用户期望速度)
 *
 * @param lat_behavior 目标横向行为
 * @param[out] lane 构建出的参考车道
 */
ErrorType BehaviorPlanner::ConstructReferenceLane(
    const LateralBehavior& lat_behavior, Lane* lane) {
  if (map_itf_ == nullptr) return kWrongStatus;

  // ---- 步骤1: 确定目标车道 ID ----
  int target_lane_id;
  if (lat_behavior == common::LateralBehavior::kLaneKeeping ||
      lat_behavior == common::LateralBehavior::kUndefined) {
    // 车道保持/未定义 -> 保持当前车道
    target_lane_id = ego_lane_id_;
  } else if (lat_behavior == common::LateralBehavior::kLaneChangeLeft) {
    // 左变道 -> 获取左侧车道 ID
    if (map_itf_->GetLeftLaneId(ego_lane_id_, &target_lane_id) != kSuccess) {
      printf("[BP]Commanding a left lane change, but no existing lane.\n");
      target_lane_id = ego_lane_id_;
      behavior_.lat_behavior = common::LateralBehavior::kLaneKeeping;
    }
  } else if (lat_behavior == common::LateralBehavior::kLaneChangeRight) {
    // 右变道 -> 获取右侧车道 ID
    if (map_itf_->GetRightLaneId(ego_lane_id_, &target_lane_id) != kSuccess) {
      printf("[BP]Commanding a right lane change, but no existing lane.\n");
      target_lane_id = ego_lane_id_;
      behavior_.lat_behavior = common::LateralBehavior::kLaneKeeping;
    }
  } else {
    assert(false);
  }

  // ---- 步骤2: 获取自车当前状态 ----
  State ego_state;
  map_itf_->GetEgoState(&ego_state);

  // ---- 步骤3: 获取目标车道的局部采样点 ----
  const decimal_t kMaxReflaneDist = 150.0;  // 向前最大距离
  const decimal_t kBackwardDist = 20.0;     // 向后最大距离
  vec_Vecf<2> samples;
  if (map_itf_->GetLocalLaneSamplesByState(
          ego_state, target_lane_id, p_route_planner_->navi_path(),
          kMaxReflaneDist, kBackwardDist, &samples) != kSuccess) {
    return kWrongStatus;
  }

  // ---- 步骤4: 采样点拟合为 Lane 对象 ----
  if (ConstructLaneFromSamples(samples, lane) != kSuccess) {
    return kWrongStatus;
  }

  // ---- 步骤5: 计算 Frenet 坐标系下的自车位置 ----
  common::StateTransformer stf(*lane);
  common::FrenetState current_fs;
  stf.GetFrenetStateFromState(ego_state, &current_fs);

  // ---- 步骤6: 曲率限制安全速度计算 ----
  // 公式: v_max = sqrt(a_lat_max / |curvature|)
  // 其中 a_lat_max = 1.5 m/s^2（舒适侧向加速度上限）
  decimal_t c, cc;
  decimal_t v_max_by_curvature;
  decimal_t v_ref = kInf;          // 初始化参考速度为无穷大
  const decimal_t sim_lat_max = 1.5;  // 最大侧向加速度 [m/s^2]
  decimal_t a_comfort = 1.67;      // 舒适减速度 [m/s^2]
  decimal_t t_forward = ego_state.velocity / a_comfort;  // 减速时间
  // 前方扫描距离：至少 20m，最多到车道末端
  decimal_t s_forward =
      std::min(std::max(20.0, t_forward * ego_state.velocity), lane->end());
  decimal_t resolution = 0.2;       // 曲率采样分辨率 [m]

  // 沿着车道方向扫描前方曲率，找出最受限制的速度
  for (decimal_t s = current_fs.vec_s[0]; s < current_fs.vec_s[0] + s_forward;
       s += resolution) {
    if (lane->GetCurvatureByArcLength(s, &c, &cc) == kSuccess) {
      // 基于曲率的速度限制：v_max = sqrt(a_lat_max / |c|)
      v_max_by_curvature = sqrt(sim_lat_max / fabs(c));
      // 取前方所有采样点中的最小速度作为参考
      v_ref = v_max_by_curvature < v_ref ? v_max_by_curvature : v_ref;
    }
  }

  // ---- 步骤7: 设定参考期望速度 ----
  // 取（曲率限制速度 - 2.0 m/s 安全余量）与用户期望速度的较小值
  // 确保不低于 0，并向下取整
  reference_desired_velocity_ =
      std::floor(std::min(std::max(v_ref - 2.0, 0.0), user_desired_velocity_));

  return kSuccess;
}

// ============================================================================
// ConstructLaneFromSamples - 离散点拟合车道
// ============================================================================

/**
 * @brief 从离散采样点拟合为 Lane 对象
 *
 * 算法步骤：
 *   1. 计算沿路径的累积弧长参数（cumulative arc length）
 *   2. 将弧长均匀划分为 num_segments=20 段
 *   3. 调用 LaneGenerator 进行三次样条拟合
 *
 * @param samples 车道中心线采样点序列 (x, y)
 * @param[out] lane 拟合后的 Lane 对象（20 段三次多项式）
 */
ErrorType BehaviorPlanner::ConstructLaneFromSamples(
    const vec_E<Vecf<2>>& samples, Lane* lane) {
  // ---- 计算累积弧长参数 ----
  double d = 0.0;
  std::vector<decimal_t> para;
  para.push_back(d);

  int num_samples = static_cast<int>(samples.size());
  for (int i = 1; i < num_samples; i++) {
    double dx = samples[i](0) - samples[i - 1](0);
    double dy = samples[i](1) - samples[i - 1](1);
    d += std::hypot(dx, dy);  // 欧式距离累加 = 弧长
    para.push_back(d);
  }

  // ---- 均匀分段 ----
  const int num_segments = 20;  // 20 段三次多项式
  Eigen::ArrayXf breaks =
      Eigen::ArrayXf::LinSpaced(num_segments, para.front(), para.back());

  // ---- 样条拟合（正则化参数 1e6 抑制过拟合）----
  const decimal_t regulator = (double)1e6;
  if (common::LaneGenerator::GetLaneBySampleFitting(
          samples, para, breaks, regulator, lane) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

// ============================================================================
// JudgeBehaviorByLaneId - 通过车道ID推断行为
// ============================================================================

/**
 * @brief 根据自车位置对应的车道 ID 推断当前横向行为
 *
 * 推断逻辑：
 *   1. 若位置车道 ID == 当前 ego_lane_id_ -> 车道保持
 *   2. 若位置车道 ID 在 potential_lk_lane_ids_ 中 -> 车道保持
 *   3. 若位置车道 ID 在 potential_lcl_lane_ids_ 中 -> 正在左变道
 *   4. 若位置车道 ID 在 potential_lcr_lane_ids_ 中 -> 正在右变道
 *   5. 否则 -> Undefined（无法推断当前行为）
 *
 * 这本质上是一种"位置->行为"的观测函数，用于判断自车实际位于哪条车道上，
 * 从而推断其当前的横向行为状态。
 *
 * @param ego_lane_id_by_pos 根据自车位置查询到的车道 ID
 * @param[out] behavior_by_lane_id 推断出的横向行为
 */
ErrorType BehaviorPlanner::JudgeBehaviorByLaneId(
    const int ego_lane_id_by_pos, LateralBehavior* behavior_by_lane_id) {
  // 直接等于当前车道 ID: 车道保持
  if (ego_lane_id_by_pos == ego_lane_id_) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneKeeping;
    return kSuccess;
  }

  // 在候选车道集合中查找
  auto it = std::find(potential_lk_lane_ids_.begin(),
                      potential_lk_lane_ids_.end(), ego_lane_id_by_pos);
  auto it_lcl = std::find(potential_lcl_lane_ids_.begin(),
                          potential_lcl_lane_ids_.end(), ego_lane_id_by_pos);
  auto it_lcr = std::find(potential_lcr_lane_ids_.begin(),
                          potential_lcr_lane_ids_.end(), ego_lane_id_by_pos);

  if (it != potential_lk_lane_ids_.end()) {
    // 在车道保持候选集中 -> 车道保持
    *behavior_by_lane_id = common::LateralBehavior::kLaneKeeping;
    return kSuccess;
  }

  if (it_lcl != potential_lcl_lane_ids_.end()) {
    // 在左变道候选集中 -> 正在左变道
    *behavior_by_lane_id = common::LateralBehavior::kLaneChangeLeft;
    return kSuccess;
  }

  if (it_lcr != potential_lcr_lane_ids_.end()) {
    // 在右变道候选集中 -> 正在右变道
    *behavior_by_lane_id = common::LateralBehavior::kLaneChangeRight;
    return kSuccess;
  }

  // 不在任何已知候选集中 -> 无法推断
  *behavior_by_lane_id = common::LateralBehavior::kUndefined;
  return kSuccess;
}

// ============================================================================
// UpdateEgoBehavior - 行为状态机
// ============================================================================

/**
 * @brief 行为状态机：根据观测行为更新系统行为状态
 *
 * 这是防止不合逻辑行为跳变的关键状态机。输入是"观测到的行为"（根据当前
 * 车道ID推断），输出是对系统行为状态的修正。
 *
 * 状态转移表：
 *
 *  当前状态          | 观测状态          | 新状态          | 说明
 *  -----------------|-------------------|-----------------|------------------------
 *  kLaneKeeping     | kLaneKeeping     | kLaneKeeping    | 正常，保持
 *  kLaneKeeping     | kLaneChangeLeft  | kUndefined      | 异常：车辆不应在左侧车道
 *  kLaneKeeping     | kLaneChangeRight | kUndefined      | 异常：车辆不应在右侧车道
 *  kLaneKeeping     | kUndefined       | kLaneKeeping    | 车道匹配不稳定，保持
 *  -----------------|-------------------|-----------------|------------------------
 *  kLaneChangeLeft  | kLaneKeeping     | kLaneChangeLeft | 仍在变道过程中
 *  kLaneChangeLeft  | kLaneChangeLeft  | kLaneKeeping    | 变道完成！解除HMI锁
 *  kLaneChangeLeft  | kLaneChangeRight | kUndefined      | 异常：不应出现反向变道
 *  kLaneChangeLeft  | kUndefined       | kLaneKeeping    | 车道跳变，取消变道
 *  -----------------|-------------------|-----------------|------------------------
 *  kLaneChangeRight | kLaneKeeping     | kLaneChangeRight| 仍在变道过程中
 *  kLaneChangeRight | kLaneChangeRight | kLaneKeeping    | 变道完成！解除HMI锁
 *  kLaneChangeRight | kLaneChangeLeft  | kUndefined      | 异常：不应出现反向变道
 *  kLaneChangeRight | kUndefined       | kLaneKeeping    | 车道跳变，取消变道
 *
 * @param behavior_by_lane_id 根据车辆位置观测到的行为
 */
ErrorType BehaviorPlanner::UpdateEgoBehavior(
    const LateralBehavior& behavior_by_lane_id) {

  // ====== 当前状态: 车道保持 ======
  if (behavior_.lat_behavior == common::LateralBehavior::kLaneKeeping) {
    if (behavior_by_lane_id == common::LateralBehavior::kLaneKeeping) {
      // 正常：保持在当前车道，不做任何修改
    } else if (behavior_by_lane_id == common::LateralBehavior::kUndefined) {
      // 车道匹配不稳定（可能处于车道线附近），保持当前行为
    } else {
      // 异常：系统认为在车道保持，但车辆位置在隔壁车道 -> Undefined
      behavior_.lat_behavior = common::LateralBehavior::kUndefined;
    }
  }
  // ====== 当前状态: 左变道中 ======
  else if (behavior_.lat_behavior ==
           common::LateralBehavior::kLaneChangeLeft) {
    if (behavior_by_lane_id == common::LateralBehavior::kLaneKeeping) {
      // 仍在原车道或变道过程中（车道 ID 尚未切换），继续变道
    } else if (behavior_by_lane_id ==
               common::LateralBehavior::kLaneChangeLeft) {
      // 变道完成：车辆位置已出现在左侧车道 -> 切回车道保持并解除 HMI 锁定
      behavior_.lat_behavior = common::LateralBehavior::kLaneKeeping;
      lock_to_hmi_ = false;
    } else if (behavior_by_lane_id == common::LateralBehavior::kUndefined) {
      // 车道跳变（匹配不稳定）-> 取消变道，保持车道
      behavior_.lat_behavior = common::LateralBehavior::kLaneKeeping;
      lock_to_hmi_ = false;
    } else {
      // 异常：左变道中出现右变道观测 -> Undefined
      behavior_.lat_behavior = common::LateralBehavior::kUndefined;
      lock_to_hmi_ = false;
    }
  }
  // ====== 当前状态: 右变道中 ======
  else if (behavior_.lat_behavior ==
           common::LateralBehavior::kLaneChangeRight) {
    if (behavior_by_lane_id == common::LateralBehavior::kLaneKeeping) {
      // 仍在原车道或变道过程中（车道 ID 尚未切换），继续变道
    } else if (behavior_by_lane_id ==
               common::LateralBehavior::kLaneChangeRight) {
      // 变道完成：车辆位置已出现在右侧车道 -> 切回车道保持并解除 HMI 锁定
      behavior_.lat_behavior = common::LateralBehavior::kLaneKeeping;
      lock_to_hmi_ = false;
    } else if (behavior_by_lane_id == common::LateralBehavior::kUndefined) {
      // 车道跳变（匹配不稳定）-> 取消变道，保持车道
      behavior_.lat_behavior = common::LateralBehavior::kLaneKeeping;
      lock_to_hmi_ = false;
    } else {
      // 异常：右变道中出现左变道观测 -> Undefined
      behavior_.lat_behavior = common::LateralBehavior::kUndefined;
      lock_to_hmi_ = false;
    }
  }  // end 枚举当前行为状态

  return kSuccess;
}

// ============================================================================
// UpdateEgoLaneId - 更新自车车道ID
// ============================================================================

/**
 * @brief 更新自车当前所在车道 ID 及其候选车道列表
 *
 * 每次规划循环中调用，刷新三个候选行为对应的车道 ID 列表：
 *   - potential_lk_lane_ids_:  车道保持可达的车道（当前车道的子车道）
 *   - potential_lcl_lane_ids_: 左变道可达的车道（左侧车道及其子车道）
 *   - potential_lcr_lane_ids_: 右变道可达的车道（右侧车道及其子车道）
 *
 * 这三个列表用于：
 *   1. JudgeBehaviorByLaneId: 判断自车当前处于何种行为状态
 *   2. MultiBehaviorJudge: 枚举可行的候选行为
 *
 * @param new_ego_lane_id 新的自车车道 ID
 */
ErrorType BehaviorPlanner::UpdateEgoLaneId(const int new_ego_lane_id) {
  ego_lane_id_ = new_ego_lane_id;

  // 刷新三个候选车道列表
  GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneKeeping,
                      &potential_lk_lane_ids_);
  GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneChangeLeft,
                      &potential_lcl_lane_ids_);
  GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneChangeRight,
                      &potential_lcr_lane_ids_);
  return kSuccess;
}

// ============================================================================
// GetPotentialLaneIds - 获取候选车道ID列表
// ============================================================================

/**
 * @brief 获取指定行为的潜在目标车道 ID 列表
 *
 * 根据行为类型不同，返回不同的车道集合：
 *
 *   - 车道保持 (kLaneKeeping / kUndefined):
 *       返回 source_lane_id 的所有子车道（下游车道）
 *       这些是在保持当前车道行驶时可能到达的车道
 *
 *   - 左变道 (kLaneChangeLeft):
 *       返回左侧相邻车道的子车道 + 左侧车道本身
 *       如果不存在左侧车道（道路边界或禁止变道），返回空列表
 *
 *   - 右变道 (kLaneChangeRight):
 *       返回右侧相邻车道的子车道 + 右侧车道本身
 *       如果不存在右侧车道，返回空列表
 *
 * @param source_lane_id 源车道 ID
 * @param beh 目标横向行为
 * @param[out] candidate_lane_ids 候选车道 ID 列表
 */
ErrorType BehaviorPlanner::GetPotentialLaneIds(
    const int source_lane_id, const LateralBehavior& beh,
    std::vector<int>* candidate_lane_ids) {
  candidate_lane_ids->clear();

  if (beh == common::LateralBehavior::kUndefined ||
      beh == common::LateralBehavior::kLaneKeeping) {
    // 车道保持：获取当前车道的子车道（下游后继车道）
    map_itf_->GetChildLaneIds(source_lane_id, candidate_lane_ids);
  } else if (beh == common::LateralBehavior::kLaneChangeLeft) {
    // 左变道：获取左侧车道的子车道 + 左侧车道本身
    int l_lane_id;
    if (map_itf_->GetLeftLaneId(source_lane_id, &l_lane_id) == kSuccess) {
      map_itf_->GetChildLaneIds(l_lane_id, candidate_lane_ids);
      candidate_lane_ids->push_back(l_lane_id);
    }
  } else if (beh == common::LateralBehavior::kLaneChangeRight) {
    // 右变道：获取右侧车道的子车道 + 右侧车道本身
    int r_lane_id;
    if (map_itf_->GetRightLaneId(source_lane_id, &r_lane_id) == kSuccess) {
      map_itf_->GetChildLaneIds(r_lane_id, candidate_lane_ids);
      candidate_lane_ids->push_back(r_lane_id);
    }
  } else {
    assert(false);
  }
  return kSuccess;
}

// ============================================================================
// 接口 setter / getter
// ============================================================================

/// 设置地图接口（依赖注入）
void BehaviorPlanner::set_map_interface(BehaviorPlannerMapItf* itf) {
  map_itf_ = itf;
}

/**
 * @brief 设置 HMI 行为命令
 *
 * 行为依赖自动驾驶等级：
 *   - L2 级: 直接执行 HMI 命令，立即设置 behavior_.lat_behavior
 *   - L3 级: HMI 作为建议，设置 lock_to_hmi_ = true
 *            MPDM 在决策时会优先考虑 hmi_behavior_
 *   - L1 级: 不处理（此函数不会被调用，因为 JoyCallback 会提前返回）
 *
 * @param hmi_behavior HMI 指定的横向行为
 */
void BehaviorPlanner::set_hmi_behavior(const LateralBehavior& hmi_behavior) {
  if (autonomous_level_ == 2) {
    // L2 级别：直接执行 HMI 命令
    behavior_.lat_behavior = hmi_behavior;
    hmi_behavior_ = hmi_behavior;
    lock_to_hmi_ = true;
  } else if (autonomous_level_ == 3) {
    // L3 级别：HMI 作为建议，MPDM 优先考虑但不强制
    hmi_behavior_ = hmi_behavior;
    lock_to_hmi_ = true;
  }
}

/// 设置自动驾驶等级
void BehaviorPlanner::set_autonomous_level(int level) {
  autonomous_level_ = level;
}

/// 设置激进等级（影响 IDM 参数和代价权重）
void BehaviorPlanner::set_aggressive_level(int level) {
  aggressive_level_ = level;
}

/// 设置用户期望速度（仅 L2+ 级别有效，不低于 0）
void BehaviorPlanner::set_user_desired_velocity(const decimal_t desired_vel) {
  if (autonomous_level_ >= 2) {
    user_desired_velocity_ = std::max(desired_vel, 0.0);
  }
}

/// 设置是否在导航中使用仿真状态
void BehaviorPlanner::set_use_sim_state(bool use_sim_state) {
  use_sim_state_ = use_sim_state;
}

/// 设置前向仿真时间步长 [s]
void BehaviorPlanner::set_sim_resolution(const decimal_t sim_resolution) {
  sim_resolution_ = sim_resolution;
}

/// 设置前向仿真时间窗口 [s]
void BehaviorPlanner::set_sim_horizon(const decimal_t sim_horizon) {
  sim_horizon_ = sim_horizon;
}

/// 获取用户设定期望速度
decimal_t BehaviorPlanner::user_desired_velocity() const {
  return user_desired_velocity_;
}

/// 获取参考期望速度（受曲率等限制后的安全速度）
decimal_t BehaviorPlanner::reference_desired_velocity() const {
  return reference_desired_velocity_;
}

/// 获取当前完整的语义行为
BehaviorPlanner::Behavior BehaviorPlanner::behavior() const {
  return behavior_;
}

/// 获取所有候选行为的前向仿真轨迹（用于可视化）
vec_E<vec_E<common::Vehicle>> BehaviorPlanner::forward_trajs() const {
  return forward_trajs_;
}

/// 获取所有候选行为列表（用于可视化）
std::vector<BehaviorPlanner::LateralBehavior>
BehaviorPlanner::forward_behaviors() const {
  return forward_behaviors_;
}

/// 获取当前自动驾驶等级
int BehaviorPlanner::autonomous_level() const { return autonomous_level_; }

}  // namespace planning
