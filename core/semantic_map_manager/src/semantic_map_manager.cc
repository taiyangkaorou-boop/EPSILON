/**
 * @file semantic_map_manager.cc
 * @author EPSILON Autonomous Driving Group
 * @brief SemanticMapManager 类实现——EPSILON 自动驾驶系统核心世界模型
 *
 * @details
 * 该文件实现了 EPSILON 规划栈的"世界模型中枢"的全部核心功能，按数据处理层级组织：
 *
 * 第1层 - 构造函数 & 配置加载：
 *   - 从JSON配置文件或显式参数初始化Agent配置
 *
 * 第2层 - UpdateSemanticMap（主入口）：
 *   - 聚合所有传感器/仿真器数据并依次调用各处理阶段
 *
 * 第3层 - 语义车道构建：
 *   - UpdateSemanticLaneSet：原始车道 -> 语义车道（含换道可用性和拓扑一致性校验）
 *   - UpdateLocalLanesAndFastLut：构建局部连续车道和快速查找表
 *
 * 第4层 - 语义车辆构建 & 行为预测：
 *   - UpdateSemanticVehicles：为周车注入最近车道、横向行为、参考车道等语义
 *   - NaiveRuleBasedLateralBehaviorPrediction：基于Frenet横向偏移的规则预测
 *   - MobilRuleBasedBehaviorPrediction：基于MOBIL模型的换道决策预测
 *
 * 第5层 - 轨迹预测 & 碰撞检测：
 *   - OpenloopTrajectoryPrediction：基于IDM模型的开环轨迹预测
 *   - CheckCollisionUsingStateAndVehicleParam：静态+动态碰撞检测
 *
 * 第6层 - 关键车辆筛选：
 *   - UpdateKeyVehicles：基于车道拓扑距离筛选对自车规划重要的车辆
 *
 * 第7层 - 查询接口：
 *   - GetNearestLaneIdUsingState：最近车道查找
 *   - IsTopologicallyReachable：拓扑可达性判断（BFS）
 *   - GetLeadingVehicleOnLane / GetFollowingVehicleOnLane：前后车查询
 *   - GetRefLaneForStateByBehavior：根据行为获取参考车道
 */

#include "semantic_map_manager/semantic_map_manager.h"

namespace semantic_map_manager {

// ======================== 第1层：构造与配置 ========================

/// @brief 通过智能体配置文件路径构造
///
/// 初始化流程：
///   1. 存储 ego_id_ 和 agent_config_path_
///   2. 创建 ConfigLoader 并设置自车ID和配置路径
///   3. 调用 ParseAgentConfig 从JSON解析配置
///   4. 启动全局计时器
SemanticMapManager::SemanticMapManager(const int &id,
                                       const std::string &agent_config_path)
    : ego_id_(id), agent_config_path_(agent_config_path) {
  p_config_loader_ = new ConfigLoader();
  p_config_loader_->set_ego_id(ego_id_);
  p_config_loader_->set_agent_config_path(agent_config_path_);
  p_config_loader_->ParseAgentConfig(&agent_config_info_);
  global_timer_.tic();
}

/// @brief 通过显式参数构造（用于非文件配置场景，如测试或简单仿真）
/// @note 默认配置：不启用开环预测、不启用跟踪噪声、不启用日志、
///        启用快速LUT、使用右手坐标系、标记为简单车道结构
SemanticMapManager::SemanticMapManager(
    const int &id, const decimal_t surrounding_search_radius,
    bool enable_openloop_prediction, bool use_right_hand_axis) {
  ego_id_ = id;
  agent_config_info_.surrounding_search_radius = surrounding_search_radius;
  agent_config_info_.enable_openloop_prediction = enable_openloop_prediction;
  agent_config_info_.enable_tracking_noise = false;
  agent_config_info_.enable_log = false;
  agent_config_info_.enable_fast_lane_lut = true;
  use_right_hand_axis_ = use_right_hand_axis;
  is_simple_lane_structure_ = true;
}

// ======================== 第2层：主入口 UpdateSemanticMap ========================

/// @brief 更新语义地图的核心入口——这是整个数据管道的主处理函数
///
/// 处理流程（严格按顺序）：
///   1. 存储时间戳及所有传感数据（ego_vehicle, lane_net, obstacle_map, vehicles）
///   2. UpdateSemanticLaneSet() —— 将原始车道转为语义车道，校验拓扑一致性
///   3. UpdateLocalLanesAndFastLut() —— 构建局部连续大车道和双向查找表（可选）
///   4. UpdateSemanticVehicles() —— 为每辆周车注入语义信息（最近车道、行为预测、参考车道）
///   5. UpdateKeyVehicles() —— 筛选对自车规划重要的关键车辆
///   6. OpenloopTrajectoryPrediction() —— 对周车进行开环轨迹预测（可选）
///   7. SaveMapToLog() —— 将车辆状态记录到CSV日志（可选）
///
/// @note 步骤2-7的顺序有严格依赖关系：必须先生成语义车道，再基于车道分析车辆
ErrorType SemanticMapManager::UpdateSemanticMap(
    const double &time_stamp, const common::Vehicle &ego_vehicle,
    const common::LaneNet &whole_lane_net,
    const common::LaneNet &surrounding_lane_net,
    const common::GridMapND<ObstacleMapType, 2> &obstacle_map,
    const std::set<std::array<decimal_t, 2>> &obstacle_grids,
    const common::VehicleSet &surrounding_vehicles) {
  TicToc timer;
  time_stamp_ = time_stamp;
  set_ego_vehicle(ego_vehicle);
  set_whole_lane_net(whole_lane_net);
  set_surrounding_lane_net(surrounding_lane_net);
  set_obstacle_map(obstacle_map);
  set_obstacle_grids(obstacle_grids);
  set_surrounding_vehicles(surrounding_vehicles);

  // * 更新车道及拓扑关系
  UpdateSemanticLaneSet();

  // * 更新关键车道及其快速查找表（可选功能）
  if (agent_config_info_.enable_fast_lane_lut) {
    UpdateLocalLanesAndFastLut();
  }

  // * 为周车注入语义信息（最近车道、行为预测、参考车道）
  UpdateSemanticVehicles();

  // * 筛选对自车规划重要的关键车辆
  UpdateKeyVehicles();

  // * 开环轨迹预测——为所有语义周车预测未来轨迹（可选功能）
  if (agent_config_info_.enable_openloop_prediction) {
    OpenloopTrajectoryPrediction();
  }

  if (agent_config_info_.enable_log) {
    SaveMapToLog();
  }
  return kSuccess;
}

/// @brief 将当前语义地图数据保存到日志文件（CSV格式）
///
/// 输出格式：每行包含
///   时间戳, 车辆ID, x, y, 速度, 加速度, 航向角, 曲率, 方向盘转角
///
/// 同时记录自车和所有周围车辆的状态。
/// 以追加模式写入 agent_config_info_.log_file。
ErrorType SemanticMapManager::SaveMapToLog() {
  std::ofstream record_file_stream;
  record_file_stream.open(agent_config_info_.log_file,
                          std::ofstream::out | std::ofstream::app);
  record_file_stream.setf(std::ios_base::fixed);

  // 构建包含自车的完整车辆集合
  common::VehicleSet vehicle_set = surrounding_vehicles_;
  vehicle_set.vehicles.insert(std::make_pair(ego_vehicle_.id(), ego_vehicle_));

  // 逐车输出CSV行
  for (auto &v : vehicle_set.vehicles) {
    record_file_stream << v.second.state().time_stamp << "," << v.first << ","
                       << v.second.state().vec_position[0] << ","
                       << v.second.state().vec_position[1] << ","
                       << v.second.state().velocity << ","
                       << v.second.state().acceleration << ","
                       << v.second.state().angle << ","
                       << v.second.state().curvature << ","
                       << v.second.state().steer << std::endl;
  }
  return kSuccess;
}

// ======================== 第3层：行为预测 ========================

/// @brief 朴素规则横向行为预测——基于Frenet横向偏移的硬分类
///
/// 算法原理（利用Frenet坐标系下的横向信息）：
///   - d（横向偏移）和 dd（横向速度）描述车辆在车道坐标系中的侧向运动
///   - 若同时满足：
///     1. 横向距离超过阈值（d > lat_distance_threshold）
///     2. 横向速度超过阈值（dd > lat_vel_threshold）
///     3. 对应侧车道存在且换道可用
///     则判定为换道意图
///
/// 坐标系方向适配：
///   - 右手坐标系（use_right_hand_axis_=true）：d>0 表示偏向左侧 -> 左换道(LCL)
///   - 左手坐标系：d>0 表示偏向左侧 -> 右换道(LCR)（因车道编号方向相反）
///
/// @param vehicle 目标车辆
/// @param nearest_lane_id 目标车辆的最近车道ID（之前已通过GetNearestLaneIdUsingState获取）
/// @param lat_probs [输出] 横向行为概率（硬分类：某行为概率1.0，其余0.0）
ErrorType SemanticMapManager::NaiveRuleBasedLateralBehaviorPrediction(
    const common::Vehicle &vehicle, const int nearest_lane_id,
    common::ProbDistOfLatBehaviors *lat_probs) {
  if (nearest_lane_id == kInvalidLaneId) {
    lat_probs->is_valid = false;
    return kWrongStatus;
  }

  // 获取最近车道的语义信息（含换道可用性和相邻车道ID）
  SemanticLane nearest_lane =
      semantic_lane_set_.semantic_lanes.at(nearest_lane_id);
  common::StateTransformer stf(nearest_lane.lane);

  // 将车辆状态转换到车道 Frenet 坐标系
  common::FrenetState fs;
  if (stf.GetFrenetStateFromState(vehicle.state(), &fs) != kSuccess) {
    lat_probs->is_valid = false;
    return kWrongStatus;
  }

  decimal_t prob_lcl = 0.0;  // 左换道概率
  decimal_t prob_lcr = 0.0;  // 右换道概率
  decimal_t prob_lk = 0.0;   // 车道保持概率

  const decimal_t lat_distance_threshold = 0.4;  // 横向偏移距离阈值（米）
  const decimal_t lat_vel_threshold = 0.35;       // 横向速度阈值（m/s）

  // 根据坐标系方向适配换道方向判定
  if (use_right_hand_axis_) {
    // 右手坐标系：d>0 为左侧偏移
    if (fs.vec_dt[0] > lat_distance_threshold &&
        fs.vec_dt[1] > lat_vel_threshold &&
        nearest_lane.l_lane_id != kInvalidLaneId &&
        nearest_lane.l_change_avbl) {
      prob_lcl = 1.0;  // 向左偏移 + 向左运动 + 左车道可换 = 左换道
      printf(
          "[NaivePrediction]vehicle %d lane id %d, lat d %lf lat dd %lf, "
          "behavior lcl.\n",
          vehicle.id(), nearest_lane_id, fs.vec_dt[0], fs.vec_dt[1]);
    } else if (fs.vec_dt[0] < -lat_distance_threshold &&
               fs.vec_dt[1] < -lat_vel_threshold &&
               nearest_lane.r_lane_id != kInvalidLaneId &&
               nearest_lane.r_change_avbl) {
      prob_lcr = 1.0;  // 向右偏移 + 向右运动 + 右车道可换 = 右换道
      printf(
          "[NaivePrediction]vehicle %d lane id %d, lat d %lf lat dd %lf, "
          "behavior lcr.\n",
          vehicle.id(), nearest_lane_id, fs.vec_dt[0], fs.vec_dt[1]);
    } else {
      prob_lk = 1.0;  // 否则为车道保持
    }
  } else {
    // 左手坐标系：左右交换
    if (fs.vec_dt[0] > lat_distance_threshold &&
        fs.vec_dt[1] > lat_vel_threshold &&
        nearest_lane.r_lane_id != kInvalidLaneId &&
        nearest_lane.r_change_avbl) {
      prob_lcr = 1.0;
      printf(
          "[NaivePrediction]vehicle %d lane id %d, lat d %lf lat dd %lf, "
          "behavior lcr.\n",
          vehicle.id(), nearest_lane_id, fs.vec_dt[0], fs.vec_dt[1]);
    } else if (fs.vec_dt[0] < -lat_distance_threshold &&
               fs.vec_dt[1] < -lat_vel_threshold &&
               nearest_lane.l_lane_id != kInvalidLaneId &&
               nearest_lane.l_change_avbl) {
      prob_lcl = 1.0;
      printf(
          "[NaivePrediction]vehicle %d lane id %d, lat d %lf lat dd %lf, "
          "behavior lcl.\n",
          vehicle.id(), nearest_lane_id, fs.vec_dt[0], fs.vec_dt[1]);
    } else {
      prob_lk = 1.0;
    }
  }

  // 填充概率分布
  lat_probs->SetEntry(common::LateralBehavior::kLaneChangeLeft, prob_lcl);
  lat_probs->SetEntry(common::LateralBehavior::kLaneChangeRight, prob_lcr);
  lat_probs->SetEntry(common::LateralBehavior::kLaneKeeping, prob_lk);
  lat_probs->is_valid = true;

  return kSuccess;
}

/// @brief MOBIL规则换道行为预测
///
/// 基于MOBIL（Minimizing Overall Braking Induced by Lane changes）模型：
///
/// 算法流程：
///   1. 为三种横向行为（保持、左换、右换）分别构建参考车道
///   2. 对每个参考车道寻找前车和后车及其Frenet状态
///   3. 将车辆、车道、前后车信息封装为MOBIL输入
///   4. 调用 common::MobilBehaviorPrediction::LateralBehaviorPrediction 进行决策评估
///
/// MOBIL决策准则（双条件）：
///   - 安全准则：换道后后车的减速度不能超过安全阈值（b_safe）
///   - 激励准则：换道对目标车自身的加速度增益 + 礼貌因子*对周围车的影响 > 阈值
///
/// 与 NaiveRuleBased 的区别：
///   - Naive: 基于当前Frenet状态做硬分类（0/1概率）
///   - MOBIL: 基于IDM跟车模型预测换道后果，产生连续概率分布
///
/// @param vehicle 目标车辆
/// @param nearby_vehicles 附近车辆集合（用于查找前后车）
/// @param res [输出] MOBIL评估后的横向行为概率分布
ErrorType SemanticMapManager::MobilRuleBasedBehaviorPrediction(
    const common::Vehicle &vehicle, const common::VehicleSet &nearby_vehicles,
    common::ProbDistOfLatBehaviors *res) {
  decimal_t lane_radius = agent_config_info_.surrounding_search_radius;

  // 为三种行为构建输入数据结构
  vec_E<common::Lane> lanes;              // 每种行为对应的参考车道
  vec_E<common::Vehicle> leading_vehicles;   // 每种行为的前车
  vec_E<common::Vehicle> following_vehicles; // 每种行为的后车
  vec_E<common::FrenetState> leading_frenet_states;   // 前车Frenet状态
  vec_E<common::FrenetState> follow_frenet_states;    // 后车Frenet状态

  // 需要评估的三种横向行为
  std::vector<LateralBehavior> behaviors{LateralBehavior::kLaneKeeping,
                                         LateralBehavior::kLaneChangeLeft,
                                         LateralBehavior::kLaneChangeRight};

  // 对每种行为构建参考车道和前后车信息
  for (const auto &behavior : behaviors) {
    // ~ 准备参考车道（根据行为从当前车道映射到目标车道）
    common::Lane ref_lane;
    GetRefLaneForStateByBehavior(vehicle.state(), std::vector<int>(), behavior,
                                 lane_radius, lane_radius, false, &ref_lane);

    // ~ 在参考车道上寻找前后车及其Frenet状态
    bool has_leading_vehicle = false, has_following_vehicle = false;
    common::Vehicle leading_vehicle, following_vehicle;
    common::FrenetState leading_frenet_state, following_frenet_state;
    GetLeadingAndFollowingVehiclesFrenetStateOnLane(
        ref_lane, vehicle.state(), nearby_vehicles, &has_leading_vehicle,
        &leading_vehicle, &leading_frenet_state, &has_following_vehicle,
        &following_vehicle, &following_frenet_state);

    // ~ 组装结果
    lanes.push_back(ref_lane);
    leading_vehicles.push_back(leading_vehicle);
    following_vehicles.push_back(following_vehicle);
    leading_frenet_states.push_back(leading_frenet_state);
    follow_frenet_states.push_back(following_frenet_state);
  }

  // 调用MOBIL核心算法进行换道决策评估
  if (common::MobilBehaviorPrediction::LateralBehaviorPrediction(
          vehicle, lanes, leading_vehicles, leading_frenet_states,
          following_vehicles, follow_frenet_states, nearby_vehicles,
          res) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

/// @brief 在参考车道上一次性获取前车和后车的Frenet状态
///
/// 组合调用：
///   1. GetLeadingVehicleOnLane -> 查找前车
///   2. GetFollowingVehicleOnLane -> 查找后车
///   3. 将查找结果转换为Frenet坐标系（通过 StateTransformer）
///
/// @note 横向搜索范围固定为 lat_range = 2.2m（约半个车道宽度）
ErrorType SemanticMapManager::GetLeadingAndFollowingVehiclesFrenetStateOnLane(
    const common::Lane &ref_lane, const common::State &ref_state,
    const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
    common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
    bool *has_following_vehicle, common::Vehicle *following_vehicle,
    common::FrenetState *following_fs) const {
  decimal_t distance_residual_ratio = 0.0;
  const decimal_t lat_range = 2.2;  // 横向搜索范围：约半个车道宽度

  // 查找前后车
  GetLeadingVehicleOnLane(ref_lane, ref_state, vehicle_set, lat_range,
                          leading_vehicle, &distance_residual_ratio);
  GetFollowingVehicleOnLane(ref_lane, ref_state, vehicle_set, lat_range,
                            following_vehicle);

  common::StateTransformer stf(ref_lane);
  *has_leading_vehicle = false;
  *has_following_vehicle = false;

  // 将前车状态转换为Frenet坐标系
  if (leading_vehicle->id() != kInvalidAgentId) {
    if (stf.GetFrenetStateFromState(leading_vehicle->state(), leading_fs) ==
        kSuccess) {
      *has_leading_vehicle = true;
    } else {
      return kWrongStatus;
    }
  }
  // 将后车状态转换为Frenet坐标系
  if (following_vehicle->id() != kInvalidAgentId) {
    if (stf.GetFrenetStateFromState(following_vehicle->state(), following_fs) ==
        kSuccess) {
      *has_following_vehicle = true;
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

/// @brief 获取自车最近的lane ID
/// @note 使用 ego_vehicle_.state() 查询，不需要导航路径约束
ErrorType SemanticMapManager::GetEgoNearestLaneId(int *ego_lane_id) const {
  int nearest_lane_id;
  decimal_t distance, arclen;
  if (GetNearestLaneIdUsingState(ego_vehicle_.state().ToXYTheta(),
                                 std::vector<int>(), &nearest_lane_id,
                                 &distance, &arclen) != kSuccess) {
    return kWrongStatus;
  }
  *ego_lane_id = nearest_lane_id;
  return kSuccess;
}

// ======================== 第3层：语义车辆构建 ========================

/// @brief 更新语义车辆集合——为每辆周车注入语义信息
///
/// 对 surrounding_vehicles_ 中的每辆车：
///   1. 查找最近车道ID和弧长位置
///   2. 进行朴素规则横向行为预测（NaiveRuleBasedLateralBehaviorPrediction）
///   3. 取最大概率行为作为当前横向行为（lat_behavior）
///   4. 根据行为构建对应的参考车道
///      - 前向长度 = max(车速 * 10s, 50m)
///      - 后向长度 = 10m
///
/// 结果存储在 semantic_surrounding_vehicles_ 中，
/// 通过 swap 原子替换（避免并发读写问题）。
ErrorType SemanticMapManager::UpdateSemanticVehicles() {
  // * 构建语义车辆集合
  // * 必要信息：车辆、最近车道ID（不带导航路径）、基于最近车道的横向行为
  // * 附加信息：预测轨迹、参考车道
  common::SemanticVehicleSet semantic_vehicles_tmp;

  for (const auto &v : surrounding_vehicles_.vehicles) {
    common::SemanticVehicle semantic_vehicle;
    semantic_vehicle.vehicle = v.second;

    // 步骤1：查找最近车道
    GetNearestLaneIdUsingState(
        semantic_vehicle.vehicle.state().ToXYTheta(), std::vector<int>(),
        &semantic_vehicle.nearest_lane_id, &semantic_vehicle.dist_to_lane,
        &semantic_vehicle.arc_len_onlane);

    // 步骤2：朴素规则横向行为预测
    NaiveRuleBasedLateralBehaviorPrediction(
        semantic_vehicle.vehicle, semantic_vehicle.nearest_lane_id,
        &semantic_vehicle.probs_lat_behaviors);

    // 步骤3：取最大概率行为作为当前行为
    semantic_vehicle.probs_lat_behaviors.GetMaxProbBehavior(
        &semantic_vehicle.lat_behavior);

    // 步骤4：根据行为构建参考车道
    decimal_t max_backward_len = 10.0;
    // 前向长度 = 速度 * 10秒，至少50m（保证足够的前向可见性）
    decimal_t forward_lane_len =
        std::max(semantic_vehicle.vehicle.state().velocity * 10.0, 50.0);
    GetRefLaneForStateByBehavior(
        semantic_vehicle.vehicle.state(), std::vector<int>(),
        semantic_vehicle.lat_behavior, forward_lane_len, max_backward_len,
        false, &semantic_vehicle.lane);

    // 插入临时集合
    semantic_vehicles_tmp.semantic_vehicles.insert(
        std::pair<int, common::SemanticVehicle>(semantic_vehicle.vehicle.id(),
                                                semantic_vehicle));
  }

  // 原子替换（swap）以避免并发问题
  {
    semantic_surrounding_vehicles_.semantic_vehicles.swap(
        semantic_vehicles_tmp.semantic_vehicles);
  }
  return kSuccess;
}

/// @brief 对所有语义周围车辆执行开环轨迹预测
///
/// 对 semantic_surrounding_vehicles_ 中的每辆车：
///   调用 TrajectoryPredictionForVehicle，使用：
///     - pred_time_ = 5.0s（预测总时长）
///     - pred_step_ = 0.2s（预测步长，产生约25个预测状态点）
///
/// 预测结果存储在 openloop_pred_trajs_ map 中（key=车辆ID, value=状态序列），
/// 后续用于 CheckCollisionUsingStateAndVehicleParam 中的动态碰撞检测。
ErrorType SemanticMapManager::OpenloopTrajectoryPrediction() {
  openloop_pred_trajs_.clear();
  for (const auto &p_sv : semantic_surrounding_vehicles_.semantic_vehicles) {
    auto semantic_vehicle = p_sv.second;
    vec_E<common::State> traj;
    // 使用IDM跟车模型在参考车道上进行开环预测
    TrajectoryPredictionForVehicle(semantic_vehicle.vehicle,
                                   semantic_vehicle.lane, pred_time_,
                                   pred_step_, &traj);
    openloop_pred_trajs_.insert({{semantic_vehicle.vehicle.id(), traj}});
  }
  return kSuccess;
}

// ======================== 第3层：语义车道构建 ========================

/// @brief 更新语义车道集合——将原始车道数据转换为语义车道
///
/// 两步处理：
///   1. 将 surrounding_lane_net_.lane_set 中的每条原始车道转换为 SemanticLane：
///      - 复制ID、方向、父子关系、左右车道关系、换道可用性、长度等
///      - 通过 LaneGenerator 从采样点拟合 Lane 对象
///   2. 一致性校验——修复拓扑不一致的情况：
///      - 若左换道目标车道不在语义集合中 -> 标记左换道不可用
///      - 若右换道目标车道不在语义集合中 -> 标记右换道不可用
///      - 清理无效的 father_id 和 child_id 引用
///
/// @note 这一步是连接"几何车道数据"和"语义推理"的关键桥梁
ErrorType SemanticMapManager::UpdateSemanticLaneSet() {
  semantic_lane_set_.clear();

  // 步骤1：将原始车道转换为语义车道
  {
    for (const auto &pe : surrounding_lane_net_.lane_set) {
      common::SemanticLane semantic_lane;
      // 复制拓扑信息
      semantic_lane.id = pe.second.id;
      semantic_lane.dir = pe.second.dir;
      semantic_lane.child_id = pe.second.child_id;
      semantic_lane.father_id = pe.second.father_id;
      // 复制换道信息
      semantic_lane.l_lane_id = pe.second.l_lane_id;
      semantic_lane.l_change_avbl = pe.second.l_change_avbl;
      semantic_lane.r_lane_id = pe.second.r_lane_id;
      semantic_lane.r_change_avbl = pe.second.r_change_avbl;
      semantic_lane.behavior = pe.second.behavior;
      semantic_lane.length = pe.second.length;

      // 从车道采样点拟合车道几何
      vec_Vecf<2> samples;
      for (const auto &pt : pe.second.lane_points) {
        samples.push_back(pt);
      }
      if (common::LaneGenerator::GetLaneBySamplePoints(
              samples, &semantic_lane.lane) != kSuccess) {
        continue;  // 拟合失败则跳过该车道
      }

      semantic_lane_set_.semantic_lanes.insert(
          std::pair<int, common::SemanticLane>(semantic_lane.id,
                                               semantic_lane));
    }
  }

  // 步骤2：一致性校验——修复拓扑不一致
  {
    for (auto &semantic_lane : semantic_lane_set_.semantic_lanes) {
      // 校验左换道：若目标车道不存在，则标记不可用
      if (semantic_lane.second.l_change_avbl) {
        auto l_it = semantic_lane_set_.semantic_lanes.find(
            semantic_lane.second.l_lane_id);
        if (l_it == semantic_lane_set_.semantic_lanes.end()) {
          semantic_lane.second.l_change_avbl = false;
          semantic_lane.second.l_lane_id = kInvalidLaneId;
        }
      }
      // 校验右换道：若目标车道不存在，则标记不可用
      if (semantic_lane.second.r_change_avbl) {
        auto r_it = semantic_lane_set_.semantic_lanes.find(
            semantic_lane.second.r_lane_id);
        if (r_it == semantic_lane_set_.semantic_lanes.end()) {
          semantic_lane.second.r_change_avbl = false;
          semantic_lane.second.r_lane_id = kInvalidLaneId;
        }
      }
      // 清理无效的父车道引用
      for (auto it = semantic_lane.second.father_id.begin();
           it < semantic_lane.second.father_id.end();) {
        auto father_it = semantic_lane_set_.semantic_lanes.find(*it);
        if (father_it == semantic_lane_set_.semantic_lanes.end()) {
          it = semantic_lane.second.father_id.erase(it);
        } else {
          ++it;
        }
      }
      // 清理无效的子车道引用
      for (auto it = semantic_lane.second.child_id.begin();
           it < semantic_lane.second.child_id.end();) {
        auto child_it = semantic_lane_set_.semantic_lanes.find(*it);
        if (child_it == semantic_lane_set_.semantic_lanes.end()) {
          it = semantic_lane.second.child_id.erase(it);
        } else {
          ++it;
        }
      }
    }
  }
  return kSuccess;
}

/// @brief 构建局部车道快速查找表（Fast LUT）
///
/// 目的：将离散的 segment 车道拼接成连续的长车道，并构建双向索引，
///      以便在 GetRefLaneForStateByBehavior 等高频查询中通过查表替代
///      实时采样和拟合，大幅加速运算。
///
/// 构建流程：
///   1. 找到自车所在的根车道（cur_lane_id）
///   2. 扩展左右相邻车道（最多5条根车道：左左-左-当前-右-右右）
///   3. 对每条根车道：
///      a. 递归向前展开（沿 child_id 链），直到累计长度 >= local_lane_length_forward_ (250m)
///      b. 递归向后展开（沿 father_id 链），直到累计长度 >= local_lane_length_backward_ (150m)
///      c. 将前后路径拼装（前向路径 + 后向反向路径）
///      d. 用车道ID序列拟合连续 Lane 对象
///      e. 分配 local_lane_id 并构建双向LUT
///
/// LUT 结构：
///   - local_to_segment_lut_：local_lane_id -> [seq_lane_id_0, seq_lane_id_1, ...]
///     用于从局部车道反查包含哪些段车道
///   - segment_to_local_lut_：seq_lane_id -> {local_lane_id_0, local_lane_id_1, ...}
///     用于从段车道ID快速找到包含它的局部车道
ErrorType SemanticMapManager::UpdateLocalLanesAndFastLut() {
  common::State ego_state = ego_vehicle_.state();
  int cur_lane_id;
  decimal_t dist_tmp, arc_len_tmp;

  // 步骤1：找到自车所在车道
  if (kSuccess != GetNearestLaneIdUsingState(ego_state.ToXYTheta(),
                                             std::vector<int>(), &cur_lane_id,
                                             &dist_tmp, &arc_len_tmp)) {
    return kWrongStatus;
  }

  // 清空现有LUT
  local_lanes_.clear();
  local_to_segment_lut_.clear();
  segment_to_local_lut_.clear();

  // * 步骤2：确定根车道集合（自车车道 + 左右各最多2层相邻车道）
  // * 目前仅考虑自车车道及其相邻车道（共最多5条根车道）
  std::vector<int> root_lane_ids;
  {
    root_lane_ids.push_back(cur_lane_id);
    // ~ 左侧车道
    if (whole_lane_net_.lane_set.at(cur_lane_id).l_lane_id > 0) {
      int l_id = whole_lane_net_.lane_set.at(cur_lane_id).l_lane_id;
      root_lane_ids.push_back(l_id);
      // ~ 左侧的左侧
      if (whole_lane_net_.lane_set.at(l_id).l_lane_id > 0) {
        int ll_id = whole_lane_net_.lane_set.at(l_id).l_lane_id;
        root_lane_ids.push_back(ll_id);
      }
    }
    // ~ 右侧车道
    if (whole_lane_net_.lane_set.at(cur_lane_id).r_lane_id > 0) {
      int r_id = whole_lane_net_.lane_set.at(cur_lane_id).r_lane_id;
      root_lane_ids.push_back(r_id);
      // ~ 右侧的右侧
      if (whole_lane_net_.lane_set.at(r_id).r_lane_id > 0) {
        int rr_id = whole_lane_net_.lane_set.at(r_id).r_lane_id;
        root_lane_ids.push_back(rr_id);
      }
    }
  }

  // 对每条根车道：向前/向后递归展开 -> 拼装 -> 拟合 -> 建立LUT
  std::vector<std::vector<int>> lane_ids_expand_front;
  for (const auto root_id : root_lane_ids) {
    // 计算自车在当前根车道上的弧长
    decimal_t arc_len;
    semantic_lane_set_.semantic_lanes.at(root_id)
        .lane.GetArcLengthByVecPosition(ego_state.vec_position, &arc_len);
    // 当前车道剩余前向长度
    decimal_t length_remain =
        semantic_lane_set_.semantic_lanes.at(root_id).length - arc_len;

    // ~ 前向递归展开
    std::vector<std::vector<int>> all_paths_forward;
    GetAllForwardLaneIdPathsWithMinimumLengthByRecursion(
        root_id, length_remain, 0.0, std::vector<int>(), &all_paths_forward);

    // ~ 后向递归展开
    std::vector<std::vector<int>> all_paths_backward;
    GetAllBackwardLaneIdPathsWithMinimumLengthByRecursion(
        root_id, arc_len, 0.0, std::vector<int>(), &all_paths_backward);

    // ~ 拼装前后路径：
    //    对于每一对 (forward_path, backward_path)
    //    assembled_path = backward_path(去掉最后一个=root) + forward_path
    std::vector<std::vector<int>> assembled_paths;
    for (const auto &f_path : all_paths_forward) {
      for (const auto &b_path : all_paths_backward) {
        std::vector<int> path = f_path;
        path.erase(path.begin());  // 去掉根车道（已在backward末尾）
        path.insert(path.begin(), b_path.begin(), b_path.end());
        assembled_paths.push_back(path);
      }
    }

    // ~ 拟合局部车道并构建LUT
    int local_lane_cnt = local_lanes_.size();
    for (const auto &path : assembled_paths) {
      common::Lane lane;
      // 从车道ID序列拟合连续的长车道对象
      if (kSuccess !=
          GetLocalLaneUsingLaneIds(ego_state, path, local_lane_length_forward_,
                                   local_lane_length_backward_, true, &lane)) {
        continue;
      }
      int local_id = local_lane_cnt++;
      local_lanes_.insert(std::pair<int, common::Lane>(local_id, lane));

      // 建立 local -> segment 映射
      local_to_segment_lut_.insert(
          std::pair<int, std::vector<int>>(local_id, path));

      // 建立 segment -> local 映射（一个段车道可能属于多个局部车道）
      for (const auto &id : path) {
        segment_to_local_lut_[id].insert(local_id);
      }
    }
  }

  has_fast_lut_ = true;
  return kSuccess;
}

/// @brief 递归获取前向车道ID路径——沿 child_id 链展开
///
/// 递归终止条件（满足任一即停止）：
///   1. 累计长度 >= local_lane_length_forward_ (250m) —— 达到最小展开长度
///   2. 当前车道没有子车道（child_id 为空）—— 到达道路终点
///
/// @param node_id 当前车道节点ID
/// @param node_length 当前车道段的长度
/// @param aggre_length 到达当前节点前已累计的长度
/// @param path_to_node 到达当前节点之前的路径
/// @param all_paths [输出] 所有满足条件的前向路径集合
void SemanticMapManager::GetAllForwardLaneIdPathsWithMinimumLengthByRecursion(
    const decimal_t &node_id, const decimal_t &node_length,
    const decimal_t &aggre_length, const std::vector<int> &path_to_node,
    std::vector<std::vector<int>> *all_paths) {
  // * 沿 child_id 方向前向遍历

  decimal_t node_aggre_length = node_length + aggre_length;
  auto path = path_to_node;
  path.push_back(node_id);

  // 终止条件：累计长度满足要求 或 无子车道
  if (node_aggre_length >= local_lane_length_forward_ ||
      whole_lane_net_.lane_set.at(node_id).child_id.empty()) {
    // 递归终止，记录该路径
    all_paths->push_back(path);
    return;
  } else {
    // 递归展开：遍历所有子车道
    auto child_ids = whole_lane_net_.lane_set.at(node_id).child_id;
    for (const auto child_id : child_ids) {
      decimal_t child_length = whole_lane_net_.lane_set.at(child_id).length;
      GetAllForwardLaneIdPathsWithMinimumLengthByRecursion(
          child_id, child_length, node_aggre_length, path, all_paths);
    }
  }
}

/// @brief 递归获取后向车道ID路径——沿 father_id 链展开
///
/// 递归终止条件（满足任一即停止）：
///   1. 累计长度 >= local_lane_length_backward_ (150m) —— 达到最小展开长度
///   2. 当前车道没有父车道（father_id 为空）—— 到达道路起点
///
/// @note 与前向版本的关键区别：终止时路径会反转（reverse），
///       使得最终输出的路径从最远的父车道指向当前根车道
void SemanticMapManager::GetAllBackwardLaneIdPathsWithMinimumLengthByRecursion(
    const decimal_t &node_id, const decimal_t &node_length,
    const decimal_t &aggre_length, const std::vector<int> &path_to_node,
    std::vector<std::vector<int>> *all_paths) {
  // * 沿 father_id 方向后向遍历

  decimal_t node_aggre_length = node_length + aggre_length;
  auto path = path_to_node;
  path.push_back(node_id);

  if (node_aggre_length >= local_lane_length_backward_ ||
      whole_lane_net_.lane_set.at(node_id).father_id.empty()) {
    // 递归终止，反转路径使其为从远到近的顺序
    std::reverse(path.begin(), path.end());
    all_paths->push_back(path);
    return;
  } else {
    // 递归展开：遍历所有父车道
    auto father_ids = whole_lane_net_.lane_set.at(node_id).father_id;
    for (const auto father_id : father_ids) {
      decimal_t father_length = whole_lane_net_.lane_set.at(father_id).length;
      GetAllBackwardLaneIdPathsWithMinimumLengthByRecursion(
          father_id, father_length, node_aggre_length, path, all_paths);
    }
  }
}

// ======================== 第7层：查询接口 ========================

/// @brief 获取所有语义车道到给定3自由度状态的距离信息
///
/// 遍历所有语义车道，对每条车道计算：
///   1. arc_len：状态投影到车道参考线的弧长位置
///   2. dist：欧氏距离（投影点到状态点的距离）
///   3. angle_diff：车道方向与状态航向角的差值（归一化）
///
/// 将距离在 lane_range_ (10m) 内的车道按距离排序后返回。
/// 返回元组格式：(距离, 弧长, 角度差, 车道ID)
///
/// @note 距离 > lane_range_ 的车道被过滤掉，减少后续处理的候选数量
ErrorType SemanticMapManager::GetDistanceToLanesUsing3DofState(
    const Vec3f &state,
    std::set<std::tuple<decimal_t, decimal_t, decimal_t, int>> *res) const {
  for (const auto &p : semantic_lane_set_.semantic_lanes) {
    decimal_t arc_len;
    // 计算状态点在车道参考线上的投影弧长
    p.second.lane.GetArcLengthByVecPosition(Vec2f(state(0), state(1)),
                                            &arc_len);

    // 获取投影点的世界坐标
    Vec2f pt;
    p.second.lane.GetPositionByArcLength(arc_len, &pt);
    // 计算欧氏距离
    double dist = std::hypot((state(0) - pt(0)), (state(1) - pt(1)));

    // 过滤距离过远的车道
    if (dist > lane_range_) continue;

    // 计算车道方向与车辆航向的角度差
    decimal_t lane_angle;
    p.second.lane.GetOrientationByArcLength(arc_len, &lane_angle);
    decimal_t angle_diff = normalize_angle(lane_angle - state(2));

    // 插入结果集合（自动按距离排序，因tuple首元素为dist）
    res->insert(std::tuple<decimal_t, decimal_t, decimal_t, int>(
        dist, arc_len, angle_diff, p.second.id));
  }
  return kSuccess;
}

/// @brief 检查两个车辆状态之间是否碰撞（基于有向包围盒OBB相交判断）
///
/// 算法：
///   1. 从车辆参数和状态构建OBB（Oriented Bounding Box）
///   2. 调用 SAT（分离轴定理）检查两个OBB是否相交
ErrorType SemanticMapManager::CheckCollisionUsingState(
    const common::VehicleParam &param_a, const common::State &state_a,
    const common::VehicleParam &param_b, const common::State &state_b,
    bool *res) {
  common::OrientedBoundingBox2D obb_a, obb_b;
  // 构建车辆A的OBB
  common::SemanticsUtils::GetOrientedBoundingBoxForVehicleUsingState(
      param_a, state_a, &obb_a);
  // 构建车辆B的OBB
  common::SemanticsUtils::GetOrientedBoundingBoxForVehicleUsingState(
      param_b, state_b, &obb_b);
  // SAT分离轴定理判定相交
  *res = common::ShapeUtils::CheckIfOrientedBoundingBoxIntersect(obb_a, obb_b);
  return kSuccess;
}

/// @brief 检查给定车辆参数状态是否与环境发生碰撞
///
/// 两步检测：
///   1. 静态碰撞检测：检查车辆OBB的4个顶点是否落在障碍物栅格地图的占据区域
///   2. 动态碰撞检测（仅当 enable_openloop_prediction 为 true）：
///      遍历所有语义周车的开环预测轨迹，在对应时间戳的预测状态上进行OBB碰撞检测
///
/// @note 静态检测先执行，一旦发现碰撞立即返回，无需再进行动态检测
/// @note 动态检测通过时间戳对齐：access_index = round((state_stamp - init_stamp) / pred_step_)
ErrorType SemanticMapManager::CheckCollisionUsingStateAndVehicleParam(
    const common::VehicleParam &vehicle_param, const common::State &state,
    bool *res) {
  // 第1步：静态碰撞检测——检查车辆顶点是否在障碍物地图的占据区域
  {
    // TODO: (@denny.ding) 添加静态碰撞检测的完整实现
    common::Vehicle vehicle;
    vehicle.set_state(state);
    vehicle.set_param(vehicle_param);
    // 获取车辆OBB的4个顶点
    vec_E<Vec2f> vertices;
    common::ShapeUtils::GetVerticesOfOrientedBoundingBox(
        vehicle.RetOrientedBoundingBox(), &vertices);

    bool is_collision = false;
    // 检查每个顶点是否落在障碍物占据区域
    for (auto &v : vertices) {
      CheckCollisionUsingGlobalPosition(v, &is_collision);
      if (is_collision) {
        *res = is_collision;
        return kSuccess;  // 发现碰撞，立即返回
      }
    }
  }

  // 第2步：动态碰撞检测——与周车的开环预测轨迹碰撞
  if (agent_config_info_.enable_openloop_prediction) {
    for (const auto &v : semantic_surrounding_vehicles_.semantic_vehicles) {
      auto state_stamp = state.time_stamp;
      auto obstacle_init_stamp = v.second.vehicle.state().time_stamp;
      auto openloop_pred_traj = openloop_pred_trajs_.at(v.first);

      // 通过时间戳对齐，找到该时间点对应的预测状态索引
      int access_index =
          std::round((state_stamp - obstacle_init_stamp) / pred_step_);
      int num_pred_states = static_cast<int>(openloop_pred_traj.size());

      // 边界检查：索引必须合法
      if (access_index < 0 || access_index >= num_pred_states) continue;

      bool is_collision = false;
      // OBB碰撞检测
      CheckCollisionUsingState(vehicle_param, state, v.second.vehicle.param(),
                               openloop_pred_traj[access_index], &is_collision);
      if (is_collision) {
        *res = is_collision;
        return kSuccess;  // 发现碰撞，立即返回
      }
    }
  }
  *res = false;
  return kSuccess;
}

/// @brief 检查给定全局坐标是否在障碍物占据区域
ErrorType SemanticMapManager::CheckCollisionUsingGlobalPosition(
    const Vec2f &p_w, bool *res) const {
  std::array<decimal_t, 2> p = {{p_w(0), p_w(1)}};
  return obstacle_map_.CheckIfEqualUsingGlobalPosition(p, GridMap2D::OCCUPIED,
                                                       res);
}

/// @brief 获取给定全局坐标位置的障碍物地图值
ErrorType SemanticMapManager::GetObstacleMapValueUsingGlobalPosition(
    const Vec2f &p_w, ObstacleMapType *res) {
  std::array<decimal_t, 2> p = {{p_w(0), p_w(1)}};
  return obstacle_map_.GetValueUsingGlobalPosition(p, res);
}

/// @brief 判断给定车道ID是否能通过拓扑连接到达导航路径中的任意节点
///
/// 使用 BFS（广度优先搜索）进行拓扑可达性检查：
///   - 从 lane_id 出发，沿 child_id、l_lane_id（左换道）、r_lane_id（右换道）展开
///   - 最大展开节点数限制：20（防止搜索爆炸）
///   - 同时统计最少需要的换道次数（num_lane_changes）
///   - 每步换道（沿 l_lane_id 或 r_lane_id 移动）计数+1
///
/// @param lane_id 起始车道ID
/// @param path 目标导航路径（车道ID序列）
/// @param num_lane_changes [输出] 最少需要的换道次数
/// @param res [输出] 是否可达
ErrorType SemanticMapManager::IsTopologicallyReachable(
    const int lane_id, const std::vector<int> &path, int *num_lane_changes,
    bool *res) const {
  if (semantic_lane_set_.semantic_lanes.count(lane_id) == 0) {
    printf("[IsTopologicallyReachable]fail to get lane id %d.\n", lane_id);
    return kWrongStatus;
  }

  // ~ 检查路径中是否有节点从 lane_id 出发可达
  const int max_expansion_nodes = 20;

  // ~ BFS 数据结构
  std::unordered_map<int, int> num_lc_map;  // 记录到每个节点的最少换道次数
  std::set<int> visited_set;                 // 已访问节点集合
  std::list<int> queue;                      // BFS队列

  visited_set.insert(lane_id);
  queue.push_back(lane_id);
  num_lc_map.insert(std::make_pair(lane_id, 0));

  int expanded_nodes = 1;
  int cur_id = lane_id;
  bool is_reachable = false;

  // BFS 主循环
  while (!queue.empty() && expanded_nodes < max_expansion_nodes) {
    cur_id = queue.front();
    queue.pop_front();
    expanded_nodes++;

    // 检查当前节点是否在目标路径中
    if (std::find(path.begin(), path.end(), cur_id) != path.end()) {
      *num_lane_changes = num_lc_map.at(cur_id);
      is_reachable = true;
      break;
    }

    // 获取当前节点的所有后继（child、左换道、右换道）
    std::vector<int> child_ids;
    auto it = semantic_lane_set_.semantic_lanes.find(cur_id);
    if (it == semantic_lane_set_.semantic_lanes.end()) {
      continue;
    } else {
      child_ids = it->second.child_id;  // 前向子车道
      if (it->second.l_change_avbl) child_ids.push_back(it->second.l_lane_id);  // 左换道
      if (it->second.r_change_avbl) child_ids.push_back(it->second.r_lane_id);  // 右换道
    }
    if (child_ids.empty()) continue;

    // 展开未访问的后继节点
    for (auto &id : child_ids) {
      if (visited_set.count(id) == 0) {
        visited_set.insert(id);
        queue.push_back(id);
        // 换道计数：沿相邻车道的移动 +1
        if (it->second.l_change_avbl && id == it->second.l_lane_id) {
          num_lc_map.insert(std::make_pair(id, num_lc_map.at(cur_id) + 1));
        } else if (it->second.r_change_avbl && id == it->second.r_lane_id) {
          num_lc_map.insert(std::make_pair(id, num_lc_map.at(cur_id) + 1));
        } else {
          num_lc_map.insert(std::make_pair(id, num_lc_map.at(cur_id)));
        }
      }
    }
  }

  *res = is_reachable;
  return kSuccess;
}

/// @brief 获取给定状态相对于导航路径的最近车道ID
///
/// 选择策略（分优先级）：
///
///   第1优先级：在 nearest_lane_range_ (1.5m) 内，取角度差最小的车道
///     - 按照 (角度差, 距离, 弧长, ID) 的元组排序
///     - 适用于车辆明确在某车道上的情况
///
///   第2优先级：若无车道在 1.5m 内且角度差 < 90度，则在所有车道中
///     搜索角度差 < 90度的最近车道
///     - 适用于车辆稍微偏离车道的情况
///
///   第3优先级：以上都不满足，直接取距离最近的车道（不限制角度差）
///     - 最终兜底方案
///
/// @param state 三维状态 (x, y, theta)
/// @param navi_path 导航路径（当前版本中未使用，预留用于约束搜索）
/// @param id [输出] 最近车道ID
/// @param distance [输出] 到最近车道的距离
/// @param arc_len [输出] 在最近车道上的弧长位置
ErrorType SemanticMapManager::GetNearestLaneIdUsingState(
    const Vec3f &state, const std::vector<int> &navi_path, int *id,
    decimal_t *distance, decimal_t *arc_len) const {
  // 获取所有车道到该状态的距离信息（按距离排序）
  // tuple: dist, arc_len, angle_diff, id
  std::set<std::tuple<decimal_t, decimal_t, decimal_t, int>> lanes_in_dist;
  if (GetDistanceToLanesUsing3DofState(state, &lanes_in_dist) != kSuccess) {
    return kWrongStatus;
  }

  if (lanes_in_dist.empty()) {
    printf("[GetNearestLaneIdUsingState]No nearest lane found.\n");
    return kWrongStatus;
  }

  // * 第1优先级：在 nearest_lane_range_ 内按角度差排序
  // tuple: angle_diff, dist, arc_len, id
  std::set<std::tuple<decimal_t, decimal_t, decimal_t, int>>
      lanes_in_angle_diff;

  // 将距离在 nearest_lane_range_ 内的车道按角度差重新排序
  for (const auto &ele : lanes_in_dist) {
    if (std::get<0>(ele) > nearest_lane_range_) {
      break;  // 由于 lanes_in_dist 按距离排序，第一个超出范围的即可停止
    }
    lanes_in_angle_diff.insert(std::tuple<decimal_t, decimal_t, decimal_t, int>(
        fabs(std::get<2>(ele)), std::get<0>(ele), std::get<1>(ele),
        std::get<3>(ele)));
  }

  // 若第1优先级无候选 或 最佳候选角度差 > 90度
  if (lanes_in_angle_diff.empty() ||
      std::get<0>(*lanes_in_angle_diff.begin()) > kPi / 2) {
    // * 第2优先级：在所有车道中寻找角度差 < 90度的最近车道
    for (const auto &ele : lanes_in_dist) {
      if (std::get<2>(ele) < kPi / 2) {
        *id = std::get<3>(ele);
        *distance = std::get<0>(ele);
        *arc_len = std::get<1>(ele);
        return kSuccess;
      }
    }
    // * 第3优先级：无满足角度要求的，取最近车道
    *id = std::get<3>(*lanes_in_dist.begin());
    *distance = std::get<0>(*lanes_in_dist.begin());
    *arc_len = std::get<1>(*lanes_in_dist.begin());
    return kSuccess;
  }

  // * 使用最小角度差车道（第1优先级命中）
  *id = std::get<3>(*lanes_in_angle_diff.begin());
  *distance = std::get<1>(*lanes_in_angle_diff.begin());
  *arc_len = std::get<2>(*lanes_in_angle_diff.begin());

  return kSuccess;
}

/// @brief 对单一车辆进行开环轨迹预测（委托给 OnLaneFsPredictor）
///
/// @note 使用基于 IDM（Intelligent Driver Model）跟车模型 + 车道参考线的
///        开环预测方法，不考虑周车之间的相互作用
ErrorType SemanticMapManager::TrajectoryPredictionForVehicle(
    const common::Vehicle &vehicle, const common::Lane &lane,
    const decimal_t &t_pred, const decimal_t &t_step,
    vec_E<common::State> *traj) {
  planning::OnLaneFsPredictor::GetPredictedTrajectory(lane, vehicle, t_pred,
                                                      t_step, traj);
  return kSuccess;
}

// ======================== 第4层：关键车辆筛选 ========================

/// @brief 更新关键车辆集合——筛选对自车规划重要的车辆
///
/// 筛选策略（两阶段）：
///
/// 阶段1 - 构建关键车道ID集合：
///   从自车当前车道及左右相邻车道出发，沿 child_id 方向前向展开
///   （最大距离 search_radius），沿 father_id 方向后向展开，
///   收集所有可达车道及其相对弧长偏移。
///
/// 阶段2 - 筛选关键车辆：
///   对每辆语义周车：
///     1. 检查其最近车道是否在关键车道集合中
///     2. 计算车辆相对于自车的弧长距离（dist = len_offset + arc_len_onlane）
///     3. 前方车辆：dist >= 0 且 |dist| < front_range（自适应裁剪范围）
///     4. 后方车辆：dist < 0 且 |dist| < s_margin（基于速度的裕量）
///     5. 排除当前车道后方已经过自车的车辆（避免选到已驶过自车的后车）
///
/// 自适应裁剪范围（front_range）计算：
///   front_range = clamp(ego_velocity * t_comfort + 100, 30, 170)
///   其中 t_comfort = max(t_pl, ego_velocity / dec_comfort)
///   即：规划时间内行驶距离 + 100m安全余量，限制在30-170m之间
///
/// @note 直接将所有 semantic_surrounding_vehicles_ 作为初始候选，
///        然后通过车道拓扑距离筛选
ErrorType SemanticMapManager::UpdateKeyVehicles() {
  // ~ 初始将所有语义周车标记为关键车辆，然后逐步筛选
  semantic_key_vehicles_ = semantic_surrounding_vehicles_;
  key_vehicles_ = surrounding_vehicles_;

  decimal_t search_radius = agent_config_info_.surrounding_search_radius;

  // ====== 阶段1：构建关键车道ID集合 ======
  std::map<int, decimal_t> key_lane_ids;  // lane_id -> 相对于自车的弧长偏移
  int cur_lane_id;
  decimal_t cur_lane_dist;
  decimal_t cur_arc_len;

  // 获取自车当前车道
  if (GetNearestLaneIdUsingState(ego_vehicle_.state().ToXYTheta(),
                                 std::vector<int>(), &cur_lane_id,
                                 &cur_lane_dist, &cur_arc_len) != kSuccess) {
    return kWrongStatus;
  }

  // 构建关键车道ID集合——沿拓扑方向展开
  {
    std::vector<int> mid_lane_ids;

    // 自车车道 + 左右相邻车道作为种子
    key_lane_ids.insert(std::pair<int, decimal_t>(cur_lane_id, -cur_arc_len));
    mid_lane_ids.push_back(cur_lane_id);

    // 左相邻车道
    if (whole_lane_net_.lane_set.at(cur_lane_id).l_change_avbl) {
      key_lane_ids.insert(std::pair<int, decimal_t>(
          whole_lane_net_.lane_set.at(cur_lane_id).l_lane_id, -cur_arc_len));
      mid_lane_ids.push_back(
          whole_lane_net_.lane_set.at(cur_lane_id).l_lane_id);
    }
    // 右相邻车道
    if (whole_lane_net_.lane_set.at(cur_lane_id).r_change_avbl) {
      key_lane_ids.insert(std::pair<int, decimal_t>(
          whole_lane_net_.lane_set.at(cur_lane_id).r_lane_id, -cur_arc_len));
      mid_lane_ids.push_back(
          whole_lane_net_.lane_set.at(cur_lane_id).r_lane_id);
    }

    // 前向展开——后继车道
    decimal_t len_front =
        whole_lane_net_.lane_set.at(cur_lane_id).length - cur_arc_len;
    for (const int &id : mid_lane_ids) {
      decimal_t len_sum = len_front;
      if (len_sum >= search_radius) continue;

      // 使用优先队列展开（pair<长度, 车道ID>，按长度排序）
      std::set<std::pair<decimal_t, int>> id_to_expand;
      id_to_expand.insert(std::pair<decimal_t, int>(len_sum, id));

      while (!id_to_expand.empty()) {
        auto it = id_to_expand.begin();
        decimal_t len_expand = it->first;
        int id_expand = it->second;
        id_to_expand.erase(it);

        // 展开子车道
        std::vector<int> succ_ids =
            whole_lane_net_.lane_set.at(id_expand).child_id;
        for (const auto &succ_id : succ_ids) {
          key_lane_ids.insert(std::pair<int, decimal_t>(succ_id, len_sum));
          decimal_t len_tmp =
              whole_lane_net_.lane_set.at(succ_id).length + len_expand;
          if (len_tmp < search_radius) {
            id_to_expand.insert(std::pair<decimal_t, int>(len_tmp, succ_id));
          }
        }
      }
    }

    // 后向展开——前驱车道（仅对左右相邻车道，不包括自车车道本身）
    auto it = std::find(mid_lane_ids.begin(), mid_lane_ids.end(), cur_lane_id);
    mid_lane_ids.erase(it);  // 移除自车车道（自车车道后方已通过 cur_arc_len 覆盖）
    decimal_t len_rear = -cur_arc_len;

    for (const auto &id : mid_lane_ids) {
      decimal_t len_sum = len_rear;
      if (fabs(len_sum) >= search_radius) continue;

      std::set<std::pair<decimal_t, int>> id_to_expand;
      id_to_expand.insert(std::pair<decimal_t, int>(len_sum, id));

      while (!id_to_expand.empty()) {
        auto it = id_to_expand.begin();
        decimal_t len_expand = it->first;
        int id_expand = it->second;
        id_to_expand.erase(it);

        // 展开父车道
        std::vector<int> pred_ids =
            whole_lane_net_.lane_set.at(id_expand).father_id;
        for (const auto &pred_id : pred_ids) {
          // 后向展开的长度为负（车道长度取负）
          decimal_t len_tmp =
              -whole_lane_net_.lane_set.at(pred_id).length + len_expand;
          key_lane_ids.insert(std::pair<int, decimal_t>(pred_id, len_tmp));
          if (fabs(len_tmp) < search_radius) {
            id_to_expand.insert(std::pair<decimal_t, int>(len_tmp, pred_id));
          }
        }
      }
    }
  }

  // ====== 阶段2：计算自适应裁剪范围并筛选关键车辆 ======
  decimal_t max_radius = 170.0;     // 最大前向距离（米）
  decimal_t min_radius = 30.0;      // 最小前向距离（米）
  decimal_t dec_comfort = 1.6;      // 舒适减速度（m/s^2）
  decimal_t t_pl = 5.0;             // 规划时域（秒）

  // 计算舒适减速时间 = max(规划时域, 当前速度/舒适减速度)
  decimal_t t_comfort =
      std::max(t_pl, ego_vehicle_.state().velocity / dec_comfort);

  // 规划视距 = 速度 * 舒适减速时间 + 100m安全余量
  decimal_t pl_horizon_length = ego_vehicle_.state().velocity * t_comfort + 100;

  // 自适应裁剪范围 = clamp(规划视距, 30, 170)
  decimal_t front_range = std::min(pl_horizon_length, max_radius);
  front_range = std::max(min_radius, front_range);

  // 筛选关键车辆
  // ~ 假设：所有附近/可达车道具有相似长度，且从相似弧长起始
  key_vehicle_ids_.clear();
  semantic_key_vehicles_.semantic_vehicles.clear();
  key_vehicles_.vehicles.clear();

  {
    for (const auto &v : semantic_surrounding_vehicles_.semantic_vehicles) {
      // 排除距离车道过远的车辆（不在任何车道上）
      if (v.second.dist_to_lane > max_distance_to_lane_) {
        continue;
      }

      int v_lane_id = v.second.nearest_lane_id;
      auto it = key_lane_ids.find(v_lane_id);

      if (it != key_lane_ids.end()) {
        decimal_t len_offset = it->second;  // 车道相对于自车的弧长偏移
        decimal_t dist = len_offset + v.second.arc_len_onlane;  // 车辆相对于自车的弧长距离

        // * 前方关键车辆筛选
        if (dist >= 0 && fabs(dist) < front_range) {
          int v_id = v.first;
          // 排除当前车道后方的已过车（arc_len 小于自车的）
          if (v.second.nearest_lane_id == cur_lane_id &&
              v.second.arc_len_onlane < cur_arc_len)
            continue;
          key_vehicle_ids_.push_back(v_id);
          semantic_key_vehicles_.semantic_vehicles.insert(
              std::pair<int, common::SemanticVehicle>(
                  v_id,
                  semantic_surrounding_vehicles_.semantic_vehicles.at(v_id)));
          key_vehicles_.vehicles.insert(std::pair<int, common::Vehicle>(
              v_id, semantic_surrounding_vehicles_.semantic_vehicles.at(v_id)
                        .vehicle));
        }

        // * 后方关键车辆筛选（需要考虑速度裕量）
        decimal_t s_margin =
            std::max(20.0, fabs(v.second.vehicle.state().velocity) * t_pl);
        if (dist < 0 && s_margin > fabs(dist)) {
          int v_id = v.first;
          // 排除当前车道后方已过车
          if (v.second.nearest_lane_id == cur_lane_id &&
              v.second.arc_len_onlane < cur_arc_len)
            continue;
          key_vehicle_ids_.push_back(v_id);
          semantic_key_vehicles_.semantic_vehicles.insert(
              std::pair<int, common::SemanticVehicle>(
                  v_id,
                  semantic_surrounding_vehicles_.semantic_vehicles.at(v_id)));
          key_vehicles_.vehicles.insert(std::pair<int, common::Vehicle>(
              v_id, semantic_surrounding_vehicles_.semantic_vehicles.at(v_id)
                        .vehicle));
        }
      }
    }
  }

  return kSuccess;
}

// ======================== 第7层：参考车道构建工具函数 ========================

/// @brief 获取给定状态在指定车道附近的局部车道采样点
///
/// 以 target_lane_id 为中心，沿车道拓扑前后扩展：
///   1. 后向扩展：沿 father_id 链收集车道段，直到累计距离 >= max_backward_dist
///   2. 前向扩展：沿 child_id 链收集车道段，直到累计距离 >= max_reflane_dist
///   3. 合并前后车道段的所有采样点，构建连续长车道
///   4. 裁剪到 [arclen - max_backward_dist, arclen + max_reflane_dist] 范围
///
/// @note 若存在 navi_path，则优先选择 navi_path 中的 father/child 作为扩展目标
ErrorType SemanticMapManager::GetLocalLaneSamplesByState(
    const common::State &state, const int lane_id,
    const std::vector<int> &navi_path, const decimal_t max_reflane_dist,
    const decimal_t max_backward_dist, vec_Vecf<2> *samples) const {
  if (semantic_lane_set_.semantic_lanes.count(lane_id) == 0) {
    printf("[GetLocalLaneSamplesByState]fail to get lane id %d.\n", lane_id);
    return kWrongStatus;
  }

  // 计算状态在目标车道上的弧长位置
  decimal_t arclen = 0.0;
  common::Lane target_lane = semantic_lane_set_.semantic_lanes.at(lane_id).lane;
  target_lane.GetArcLengthByVecPosition(state.vec_position, &arclen);

  // ====== 后向扩展 ======
  decimal_t accum_dist_backward = 0.0;
  std::vector<int> ids_back;
  {
    if (arclen < max_backward_dist) {
      // 目标车道剩余长度不足，需要向前驱车道扩展
      accum_dist_backward += arclen;
      int id_tmp = lane_id;
      while (accum_dist_backward < max_backward_dist) {
        std::vector<int> father_ids =
            semantic_lane_set_.semantic_lanes.at(id_tmp).father_id;
        if (!father_ids.empty()) {
          // 优先选择在导航路径中的前驱车道
          int father_id = father_ids.front();
          for (auto &id : father_ids) {
            if (std::find(navi_path.begin(), navi_path.end(), id) !=
                navi_path.end()) {
              father_id = id;
              break;
            }
          }
          accum_dist_backward +=
              semantic_lane_set_.semantic_lanes.at(father_id).length;
          ids_back.push_back(father_id);
          id_tmp = father_id;
        } else {
          break;  // 无前驱车道，停止扩展
        }
      }
    } else {
      accum_dist_backward = arclen;
    }
  }

  // 反转后向车道ID序列（使其从最远的父车道指向目标车道）
  std::reverse(ids_back.begin(), ids_back.end());

  // ====== 前向扩展 ======
  std::vector<int> ids_front;
  decimal_t accum_dist_forward = 0.0;
  {
    decimal_t dist_remain_target_lane =
        semantic_lane_set_.semantic_lanes.at(lane_id).length - arclen;

    if (dist_remain_target_lane < max_reflane_dist) {
      // 目标车道剩余长度不足，需要向子车道扩展
      int id_tmp = lane_id;
      ids_front.push_back(lane_id);
      accum_dist_forward += dist_remain_target_lane;

      while (accum_dist_forward < max_reflane_dist) {
        std::vector<int> child_ids =
            semantic_lane_set_.semantic_lanes.at(id_tmp).child_id;
        if (!child_ids.empty()) {
          // 优先选择在导航路径中的子车道
          int child_id = child_ids.front();
          for (auto &id : child_ids) {
            if (std::find(navi_path.begin(), navi_path.end(), id) !=
                navi_path.end()) {
              child_id = id;
              break;
            }
          }
          accum_dist_forward +=
              semantic_lane_set_.semantic_lanes.at(child_id).length;
          ids_front.push_back(child_id);
          id_tmp = child_id;
        } else {
          break;
        }
      }
    } else {
      ids_front.push_back(lane_id);
      accum_dist_forward = dist_remain_target_lane;
    }
  }

  // ====== 合并前后车道段，构建完整车道ID序列 ======
  std::vector<int> lane_id_all;
  lane_id_all.insert(lane_id_all.end(), ids_back.begin(), ids_back.end());
  lane_id_all.insert(lane_id_all.end(), ids_front.begin(), ids_front.end());

  // 提取所有车道段的采样点
  vec_Vecf<2> raw_samples;
  for (const auto &id : lane_id_all) {
    if (raw_samples.empty() &&
        (int)surrounding_lane_net_.lane_set.at(id).lane_points.size() > 0) {
      raw_samples.push_back(
          surrounding_lane_net_.lane_set.at(id).lane_points[0]);
    }
    for (int i = 1;
         i < (int)surrounding_lane_net_.lane_set.at(id).lane_points.size();
         ++i) {
      raw_samples.push_back(
          surrounding_lane_net_.lane_set.at(id).lane_points[i]);
    }
  }

  // 从采样点拟合连续长车道
  common::Lane long_lane;
  if (common::LaneGenerator::GetLaneBySamplePoints(raw_samples, &long_lane) !=
      kSuccess) {
    return kWrongStatus;
  }

  // 裁剪到有效范围并采样
  decimal_t acc_dist_tmp;
  decimal_t sample_start =
      std::max(0.0, accum_dist_backward - max_backward_dist);
  decimal_t forward_sample_len = std::min(max_reflane_dist, accum_dist_forward);
  SampleLane(long_lane, sample_start,
             sample_start + forward_sample_len +
                 std::min(accum_dist_backward, max_backward_dist),
             1.0, samples, &acc_dist_tmp);

  return kSuccess;
}

/// @brief 通过车道ID序列构建局部连续长车道
///
/// 算法：
///   1. 从 whole_lane_net_ 中提取所有车道段的采样点
///   2. 拟合为一条连续的 long_lane
///   3. 计算车辆状态的弧长位置
///   4. 裁剪到 [arc_len - backward_len, arc_len + forward_len] 范围
///
/// @param state 车辆状态
/// @param lane_ids 车道ID序列
/// @param forward_length 前向裁剪长度
/// @param backward_length 后向裁剪长度
/// @param is_high_quality 是否高质量样条拟合
/// @param lane [输出] 裁剪后的局部车道
ErrorType SemanticMapManager::GetLocalLaneUsingLaneIds(
    const common::State &state, const std::vector<int> &lane_ids,
    const decimal_t forward_length, const decimal_t backward_length,
    const bool &is_high_quality, common::Lane *lane) {
  // 提取所有车道段的采样点
  vec_Vecf<2> raw_samples;
  for (const auto &id : lane_ids) {
    if (raw_samples.empty() &&
        whole_lane_net_.lane_set.at(id).lane_points.size() > 0) {
      raw_samples.push_back(whole_lane_net_.lane_set.at(id).lane_points[0]);
    }
    for (int i = 1; i < whole_lane_net_.lane_set.at(id).lane_points.size();
         ++i) {
      raw_samples.push_back(whole_lane_net_.lane_set.at(id).lane_points[i]);
    }
  }

  // 拟合连续长车道
  common::Lane long_lane;
  if (common::LaneGenerator::GetLaneBySamplePoints(raw_samples, &long_lane) !=
      kSuccess) {
    return kWrongStatus;
  }

  // 获取弧长位置
  decimal_t arc_len;
  long_lane.GetArcLengthByVecPosition(state.vec_position, &arc_len);

  // 裁剪到目标范围
  vec_Vecf<2> samples;
  decimal_t acc_dist_tmp;
  decimal_t sample_start = std::max(0.0, arc_len - backward_length);
  decimal_t sample_end = std::min(arc_len + forward_length, long_lane.end());
  SampleLane(long_lane, sample_start, sample_end, 1.0, &samples, &acc_dist_tmp);

  if (kSuccess != GetLaneBySampledPoints(samples, is_high_quality, lane)) {
    return kWrongStatus;
  }

  return kSuccess;
}

/// @brief 根据给定横向行为获取对应的参考车道
///
/// 步骤：
///   1. GetNearestLaneIdUsingState：获取车辆当前最近车道
///   2. 检查距离是否合法（<= max_distance_to_lane_）
///   3. GetTargetLaneId：根据行为映射目标车道ID
///   4. 若启用快速LUT：直接在 segment_to_local_lut_ 中查表获取预构建的局部车道
///   5. 否则：通过 GetLocalLaneSamplesByState 实时采样并拟合
///
/// @note 当快速LUT有效时，取 segment_to_local_lut_ 的第一个候选局部车道
///        （一个段车道可能属于多个预构建局部车道，取第一个即可）
ErrorType SemanticMapManager::GetRefLaneForStateByBehavior(
    const common::State &state, const std::vector<int> &navi_path,
    const LateralBehavior &behavior, const decimal_t &max_forward_len,
    const decimal_t &max_back_len, const bool is_high_quality,
    common::Lane *lane) const {
  Vec3f state_3dof(state.vec_position(0), state.vec_position(1), state.angle);

  // 步骤1：获取最近车道
  int current_lane_id;
  decimal_t distance_to_lane;
  decimal_t arc_len;
  if (GetNearestLaneIdUsingState(state_3dof, navi_path, &current_lane_id,
                                 &distance_to_lane, &arc_len) != kSuccess) {
    printf("[GetRefLaneForStateByBehavior]Cannot get nearest lane.\n");
    return kWrongStatus;
  }

  // 步骤2：距离合法性检查
  if (distance_to_lane > max_distance_to_lane_) {
    return kWrongStatus;
  }

  // 步骤3：根据行为映射目标车道ID
  int target_lane_id;
  if (GetTargetLaneId(current_lane_id, behavior, &target_lane_id) != kSuccess) {
    return kWrongStatus;
  }

  // 步骤4：尝试使用快速LUT查表
  if (agent_config_info_.enable_fast_lane_lut && has_fast_lut_) {
    if (segment_to_local_lut_.end() !=
        segment_to_local_lut_.find(target_lane_id)) {
      // * 这里简单取第一个候选局部车道（多个候选应指向同一空间区域）
      int id = *segment_to_local_lut_.at(target_lane_id).begin();
      *lane = local_lanes_.at(id);
      return kSuccess;
    }
  }

  // 步骤5：快速LUT未命中或未启用——实时采样并拟合
  // ~ 参考车道长度应与最大速度和最大前向仿真时间匹配
  // ~ 当前设置对应 30m/s * 7.5s 前向距离
  vec_Vecf<2> samples;
  if (GetLocalLaneSamplesByState(state, target_lane_id, navi_path,
                                 max_forward_len, max_back_len,
                                 &samples) != kSuccess) {
    printf("[GetRefLaneForStateByBehavior]Cannot get local lane samples.\n");
    return kWrongStatus;
  }

  if (kSuccess != GetLaneBySampledPoints(samples, is_high_quality, lane)) {
    return kWrongStatus;
  }

  return kSuccess;
}

/// @brief 通过采样点拟合车道
///
/// 两种拟合模式：
///   - 高质量模式（is_high_quality=true）：
///     使用分段样条拟合（num_segments=20），带有正则化（regulator=1e6）
///     -> 适合于需要高精度车道表示的场景
///
///   - 标准模式（is_high_quality=false）：
///     使用默认的 GetLaneBySamplePoints 拟合
///     -> 速度快，适合临时造车道
ErrorType SemanticMapManager::GetLaneBySampledPoints(
    const vec_Vecf<2> &samples, const bool &is_high_quality,
    common::Lane *lane) const {
  if (is_high_quality) {
    // 高质量模式：分段样条拟合
    double d = 0.0;
    std::vector<decimal_t> para;
    para.push_back(d);

    // 构建累积弧长参数化
    int num_samples = static_cast<int>(samples.size());
    for (int i = 1; i < num_samples; i++) {
      double dx = samples[i](0) - samples[i - 1](0);
      double dy = samples[i](1) - samples[i - 1](1);
      d += std::hypot(dx, dy);
      para.push_back(d);
    }

    // 20段均匀分段
    const int num_segments = 20;
    Eigen::ArrayXf breaks =
        Eigen::ArrayXf::LinSpaced(num_segments, para.front(), para.back());

    // 正则化系数（防止过拟合）
    const decimal_t regulator = (double)1e6;
    if (common::LaneGenerator::GetLaneBySampleFitting(
            samples, para, breaks, regulator, lane) != kSuccess) {
      return kWrongStatus;
    }
  } else {
    // 标准模式：默认拟合
    if (common::LaneGenerator::GetLaneBySamplePoints(samples, lane) !=
        kSuccess) {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

/// @brief 在车道上均匀采样
///
/// @param lane 目标车道
/// @param s0 起始弧长
/// @param s1 终止弧长
/// @param step 采样步长（通常 1.0m）
/// @param samples [输出] 采样点向量
/// @param accum_dist [输出] 累计采样距离
ErrorType SemanticMapManager::SampleLane(const common::Lane &lane,
                                         const decimal_t &s0,
                                         const decimal_t &s1,
                                         const decimal_t &step,
                                         vec_E<Vecf<2>> *samples,
                                         decimal_t *accum_dist) const {
  Vecf<2> pt;
  for (decimal_t s = s0; s < s1; s += step) {
    lane.GetPositionByArcLength(s, &pt);
    samples->push_back(pt);
    (*accum_dist) += step;
  }
  return kSuccess;
}

/// @brief 根据当前车道ID和横向行为获取目标车道ID
///
/// 映射规则：
///   - kLaneKeeping / kUndefined -> 返回当前车道ID
///   - kLaneChangeLeft -> 若左换道可用，返回 l_lane_id；否则返回 kWrongStatus
///   - kLaneChangeRight -> 若右换道可用，返回 r_lane_id；否则返回 kWrongStatus
ErrorType SemanticMapManager::GetTargetLaneId(const int lane_id,
                                              const LateralBehavior &behavior,
                                              int *target_lane_id) const {
  auto it = semantic_lane_set_.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set_.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    if (behavior == common::LateralBehavior::kLaneKeeping ||
        behavior == common::LateralBehavior::kUndefined) {
      *target_lane_id = lane_id;
    } else if (behavior == common::LateralBehavior::kLaneChangeLeft) {
      if (it->second.l_change_avbl) {
        *target_lane_id = it->second.l_lane_id;
      } else {
        return kWrongStatus;
      }
    } else if (behavior == common::LateralBehavior::kLaneChangeRight) {
      if (it->second.r_change_avbl) {
        *target_lane_id = it->second.r_lane_id;
      } else {
        return kWrongStatus;
      }
    } else {
      assert(false);
    }
  }
  return kSuccess;
}

/// @brief 获取参考车道上给定状态的前方车辆（最近的前车）
///
/// 搜索算法：
///   1. 沿参考车道弧长方向，从 ref_state 位置开始向后（delta_s方向）步进搜索
///   2. 步长 = lat_range / 1.4（约1.57m，确保搜索密度不遗漏车辆）
///   3. 最大搜索距离 = 120m
///   4. 对每一步，检查 vehicle_set 中是否有车辆在该搜索点附近（lat_range 半径内）
///   5. 找到的第一辆车即为最近前车
///
/// @param ref_lane 参考车道
/// @param ref_state 参考状态
/// @param vehicle_set 候选车辆集合
/// @param lat_range 横向搜索半径（米）
/// @param leading_vehicle [输出] 前车
/// @param distance_residual_ratio [输出] 距离剩余比例 = (120 - delta_s) / 120
ErrorType SemanticMapManager::GetLeadingVehicleOnLane(
    const common::Lane &ref_lane, const common::State &ref_state,
    const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
    common::Vehicle *leading_vehicle,
    decimal_t *distance_residual_ratio) const {
  /**
   *    前车搜索示意图：
   *    [>>>>]    ------->    [>>>>]    (搜索方向: s增大)
   *      | offset               |
   *      |                      |
   *  ------------------------------------ref lane (参考车道)
   *      lane_pt                lane_pt+1
   */

  common::StateTransformer stf(ref_lane);
  common::FrenetState ref_fs;
  Vecf<2> lane_pt;

  // 将参考状态转换到车道Frenet坐标系
  if (stf.GetFrenetStateFromState(ref_state, &ref_fs) != kSuccess) {
    return kWrongStatus;
  }
  ref_lane.GetPositionByArcLength(ref_fs.vec_s[0], &lane_pt);

  const decimal_t lane_width = 3.5;
  const decimal_t search_lat_radius = lat_range;
  const decimal_t max_forward_search_dist = 120.0;  // 最大前向搜索距离
  decimal_t search_lon_offset = 0.0;
  decimal_t resolution = search_lat_radius / 1.4;  // 纵向搜索步长

  int leading_vehicle_id = kInvalidAgentId;
  bool find_leading_vehicle_in_set = false;

  // 沿参考车道弧长方向步进搜索（s从ref_fs.s开始递增）
  for (decimal_t s = ref_fs.vec_s[0] + resolution + search_lon_offset;
       s < ref_fs.vec_s[0] + max_forward_search_dist + search_lon_offset;
       s += resolution) {
    decimal_t delta_s = s - ref_fs.vec_s[0];  // 当前搜索点距离参考位置的前向距离
    ref_lane.GetPositionByArcLength(s, &lane_pt);

    // 检查车辆集合中是否有车辆在此搜索点附近
    for (const auto &entry : vehicle_set.vehicles) {
      if (entry.second.id() == kInvalidAgentId) continue;
      // 欧氏距离判定：车辆是否在搜索点 lat_range 半径内
      if ((lane_pt - entry.second.state().vec_position).squaredNorm() <
          search_lat_radius * search_lat_radius) {
        find_leading_vehicle_in_set = true;
        leading_vehicle_id = entry.first;
        // 距离剩余比例：距离参考位置越远，剩余比例越小
        *distance_residual_ratio =
            (max_forward_search_dist - delta_s) / max_forward_search_dist;
        break;
      }
    }

    if (find_leading_vehicle_in_set) break;
  }

  if (find_leading_vehicle_in_set) {
    auto it = vehicle_set.vehicles.find(leading_vehicle_id);
    *leading_vehicle = it->second;
  } else {
    return kWrongStatus;
  }
  return kSuccess;
}

/// @brief 获取参考车道上给定状态的后方车辆（最近的后车）
///
/// 搜索算法：
///   1. 从 ref_state 位置向后搜索（delta_s 递增但方向为 s 减小）
///   2. 搜索步长和横向半径与 GetLeadingVehicleOnLane 相同
///   3. 最大后向搜索距离 = min(ref_fs.s - lane.begin(), 100.0)
///      限制在车道起点和前100m范围内
///   4. 同样找到的第一辆车即为最近后车
///
/// @param ref_lane 参考车道
/// @param ref_state 参考状态
/// @param vehicle_set 候选车辆集合
/// @param lat_range 横向搜索半径（米）
/// @param following_vehicle [输出] 后车
ErrorType SemanticMapManager::GetFollowingVehicleOnLane(
    const common::Lane &ref_lane, const common::State &ref_state,
    const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
    common::Vehicle *following_vehicle) const {
  common::StateTransformer stf(ref_lane);
  common::FrenetState ref_fs;
  if (stf.GetFrenetStateFromState(ref_state, &ref_fs) != kSuccess) {
    return kWrongStatus;
  }

  const decimal_t lane_width_tol = lat_range;
  // 后向搜索最大距离 = min(到车道起点距离, 100m)
  decimal_t max_backward_search_dist =
      std::min(ref_fs.vec_s[0] - ref_lane.begin(), 100.0);

  int following_vehicle_id = kInvalidAgentId;
  Vecf<2> lane_pt;
  bool find_following_vehicle_in_set = false;

  // 沿参考车道弧长方向向后步进搜索（delta_s递增，s = ref_fs.s - delta_s）
  for (decimal_t delta_s = lane_width_tol / 1.4;
       delta_s < max_backward_search_dist - 2.0 * lane_width_tol;
       delta_s += lane_width_tol / 1.4) {
    ref_lane.GetPositionByArcLength(ref_fs.vec_s[0] - delta_s, &lane_pt);
    for (auto &entry : vehicle_set.vehicles) {
      if (entry.second.id() == kInvalidAgentId) continue;
      if ((lane_pt - entry.second.state().vec_position).norm() <
          lane_width_tol) {
        find_following_vehicle_in_set = true;
        following_vehicle_id = entry.first;
        break;
      }
    }

    if (find_following_vehicle_in_set) break;
  }

  if (find_following_vehicle_in_set) {
    auto it = vehicle_set.vehicles.find(following_vehicle_id);
    *following_vehicle = it->second;
  } else {
    return kWrongStatus;
  }
  return kSuccess;
}

/// @brief 获取限速值——委托给 TrafficSignalManager
ErrorType SemanticMapManager::GetSpeedLimit(const State &state,
                                            const Lane &lane,
                                            decimal_t *speed_limit) const {
  return traffic_singal_manager_.GetSpeedLimit(state, lane, speed_limit);
}

/// @brief 获取停车状态——委托给 TrafficSignalManager
ErrorType SemanticMapManager::GetTrafficStoppingState(
    const State &state, const Lane &lane, State *stopping_state) const {
  return traffic_singal_manager_.GetTrafficStoppingState(state, lane,
                                                         stopping_state);
}

/// @brief 判断局部拼接车道是否包含给定的段车道ID
///
/// 通过 local_to_segment_lut_ 查表获取局部车道所包含的段车道列表，
/// 然后检查目标 seg_lane_id 是否在此列表中。
///
/// @note 仅在快速LUT已构建时有效（has_fast_lut_ = true），否则返回false
bool SemanticMapManager::IsLocalLaneContainsLane(const int &local_lane_id,
                                                 const int &seg_lane_id) const {
  if (!has_fast_lut_) return false;
  auto ids = local_to_segment_lut_.at(local_lane_id);
  if (ids.end() != std::find(ids.begin(), ids.end(), seg_lane_id)) {
    return true;
  }
  return false;
}

/// @brief 计算车道网络上两点间的距离（Dijkstra-like图搜索，未完整实现）
///
/// TODO(lu.zhang): 建议未来改用通用图搜索算法
///
/// 算法的意图是：
///   1. 使用优先队列进行最小成本路径搜索
///   2. 后继节点包括：子车道（成本=车道长度）、左换道（成本=10.0）、右换道（成本=10.0）
///   3. 节点成本通过 pair<累计成本, 节点ID> 在优先队列中排序
///
/// @note 此函数未完成——while循环没有将后继节点插入优先队列，
///        也没有将最终距离写入 *dist
ErrorType SemanticMapManager::GetDistanceOnLaneNet(const int &lane_id_0,
                                                   const decimal_t &arc_len_0,
                                                   const int &lane_id_1,
                                                   const decimal_t &arc_len_1,
                                                   decimal_t *dist) const {
  std::unordered_set<int> visited_list;
  std::set<std::pair<decimal_t, int>> pq;  // 优先队列：pair<成本, 节点ID>
  pq.insert(std::pair<decimal_t, int>(0.0, lane_id_0));
  decimal_t cost_lane_change = 10.0;  // 换道成本（米）

  int tar_node = lane_id_1;

  while (!pq.empty()) {
    int cur_node = pq.begin()->second;

    if (cur_node == tar_node) {
      // 到达目标节点
      break;
    }
    visited_list.insert(cur_node);
    std::vector<std::pair<int, decimal_t>> succ_nodes;

    // 收集所有后继节点及对应成本
    {
      // 子车道（前向）：成本 = 车道长度
      if (!whole_lane_net_.lane_set.at(cur_node).child_id.empty()) {
        auto ids = whole_lane_net_.lane_set.at(cur_node).child_id;
        auto cost = whole_lane_net_.lane_set.at(cur_node).length;
        for (const auto &id : ids) {
          succ_nodes.push_back(std::pair<int, decimal_t>(id, cost));
        }
      }

      // 左换道：成本 = cost_lane_change (10m)
      if (whole_lane_net_.lane_set.at(cur_node).l_change_avbl) {
        auto id = whole_lane_net_.lane_set.at(cur_node).l_lane_id;
        auto cost = cost_lane_change;
        succ_nodes.push_back(std::pair<int, decimal_t>(id, cost));
      }

      // 右换道：成本 = cost_lane_change (10m)
      if (whole_lane_net_.lane_set.at(cur_node).r_change_avbl) {
        auto id = whole_lane_net_.lane_set.at(cur_node).r_lane_id;
        auto cost = cost_lane_change;
        succ_nodes.push_back(std::pair<int, decimal_t>(id, cost));
      }
    }

    // TODO: 未将后继节点插入优先队列（算法不完整）
  }

  return kSuccess;
}

}  // namespace semantic_map_manager
