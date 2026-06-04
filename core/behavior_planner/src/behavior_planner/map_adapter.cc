/**
 * @file map_adapter.cc
 * @brief 行为规划器地图适配器的实现
 *
 * 实现 BehaviorPlannerMapAdapter 类的全部方法，将 BehaviorPlannerMapItf
 * 接口调用转发到底层 SemanticMapManager，并附带必要的有效性检查。
 *
 * ## 设计要点
 *   1. 所有方法首先检查 is_valid_ 标志，未绑定有效地图时返回 kWrongStatus
 *   2. 车道拓扑查询（GetLeftLaneId/GetRightLaneId）额外检查变道可行性标志
 *      （l_change_avbl / r_change_avbl），防止规划器请求不存在的变道
 *   3. GetLaneByLaneId 和 GetRefLaneForStateByBehavior 额外验证返回的 Lane
 *      对象有效性，确保上游拿到的是可用的车道数据
 *   4. GetKeyVehicles/GetKeySemanticVehicles 目前直接返回全部周围车辆，
 *      留 TODO 供后续添加筛选策略
 *   5. GetPredictedBehavior 从 semantic_surrounding_vehicles 中查找预测行为
 *
 * ## 错误处理策略
 *   错误信息通过 printf 输出到控制台，同时返回 kWrongStatus。
 *   上游 BehaviorPlanner 在每个步骤检查返回值。
 */
#include "behavior_planner/map_adapter.h"

namespace planning {

// ============================================================================
// 有效性检查
// ============================================================================

/// 检查地图适配器是否已绑定有效数据源
bool BehaviorPlannerMapAdapter::IsValid() { return is_valid_; }

// ============================================================================
// 自车状态查询
// ============================================================================

/// 获取自车状态（位置 x, y, 航向角 angle, 曲率 curvature, 速度 velocity）
ErrorType BehaviorPlannerMapAdapter::GetEgoState(State *state) {
  if (!is_valid_) return kWrongStatus;
  *state = map_->ego_vehicle().state();
  return kSuccess;
}

/// 获取自车唯一标识符
ErrorType BehaviorPlannerMapAdapter::GetEgoId(int *id) {
  if (!is_valid_) return kWrongStatus;
  *id = map_->ego_id();
  return kSuccess;
}

/// 获取自车完整车辆对象（含运动状态和几何参数）
ErrorType BehaviorPlannerMapAdapter::GetEgoVehicle(common::Vehicle *vehicle) {
  if (!is_valid_) return kWrongStatus;
  *vehicle = map_->ego_vehicle();
  return kSuccess;
}

/**
 * @brief 根据自车位置确定当前所在车道 ID
 *
 * 算法步骤：
 *   1. 获取自车当前位置和航向角
 *   2. 调用 SemanticMapManager 的最邻近车道查询
 *   3. 返回与导航路径匹配的车道 ID
 *
 * @param navi_path 导航路径（车道 ID 序列）
 * @param[out] lane_id 自车当前所在的车道 ID
 */
ErrorType BehaviorPlannerMapAdapter::GetEgoLaneIdByPosition(
    const std::vector<int> &navi_path, int *lane_id) {
  if (!is_valid_) {
    printf("[GetEgoLaneIdByPosition]Interface not valid.\n");
    return kWrongStatus;
  }

  int ego_lane_id = kInvalidLaneId;
  decimal_t distance_to_lane;
  decimal_t arc_len;

  // 构造简化状态（x, y, 航向角），用于车道匹配
  Vec3f state_3dof(map_->ego_vehicle().state().vec_position(0),
                   map_->ego_vehicle().state().vec_position(1),
                   map_->ego_vehicle().state().angle);

  std::set<std::tuple<decimal_t, decimal_t, int>> dist_set;

  // 调用底层地图管理器查询最近车道
  if (map_->GetNearestLaneIdUsingState(state_3dof, navi_path, &ego_lane_id,
                                       &distance_to_lane,
                                       &arc_len) != kSuccess) {
    printf("[GetEgoLaneIdByPosition]Cannot get nearest lane.\n");
    return kWrongStatus;
  }

  *lane_id = ego_lane_id;
  return kSuccess;
}

// ============================================================================
// 最近车道查询
// ============================================================================

/// 根据给定状态（x, y, 航向角）查询最近车道 ID 及相关距离信息
ErrorType BehaviorPlannerMapAdapter::GetNearestLaneIdUsingState(
    const Vec3f &state, const std::vector<int> &navi_path, int *id,
    decimal_t *distance, decimal_t *arc_len) {
  if (!is_valid_) {
    printf("[GetNearestLaneIdUsingState]Interface not valid.\n");
    return kWrongStatus;
  }
  std::set<std::tuple<decimal_t, decimal_t, int>> dist_set;
  if (map_->GetNearestLaneIdUsingState(state, navi_path, id, distance,
                                       arc_len) != kSuccess) {
    printf("[GetNearestLaneIdUsingState]Cannot get nearest lane.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

// ============================================================================
// 拓扑可达性检查
// ============================================================================

/// 检查目标车道是否可从当前车道通过变道操作到达
ErrorType BehaviorPlannerMapAdapter::IsTopologicallyReachable(
    const int lane_id, const std::vector<int> &path, int *num_lane_changes,
    bool *res) {
  if (!is_valid_) {
    printf("[GetNearestLaneIdUsingState]Interface not valid.\n");
    return kWrongStatus;
  }
  if (map_->IsTopologicallyReachable(lane_id, path, num_lane_changes, res) !=
      kSuccess) {
    printf("[GetNearestLaneIdUsingState]Cannot get nearest lane.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

// ============================================================================
// 车道拓扑查询
// ============================================================================

/**
 * @brief 获取右侧相邻车道 ID
 *
 * 从语义车道集合中查找源车道的右邻居。
 * 额外检查 r_change_avbl 标志，仅当允许右变道时才返回有效 ID。
 *
 * @param lane_id 源车道 ID
 * @param[out] r_lane_id 右侧车道 ID
 */
ErrorType BehaviorPlannerMapAdapter::GetRightLaneId(const int lane_id,
                                                    int *r_lane_id) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    // 检查是否允许右变道（r_change_avbl 标志由地图或交通规则决定）
    if (it->second.r_change_avbl) {
      *r_lane_id = it->second.r_lane_id;
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

/**
 * @brief 获取左侧相邻车道 ID
 *
 * 从语义车道集合中查找源车道的左邻居。
 * 额外检查 l_change_avbl 标志，仅当允许左变道时才返回有效 ID。
 *
 * @param lane_id 源车道 ID
 * @param[out] l_lane_id 左侧车道 ID
 */
ErrorType BehaviorPlannerMapAdapter::GetLeftLaneId(const int lane_id,
                                                   int *l_lane_id) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    // 检查是否允许左变道（l_change_avbl 标志）
    if (it->second.l_change_avbl) {
      *l_lane_id = it->second.l_lane_id;
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

// ============================================================================
// 车道几何查询
// ============================================================================

/**
 * @brief 根据车道 ID 获取 Lane 对象
 *
 * 从语义车道集合中查找指定车道，返回其几何描述（中心线等）。
 * 额外检查返回的 Lane 对象是否有效。
 */
ErrorType BehaviorPlannerMapAdapter::GetLaneByLaneId(const int lane_id,
                                                     Lane *lane) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    *lane = it->second.lane;
    // 验证车道几何数据的有效性
    if (!lane->IsValid()) {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

/// 获取下游后继车道 ID 列表（子车道），用于车道保持的候选车道枚举
ErrorType BehaviorPlannerMapAdapter::GetChildLaneIds(
    const int lane_id, std::vector<int> *child_ids) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    // 将子车道 ID 集合复制到输出列表
    child_ids->assign(it->second.child_id.begin(), it->second.child_id.end());
  }
  return kSuccess;
}

/// 获取上游前驱车道 ID 列表（父车道）
ErrorType BehaviorPlannerMapAdapter::GetFatherLaneIds(
    const int lane_id, std::vector<int> *father_ids) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    father_ids->assign(it->second.father_id.begin(),
                       it->second.father_id.end());
  }
  return kSuccess;
}

/// 获取车道在指定状态附近的局部采样点序列
ErrorType BehaviorPlannerMapAdapter::GetLocalLaneSamplesByState(
    const State &state, const int lane_id, const std::vector<int> &navi_path,
    const decimal_t max_reflane_dist, const decimal_t max_backward_dist,
    vec_Vecf<2> *samples) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetLocalLaneSamplesByState(state, lane_id, navi_path,
                                       max_reflane_dist, max_backward_dist,
                                       samples) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

/// 根据自车状态和目标行为构建参考车道（附带有效性验证）
ErrorType BehaviorPlannerMapAdapter::GetRefLaneForStateByBehavior(
    const State &state, const std::vector<int> &navi_path,
    const LateralBehavior &behavior, const decimal_t &max_forward_len,
    const decimal_t &max_back_len, const bool is_high_quality, Lane *lane) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetRefLaneForStateByBehavior(state, navi_path, behavior,
                                         max_forward_len, max_back_len,
                                         is_high_quality, lane) != kSuccess) {
    return kWrongStatus;
  }
  // 额外验证：确保返回的车道几何数据是有效的
  if (!lane->IsValid()) {
    return kWrongStatus;
  }
  return kSuccess;
}

// ============================================================================
// IDM 前车查询
// ============================================================================

/// 获取指定车道参考状态前方最近的车辆（用于 IDM 跟驰模型的前车输入）
ErrorType BehaviorPlannerMapAdapter::GetLeadingVehicleOnLane(
    const common::Lane &ref_lane, const common::State &ref_state,
    const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
    common::Vehicle *leading_vehicle, decimal_t *distance_residual_ratio) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetLeadingVehicleOnLane(ref_lane, ref_state, vehicle_set, lat_range,
                                    leading_vehicle,
                                    distance_residual_ratio) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

// ============================================================================
// 关键车辆获取
// ============================================================================

/**
 * @brief 获取关键车辆集合
 *
 * 当前实现：直接返回所有周围车辆。
 * TODO: 后续可添加车辆选择策略，根据距离、相关性等筛选最有影响力的车辆。
 */
ErrorType BehaviorPlannerMapAdapter::GetKeyVehicles(
    common::VehicleSet *key_vehicle_set) {
  if (!is_valid_) return kWrongStatus;
  // TODO: (@denny.ding) add vehicle selection strategy here
  *key_vehicle_set = map_->surrounding_vehicles();
  return kSuccess;
}

/**
 * @brief 获取带语义信息的关键车辆集合
 *
 * 与 GetKeyVehicles 的区别：返回 SemanticVehicleSet，包含每辆车的
 * 预测横向行为和参考车道信息，用于多智能体前向仿真。
 * 当前实现：直接返回全部语义关键车辆。
 * TODO: 后续可添加车辆选择策略。
 */
ErrorType BehaviorPlannerMapAdapter::GetKeySemanticVehicles(
    common::SemanticVehicleSet *key_vehicle_set) {
  if (!is_valid_) return kWrongStatus;
  // TODO: (@denny.ding) add vehicle selection strategy here
  *key_vehicle_set = map_->semantic_key_vehicles();
  return kSuccess;
}

// ============================================================================
// 车道网络与碰撞检测
// ============================================================================

/// 获取完整车道网络
ErrorType BehaviorPlannerMapAdapter::GetWholeLaneNet(
    common::LaneNet *lane_net) {
  if (!is_valid_) return kWrongStatus;
  *lane_net = map_->whole_lane_net();
  return kSuccess;
}

/// 检查两车在给定状态下是否发生碰撞
ErrorType BehaviorPlannerMapAdapter::CheckCollisionUsingState(
    const common::VehicleParam &param_a, const common::State &state_a,
    const common::VehicleParam &param_b, const common::State &state_b,
    bool *res) {
  if (!is_valid_) return kWrongStatus;
  if (map_->CheckCollisionUsingState(param_a, state_a, param_b, state_b, res) !=
      kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

/// 检查给定车辆是否与任何障碍物发生碰撞
ErrorType BehaviorPlannerMapAdapter::CheckIfCollision(
    const common::VehicleParam &vehicle_param, const State &state, bool *res) {
  if (!is_valid_) return kWrongStatus;
  map_->CheckCollisionUsingStateAndVehicleParam(vehicle_param, state, res);
  return kSuccess;
}

// ============================================================================
// 限速与行为预测
// ============================================================================

/// 获取指定位置的限速值
ErrorType BehaviorPlannerMapAdapter::GetSpeedLimit(const State &state,
                                                   const Lane &lane,
                                                   decimal_t *speed_limit) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetSpeedLimit(state, lane, speed_limit) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

/**
 * @brief 获取指定车辆的预测横向行为
 *
 * 从 semantic_surrounding_vehicles 中根据 vehicle_id 查找对应车辆的
 * 预测横向行为（lat_behavior），用于 MPDM 前向仿真中的其他交通参与者建模。
 *
 * @param vehicle_id 目标车辆 ID
 * @param[out] lat_behavior 预测的横向行为
 */
ErrorType BehaviorPlannerMapAdapter::GetPredictedBehavior(
    const int vehicle_id, common::LateralBehavior *lat_behavior) {
  if (!is_valid_) return kWrongStatus;
  if (vehicle_id == kInvalidAgentId) return kWrongStatus;
  auto semantic_vehicle_set = map_->semantic_surrounding_vehicles();
  *lat_behavior =
      semantic_vehicle_set.semantic_vehicles.at(vehicle_id).lat_behavior;
  return kSuccess;
}

// ============================================================================
// 数据注入
// ============================================================================

/**
 * @brief 注入最新地图数据源
 *
 * 由 BehaviorPlannerServer::PlanCycleCallback 在每次规划循环中调用，
 * 将最新的语义地图共享指针存入 adapter 内部。
 *
 * @param map_ptr SemanticMapManager 的共享指针
 */
void BehaviorPlannerMapAdapter::set_map(
    std::shared_ptr<IntegratedMap> map_ptr) {
  map_ = map_ptr;
  is_valid_ = true;
}

}  // namespace planning
