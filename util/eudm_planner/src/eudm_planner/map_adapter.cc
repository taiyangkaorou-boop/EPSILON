/**
 * @file map_adapter.cc
 * @author EPSILON Autonomous Driving Team
 * @brief 地图适配器实现——将SemanticMapManager适配为EudmPlannerMapItf
 * @version 0.1
 * @date 2019-07-07
 * @copyright Copyright (c) 2019
 *
 * 本文件实现了EudmPlannerMapAdapter类，该类是适配器模式中
 * 的"适配器"角色：将ROS语义地图管理器（SemanticMapManager）
 * 适配为EUDM规划器所需的标准地图接口（EudmPlannerMapItf）。
 *
 * ## 实现模式
 *
 * 大多数方法采用简单的委托模式：
 * 1. 检查is_valid_（确保地图已加载）
 * 2. 将调用委托给map_的对应方法
 * 3. 检查返回值
 * 4. 返回结果
 *
 * 特殊方法：
 * - GetEgoLaneIdByPosition: 构造state_3dof后委托
 * - IsLaneConsistent: 使用BFS搜索替代简单比较
 * - GetLeftLaneId/GetRightLaneId: 附加变更可用性检查
 * - GetChildLaneIds: 将set数据assign到vector输出
 */

#include "eudm_planner/map_adapter.h"

namespace planning {

/// @brief 检查地图适配器是否有效
bool EudmPlannerMapAdapter::IsValid() { return is_valid_; }

/// @brief 获取自车状态——直接委托给map_->ego_vehicle().state()
ErrorType EudmPlannerMapAdapter::GetEgoState(State *state) {
  if (!is_valid_) return kWrongStatus;
  *state = map_->ego_vehicle().state();
  return kSuccess;
}

/// @brief 获取自车ID
ErrorType EudmPlannerMapAdapter::GetEgoId(int *id) {
  if (!is_valid_) return kWrongStatus;
  *id = map_->ego_id();
  return kSuccess;
}

/// @brief 获取自车完整车辆对象
ErrorType EudmPlannerMapAdapter::GetEgoVehicle(common::Vehicle *vehicle) {
  if (!is_valid_) return kWrongStatus;
  *vehicle = map_->ego_vehicle();
  return kSuccess;
}

/// @brief 根据位置获取自车所在车道ID
///
/// 构造3自由度状态（x, y, theta），然后委托给SemanticMapManager的
/// GetNearestLaneIdUsingState方法查询最近车道ID。
ErrorType EudmPlannerMapAdapter::GetEgoLaneIdByPosition(
    const std::vector<int> &navi_path, int *lane_id) {
  if (!is_valid_) {
    printf("[GetEgoLaneIdByPosition]Interface not valid.\n");
    return kWrongStatus;
  }

  int ego_lane_id = kInvalidLaneId;
  decimal_t distance_to_lane;
  decimal_t arc_len;

  // 构造3自由度状态：位置(x, y) + 朝向(theta)
  Vec3f state_3dof(map_->ego_vehicle().state().vec_position(0),
                   map_->ego_vehicle().state().vec_position(1),
                   map_->ego_vehicle().state().angle);

  // 委托给地图管理器查询最近车道
  if (map_->GetNearestLaneIdUsingState(state_3dof, navi_path, &ego_lane_id,
                                       &distance_to_lane,
                                       &arc_len) != kSuccess) {
    printf("[GetEgoLaneIdByPosition]Cannot get nearest lane.\n");
    return kWrongStatus;
  }

  *lane_id = ego_lane_id;
  return kSuccess;
}

/// @brief 查询距离指定状态最近的车道ID
ErrorType EudmPlannerMapAdapter::GetNearestLaneIdUsingState(
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

/// @brief 检查车道拓扑可达性——委托给底层地图管理器
ErrorType EudmPlannerMapAdapter::IsTopologicallyReachable(
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

/// @brief 获取右侧相邻车道ID
///
/// 从语义车道集合中查找指定车道，检查右侧变道是否可用（r_change_avbl），
/// 若可用则返回右侧车道ID。
ErrorType EudmPlannerMapAdapter::GetRightLaneId(const int lane_id,
                                                int *r_lane_id) {
  if (!is_valid_) return kWrongStatus;
  auto p_semantic_lane_set = map_->semantic_lane_set_cptr();
  auto it = p_semantic_lane_set->semantic_lanes.find(lane_id);
  if (it == p_semantic_lane_set->semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    if (it->second.r_change_avbl) {
      // 右侧变道可用
      *r_lane_id = it->second.r_lane_id;
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

/// @brief 获取左侧相邻车道ID
ErrorType EudmPlannerMapAdapter::GetLeftLaneId(const int lane_id,
                                               int *l_lane_id) {
  if (!is_valid_) return kWrongStatus;
  auto p_semantic_lane_set = map_->semantic_lane_set_cptr();
  auto it = p_semantic_lane_set->semantic_lanes.find(lane_id);
  if (it == p_semantic_lane_set->semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    if (it->second.l_change_avbl) {
      // 左侧变道可用
      *l_lane_id = it->second.l_lane_id;
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

/// @brief 通过车道ID获取车道对象
///
/// 从语义车道集合中查找并返回完整的车道几何信息。
/// 附带有效性检查：若车道对象无效则返回错误。
ErrorType EudmPlannerMapAdapter::GetLaneByLaneId(const int lane_id,
                                                 Lane *lane) {
  if (!is_valid_) return kWrongStatus;
  auto p_semantic_lane_set = map_->semantic_lane_set_cptr();
  auto it = p_semantic_lane_set->semantic_lanes.find(lane_id);
  if (it == p_semantic_lane_set->semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    *lane = it->second.lane;
    if (!lane->IsValid()) {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

/// @brief 获取后继车道ID列表（子车道）
///
/// 使用assign进行深拷贝，将set数据转换为vector输出。
ErrorType EudmPlannerMapAdapter::GetChildLaneIds(const int lane_id,
                                                 std::vector<int> *child_ids) {
  if (!is_valid_) return kWrongStatus;
  auto p_semantic_lane_set = map_->semantic_lane_set_cptr();
  auto it = p_semantic_lane_set->semantic_lanes.find(lane_id);
  if (it == p_semantic_lane_set->semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    // 注意：这里是assign（深拷贝），将child_id容器中的内容复制到child_ids
    child_ids->assign(it->second.child_id.begin(), it->second.child_id.end());
  }
  return kSuccess;
}

/// @brief 检查新旧车道ID是否拓扑一致
///
/// 通过BFS（广度优先搜索）在旧车道的后继车道集合中查找新车道。
/// 如果新车道ID在旧车道后继车道的前20个节点中找到，则认为一致。
///
/// 这是判断变道操作是否成功完成的关键逻辑：
/// - 如果新车道ID在旧车道的前驱-后继拓扑链上，说明变道已经完成
///
/// @param lane_id_old 旧车道ID
/// @param lane_id_new 新车道ID
/// @return true表示新车道在旧车道的后继拓扑中（变道完成）
bool EudmPlannerMapAdapter::IsLaneConsistent(const int lane_id_old,
                                             const int lane_id_new) {
  // 相同ID直接返回一致
  if (lane_id_new == lane_id_old) return true;

  auto p_semantic_lane_set = map_->semantic_lane_set_cptr();
  const int max_nodes_expanded = 20;  // 最大搜索节点数，防止性能问题
  int expanded_nodes = 1;
  std::set<int> visited_set;          // 已访问节点集合
  std::list<int> queue;               // BFS队列

  visited_set.insert(lane_id_old);
  queue.push_back(lane_id_old);

  // BFS搜索
  int cur_id;
  while (!queue.empty() && expanded_nodes < max_nodes_expanded) {
    cur_id = queue.front();
    queue.pop_front();
    expanded_nodes++;

    // 获取当前节点的所有后继车道ID
    std::vector<int> child_ids;
    auto it = p_semantic_lane_set->semantic_lanes.find(cur_id);
    if (it == p_semantic_lane_set->semantic_lanes.end()) {
      continue;
    } else {
      child_ids = it->second.child_id;
    }
    if (child_ids.empty()) continue;

    // 将所有未访问的后继节点加入队列
    for (auto &id : child_ids) {
      if (visited_set.count(id) == 0) {
        visited_set.insert(id);
        queue.push_back(id);
      }
    }
  }

  // 在visited_set中查找新车道ID
  if (visited_set.find(lane_id_new) != visited_set.end()) return true;
  return false;
}

/// @brief 获取前驱车道ID列表（父车道）
ErrorType EudmPlannerMapAdapter::GetFatherLaneIds(
    const int lane_id, std::vector<int> *father_ids) {
  if (!is_valid_) return kWrongStatus;
  auto p_semantic_lane_set = map_->semantic_lane_set_cptr();
  auto it = p_semantic_lane_set->semantic_lanes.find(lane_id);
  if (it == p_semantic_lane_set->semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    father_ids->assign(it->second.father_id.begin(),
                       it->second.father_id.end());
  }
  return kSuccess;
}

/// @brief 获取局部车道采样点——委托给底层地图管理器
ErrorType EudmPlannerMapAdapter::GetLocalLaneSamplesByState(
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

/// @brief 根据状态和行为获取参考车道
///
/// 额外检查返回的车道是否有效（lane->IsValid()），
/// 无效时返回错误。
ErrorType EudmPlannerMapAdapter::GetRefLaneForStateByBehavior(
    const State &state, const std::vector<int> &navi_path,
    const LateralBehavior &behavior, const decimal_t &max_forward_len,
    const decimal_t &max_back_len, const bool is_high_quality, Lane *lane) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetRefLaneForStateByBehavior(state, navi_path, behavior,
                                         max_forward_len, max_back_len,
                                         is_high_quality, lane) != kSuccess) {
    return kWrongStatus;
  }
  // 额外有效性检查
  if (!lane->IsValid()) {
    return kWrongStatus;
  }
  return kSuccess;
}

/// @brief 获取参考车道上前方最近的车辆
///
/// 用于IDM前向仿真：查找自车前方的车辆作为IDM模型的前车输入。
ErrorType EudmPlannerMapAdapter::GetLeadingVehicleOnLane(
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

/// @brief 获取参考车道上前后车辆的Frenet状态
///
/// 返回前后车辆在Frenet坐标系下的状态（s, d坐标及导数），
/// 用于变道间隙评估和RSS安全检查。
ErrorType
EudmPlannerMapAdapter::GetLeadingAndFollowingVehiclesFrenetStateOnLane(
    const common::Lane &ref_lane, const common::State &ref_state,
    const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
    common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
    bool *has_following_vehicle, common::Vehicle *following_vehicle,
    common::FrenetState *following_fs) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetLeadingAndFollowingVehiclesFrenetStateOnLane(
          ref_lane, ref_state, vehicle_set, has_leading_vehicle,
          leading_vehicle, leading_fs, has_following_vehicle, following_vehicle,
          following_fs) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

/// @brief 获取周围车辆集合
ErrorType EudmPlannerMapAdapter::GetSurroundingVehicles(
    common::VehicleSet *surrounding_vehicle_set) {
  if (!is_valid_) return kWrongStatus;
  *surrounding_vehicle_set = map_->surrounding_vehicles();
  return kSuccess;
}

/// @brief 获取关键车辆集合
ErrorType EudmPlannerMapAdapter::GetKeyVehicles(
    common::VehicleSet *key_vehicle_set) {
  if (!is_valid_) return kWrongStatus;
  *key_vehicle_set = map_->key_vehicles();
  return kSuccess;
}

/// @brief 获取带语义信息的关键车辆
///
/// 语义车辆含有关键的行为概率分布信息，
/// 是EUDM建模不确定性的核心数据来源。
ErrorType EudmPlannerMapAdapter::GetKeySemanticVehicles(
    common::SemanticVehicleSet *key_vehicle_set) {
  if (!is_valid_) return kWrongStatus;
  *key_vehicle_set = map_->semantic_key_vehicles();
  return kSuccess;
}

/// @brief 获取完整车道网络
ErrorType EudmPlannerMapAdapter::GetWholeLaneNet(common::LaneNet *lane_net) {
  if (!is_valid_) return kWrongStatus;
  *lane_net = map_->whole_lane_net();
  return kSuccess;
}

/// @brief 基于状态的碰撞检测——委托给底层地图管理器
ErrorType EudmPlannerMapAdapter::CheckCollisionUsingState(
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

/// @brief 设置地图数据指针（依赖注入）
///
/// 每个规划周期由EudmManager调用，注入最新的SemanticMapManager。
/// 设置后is_valid_变为true，其他方法才能正常使用。
void EudmPlannerMapAdapter::set_map(std::shared_ptr<IntegratedMap> map_ptr) {
  map_ = map_ptr;
  is_valid_ = true;
}

}  // namespace planning
