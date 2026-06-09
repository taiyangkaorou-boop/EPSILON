/**
 * @file map_adpater.cc
 * @author HKUST Aerial Robotics Group
 * @brief 地图适配器实现 —— 将 SemanticMapManager 适配为 SscPlannerMapItf 接口
 *
 * [概述]
 * SscPlannerAdapter 是一个薄封装适配器，所有接口方法均简单委托给
 * 内部的 SemanticMapManager 共享指针。其主要职责是:
 *   - 持有环境的快照 (通过 set_map 注入)
 *   - 将 SemanticMapManager 的具体接口映射为 SscPlannerMapItf 的统一接口
 *   - 在每次查询前检查 is_valid_ 标志以确保数据可用
 *
 * [数据流]
 *   SscPlannerServer::PlanCycleCallback
 *     → SscPlannerAdapter::set_map(smm_shared_ptr)  [保存快照]
 *   SscPlanner::RunOnce
 *     → SscPlannerAdapter::GetEgoVehicle(...)       [查询快照]
 *     → SscPlannerAdapter::GetForwardTrajectories(...)
 *     → ...
 *
 * [线程安全]
 *   set_map() 和所有 Get* 方法在同一条主规划线程中调用, 不需要额外同步。
 *
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#include "ssc_planner/map_adapter.h"

#include <algorithm>

namespace planning {

/// @brief 设置/更新底层地图数据
/// @param map SemanticMapManager 的共享指针 (一个完整的环境快照)
ErrorType SscPlannerAdapter::set_map(std::shared_ptr<IntegratedMap> map) {
  map_ = map;  // 持有环境快照
  is_valid_ = true;
  return kSuccess;
}

/// @brief 检查地图是否有效 (是否已调用 set_map)
bool SscPlannerAdapter::IsValid() { return is_valid_; }

/// @brief 获取地图快照的时间戳
decimal_t SscPlannerAdapter::GetTimeStamp() { return map_->time_stamp(); }

/// @brief 获取自车完整信息 (状态 + 车辆物理参数)
ErrorType SscPlannerAdapter::GetEgoVehicle(Vehicle* vehicle) {
  if (!is_valid_) return kWrongStatus;
  *vehicle = map_->ego_vehicle();
  return kSuccess;
}

/// @brief 获取自车状态 (仅状态, 不含参数)
ErrorType SscPlannerAdapter::GetEgoState(State* state) {
  if (!is_valid_) return kWrongStatus;
  *state = map_->ego_vehicle().state();
  return kSuccess;
}

/// @brief 获取局部参考车道 (委托到自车参考车道)
///
/// SSC 规划器需要一条参考车道来建立 StateTransformer (笛卡尔 <-> Frenet)。
/// 适配器从 ego_behavior().ref_lane 中获取该车道。
ErrorType SscPlannerAdapter::GetLocalReferenceLane(Lane* lane) {
  if (!is_valid_) return kWrongStatus;
  auto ref_lane = map_->ego_behavior().ref_lane;
  if (!ref_lane.IsValid()) {
    printf("[GetEgoReferenceLane]No reference lane existing.\n");
    return kWrongStatus;
  }
  *lane = ref_lane;
  return kSuccess;
}

/// @brief 获取自车前向仿真轨迹 (不含周围车辆)
ErrorType SscPlannerAdapter::GetForwardTrajectories(
    std::vector<LateralBehavior>* behaviors,
    vec_E<vec_E<common::Vehicle>>* trajs) {
  if (!is_valid_) return kWrongStatus;
  if (map_->ego_behavior().forward_behaviors.size() < 1) return kWrongStatus;
  *behaviors = map_->ego_behavior().forward_behaviors;
  *trajs = map_->ego_behavior().forward_trajs;
  return kSuccess;
}

/// @brief 获取自车前向仿真轨迹 + 周围车辆预测轨迹
///
/// 这两个数据结构一同打包传递给 SscPlanner::StateTransformForInputData,
/// 以进行批量坐标变换。sur_trajs 的结构为:
///   [behavior_idx][vehicle_id] = list<Vehicle_trajectory>
ErrorType SscPlannerAdapter::GetForwardTrajectories(
    std::vector<LateralBehavior>* behaviors,
    vec_E<vec_E<common::Vehicle>>* trajs,
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>* sur_trajs) {
  if (!is_valid_) return kWrongStatus;
  if (map_->ego_behavior().forward_behaviors.size() < 1) return kWrongStatus;
  *behaviors = map_->ego_behavior().forward_behaviors;
  *trajs = map_->ego_behavior().forward_trajs;
  *sur_trajs = map_->ego_behavior().surround_trajs;
  return kSuccess;
}

/// @brief 获取周围车辆当前确定性预测轨迹的存在概率
///
/// MVP-2 暂不生成 LK/LCL/LCR 多模态周车轨迹，因此这里只读取
/// SemanticVehicle 中已经存在的 argmax 横向行为概率，用作当前单条
/// 周车预测轨迹写入 risk grid 的权重。
ErrorType SscPlannerAdapter::GetSurroundingTrajectoryExistenceProbabilities(
    std::unordered_map<int, decimal_t>* traj_probs) {
  if (!is_valid_ || traj_probs == nullptr) return kWrongStatus;

  traj_probs->clear();
  const auto semantic_vehicle_set = map_->semantic_surrounding_vehicles();
  for (const auto& entry : semantic_vehicle_set.semantic_vehicles) {
    const int vehicle_id = entry.first;
    const auto& semantic_vehicle = entry.second;
    decimal_t existence_prob = 1.0;

    const auto& probs = semantic_vehicle.probs_lat_behaviors;
    if (probs.is_valid &&
        semantic_vehicle.lat_behavior != common::LateralBehavior::kUndefined) {
      const auto prob_it = probs.probs.find(semantic_vehicle.lat_behavior);
      if (prob_it != probs.probs.end()) {
        existence_prob = std::max<decimal_t>(
            0.0, std::min<decimal_t>(1.0, prob_it->second));
      }
    }

    traj_probs->insert({vehicle_id, existence_prob});
  }
  return kSuccess;
}

/// @brief 获取周围车辆多模态预测轨迹，供 SSC risk grid 使用
///
/// MVP-3 仅生成 risk grid 旁路输入：对每辆语义周车，读取 LK/LCL/LCR
/// 的行为概率；概率大于阈值且参考车道可构建时，调用语义地图已有开环
/// 预测器生成一条 Vehicle 轨迹。原始 deterministic surround_trajs 不变。
ErrorType SscPlannerAdapter::GetMultiModalSurroundingTrajectories(
    MultiModalSurroundingTrajectories* multimodal_trajs) {
  if (!is_valid_ || multimodal_trajs == nullptr) return kWrongStatus;

  multimodal_trajs->clear();
  const auto semantic_vehicle_set = map_->semantic_surrounding_vehicles();
  const std::vector<common::LateralBehavior> candidate_behaviors = {
      common::LateralBehavior::kLaneKeeping,
      common::LateralBehavior::kLaneChangeLeft,
      common::LateralBehavior::kLaneChangeRight};

  constexpr decimal_t kMinModeProbability = 1.0e-6;
  constexpr decimal_t kPredictionTime = 5.0;
  constexpr decimal_t kPredictionStep = 0.2;
  constexpr decimal_t kBackwardLaneLength = 10.0;

  for (const auto& entry : semantic_vehicle_set.semantic_vehicles) {
    const int vehicle_id = entry.first;
    const auto& semantic_vehicle = entry.second;
    const auto& probs = semantic_vehicle.probs_lat_behaviors;

    if (!probs.is_valid) continue;

    vec_E<SurroundingVehicleTrajectoryMode> modes;
    for (const auto& behavior : candidate_behaviors) {
      const auto prob_it = probs.probs.find(behavior);
      if (prob_it == probs.probs.end()) continue;

      const decimal_t probability = std::max<decimal_t>(
          0.0, std::min<decimal_t>(1.0, prob_it->second));
      if (probability <= kMinModeProbability) continue;

      common::Lane ref_lane;
      const decimal_t forward_lane_len =
          std::max(semantic_vehicle.vehicle.state().velocity * kPredictionTime,
                   50.0);
      if (map_->GetRefLaneForStateByBehavior(
              semantic_vehicle.vehicle.state(), std::vector<int>(), behavior,
              forward_lane_len, kBackwardLaneLength, false, &ref_lane) !=
          kSuccess) {
        continue;
      }

      vec_E<common::State> pred_states;
      if (map_->TrajectoryPredictionForVehicle(semantic_vehicle.vehicle,
                                               ref_lane, kPredictionTime,
                                               kPredictionStep,
                                               &pred_states) != kSuccess) {
        continue;
      }
      if (pred_states.empty()) continue;

      SurroundingVehicleTrajectoryMode mode;
      mode.vehicle_id = vehicle_id;
      mode.lat_behavior = behavior;
      mode.probability = probability;
      for (const auto& state : pred_states) {
        common::Vehicle vehicle = semantic_vehicle.vehicle;
        vehicle.set_state(state);
        mode.traj.emplace_back(vehicle);
      }
      modes.emplace_back(mode);
    }

    if (!modes.empty()) {
      multimodal_trajs->insert({vehicle_id, modes});
    }
  }
  return kSuccess;
}

/// @brief 获取自车当前离散横向行为
///
/// 行为来源于 EUDM (紧急-效用决策模型) 或其他行为规划器的输出,
/// 可能的值为: kLaneKeeping, kLeftLaneChange, kRightLaneChange
ErrorType SscPlannerAdapter::GetEgoDiscretBehavior(
    LateralBehavior* lat_behavior) {
  if (!is_valid_) return kWrongStatus;
  if (map_->ego_behavior().lat_behavior == common::LateralBehavior::kUndefined)
    return kWrongStatus;
  *lat_behavior = map_->ego_behavior().lat_behavior;
  return kSuccess;
}

/// @brief 根据车道 ID 查询车道信息 (通过语义车道集合查找)
ErrorType SscPlannerAdapter::GetLaneByLaneId(const int lane_id, Lane* lane) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    *lane = it->second.lane;
  }
  return kSuccess;
}

/// @brief 获取自车参考车道 (与 GetLocalReferenceLane 逻辑相同)
ErrorType SscPlannerAdapter::GetEgoReferenceLane(Lane* lane) {
  if (!is_valid_) return kWrongStatus;
  auto ref_lane = map_->ego_behavior().ref_lane;
  if (!ref_lane.IsValid()) {
    printf("[GetEgoReferenceLane]No reference lane existing.\n");
    return kWrongStatus;
  }
  *lane = ref_lane;
  return kSuccess;
}

/// @brief 获取 2D 障碍物占据栅格地图
ErrorType SscPlannerAdapter::GetObstacleMap(GridMap2D* grid_map) {
  if (!is_valid_) return kWrongStatus;
  *grid_map = map_->obstacle_map();
  return kSuccess;
}

/// @brief 检查给定状态是否与障碍物碰撞
///
/// 通过 SemanticMapManager::CheckCollisionUsingStateAndVehicleParam 执行,
/// 内部可能使用车辆轮廓与占据栅格地图的碰撞检测。
ErrorType SscPlannerAdapter::CheckIfCollision(
    const common::VehicleParam& vehicle_param, const State& state, bool* res) {
  if (!is_valid_) return kWrongStatus;
  map_->CheckCollisionUsingStateAndVehicleParam(vehicle_param, state, res);
  return kSuccess;
}

/// @brief 获取障碍物占据栅格集合
///
/// 返回 std::set<std::array<decimal_t, 2>>, 每个元素是一个 (x, y) 坐标对,
/// 表示障碍物栅格的中心点在全局笛卡尔坐标系中的位置。
ErrorType SscPlannerAdapter::GetObstacleGrids(
    std::set<std::array<decimal_t, 2>>* obs_grids) {
  if (!is_valid_) return kWrongStatus;
  *obs_grids = map_->obstacle_grids();
  return kSuccess;
}

}  // namespace planning
