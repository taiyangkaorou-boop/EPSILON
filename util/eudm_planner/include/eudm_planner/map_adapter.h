/**
 * @file map_adapter.h
 * @author EPSILON Autonomous Driving Team
 * @brief 地图适配器——EudmPlannerMapItf接口的具体实现
 * @version 0.1
 * @date 2019-07-07
 * @copyright Copyright (c) 2019
 *
 * ## 设计模式：适配器模式（Adapter Pattern）
 *
 * EudmPlannerMapAdapter实现了EudmPlannerMapItf纯虚接口，
 * 内部委托（delegate）给ROS SemanticMapManager对象。
 *
 * ## 职责
 *
 * 1. **接口转换**：将SemanticMapManager（ROS集成）的API转换为
 *    EUDM规划器所需的标准地图接口
 * 2. **有效性检查**：在所有操作前检查map_是否有效，
 *    确保不会在未加载地图数据时访问
 * 3. **数据转发**：直接将函数调用转发到SemanticMapManager的对应方法，
 *    并在调用前后添加有效性检查和错误处理
 *
 * ## 与EUDM系统的关系
 *
 * ```
 * EudmPlanner (依赖) --> EudmPlannerMapItf (接口)
 *                              ^
 *                              | 实现
 *                    EudmPlannerMapAdapter (适配器)
 *                              |
 *                              | 委托
 *                    SemanticMapManager (ROS地图管理器)
 * ```
 *
 * 这种设计使得EUDM规划器不直接依赖ROS的SemanticMapManager，
 * 可以通过注入不同的适配器实现来支持不同的地图数据源。
 *
 * @see map_interface.h EudmPlannerMapItf（纯虚接口定义）
 * @see semantic_map_manager/semantic_map_manager.h（具体地图实现）
 */

#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_MAP_ADAPTER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_MAP_ADAPTER_H_

#include <iostream>
#include <set>

#include "common/basics/semantics.h"
#include "eudm_planner/map_interface.h"
#include "semantic_map_manager/semantic_map_manager.h"

namespace planning {

/// @class EudmPlannerMapAdapter
/// @brief 地图适配器——将SemanticMapManager适配为EudmPlannerMapItf接口
///
/// 实现适配器模式（Adapter Pattern），作为EUDM规划器和
/// ROS语义地图管理器之间的桥梁。所有地图查询操作都
/// 通过委托调用SemanticMapManager的相应方法实现。
///
/// ## 关键特性
/// - 所有方法在调用前检查is_valid_，无效时返回kWrongStatus
/// - IsLaneConsistent使用BFS搜索实现车道拓扑一致性检查
/// - set_map用于注入新的地图数据指针（每个规划周期调用一次）
class EudmPlannerMapAdapter : public EudmPlannerMapItf {
 public:
  using IntegratedMap = semantic_map_manager::SemanticMapManager;

  /// @name EudmPlannerMapItf接口实现
  /// @{

  /// @brief 检查地图是否已加载且有效
  bool IsValid() override;

  /// @brief 获取自车状态（委托给map_->ego_vehicle().state()）
  ErrorType GetEgoState(State *state) override;

  /// @brief 获取自车ID
  ErrorType GetEgoId(int *id) override;

  /// @brief 获取自车完整的车辆对象
  ErrorType GetEgoVehicle(common::Vehicle *vehicle) override;

  /// @brief 根据位置获取自车所在车道ID
  ///
  /// 调用map_->GetNearestLaneIdUsingState获取距离自车位置最近的车道ID。
  ErrorType GetEgoLaneIdByPosition(const std::vector<int> &navi_path,
                                   int *lane_id) override;

  /// @brief 获取距离指定状态最近的车道ID
  ErrorType GetNearestLaneIdUsingState(const Vec3f &state,
                                       const std::vector<int> &navi_path,
                                       int *id, decimal_t *distance,
                                       decimal_t *arc_len) override;

  /// @brief 检查车道拓扑可达性
  ErrorType IsTopologicallyReachable(const int lane_id,
                                     const std::vector<int> &path,
                                     int *num_lane_changes, bool *res) override;

  /// @brief 检查新旧车道ID的拓扑一致性（BFS搜索）
  ///
  /// 从旧车道ID出发，通过BFS在其后继车道集合中搜索新车道ID。
  /// 如果找到，说明两车道在拓扑上是上下级（前驱-后继）关系。
  /// 最大扩展节点数限制为20，防止搜索过于深入。
  bool IsLaneConsistent(const int lane_id_old, const int lane_id_new) override;

  /// @brief 获取右侧相邻车道ID
  ErrorType GetRightLaneId(const int lane_id, int *r_lane_id) override;

  /// @brief 获取左侧相邻车道ID
  ErrorType GetLeftLaneId(const int lane_id, int *l_lane_id) override;

  /// @brief 获取后继车道ID列表
  ErrorType GetChildLaneIds(const int lane_id,
                            std::vector<int> *child_ids) override;

  /// @brief 获取前驱车道ID列表
  ErrorType GetFatherLaneIds(const int lane_id,
                             std::vector<int> *father_ids) override;

  /// @brief 通过车道ID获取车道对象
  ErrorType GetLaneByLaneId(const int lane_id, Lane *lane) override;

  /// @brief 获取局部车道采样点
  ErrorType GetLocalLaneSamplesByState(const State &state, const int lane_id,
                                       const std::vector<int> &navi_path,
                                       const decimal_t max_reflane_dist,
                                       const decimal_t max_backward_dist,
                                       vec_Vecf<2> *samples) override;

  /// @brief 根据行为获取参考车道
  ErrorType GetRefLaneForStateByBehavior(
      const State &state, const std::vector<int> &navi_path,
      const LateralBehavior &behavior, const decimal_t &max_forward_len,
      const decimal_t &max_back_len, const bool is_high_quality, Lane *lane);

  /// @brief 获取关键车辆集合
  ErrorType GetKeyVehicles(common::VehicleSet *key_vehicle_set) override;

  /// @brief 获取周围车辆集合
  ErrorType GetSurroundingVehicles(
      common::VehicleSet *key_vehicle_set) override;

  /// @brief 获取带语义信息的关键车辆（含行为概率分布）
  ///
  /// 委托给map_->semantic_key_vehicles()。
  /// 返回的SemanticVehicleSet包含每辆车的行为概率分布，
  /// 是EUDM建模不确定性的数据来源。
  ErrorType GetKeySemanticVehicles(
      common::SemanticVehicleSet *key_vehicle_set) override;

  /// @brief 获取完整车道网络
  ErrorType GetWholeLaneNet(common::LaneNet *lane_net) override;

  /// @brief 基于状态的碰撞检测
  ErrorType CheckCollisionUsingState(const common::VehicleParam &param_a,
                                     const common::State &state_a,
                                     const common::VehicleParam &param_b,
                                     const common::State &state_b,
                                     bool *res) override;

  /// @brief 获取参考车道上前方最近的车辆
  ErrorType GetLeadingVehicleOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
      common::Vehicle *leading_vehicle,
      decimal_t *distance_residual_ratio) override;

  /// @brief 获取参考车道上前后车辆的Frenet状态
  ErrorType GetLeadingAndFollowingVehiclesFrenetStateOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
      common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
      bool *has_following_vehicle, common::Vehicle *following_vehicle,
      common::FrenetState *following_fs) override;
  /// @}

  /// @brief 设置地图数据指针（依赖注入）
  ///
  /// 每个规划周期由EudmManager调用，传入最新的SemanticMapManager。
  /// 设置后is_valid_变为true，其他方法才能正常使用。
  ///
  /// @param map_ptr SemanticMapManager共享指针
  void set_map(std::shared_ptr<IntegratedMap> map_ptr);

  /// @brief 获取地图数据指针
  std::shared_ptr<IntegratedMap> map() { return map_; }

 private:
  std::shared_ptr<IntegratedMap> map_;  ///< 语义地图管理器指针（底层数据源）
  bool is_valid_ = false;               ///< 地图数据是否已加载且有效
};

}  // namespace planning

#endif
