/**
 * @file map_interface.h
 * @author EPSILON Autonomous Driving Team
 * @brief EUDM规划器的纯虚拟地图接口
 * @version 0.1
 * @date 2019-07-07
 * @copyright Copyright (c) 2019
 *
 * ## 设计目的
 *
 * EudmPlannerMapItf是一个纯虚接口类（pure virtual interface），
 * 定义了EUDM规划器与地图系统之间的所有必需交互。
 *
 * 采用接口分离原则（ISP）和依赖倒置原则（DIP）：
 * - EudmPlanner依赖于该抽象接口，而非具体的地图实现
 * - 不同的地图实现（如ROS SemanticMapManager适配器）只需实现该接口
 * - 便于单元测试（可以注入mock地图实现）
 *
 * ## 接口分类
 *
 * **自车信息获取**：
 * - GetEgoState/GetEgoId/GetEgoVehicle: 获取自车状态
 * - GetEgoLaneIdByPosition: 根据位置获取车道ID
 *
 * **车道拓扑查询**：
 * - GetRightLaneId/GetLeftLaneId: 获取左右相邻车道
 * - GetChildLaneIds/GetFatherLaneIds: 获取前后继车道
 * - IsTopologicallyReachable: 拓扑可达性检查
 * - IsLaneConsistent: 两车道是否在拓扑上一致（通过BFS检查）
 *
 * **参考线和几何**：
 * - GetRefLaneForStateByBehavior: 根据行为获取参考车道
 * - GetLaneByLaneId: 通过ID获取车道对象
 * - GetLocalLaneSamplesByState: 获取局部车道采样点
 * - GetNearestLaneIdUsingState: 获取距离状态最近的车道ID
 *
 * **交通参与者**：
 * - GetKeyVehicles/GetSurroundingVehicles: 获取关键/周围车辆
 * - GetKeySemanticVehicles: 获取带语义信息的车辆
 * - GetLeadingVehicleOnLane: 获取车道上前方车辆
 * - GetLeadingAndFollowingVehiclesFrenetStateOnLane: 获取前后车辆Frenet状态
 *
 * **碰撞检测**：
 * - CheckCollisionUsingState: 基于状态的碰撞检测
 *
 * @see map_adapter.h EudmPlannerMapAdapter（该接口的具体实现）
 * @see eudm_planner.h
 */

#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_MAP_INTERFACE_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_MAP_INTERFACE_H_

#include <iostream>
#include <set>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"

namespace planning {

/// @class EudmPlannerMapItf
/// @brief EUDM规划器的纯虚拟地图接口
///
/// 该接口定义了EUDM行为规划器所需的所有地图交互操作。
/// 任何具体的地图实现（如ROS SematicMapManager适配器）必须实现此接口。
///
/// 所有方法都是纯虚函数（virtual = 0），形成EUDM规划器与地图系统的
/// 解耦边界。这支持：
/// 1. 单元测试：注入mock地图实现
/// 2. 地图实现切换：无需修改EUDM规划器代码
/// 3. 接口清晰：明确列出EUDM需要的地图功能
class EudmPlannerMapItf {
 public:
  using State = common::State;
  using Lane = common::Lane;
  using Behavior = common::SemanticBehavior;
  using Vehicle = common::Vehicle;
  using LateralBehavior = common::LateralBehavior;

  /// @name 基础检查
  /// @{

  /// @brief 检查地图接口是否有效（是否已加载地图数据）
  /// @return true表示地图数据已加载，可以使用其他方法
  virtual bool IsValid() = 0;
  /// @}

  /// @name 自车信息获取
  /// @{

  /// @brief 获取自车状态（位置、速度、加速度、曲率等）
  virtual ErrorType GetEgoState(State *state) = 0;

  /// @brief 获取自车ID
  virtual ErrorType GetEgoId(int *id) = 0;

  /// @brief 获取自车完整的车辆对象
  virtual ErrorType GetEgoVehicle(common::Vehicle *vehicle) = 0;

  /// @brief 根据自车位置获取所在的车道ID
  /// @param navi_path 导航路径（当前不使用，预留用于路线约束）
  /// @param lane_id [out] 输出的车道ID
  virtual ErrorType GetEgoLaneIdByPosition(const std::vector<int> &navi_path,
                                           int *lane_id) = 0;

  /// @brief 获取距离指定状态最近的车道ID（同时返回距离和弧长）
  ///
  /// 用于判断自车/周围车辆当前属于哪个车道。
  ///
  /// @param state 查询状态（x, y, theta）
  /// @param navi_path 导航路径
  /// @param id [out] 输出的最近车道ID
  /// @param distance [out] 到车道的垂直距离
  /// @param arc_len [out] 在车道上的弧长
  virtual ErrorType GetNearestLaneIdUsingState(
      const Vec3f &state, const std::vector<int> &navi_path, int *id,
      decimal_t *distance, decimal_t *arc_len) = 0;
  /// @}

  /// @name 车道拓扑查询
  /// @{

  /// @brief 检查目标车道是否从源车道拓扑可达
  ///
  /// 沿着车道的有向图，检查是否可以通过若干次变道和前进
  /// 从源车道到达目标车道。
  ///
  /// @param num_lane_changes [out] 需要的变道次数
  /// @param res [out] 是否可达
  virtual ErrorType IsTopologicallyReachable(const int lane_id,
                                             const std::vector<int> &path,
                                             int *num_lane_changes,
                                             bool *res) = 0;

  /// @brief 检查两个车道是否拓扑一致（新车道是否是旧车道的后继）
  ///
  /// 通过BFS搜索，在旧车道的后继集合中查找新车道的存在。
  /// 用于判断变道操作是否成功完成。
  ///
  /// @return true表示新旧车道在拓扑上前后继关系
  virtual bool IsLaneConsistent(const int lane_id_old, const int lane_id_new) = 0;

  /// @brief 获取关键车辆集合（包含语义信息的车辆）
  virtual ErrorType GetKeyVehicles(common::VehicleSet *key_vehicle_set) = 0;

  /// @brief 获取周围车辆集合
  virtual ErrorType GetSurroundingVehicles(
      common::VehicleSet *key_vehicle_set) = 0;

  /// @brief 获取带语义信息的关键车辆（包含行为概率分布）
  ///
  /// 返回的SemanticVehicleSet中的每辆车包含：
  /// - 车辆状态（位置、速度等）
  /// - 横向行为概率分布（ProbDistOfLatBehaviors）
  /// - 当前横向行为
  ///
  /// 这是EUDM建模周围车辆行为不确定性的核心数据来源。
  virtual ErrorType GetKeySemanticVehicles(
      common::SemanticVehicleSet *key_vehicle_set) = 0;

  /// @brief 获取指定车道的右侧相邻车道ID
  virtual ErrorType GetRightLaneId(const int lane_id, int *r_lane_id) = 0;

  /// @brief 获取指定车道的左侧相邻车道ID
  virtual ErrorType GetLeftLaneId(const int lane_id, int *l_lane_id) = 0;

  /// @brief 获取指定车道的后继车道ID列表
  virtual ErrorType GetChildLaneIds(const int lane_id,
                                    std::vector<int> *child_ids) = 0;

  /// @brief 获取指定车道的前驱车道ID列表
  virtual ErrorType GetFatherLaneIds(const int lane_id,
                                     std::vector<int> *father_ids) = 0;

  /// @brief 通过车道ID获取完整的车道对象
  virtual ErrorType GetLaneByLaneId(const int lane_id, Lane *lane) = 0;
  /// @}

  /// @name 参考线和几何
  /// @{

  /// @brief 获取车道在状态附近的局部采样点
  ///
  /// 用于路径可视化和控制参考。
  ///
  /// @param max_reflane_dist 参考线前方最大距离
  /// @param max_backward_dist 参考线后方最大距离
  /// @param samples [out] 输出的采样点列表（x,y坐标）
  virtual ErrorType GetLocalLaneSamplesByState(
      const State &state, const int lane_id, const std::vector<int> &navi_path,
      const decimal_t max_reflane_dist, const decimal_t max_backward_dist,
      vec_Vecf<2> *samples) = 0;

  /// @brief 根据状态和行为获取参考车道
  ///
  /// 给定自车状态和预期的横向行为（保持/左变道/右变道），
  /// 返回对应方向上的参考车道（含车道几何信息）。
  /// 参考车道用于Frenet坐标变换和前向仿真。
  ///
  /// @param behavior 横向行为
  /// @param max_forward_len 参考线前方最大长度
  /// @param max_back_len 参考线后方最大长度
  /// @param is_high_quality 是否要求高质量车道
  /// @param lane [out] 输出的参考车道
  virtual ErrorType GetRefLaneForStateByBehavior(
      const State &state, const std::vector<int> &navi_path,
      const LateralBehavior &behavior, const decimal_t &max_forward_len,
      const decimal_t &max_back_len, const bool is_high_quality,
      Lane *lane) = 0;

  /// @brief 获取完整的车道网络
  virtual ErrorType GetWholeLaneNet(common::LaneNet *lane_net) = 0;
  /// @}

  /// @name 碰撞检测
  /// @{

  /// @brief 基于状态检测两车是否碰撞
  ///
  /// 使用车辆参数（长宽）和状态（位置、朝向）进行几何碰撞检测。
  ///
  /// @param param_a 车辆A的参数（长/宽/后轴位置等）
  /// @param state_a 车辆A的状态
  /// @param param_b 车辆B的参数
  /// @param state_b 车辆B的状态
  /// @param res [out] 是否发生碰撞
  virtual ErrorType CheckCollisionUsingState(
      const common::VehicleParam &param_a, const common::State &state_a,
      const common::VehicleParam &param_b, const common::State &state_b,
      bool *res) = 0;
  /// @}

  /// @name 交通参与者交互
  /// @{

  /// @brief 获取参考车道上位于参考状态前方的车辆
  ///
  /// 在指定车道和横向范围内搜索前方最近的车辆。
  /// 返回距离残留比（distance_residual_ratio），指示自车与前车的距离
  /// 相对于前车有效影响范围的比例（用于调整效率代价中的前车影响权重）。
  ///
  /// @param ref_lane 参考车道
  /// @param ref_state 参考状态（通常是自车状态）
  /// @param vehicle_set 待搜索的车辆集合
  /// @param lat_range 横向搜索范围
  /// @param leading_vehicle [out] 输出的前车
  /// @param distance_residual_ratio [out] 距离残留比
  virtual ErrorType GetLeadingVehicleOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
      common::Vehicle *leading_vehicle, decimal_t *distance_residual_ratio) = 0;

  /// @brief 获取参考车道上参考状态前方和后方的车辆（含Frenet状态）
  ///
  /// 同时查找前车和后车，并返回它们在参考车道Frenet坐标系下的状态。
  /// 用于变道时的间隙查找和RSS预检查。
  ///
  /// @param has_leading_vehicle [out] 是否存在前车
  /// @param leading_vehicle [out] 前车对象
  /// @param leading_fs [out] 前车的Frenet状态
  /// @param has_following_vehicle [out] 是否存在后车
  /// @param following_vehicle [out] 后车对象
  /// @param following_fs [out] 后车的Frenet状态
  virtual ErrorType GetLeadingAndFollowingVehiclesFrenetStateOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
      common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
      bool *has_following_vehicle, common::Vehicle *following_vehicle,
      common::FrenetState *following_fs) = 0;
  /// @}
};

}  // namespace planning

#endif
