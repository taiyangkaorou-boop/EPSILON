/**
 * @file map_interface.h
 * @author HKUST Aerial Robotics Group
 * @brief SSC 规划器的地图接口 —— 纯虚基类
 *
 * [概述]
 * SscPlannerMapItf 定义了 SSC 规划器与外部环境数据源之间的抽象契约。
 * 它采用"依赖倒置"设计原则：SscPlanner 只依赖此接口，不直接依赖
 * SemanticMapManager 等具体实现。这使得规划器可以：
 *   - 在 ROS 仿真环境中通过 MapAdapter 获取实时数据
 *   - 在离线测试中通过 Mock 对象注入定制数据
 *   - 轻松适配不同的环境感知模块
 *
 * [接口分类]
 *   - 自车信息:  GetEgoVehicle, GetEgoState, GetEgoDiscretBehavior
 *   - 参考车道:  GetEgoReferenceLane, GetLocalReferenceLane, GetLaneByLaneId
 *   - 障碍物信息: GetObstacleMap, GetObstacleGrids, CheckIfCollision
 *   - 轨迹信息:  GetForwardTrajectories (两个重载版本)
 *   - 元信息:    IsValid, GetTimeStamp
 *
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_INTERFACE_H__
#define _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_INTERFACE_H__

#include <array>
#include <set>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"

namespace planning {

/// @brief 周围车辆某一种横向行为模态下的预测轨迹
/// @note 这是 SSC risk grid 专用旁路结构，暂不写入 core/common 全局语义契约。
struct SurroundingVehicleTrajectoryMode {
  int vehicle_id = kInvalidAgentId;  ///< 周围车辆 ID
  common::LateralBehavior lat_behavior{common::LateralBehavior::kUndefined};  ///< 该模态对应的横向行为
  decimal_t probability{0.0};  ///< 该模态概率，来自 SemanticVehicle.probs_lat_behaviors
  vec_E<common::Vehicle> traj;  ///< 全局坐标系下的预测轨迹
};

/// @brief 周围车辆多模态预测轨迹集合，key=车辆ID
using MultiModalSurroundingTrajectories =
    std::unordered_map<int, vec_E<SurroundingVehicleTrajectoryMode>>;

/// @class SscPlannerMapItf
/// @brief SSC 规划器地图接口 —— 所有环境数据提供者的抽象基类
///
/// 此接口定义了 SSC 规划器获取环境信息所需的全套方法。
/// 具体实现（如 SscPlannerAdapter）负责从语义地图管理器
/// 或其他数据源提取相应数据。
class SscPlannerMapItf {
 public:
  using ObstacleMapType = uint8_t;                          ///< 障碍物地图数据类型
  using State = common::State;                              ///< 全局状态类型
  using Lane = common::Lane;                                ///< 车道类型
  using Vehicle = common::Vehicle;                          ///< 车辆类型
  using LateralBehavior = common::LateralBehavior;          ///< 横向行为类型
  using Behavior = common::SemanticBehavior;                ///< 语义行为类型
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;  ///< 2D占据栅格地图

  // =========================================================================
  // 元信息
  // =========================================================================

  /// @brief 检查地图数据是否可用（已加载且未过期）
  /// @return true 表示数据有效可安全读取
  virtual bool IsValid() = 0;

  /// @brief 获取地图数据的时间戳
  /// @return 时间戳（秒）
  virtual decimal_t GetTimeStamp() = 0;

  // =========================================================================
  // 自车信息
  // =========================================================================

  /// @brief 获取自车完整车辆信息（含状态、参数）
  virtual ErrorType GetEgoVehicle(Vehicle* vehicle) = 0;

  /// @brief 获取自车状态（仅状态，不含车辆参数）
  virtual ErrorType GetEgoState(State* state) = 0;

  /// @brief 获取自车当前的离散横向行为决策
  /// @param lat_behavior 输出: LateralBehavior 枚举值
  virtual ErrorType GetEgoDiscretBehavior(LateralBehavior* lat_behavior) = 0;

  // =========================================================================
  // 参考车道
  // =========================================================================

  /// @brief 获取自车的参考车道（与行为决策关联）
  virtual ErrorType GetEgoReferenceLane(Lane* lane) = 0;

  /// @brief 获取局部参考车道（自车附近区域的车道信息）
  virtual ErrorType GetLocalReferenceLane(Lane* lane) = 0;

  /// @brief 根据车道 ID 查询车道信息
  virtual ErrorType GetLaneByLaneId(const int lane_id, Lane* lane) = 0;

  // =========================================================================
  // 障碍物信息
  // =========================================================================

  /// @brief 获取全局坐标系下的 2D 障碍物占据栅格地图
  virtual ErrorType GetObstacleMap(GridMap2D* grid_map) = 0;

  /// @brief 获取障碍物占据栅格的坐标集合（离散化后的位置列表）
  virtual ErrorType GetObstacleGrids(
      std::set<std::array<decimal_t, 2>>* obs_grids) = 0;

  /// @brief 检查给定车辆在给定状态下是否与障碍物发生碰撞
  /// @param vehicle_param 车辆物理参数
  /// @param state         全局坐标下的车辆状态
  /// @param res           输出: true=碰撞, false=无碰撞
  virtual ErrorType CheckIfCollision(const common::VehicleParam& vehicle_param,
                                     const State& state, bool* res) = 0;

  // =========================================================================
  // 前向轨迹（来自行为规划层的前向仿真结果）
  // =========================================================================

  /// @brief 获取各行为下的自车前向仿真轨迹（不含周围车辆轨迹）
  /// @param behaviors 输出: 各行为标签
  /// @param trajs     输出: [behavior][state] 前向轨迹
  virtual ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs) = 0;

  /// @brief 获取各行为下的自车前向轨迹 + 周围车辆预测轨迹
  /// @param behaviors 输出: 各行为标签
  /// @param trajs     输出: [behavior][state] 自车前向轨迹
  /// @param sur_trajs 输出: [behavior][vehicle_id][state] 周围车辆轨迹
  virtual ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs,
      vec_E<std::unordered_map<int, vec_E<Vehicle>>>* sur_trajs) = 0;

  /// @brief 获取周围车辆当前确定性预测轨迹的存在概率
  /// @param traj_probs 输出: key=车辆ID, value=该车当前 argmax 横向行为概率
  /// @note MVP-2 只把已有单条确定性周车轨迹按概率加权，不生成多模态轨迹。
  virtual ErrorType GetSurroundingTrajectoryExistenceProbabilities(
      std::unordered_map<int, decimal_t>* traj_probs) = 0;

  /// @brief 获取周围车辆多模态预测轨迹，供 risk grid 使用
  /// @param multimodal_trajs 输出: key=车辆ID, value=该车多个横向行为模态
  /// @note MVP-3 只让 risk grid 消费该旁路，不改变 binary map/corridor/QP。
  virtual ErrorType GetMultiModalSurroundingTrajectories(
      MultiModalSurroundingTrajectories* multimodal_trajs) = 0;
};

}  // namespace planning

#endif  // _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_INTERFACE_H__
