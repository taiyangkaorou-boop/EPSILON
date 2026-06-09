/**
 * @file map_adapter.h
 * @author HKUST Aerial Robotics Group
 * @brief 地图适配器 —— SscPlannerMapItf 的具体实现
 *
 * [概述]
 * SscPlannerAdapter 实现了 SscPlannerMapItf 纯虚接口，作为
 * SemanticMapManager（语义地图管理器）与 SscPlanner 之间的适配层。
 *
 * [设计模式]
 *   Adapter（适配器）模式：将 SemanticMapManager 的接口适配为
 *   SscPlannerMapItf 所定义的统一查询接口。
 *
 * [数据流]
 *   SemanticMapManager (共享指针)
 *        |
 *   SscPlannerAdapter::set_map()  ←  由服务器每帧调用
 *        |
 *   SscPlannerAdapter::GetXxx()   ←  由规划器查询使用
 *
 * [使用方式]
 *   1. 外部通过 set_map() 注入 SemanticMapManager 的快照
 *   2. SscPlanner 通过 SscPlannerMapItf* 指针调用各 Get* 方法
 *   3. 适配器内部委托 SemanticMapManager 完成实际查询
 *
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_ADAPTER_H__
#define _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_ADAPTER_H__

#include "common/basics/semantics.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "ssc_planner/map_interface.h"

namespace planning {

/// @class SscPlannerAdapter
/// @brief 地图适配器 —— 将 SemanticMapManager 适配为 SscPlannerMapItf 接口
///
/// 该类实现了适配器模式，负责在 SemanticMapManager 的具体数据结构和
/// SscPlanner 期望的抽象查询接口之间进行桥接。
class SscPlannerAdapter : public SscPlannerMapItf {
 public:
  using IntegratedMap = semantic_map_manager::SemanticMapManager;  ///< 被适配的地图类型

  /// @brief 检查地图是否有效
  bool IsValid() override;

  /// @brief 获取地图时间戳
  decimal_t GetTimeStamp() override;

  /// @brief 配置多模态周车预测时间参数
  /// @param prediction_time 预测时长，单位 s
  /// @param prediction_step 预测采样间隔，单位 s
  /// @note 由 SscPlanner::Init() 根据 proto 配置和 SSC map 时间域传入。
  void ConfigureMultiModalPrediction(const decimal_t prediction_time,
                                     const decimal_t prediction_step) override;

  /// @brief 获取自车车辆信息
  ErrorType GetEgoVehicle(Vehicle* vehicle) override;

  /// @brief 获取自车状态
  ErrorType GetEgoState(State* state) override;

  /// @brief 获取自车参考车道
  ErrorType GetEgoReferenceLane(Lane* lane) override;

  /// @brief 获取局部参考车道（delegate 到自车参考车道）
  ErrorType GetLocalReferenceLane(Lane* lane) override;

  /// @brief 根据 ID 查询车道
  ErrorType GetLaneByLaneId(const int lane_id, Lane* lane) override;

  /// @brief 获取 2D 障碍物占据栅格地图
  ErrorType GetObstacleMap(GridMap2D* grid_map) override;

  /// @brief 检查碰撞
  ErrorType CheckIfCollision(const common::VehicleParam& vehicle_param,
                             const State& state, bool* res) override;

  /// @brief 获取自车前向轨迹（不含周围车辆）
  ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs) override;

  /// @brief 获取自车当前离散行为
  ErrorType GetEgoDiscretBehavior(LateralBehavior* lat_behavior) override;

  /// @brief 获取自车前向轨迹 + 周围车辆预测轨迹
  ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs,
      vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>* sur_trajs)
      override;

  /// @brief 获取周围车辆当前确定性预测轨迹的存在概率
  ErrorType GetSurroundingTrajectoryExistenceProbabilities(
      std::unordered_map<int, decimal_t>* traj_probs) override;

  /// @brief 获取周围车辆多模态预测轨迹，供 SSC risk grid 使用
  ErrorType GetMultiModalSurroundingTrajectories(
      MultiModalSurroundingTrajectories* multimodal_trajs) override;

  /// @brief 获取按自车候选行为条件化的周围车辆多模态预测轨迹
  /// @param multimodal_trajs_by_ego_behavior 输出 [ego_behavior_index][vehicle_id][mode]
  /// @note 该接口会用 ego_behavior().surround_trajs[i] 覆盖对应候选下的
  ///       周车 argmax 模态，使 LK/LCL/LCR 自车候选可以拥有不同风险场。
  ErrorType GetBehaviorConditionedMultiModalSurroundingTrajectories(
      BehaviorConditionedMultiModalSurroundingTrajectories*
          multimodal_trajs_by_ego_behavior) override;

  /// @brief 获取障碍物占据栅格集合
  ErrorType GetObstacleGrids(
      std::set<std::array<decimal_t, 2>>* obs_grids) override;

  /// @brief 设置/更新底层 SemanticMapManager 数据
  /// @param map SemanticMapManager 的共享指针（持有环境快照）
  /// @return 错误码
  ErrorType set_map(std::shared_ptr<IntegratedMap> map);

 private:
  /// 持有 SemanticMapManager 的共享指针，通过它代理所有查询
  std::shared_ptr<IntegratedMap> map_;
  /// 内部有效性标志
  bool is_valid_ = false;
  /// 多模态预测时长，默认 5s；初始化后通常会被 SSC map horizon 覆盖
  decimal_t multimodal_prediction_time_ = 5.0;
  /// 多模态预测采样间隔，默认 0.2s
  decimal_t multimodal_prediction_step_ = 0.2;
};

}  // namespace planning

#endif
