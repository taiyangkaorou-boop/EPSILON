/**
 * @file map_adapter.h
 * @brief 行为规划器地图适配器（具体实现类）
 *
 * 本文件定义了 BehaviorPlannerMapAdapter 类，是 BehaviorPlannerMapItf 纯虚接口
 * 的具体实现。其核心职责是将 SemanticMapManager 的数据适配为行为规划器所需的地图
 * 查询接口。
 *
 * ## 设计模式：适配器模式（Adapter Pattern）
 *   - Target（目标接口）: BehaviorPlannerMapItf
 *   - Adaptee（被适配者）: SemanticMapManager
 *   - Adapter（适配器）: BehaviorPlannerMapAdapter
 *
 * ## 数据流
 *   BehaviorPlannerServer::PlanCycleCallback
 *     -> map_adapter_.set_map(smm_ptr)        // 注入最新地图数据
 *     -> bp_.set_map_interface(&map_adapter_)  // 绑定到规划器
 *     -> bp_.RunOnce()                         // 规划器通过接口调用地图查询
 *       -> map_adapter_.GetEgoState()          // 转发到 SemanticMapManager
 *       -> map_adapter_.GetLeftLaneId()        // 转发并检查变道可行性标志
 *
 * ## 典型使用方式
 * BehaviorPlanner 通过 BehaviorPlannerMapItf 指针调用方法，实际运行时会由
 * BehaviorPlannerMapAdapter 将调用转发到底层 SemanticMapManager，
 * 同时增加有效性检查和车道拓扑合法性校验。
 */
#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_MAP_ADAPTER_H_
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_MAP_ADAPTER_H_

#include "behavior_planner/map_interface.h"
#include "common/basics/semantics.h"
#include "semantic_map_manager/semantic_map_manager.h"

namespace planning {

/**
 * @class BehaviorPlannerMapAdapter
 * @brief 将 SemanticMapManager 适配为 BehaviorPlannerMapItf 接口
 *
 * 每个方法实现包含三步：
 *   1. 检查 is_valid_ 标志（是否已注入有效地图数据）
 *   2. 调用底层 map_（SemanticMapManager）对应方法
 *   3. 附加拓扑合法性检查（如左/右变道是否允许）
 */
class BehaviorPlannerMapAdapter : public BehaviorPlannerMapItf {
 public:
  /// 集成地图管理器类型别名
  using IntegratedMap = semantic_map_manager::SemanticMapManager;

  /** @name 有效性检查 */
  ///@{
  /// 检查地图适配器是否已绑定有效地图数据
  bool IsValid() override;
  ///@}

  /** @name 自车状态查询 */
  ///@{
  /// 获取自车状态（位置、航向、速度等）
  ErrorType GetEgoState(State *state) override;
  /// 获取自车唯一标识符
  ErrorType GetEgoId(int *id) override;
  /// 获取自车完整车辆对象（状态 + 几何参数）
  ErrorType GetEgoVehicle(common::Vehicle *vehicle) override;
  /// 根据自车位置确定当前所在车道 ID
  ErrorType GetEgoLaneIdByPosition(const std::vector<int> &navi_path,
                                   int *lane_id) override;
  ///@}

  /** @name 车道定位查询 */
  ///@{
  /// 根据给定状态查询最近车道 ID 及相关距离信息
  ErrorType GetNearestLaneIdUsingState(const Vec3f &state,
                                       const std::vector<int> &navi_path,
                                       int *id, decimal_t *distance,
                                       decimal_t *arc_len) override;
  /// 检查目标车道是否拓扑可达
  ErrorType IsTopologicallyReachable(const int lane_id,
                                     const std::vector<int> &path,
                                     int *num_lane_changes, bool *res) override;
  ///@}

  /** @name 车道拓扑查询 */
  ///@{
  /// 获取右侧相邻车道 ID（检查 r_change_avbl 标志）
  ErrorType GetRightLaneId(const int lane_id, int *r_lane_id) override;
  /// 获取左侧相邻车道 ID（检查 l_change_avbl 标志）
  ErrorType GetLeftLaneId(const int lane_id, int *l_lane_id) override;
  /// 获取下游后继车道 ID 列表
  ErrorType GetChildLaneIds(const int lane_id,
                            std::vector<int> *child_ids) override;
  /// 获取上游前驱车道 ID 列表
  ErrorType GetFatherLaneIds(const int lane_id,
                             std::vector<int> *father_ids) override;
  ///@}

  /** @name 车道几何数据查询 */
  ///@{
  /// 根据车道 ID 获取 Lane 对象（检查 Lane 有效性）
  ErrorType GetLaneByLaneId(const int lane_id, Lane *lane) override;
  /// 获取车道在指定状态附近的局部采样点
  ErrorType GetLocalLaneSamplesByState(const State &state, const int lane_id,
                                       const std::vector<int> &navi_path,
                                       const decimal_t max_reflane_dist,
                                       const decimal_t max_backward_dist,
                                       vec_Vecf<2> *samples) override;
  /// 根据自车状态和目标行为构建参考车道（附带有效性检查）
  ErrorType GetRefLaneForStateByBehavior(
      const State &state, const std::vector<int> &navi_path,
      const LateralBehavior &behavior, const decimal_t &max_forward_len,
      const decimal_t &max_back_len, const bool is_high_quality, Lane *lane) override;
  ///@}

  /** @name 周围车辆查询 */
  ///@{
  /// 获取关键车辆集合（TODO: 添加车辆选择策略）
  ErrorType GetKeyVehicles(common::VehicleSet *key_vehicle_set) override;
  /// 获取带语义信息的关键车辆集合（含行为预测和参考车道）
  ErrorType GetKeySemanticVehicles(
      common::SemanticVehicleSet *key_vehicle_set) override;
  ///@}

  /** @name 车道网络 */
  ///@{
  /// 获取完整车道网络
  ErrorType GetWholeLaneNet(common::LaneNet *lane_net) override;
  ///@}

  /** @name 碰撞检测 */
  ///@{
  /// 检查两车在给定状态下是否碰撞
  ErrorType CheckCollisionUsingState(const common::VehicleParam &param_a,
                                     const common::State &state_a,
                                     const common::VehicleParam &param_b,
                                     const common::State &state_b,
                                     bool *res) override;
  /// 检查给定车辆是否与任何障碍物碰撞
  ErrorType CheckIfCollision(const common::VehicleParam &vehicle_param,
                             const State &state, bool *res) override;
  ///@}

  /** @name 其他查询 */
  ///@{
  /// 获取特定车辆的前车（用于 IDM 跟驰）
  ErrorType GetLeadingVehicleOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
      common::Vehicle *leading_vehicle,
      decimal_t *distance_residual_ratio) override;
  /// 获取限速值
  ErrorType GetSpeedLimit(const State &state, const Lane &lane,
                          decimal_t *speed_limit) override;
  /// 获取指定车辆的预测横向行为（从语义车辆集合中查找）
  ErrorType GetPredictedBehavior(
      const int vehicle_id, common::LateralBehavior *lat_behavior) override;
  ///@}

  /**
   * @brief 注入最新的地图数据源
   * @param map_ptr SemanticMapManager 的共享指针
   * @note 调用后 is_valid_ 置为 true
   */
  void set_map(std::shared_ptr<IntegratedMap> map_ptr);

 private:
  /// 底层地图数据源的共享指针
  std::shared_ptr<IntegratedMap> map_;

  /// 有效性标志，set_map 后为 true
  bool is_valid_ = false;
};

}  // namespace planning

#endif  // _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_MAP_ADAPTER_H_
