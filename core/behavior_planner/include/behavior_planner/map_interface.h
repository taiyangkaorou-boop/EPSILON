/**
 * @file map_interface.h
 * @brief 行为规划器地图接口（纯虚抽象基类）
 *
 * 本文件定义了 BehaviorPlannerMapItf 纯虚接口，是行为规划器与地图数据源之间的
 * 抽象契约层。通过依赖倒置原则（DIP），将 BehaviorPlanner 与具体的语义地图实现
 * （如 SemanticMapManager）解耦。
 *
 * ## 设计目的
 *   - 解耦：BehaviorPlanner 不直接依赖 SemanticMapManager，而是依赖本接口
 *   - 可测试性：可通过 mock 实现进行单元测试
 *   - 可扩展性：支持不同的地图数据源（仿真器、实车感知等）
 *
 * ## 接口分类
 *   1. 自车状态查询：GetEgoState, GetEgoId, GetEgoVehicle, GetEgoLaneIdByPosition
 *   2. 车道拓扑查询：GetRightLaneId, GetLeftLaneId, GetChildLaneIds, GetFatherLaneIds, IsTopologicallyReachable
 *   3. 车道几何查询：GetLaneByLaneId, GetLocalLaneSamplesByState, GetRefLaneForStateByBehavior, GetWholeLaneNet
 *   4. 周围车辆查询：GetKeyVehicles, GetKeySemanticVehicles, GetLeadingVehicleOnLane
 *   5. 碰撞检测：CheckCollisionUsingState, CheckIfCollision
 *   6. 其他：GetSpeedLimit, GetPredictedBehavior, GetNearestLaneIdUsingState
 *
 * ## 实现类
 * 当前实现为 BehaviorPlannerMapAdapter（map_adapter.h），将接口调用转发到
 * SemanticMapManager。
 */
#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_MAP_INTERFACE_H_
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_MAP_INTERFACE_H_

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"

namespace planning {

/**
 * @class BehaviorPlannerMapItf
 * @brief 行为规划器地图服务的纯虚接口
 *
 * 所有方法均为纯虚函数，由具体的地图适配器实现。
 * 接口覆盖了行为规划器所需的全部地图查询能力。
 */
class BehaviorPlannerMapItf {
 public:
  /// 车辆状态类型（位置 x, y, 航向角 angle, 曲率 curvature, 速度 velocity）
  using State = common::State;

  /// 车道类型（含中心线几何、宽度、限速等）
  using Lane = common::Lane;

  /// 语义行为类型
  using Behavior = common::SemanticBehavior;

  /// 车辆类型（含运动状态、几何参数）
  using Vehicle = common::Vehicle;

  /// 横向行为枚举
  using LateralBehavior = common::LateralBehavior;

  /**
   * @brief 检查地图接口是否有效
   * @return true 如果地图数据已加载且可用
   */
  virtual bool IsValid() = 0;

  /**
   * @brief 获取自车当前状态（位置、航向、速度等）
   * @param[out] state 自车状态
   * @note 这是 MPDM 算法最基础的输入之一
   */
  virtual ErrorType GetEgoState(State *state) = 0;

  /**
   * @brief 获取自车 ID
   * @param[out] id 自车唯一标识符
   */
  virtual ErrorType GetEgoId(int *id) = 0;

  /**
   * @brief 获取自车完整信息（状态 + 几何参数）
   * @param[out] vehicle 自车对象
   */
  virtual ErrorType GetEgoVehicle(common::Vehicle *vehicle) = 0;

  /**
   * @brief 根据自车位置确定当前所在车道 ID
   * @param navi_path 导航路径（车道 ID 序列）
   * @param[out] lane_id 自车所在车道 ID
   * @note 结合导航路径进行车道匹配，避免匹配到导航无关车道
   */
  virtual ErrorType GetEgoLaneIdByPosition(const std::vector<int> &navi_path,
                                           int *lane_id) = 0;

  /**
   * @brief 根据给定状态查询最近车道 ID
   * @param state 查询状态（位置 + 航向）
   * @param navi_path 导航路径约束
   * @param[out] id 最近车道 ID
   * @param[out] distance 到车道的垂直距离
   * @param[out] arc_len 在车道上的弧长投影
   */
  virtual ErrorType GetNearestLaneIdUsingState(
      const Vec3f &state, const std::vector<int> &navi_path, int *id,
      decimal_t *distance, decimal_t *arc_len) = 0;

  /**
   * @brief 检查目标车道是否可从当前车道拓扑可达
   *
   * @param lane_id 查询的目标车道 ID
   * @param path 导航路径（车道 ID 列表）
   * @param[out] num_lane_changes 所需变道次数
   * @param[out] res 是否可达
   * @note 用于验证变道行为是否合法
   */
  virtual ErrorType IsTopologicallyReachable(const int lane_id,
                                             const std::vector<int> &path,
                                             int *num_lane_changes,
                                             bool *res) = 0;

  /**
   * @brief 获取关键车辆集合（用于前向仿真）
   *
   * 关键车辆是 MPDM 仿真中的交互主体，包括自车周围一定范围内的车辆。
   *
   * @param[out] key_vehicle_set 关键车辆集合
   */
  virtual ErrorType GetKeyVehicles(common::VehicleSet *key_vehicle_set) = 0;

  /**
   * @brief 获取关键语义车辆集合（含行为预测和参考车道）
   *
   * 与 GetKeyVehicles 的区别：返回的是 SemanticVehicleSet，包含每辆车的
   * 预测横向行为（lat_behavior）和参考车道（lane），用于多智能体仿真。
   *
   * @param[out] key_vehicle_set 关键语义车辆集合
   */
  virtual ErrorType GetKeySemanticVehicles(
      common::SemanticVehicleSet *key_vehicle_set) = 0;

  /**
   * @brief 获取指定车道的右侧相邻车道 ID
   * @param lane_id 源车道 ID
   * @param[out] r_lane_id 右侧车道 ID（仅当存在右侧变道条件时）
   */
  virtual ErrorType GetRightLaneId(const int lane_id, int *r_lane_id) = 0;

  /**
   * @brief 获取指定车道的左侧相邻车道 ID
   * @param lane_id 源车道 ID
   * @param[out] l_lane_id 左侧车道 ID（仅当存在左侧变道条件时）
   */
  virtual ErrorType GetLeftLaneId(const int lane_id, int *l_lane_id) = 0;

  /**
   * @brief 获取指定车道的子车道（下游后继车道）ID 列表
   * @param lane_id 源车道 ID
   * @param[out] child_ids 子车道 ID 列表（可能多个，如分叉路口）
   */
  virtual ErrorType GetChildLaneIds(const int lane_id,
                                    std::vector<int> *child_ids) = 0;

  /**
   * @brief 获取指定车道的父车道（上游前驱车道）ID 列表
   * @param lane_id 源车道 ID
   * @param[out] father_ids 父车道 ID 列表（可能多个，如合流路口）
   */
  virtual ErrorType GetFatherLaneIds(const int lane_id,
                                     std::vector<int> *father_ids) = 0;

  /**
   * @brief 根据车道 ID 获取 Lane 对象
   * @param lane_id 车道 ID
   * @param[out] lane 车道对象（含中心线几何）
   */
  virtual ErrorType GetLaneByLaneId(const int lane_id, Lane *lane) = 0;

  /**
   * @brief 获取车道在指定状态附近的局部采样点
   *
   * @param state 参考状态
   * @param lane_id 车道 ID
   * @param navi_path 导航路径
   * @param max_reflane_dist 向前最大采样距离
   * @param max_backward_dist 向后最大采样距离
   * @param[out] samples 车道中心线采样点序列（Vecf<2> x, y）
   */
  virtual ErrorType GetLocalLaneSamplesByState(
      const State &state, const int lane_id, const std::vector<int> &navi_path,
      const decimal_t max_reflane_dist, const decimal_t max_backward_dist,
      vec_Vecf<2> *samples) = 0;

  /**
   * @brief 根据自车状态和目标行为构建高质量参考车道
   *
   * 这是 MPDM 中为自车和前向仿真构建参考车道的核心接口。
   *
   * @param state 自车状态
   * @param navi_path 导航路径
   * @param behavior 目标横向行为（决定车道 ID）
   * @param max_forward_len 向前最大长度
   * @param max_back_len 向后最大长度
   * @param is_high_quality 是否生成高质量车道（含更密集采样）
   * @param[out] lane 生成的参考车道
   */
  virtual ErrorType GetRefLaneForStateByBehavior(
      const State &state, const std::vector<int> &navi_path,
      const LateralBehavior &behavior, const decimal_t &max_forward_len,
      const decimal_t &max_back_len, const bool is_high_quality,
      Lane *lane) = 0;

  /**
   * @brief 获取完整的车道网络
   * @param[out] lane_net 车道网络对象
   */
  virtual ErrorType GetWholeLaneNet(common::LaneNet *lane_net) = 0;

  /**
   * @brief 检查两车在给定状态下是否碰撞（使用车辆几何）
   * @param param_a 车辆 A 的几何参数
   * @param state_a 车辆 A 的状态
   * @param param_b 车辆 B 的几何参数
   * @param state_b 车辆 B 的状态
   * @param[out] res true 如果发生碰撞
   */
  virtual ErrorType CheckCollisionUsingState(
      const common::VehicleParam &param_a, const common::State &state_a,
      const common::VehicleParam &param_b, const common::State &state_b,
      bool *res) = 0;

  /**
   * @brief 检查给定车辆是否与任何障碍物碰撞
   * @param vehicle_param 车辆几何参数
   * @param state 车辆状态
   * @param[out] res true 如果发生碰撞
   */
  virtual ErrorType CheckIfCollision(const common::VehicleParam &vehicle_param,
                                     const State &state, bool *res) = 0;

  /**
   * @brief 获取指定车道上的前车（Leading Vehicle）
   *
   * 这是 IDM 跟驰模型的关键输入：查找自车所在车道前方最近的车辆。
   *
   * @param ref_lane 参考车道
   * @param ref_state 参考状态（自车当前位置）
   * @param vehicle_set 待搜索的车辆集合
   * @param lat_range 横向搜索范围（车道宽度容忍度）
   * @param[out] leading_vehicle 前车
   * @param[out] distance_residual_ratio 距离残差比（自车与前车距离 / IDM期望距离）
   */
  virtual ErrorType GetLeadingVehicleOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
      common::Vehicle *leading_vehicle, decimal_t *distance_residual_ratio) = 0;

  /**
   * @brief 获取指定状态所在位置的限速
   * @param state 查询状态
   * @param lane 所在车道
   * @param[out] speed_limit 限速值 [m/s]
   */
  virtual ErrorType GetSpeedLimit(const State &state, const Lane &lane,
                                  decimal_t *speed_limit) = 0;

  /**
   * @brief 获取指定车辆的预测横向行为
   *
   * 用于 MPDM 前向仿真中确定周围车辆的行为意图。
   *
   * @param vehicle_id 车辆 ID
   * @param[out] lat_behavior 预测的横向行为
   */
  virtual ErrorType GetPredictedBehavior(
      const int vehicle_id, common::LateralBehavior *lat_behavior) = 0;
};

}  // namespace planning

#endif  // _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_MAP_INTERFACE_H_
