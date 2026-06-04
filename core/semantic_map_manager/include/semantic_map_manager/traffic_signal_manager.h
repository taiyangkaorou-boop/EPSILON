/**
 * @file traffic_signal_manager.h
 * @author EPSILON Autonomous Driving Group
 * @brief 交通信号管理器——限速标志、红绿灯、停车标志的注册与查询
 *
 * @details
 * TrafficSignalManager 管理车道级别的交通规则约束，供规划器查询当前状态下车道
 * 上的限速值和停车状态。
 *
 * 核心概念——交叉口类型分类（IntersectionType）：
 *   - kNotIntersect：车辆当前状态不与信号区域相交
 *   - kSignalAhead：　车辆在信号区域前方，即将进入约束区域
 *   - kSignalControlled：车辆已在信号约束区域内
 *
 * 限速生效逻辑（GetSpeedLimit）：
 *   1. 将车辆状态转换为目标车道的Frenet坐标系
 *   2. 遍历所有 speed_limit_list_ 中的信号
 *   3. 对每个信号调用 CheckIntersectionTypeWithSignal 判断相交类型
 *   4. 计算有效减速距离 effect_speed_limit_dist：
 *      需要减速度时 = |v_limit^2 - v_cur^2| / (2*a_est)，其中 a_est=1.0 m/s^2
 *   5. 若为 kSignalAhead 且距离 < 有效减速距离，或为 kSignalControlled，
 *      则取最严格的限速值
 *
 * 信号相交判断（CheckIntersectionTypeWithSignal）：
 *   - 将信号的起点/终点投影到车道Frenet坐标系
 *   - 判断车辆Frenet状态是否在信号的横/纵向范围内
 *   - 车辆在信号前方（s < start_s）-> kSignalAhead
 *   - 车辆在信号内部（start_s <= s < end_s）-> kSignalControlled
 *   - 其他情况 -> kNotIntersect
 *
 * @version 0.1
 * @date 2019-03-20
 */

#ifndef _CORE_SEMANTIC_MAP_MANAGER_INC_TRAFFIC_SIGNAL_MANAGER_H__
#define _CORE_SEMANTIC_MAP_MANAGER_INC_TRAFFIC_SIGNAL_MANAGER_H__

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"

namespace semantic_map_manager {

/// @class TrafficSignalManager
/// @brief 管理车道级别的交通规则约束（限速、红绿灯、停车标志等）
///
/// 负责加载、更新和查询交通信号，将地理空间的信号定义与车辆的车道级状态
/// 进行Frenet坐标系的关联查询，为规划器提供实时的限速和停车约束。
class TrafficSignalManager {
 public:
  using State = common::State;
  using Lane = common::Lane;
  using SpeedLimit = common::SpeedLimit;
  using TrafficLight = common::TrafficLight;

  /// @brief 交叉口类型枚举——车辆与交通信号区域的空间关系
  enum IntersectionType {
    kNotIntersect = 0,    ///< 未相交——车辆不在信号区域及有效影响范围内
    kSignalAhead,         ///< 信号在前方——车辆即将进入约束区域，需准备减速
    kSignalControlled     ///< 信号控制中——车辆当前处于约束区域内，须遵守限速
  };

  TrafficSignalManager();
  ErrorType Init();
  /// @brief 加载交通信号数据（限速标志等）
  /// @note 当前实现为硬编码（TODO 注释指出应改为由物理仿真器驱动）
  ErrorType LoadSignals();
  /// @brief 更新信号状态
  /// @param time_elapsed 已经过的仿真时间
  /// @note 根据 valid_time 范围自动移除过期的限速信号
  ErrorType UpdateSignals(const decimal_t time_elapsed);

  /// @brief 获取某状态在指定车道上的限速值
  /// @param state 车辆状态
  /// @param lane 参考车道
  /// @param speed_limit [输出] 当前适用的限速值（m/s）或 kInf（无限速）
  ErrorType GetSpeedLimit(const State& state, const Lane& lane,
                          decimal_t* speed_limit) const;

  /// @brief 获取某状态在指定车道上的停车状态（用于红绿灯/停车标志）
  /// @param state 车辆状态
  /// @param lane 参考车道
  /// @param stopping_state [输出] 如果需要停车则为停止点的状态，否则为当前状态
  ErrorType GetTrafficStoppingState(const State& state, const Lane& lane,
                                    State* stopping_state) const;

  inline vec_E<SpeedLimit> speed_limit_list() const {
    return speed_limit_list_;
  }
  inline vec_E<TrafficLight> traffic_light_list() const {
    return traffic_light_list_;
  }

 private:
  /// @brief 检查Frenet状态与交通信号的空间关系
  ///
  /// 判断逻辑：
  ///   1. 将信号的 start_point/end_point 投影到车道Frenet坐标系
  ///   2. 检查投影点的横坐标(s)是否有序（start_s < end_s）
  ///   3. 检查车辆的横向位置(d)是否在信号的 lateral_range 范围内
  ///   4. 根据车辆的 s 与信号区间的位置关系分类：
  ///      - s < start_s -> kSignalAhead（信号在前方）
  ///      - start_s <= s < end_s -> kSignalControlled（受信号控制）
  ///      - s >= end_s -> kNotIntersect（不在信号范围内）
  ///
  /// @param fs 车辆在车道上的Frenet状态
  /// @param lane 参考车道
  /// @param signal 交通信号（含空间范围定义）
  /// @param intersection_type [输出] 相交类型
  /// @param dist_to_startpt [输出] 到信号起点的弧长距离
  /// @param dist_to_endpt [输出] 到信号终点的弧长距离
  ErrorType CheckIntersectionTypeWithSignal(const common::FrenetState& fs,
                                            const Lane& lane,
                                            const common::TrafficSignal& signal,
                                            IntersectionType* intersection_type,
                                            decimal_t* dist_to_startpt,
                                            decimal_t* dist_to_endpt) const;

  vec_E<SpeedLimit> speed_limit_list_;    ///< 限速标志列表
  vec_E<TrafficLight> traffic_light_list_; ///< 红绿灯列表
};

}  // namespace semantic_map_manager
#endif
