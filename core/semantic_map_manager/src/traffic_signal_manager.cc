/**
 * @file traffic_signal_manager.cc
 * @author EPSILON Autonomous Driving Group
 * @brief TrafficSignalManager 类实现——交通信号管理
 *
 * @details
 * 该文件实现了交通信号的加载、更新和查询逻辑：
 *   - LoadSignals()：加载限速标志/红绿灯（当前为硬编码占位）
 *   - UpdateSignals()：根据仿真时间移除过期信号
 *   - GetSpeedLimit()：查询某状态在某车道上的有效限速值
 *   - CheckIntersectionTypeWithSignal()：判断Frenet状态与信号区域的空间关系
 *   - GetTrafficStoppingState()：获取停车状态（未完整实现）
 *
 * 关键算法——限速生效判定：
 *   对于每个限速信号，计算车辆当前速度下减速到限速值所需的距离
 *   （基于匀减速假设a=1.0 m/s^2），若车辆在信号前方的距离小于该减速距离，
 *   或已在信号区域内，则限速生效。
 *
 * @note TODO: (@denny.ding) 交通信号应由物理仿真器控制，当前为硬编码实现
 */

#include "semantic_map_manager/traffic_signal_manager.h"

namespace semantic_map_manager {

/// @brief 构造函数——自动加载交通信号数据
TrafficSignalManager::TrafficSignalManager() { LoadSignals(); }

ErrorType TrafficSignalManager::Init() { return kSuccess; }

/// @brief 加载交通信号（限速标志、红绿灯等）
///
/// @note 当前实现为硬编码占位（所有信号定义均被注释掉）
///       TODO: (@denny.ding) 交通信号应由物理仿真器控制
///
/// 信号定义包含以下属性：
///   - start_point / end_point：信号的起止经纬度位置（世界坐标）
///   - vel_range：限速速度范围 [min, max]，若 max 为 0 则视为红灯/禁止通行
///   - lateral_range：横向有效范围 [min, max]
///   - start_angle / end_angle：起止方向角
///   - valid_time：有效时间范围 [start, end]（用于临时限速如施工区）
ErrorType TrafficSignalManager::LoadSignals() {
  // TODO: (@denny.ding) 交通信号应由物理仿真器控制
  // ~ 以下为硬编码的限速示例（全部注释掉了）

  // * 限速标志示例说明：
  //   每个 SpeedLimit 包含起止点坐标、限速值、横向有效范围和方向角
  //   当前所有硬编码示例均被注释，等待物理仿真器集成

  // * For google_urban 场景的硬编码示例（已注释）
  // * For highway 场景的硬编码示例（已注释）

  return kSuccess;
}

/// @brief 更新信号状态——根据仿真时间移除过期的临时限速
///
/// 扫描 speed_limit_list_ 中所有信号的 valid_time 属性，
/// 若当前仿真时间不在 [valid_time[0], valid_time[1]] 范围内，
/// 则该信号被移除。这用于处理临时性交通约束（如施工区限速）。
///
/// @param time_elapsed 已经过的仿真时间（秒）
ErrorType TrafficSignalManager::UpdateSignals(const decimal_t time_elapsed) {
  for (auto it = speed_limit_list_.begin(); it < speed_limit_list_.end();) {
    Vec2f valid_time = it->valid_time();
    // 若当前时间不在有效时间范围内，移除该信号
    if (time_elapsed < valid_time[0] || time_elapsed > valid_time[1]) {
      printf("[xxxxx]signal erased at %lf.\n", time_elapsed);
      it = speed_limit_list_.erase(it);
    } else {
      ++it;
    }
  }
  return kSuccess;
}

/// @brief 获取某状态在某车道上的有效限速值
///
/// 算法流程：
///   1. 将车辆状态转换为目标车道的Frenet坐标系
///   2. 遍历所有限速信号 speed_limit_list_
///   3. 对每个信号调用 CheckIntersectionTypeWithSignal 判断相交类型
///   4. 计算有效减速距离 effect_speed_limit_dist：
///      若当前速度 > 限速值，则
///        effect_dist = |v_limit^2 - v_cur^2| / (2 * a_est)
///      其中 a_est = 1.0 m/s^2（舒适减速度假设）
///      否则 effect_dist = 0（当前速度已满足限速）
///   5. 若相交类型为 kSignalAhead 且距离 < 有效减速距离，
///      或为 kSignalControlled，则适用该限速
///   6. 取所有适用限速中的最严格值（最小值）
///
/// @param state 车辆状态
/// @param lane 参考车道
/// @param speed_limit [输出] 限速值（m/s），若无限制则为 kInf
ErrorType TrafficSignalManager::GetSpeedLimit(const State& state,
                                              const Lane& lane,
                                              decimal_t* speed_limit) const {
  // 将笛卡尔状态转换为 Frenet 状态（在 lane 参考线上）
  common::StateTransformer stf(lane);
  common::FrenetState ref_fs;
  if (stf.GetFrenetStateFromState(state, &ref_fs) != kSuccess) {
    // printf("[GetSpeedLimit]Cannot get ref state frenet state.\n");
    return kWrongStatus;
  }

  const decimal_t acc_esti = 1.0;  // 估算舒适减速度 (m/s^2)
  decimal_t limit = kInf;          // 初始化为无限制

  for (auto& speed_limit : speed_limit_list_) {
    IntersectionType intersection_type;
    decimal_t dist_to_startpt;
    decimal_t dist_to_endpt;

    // 判断车辆与信号的空间关系
    if (CheckIntersectionTypeWithSignal(ref_fs, lane, speed_limit,
                                        &intersection_type, &dist_to_startpt,
                                        &dist_to_endpt) != kSuccess) {
      continue;
    }

    if (intersection_type != kNotIntersect) {
      // 计算有效减速距离（匀减速模型）
      //   v_f^2 = v_0^2 + 2*a*d  =>  d = |v_f^2 - v_0^2| / (2*|a|)
      decimal_t effect_speed_limit_dist =
          state.velocity > speed_limit.max_velocity()
              ? fabs(speed_limit.max_velocity() * speed_limit.max_velocity() -
                     state.velocity * state.velocity) /
                    (2.0 * acc_esti)
              : 0.0;

      // 判断限速是否生效：
      //   kSignalAhead + 距离 < 减速距离 => 需要开始减速
      //   kSignalControlled => 已经在限速区内
      if ((intersection_type == kSignalAhead &&
           dist_to_startpt < effect_speed_limit_dist) ||
          intersection_type == kSignalControlled) {
        // 取最严格的限速值
        limit = limit > speed_limit.max_velocity() ? speed_limit.max_velocity()
                                                   : limit;
      }
    }
  }
  *speed_limit = limit;
  return kSuccess;
}

/// @brief 判断 Frenet 状态与交通信号区域的空间关系
///
/// 判断逻辑：
///   1. 将信号的 start_point 和 end_point 投影到车道 Frenet 坐标系
///   2. 检查投影点的纵坐标(s)是否有序（start_s < end_s）
///   3. 检查车辆的横向位置(d)是否在信号的 lateral_range 范围内
///   4. 根据车辆的 s 坐标与信号区间的关系分类：
///      - s < start_s：车辆在信号前方 => kSignalAhead
///      - start_s <= s < end_s：车辆在信号区域内 => kSignalControlled
///      - s >= end_s：车辆已通过 => kNotIntersect
///
/// @param fs 车辆在车道上的Frenet状态
/// @param lane 参考车道
/// @param signal 交通信号定义
/// @param intersection_type [输出] 相交类型
/// @param dist_to_startpt [输出] 到信号起点的弧长距离（负值表示未到达）
/// @param dist_to_endpt [输出] 到信号终点的弧长距离
ErrorType TrafficSignalManager::CheckIntersectionTypeWithSignal(
    const common::FrenetState& fs, const Lane& lane,
    const common::TrafficSignal& signal, IntersectionType* intersection_type,
    decimal_t* dist_to_startpt, decimal_t* dist_to_endpt) const {
  // ~ 注意：使用朴素方法判断限速是否对当前 <state, lane> 生效
  common::StateTransformer stf(lane);
  Vec2f start_pt_fs, end_pt_fs;

  // 将信号的起止点投影到 Frenet 坐标系
  if (stf.GetFrenetPointFromPoint(signal.start_point(), &start_pt_fs) !=
      kSuccess) {
    return kWrongStatus;
  }
  if (stf.GetFrenetPointFromPoint(signal.end_point(), &end_pt_fs) != kSuccess) {
    return kWrongStatus;
  }

  Vec2f lateral_range = signal.lateral_range();
  IntersectionType int_type = kNotIntersect;
  decimal_t dist_to_start = 0.0;
  decimal_t dist_to_end = 0.0;

  // * 判断是否在信号范围内（投影需正序且在横向范围内）
  if (start_pt_fs[0] < end_pt_fs[0] + kEPS) {
    // 检查起止点的横向位置是否都在 lateral_range 内
    if (start_pt_fs[1] + lateral_range(1) > 0.0 &&
        start_pt_fs[1] + lateral_range(0) < 0.0 &&
        end_pt_fs[1] + lateral_range(1) > 0.0 &&
        end_pt_fs[1] + lateral_range(0) < 0.0) {

      if (fs.vec_s[0] < start_pt_fs[0]) {
        // 车辆在信号前方
        int_type = kSignalAhead;
        dist_to_start = start_pt_fs[0] - fs.vec_s[0];
        dist_to_end = end_pt_fs[0] - fs.vec_s[0];
      } else if (fs.vec_s[0] >= start_pt_fs[0] && fs.vec_s[0] < end_pt_fs[0]) {
        // 车辆在信号区域内
        int_type = kSignalControlled;
        dist_to_start = start_pt_fs[0] - fs.vec_s[0];
        dist_to_end = end_pt_fs[0] - fs.vec_s[0];
      } else {
        // 车辆已通过信号区域
        int_type = kNotIntersect;
      }
    }
  }

  *intersection_type = int_type;
  *dist_to_startpt = dist_to_start;
  *dist_to_endpt = dist_to_end;
  return kSuccess;
}

/// @brief 获取交通停车状态——根据交通信号给出停车点
/// @note 当前未实现，直接返回 kSuccess
ErrorType TrafficSignalManager::GetTrafficStoppingState(
    const State& state, const Lane& lane, State* stopping_state) const {
  return kSuccess;
}

}  // namespace semantic_map_manager
