/**
 * @file waypoint.h
 * @brief 路点（Waypoint）模板结构体定义
 *
 * 本文件定义了 EPSILON 自动驾驶规划系统中通用的路点（Waypoint）数据结构。
 * 路点是轨迹规划的基本单元，包含了在某一时刻（或某一参数值）处的完整
 * 运动学状态信息：位置、速度、加速度和加加速度（jerk）。
 *
 * 关键特性：
 *   - 模板化设计：支持任意维度（N_DIM=2 为二维路点，N_DIM=3 为三维路点）
 *   - 固定约束标志位：通过 fix_pos、fix_vel、fix_acc、fix_jrk 标志位
 *     指定哪些分量是硬约束（必须在轨迹规划中精确满足）
 *   - 时间戳标记：通过 stamped 标志位区分是否需要时间对齐
 *
 * 典型应用场景：
 *   - 作为样条曲线生成器的输入：通过一系列带有约束标志的路点，
 *     生成满足边界条件和（部分）中间约束的光滑样条轨迹
 *   - 在 QP（二次规划）优化问题中作为约束条件的形式化表示
 *   - 轨迹拼接与重规划中的连接点定义
 *
 * 类型别名：
 *   - Waypoint2D: 二维路点，用于平面位置规划
 *   - Waypoint3D: 三维路点，用于包含时间维度的时空轨迹规划
 */

#ifndef _COMMON_INC_COMMON_STATE_WAYPOINT_H__
#define _COMMON_INC_COMMON_STATE_WAYPOINT_H__

#include "common/basics/basics.h"

namespace common {

/**
 * @struct Waypoint
 * @brief 通用路点模板结构体
 *
 * 路点封装了在某一时刻 t 处的运动学状态（位置、速度、加速度、加加速度），
 * 并通过 fix_* 标志位标注关键约束点，用于轨迹插值、拟合和平滑优化。
 *
 * @tparam N_DIM 路点的空间维度（2 表示二维，3 表示三维/含时间）
 *
 * 各约束标志位的含义：
 *   - fix_pos: 位置必须精确通过
 *   - fix_vel: 速度必须精确满足
 *   - fix_acc: 加速度必须精确满足
 *   - fix_jrk: 加加速度必须精确满足
 *   - stamped: 路点时间戳是否有效（用于时间对齐的轨迹）
 */
template <int N_DIM>
struct Waypoint {
  /// 路点位置向量，维度 N_DIM
  Vecf<N_DIM> pos;

  /// 路点速度向量，维度 N_DIM
  Vecf<N_DIM> vel;

  /// 路点加速度向量，维度 N_DIM
  Vecf<N_DIM> acc;

  /// 路点加加速度（jerk）向量，维度 N_DIM
  Vecf<N_DIM> jrk;

  /// 路点对应的时间/参数值
  decimal_t t{0.0};

  /// 位置是否必须精确满足的约束标志
  bool fix_pos = false;

  /// 速度是否必须精确满足的约束标志
  bool fix_vel = false;

  /// 加速度是否必须精确满足的约束标志
  bool fix_acc = false;

  /// 加加速度是否必须精确满足的约束标志
  bool fix_jrk = false;

  /// 时间戳是否有效的标志（用于标识该路点是否包含有效的时间信息）
  bool stamped = false;
};

/// 二维路点类型别名，用于平面位置规划的 2D 场景
typedef Waypoint<2> Waypoint2D;

/// 三维路点类型别名，用于包含时间维度的时空轨迹规划的 3D 场景
typedef Waypoint<3> Waypoint3D;

}  // namespace common

#endif
