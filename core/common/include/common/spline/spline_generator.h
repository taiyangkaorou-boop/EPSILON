/**
 * @file spline_generator.h
 * @brief 样条曲线生成器模板类
 *
 * 本文件定义了 EPSILON 自动驾驶规划系统中的样条曲线生成器
 * SplineGenerator，提供从离散采样点、状态序列、空间走廊等
 * 数据源生成各种类型样条曲线的静态方法。
 *
 * SplineGenerator 是一个纯静态工具类模板，支持不同的多项式次数
 * (N_DEG) 和空间维度 (N_DIM) 组合。主要功能包括：
 *
 *   1. 插值类方法（精确通过所有给定数据点）：
 *      - GetCubicSplineBySampleInterpolation: 三次样条插值
 *        （自然三次样条，Natural Cubic Spline）
 *      - GetSplineFromStateVec: 从笛卡尔状态序列生成样条
 *      - GetSplineFromFreeStateVec: 从自由空间状态序列生成样条
 *      - GetWaypointsFromPositionSamples: 从位置采样生成带导数信息的 Waypoints
 *
 *   2. 拟合类方法（在最小二乘意义下逼近数据点）：
 *      - GetQuinticSplineBySampleFitting: 五次样条拟合，带连续性约束，
 *        适用于有噪声的采样数据
 *
 *   3. 优化类方法（在约束空间内求解最优轨迹）：
 *      - GetBezierSplineUsingCorridor: 在时空语义走廊约束下生成最优 Bezier 曲线，
 *        利用 Bezier 曲线的凸包性质保证整条曲线位于安全区域内。
 *        支持参考轨迹的邻近度加权（weight_proximity）。
 *
 * 样条生成器是连接原始感知/规划数据与数学曲线表示之间的桥梁，
 * 生成的样条曲线供 Lane、轨迹规划、碰撞检测等模块使用。
 *
 * 常见的 (N_DEG, N_DIM) 组合：
 *   - (3, 2): 三次二维样条，用于简单路径插值
 *   - (5, 2): 五次二维样条，用于满足加速度连续性的路径拟合
 *   - (5, 3): 五次三维样条，用于 (x, y, t) 时空轨迹优化
 */

#ifndef _CORE_COMMON_INC_COMMON_SPLINE_GENERATOR_H__
#define _CORE_COMMON_INC_COMMON_SPLINE_GENERATOR_H__

#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/basics/shapes.h"
#include "common/math/calculations.h"
#include "common/solver/qp_solver.h"
#include "common/spline/bezier.h"
#include "common/spline/lookup_table.h"
#include "common/spline/polynomial.h"
#include "common/spline/spline.h"
#include "common/state/state.h"
#include "common/state/waypoint.h"
#include "tk_spline/spline.h"

namespace common {

/**
 * @class SplineGenerator
 * @brief 样条曲线生成器（纯静态工具类模板）
 *
 * 提供一系列静态方法，从不同的输入数据类型生成样条曲线，
 * 涵盖插值、拟合和约束优化三种策略。
 *
 * @tparam N_DEG 生成多项式样条的次数
 * @tparam N_DIM 曲线的空间维度
 */
template <int N_DEG, int N_DIM>
class SplineGenerator {
 public:
  /// 生成的分段多项式样条类型
  typedef Spline<N_DEG, N_DIM> SplineType;

  /// 生成的 Bezier 样条类型
  typedef BezierSpline<N_DEG, N_DIM> BezierSplineType;

  /**
   * @brief 通过三次样条插值生成样条曲线（自然三次样条）
   *
   * 此方法通过解一个三对角线性系统构建自然三次样条（Natural Cubic Spline），
   * 使得样条曲线在断点处满足 C^2 连续性（位置、一阶导数、二阶导数连续）。
   * 自然边界条件：两端点的二阶导数为零。
   *
   * @param samples 采样点向量，每个元素为 N_DIM 维坐标，曲线将精确通过这些点
   * @param para 采样点对应的参数化值（通常为弧长），长度必须与 samples 相同
   * @param spline 输出参数，生成的三次样条曲线（封装在 N_DEG 次的 Spline 中）
   * @return ErrorType 操作是否成功
   */
  static ErrorType GetCubicSplineBySampleInterpolation(
      const vec_Vecf<N_DIM>& samples, const std::vector<decimal_t>& para,
      SplineType* spline);

  /**
   * @brief 通过带连续性约束的最小二乘拟合生成五次样条
   *
   * 此方法通过求解一个 QP（二次规划）问题，在同时满足分段间连续性约束
   * 的前提下，最小化样条曲线与采样数据的偏差。优化目标包含：
   *   1. 拟合误差项：在各采样参数值上计算样条值与实际值的偏差
   *   2. 光滑正则项：由 regulator 系数加权的曲线能量（高阶导数的平方积分）
   *
   * 分段信息由 breaks 断点数组指定。breaks 中的每个元素定义了一个新段的
   * 起始参数值。
   *
   * @param samples 采样点向量
   * @param para 采样点对应的参数化值向量（通常为弧长）
   * @param breaks 分段断点数组，如 [0, 5, 10, 15] 表示三段样条
   * @param regulator 正则化系数，控制光滑项权重。值越大结果越光滑，
   *                  但可能偏离采样点更远。建议范围 [1e6, 1e8)
   * @param spline 输出参数，生成的五次样条曲线
   * @return ErrorType 操作是否成功
   *
   * @note 当 breaks 数量很大时，由于 QP 问题的规模增大，此方法可能较耗时
   */
  static ErrorType GetQuinticSplineBySampleFitting(
      const vec_Vecf<N_DIM>& samples, const std::vector<decimal_t>& para,
      const Eigen::ArrayXf& breaks, const decimal_t regulator,
      SplineType* spline);

  /**
   * @brief 从位置采样点生成带导数信息的 Waypoint 向量
   *
   * 先通过样条插值获得位置样条，然后对样条求导获得每个采样点处的
   * 速度、加速度和加加速度信息，组装为 Waypoint 对象。
   *
   * @param samples 位置采样点向量
   * @param para 参数化值向量（通常为弧长）
   * @param waypoints 输出参数，包含完整运动学信息的 Waypoint 向量
   * @return ErrorType 操作是否成功
   */
  static ErrorType GetWaypointsFromPositionSamples(
      const vec_Vecf<N_DIM>& samples, const std::vector<decimal_t>& para,
      vec_E<Waypoint<N_DIM>>* waypoints);

  /**
   * @brief 从笛卡尔状态向量生成样条曲线
   *
   * 从 State 向量中提取位置信息（x, y），连同指定的参数化值，
   * 通过样条插值或拟合生成光滑曲线。
   *
   * @param para 参数化值向量（通常为时间或弧长）
   * @param state_vec 笛卡尔 State 向量，从中提取位置 (x, y)
   * @param spline 输出参数，生成的样条曲线
   * @return ErrorType 操作是否成功
   */
  static ErrorType GetSplineFromStateVec(const std::vector<decimal_t>& para,
                                         const vec_E<State>& state_vec,
                                         SplineType* spline);

  /**
   * @brief 从自由空间状态向量生成样条曲线
   *
   * 从 FreeState 向量中提取位置信息 (x, y)，连同指定的参数化值，
   * 通过样条插值或拟合生成光滑曲线。
   *
   * @param para 参数化值向量（通常为时间）
   * @param free_state_vec FreeState 向量，从中提取位置 (x, y)
   * @param spline 输出参数，生成的样条曲线
   * @return ErrorType 操作是否成功
   */
  static ErrorType GetSplineFromFreeStateVec(
      const std::vector<decimal_t>& para,
      const vec_E<FreeState>& free_state_vec, SplineType* spline);

  /**
   * @brief 在时空语义走廊约束下生成最优 Bezier 样条曲线
   *
   * 此方法是 Bezier 样条在约束空间优化的典型应用。其核心思想是：
   *   由于 Bezier 曲线具有凸包性质（曲线完全位于控制点凸包内），
   *   只要确保所有控制点位于安全走廊内，整条曲线就一定安全。
   *
   * 优化目标：
   *   1. 曲线能量最小化（通常为 jerk 的平方积分，通过 Hessian 矩阵计算）
   *   2. 与参考轨迹的邻近度加权（weight_proximity 控制的引导项）
   *
   * 约束条件：
   *   1. 控制点位于语义走廊 cube 内（凸约束，保证安全性）
   *   2. 起始段满足 start_constraints 指定的边界条件
   *   3. 终止段满足 end_constraints 指定的边界条件
   *
   * 典型用法：参数化为二维 t（二维位置 + 时间），走廊为三维（2D + t）。
   *
   * @param cubes 时空语义立方体序列（N_DIM+1 维，如 3D 表示 2D 空间 + 时间）
   * @param start_constraints 起点约束向量 [pos, vel, acc, ...]，每项是 N_DIM 维向量
   * @param end_constraints 终点约束向量 [pos, vel, acc, ...]
   * @param ref_stamps 参考时间戳向量，用于建立分段的时间参数化
   * @param ref_points 参考轨迹点向量，作为优化的引导目标
   * @param weight_proximity 邻近度权重，控制参考轨迹引导项的强度
   * @param bezier_spline 输出参数，生成的 Bezier 样条曲线
   * @return ErrorType 操作是否成功
   */
  static ErrorType GetBezierSplineUsingCorridor(
      const vec_E<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
      const vec_E<Vecf<N_DIM>>& start_constraints,
      const vec_E<Vecf<N_DIM>>& end_constraints,
      const std::vector<decimal_t>& ref_stamps,
      const vec_E<Vecf<N_DIM>>& ref_points, const decimal_t& weight_proximity,
      BezierSplineType* bezier_spline);

  /**
   * @brief 在时空语义走廊约束下生成最优 Bezier 样条曲线（简化版，无参考轨迹引导）
   *
   * 与上面的重载版本相同，但不包含参考轨迹邻近度引导（相当于 weight_proximity = 0）。
   *
   * @param cubes 时空语义立方体序列
   * @param start_constraints 起点约束向量
   * @param end_constraints 终点约束向量
   * @param bezier_spline 输出参数，生成的 Bezier 样条曲线
   * @return ErrorType 操作是否成功
   */
  static ErrorType GetBezierSplineUsingCorridor(
      const vec_E<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
      const vec_E<Vecf<N_DIM>>& start_constraints,
      const vec_E<Vecf<N_DIM>>& end_constraints,
      BezierSplineType* bezier_spline);

};  // class spline generator

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_SPLINE_GENERATOR_H__
