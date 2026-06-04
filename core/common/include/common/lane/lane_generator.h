/**
 * @file lane_generator.h
 * @brief 车道线生成器
 *
 * 本文件定义了 EPSILON 自动驾驶规划系统中的 LaneGenerator 类，
 * 提供从离散采样点生成样条曲线表示的车道线（Lane 对象）的功能。
 *
 * LaneGenerator 是一个纯静态工具类，所有方法均为静态方法，不需要
 * 实例化即可使用。它支持三种车道线生成策略：
 *
 *   1. GetLaneBySampleInterpolation:
 *      通过样本点的样条插值生成车道线。要求车道线精确通过所有样本点，
 *      适用于高质量感知数据（如高精地图车道中心线采样点）。
 *      需要用户提供参数值（通常是弧长），用于建立采样点与参数之间的对应关系。
 *
 *   2. GetLaneBySamplePoints:
 *      根据样本点的顺序自动生成参数化（使用累积弦长作为近似的弧长参数化），
 *      然后通过样条插值生成车道线。此方法的参数化精度低于手动指定参数的方法，
 *      但使用起来更简便。
 *
 *   3. GetLaneBySampleFitting:
 *      通过带连续性约束的最小二乘拟合生成车道线。车道线不必精确通过每个样本点，
 *      而是在全局最优的意义下逼近样本点，同时保证分段之间的连续性。
 *      此方法适用于有噪声的传感器数据或地图数据。
 *      正则化系数 regulator 建议范围为 [1e6, 1e8)。
 *
 * 生成的车道线可用于建立 Frenet 坐标系参考框架，支撑后续的轨迹规划任务。
 */

#ifndef _CORE_COMMON_INC_COMMON_LANE_LANE_GENERATOR_H__
#define _CORE_COMMON_INC_COMMON_LANE_LANE_GENERATOR_H__

#include "common/lane/lane.h"

#include "common/basics/basics.h"
#include "common/basics/config.h"

namespace common {

/**
 * @class LaneGenerator
 * @brief 车道线生成器（纯静态工具类）
 *
 * 提供多种从离散采样点生成 Lane 对象的静态方法，包括：
 *   - 精确插值（interpolation）方法，要求曲线精确通过所有采样点
 *   - 最小二乘拟合（fitting）方法，在最优逼近采样点的同时保证曲线光滑
 *
 * 所有方法都是静态的，无需实例化 LaneGenerator 即可调用。
 */
class LaneGenerator {
 public:
  /**
   * @brief 通过样本点样条插值生成车道线
   *
   * 生成的样条曲线将精确通过所有给定的采样点。需要用户提供采样点对应的
   * 参数值（通常为弧长），以建立采样点位置与曲线参数之间的映射关系。
   *
   * @param samples 车道线采样点向量，每个元素为二维坐标 (x, y)
   * @param para 采样点对应的参数值向量（通常为弧长），长度应与 samples 相同
   * @param lane 输出参数，生成的 Lane 对象
   * @return ErrorType 操作是否成功
   */
  static ErrorType GetLaneBySampleInterpolation(
      const vec_Vecf<LaneDim>& samples, const std::vector<decimal_t>& para,
      Lane* lane);

  /**
   * @brief 通过样本点自动参数化并插值生成车道线
   *
   * 与 GetLaneBySampleInterpolation 的区别在于，此方法自动使用累积弦长
   * 作为参数化（无需用户提供 para），然后进行样条插值。适用于参数值未知
   * 但采样点间距较为均匀的场景。
   *
   * @param samples 车道线采样点向量，每个元素为二维坐标 (x, y)
   * @param lane 输出参数，生成的 Lane 对象
   * @return ErrorType 操作是否成功
   */
  static ErrorType GetLaneBySamplePoints(const vec_Vecf<LaneDim>& samples,
                                         Lane* lane);

  /**
   * @brief 通过最小二乘拟合生成车道线（带连续性约束）
   *
   * 生成的样条曲线不要求精确通过所有采样点，而是在满足分段间连续性约束的
   * 前提下，最小化与采样点的累积误差。此方法对噪声数据更鲁棒。
   *
   * 该函数的优化目标包含两项：
   *   - 数据拟合项：最小化样条曲线与采样点的偏差（在指定的参数值上求值）
   *   - 光滑正则项：惩罚各分段的高阶导数能量（由 regulator 系数加权）
   *
   * @param samples 车道线采样点向量，每个元素为二维坐标 (x, y)
   * @param para 采样点对应的参数值向量（通常为弧长）
   * @param breaks 分段断点数组，定义样条曲线的分段数。例如，若有 N 个断点，
   *               则生成 N-1 个多项式段
   * @param regulator 正则化系数，控制光滑项相对于拟合项的权重。
   *                  建议取值范围为 [1e6, 1e8)，值越大结果越光滑但离群点偏差越大
   * @param lane 输出参数，生成的 Lane 对象
   * @return ErrorType 操作是否成功
   */
  static ErrorType GetLaneBySampleFitting(const vec_Vecf<LaneDim>& samples,
                                          const std::vector<decimal_t>& para,
                                          const Eigen::ArrayXf& breaks,
                                          const decimal_t regulator,
                                          Lane* lane);
};

}  // namespace common

#endif
