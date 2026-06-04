/**
 * @file lane_generator.cc
 * @brief 车道生成器类（LaneGenerator）的实现文件
 *
 * 本文件实现了从离散采样点生成连续车道参考曲线的功能。
 * LaneGenerator 封装了样条曲线生成算法（三次样条插值或五次样条拟合），
 * 将离散的道路采样点转换为连续可微的 Lane 对象。
 *
 * ==================== 核心功能 ====================
 *
 * 1. GetLaneBySampleInterpolation:
 *    通过三次样条插值从采样点生成车道。
 *    要求采样点已通过参数化（para）指定弧长值。
 *
 * 2. GetLaneBySamplePoints:
 *    通过三次样条插值从采样点生成车道（自动参数化）。
 *    自动根据相邻采样点之间的欧氏距离累积计算弧长参数。
 *
 * 3. GetLaneBySampleFitting:
 *    通过五次样条拟合从采样点生成车道（带正则化）。
 *    使用分段五次多项式，在指定的断点（breaks）处保证连续性，
 *    并通过正则化参数（regulator）控制曲线的光滑度。
 *
 * ==================== 样条选择策略 ====================
 *
 * - 三次样条插值（Cubic Spline Interpolation）：
 *   优点：计算快速，对于规则采样点效果好
 *   缺点：对噪声敏感，不能调节光滑度
 *   适用场景：精确的地图数据，采样点密度高
 *
 * - 五次样条拟合（Quintic Spline Fitting）：
 *   优点：通过正则化项可以调节光滑度与拟合精度的平衡
 *   缺点：计算量较大，需要求解二次规划问题
 *   适用场景：含噪声的采样点，需要光滑的参考曲线
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/lane/lane_generator.h"
#include "common/basics/config.h"
#include "common/spline/spline_generator.h"

namespace common {

/*
 * GetLaneBySampleInterpolation - 通过三次样条插值从采样点生成车道
 *
 * 使用 SplineGenerator 的三次样条插值功能，在给定的弧长参数化
 * 下对采样点进行精确插值，生成连续的车道参考曲线。
 *
 * @param samples 车道采样点序列（笛卡尔坐标）
 * @param para 对应的弧长参数序列（每个采样点在曲线上的弧长值）
 * @param lane 输出参数，生成的车道对象
 * @return kSuccess 生成成功；kWrongStatus 采样点不足或插值失败
 */
ErrorType LaneGenerator::GetLaneBySampleInterpolation(
    const vec_Vecf<LaneDim>& samples, const std::vector<decimal_t>& para,
    Lane* lane) {
  Spline<LaneDegree, LaneDim> spline;
  SplineGenerator<LaneDegree, LaneDim> spline_generator;

  if (spline_generator.GetCubicSplineBySampleInterpolation(
          samples, para, &spline) != kSuccess) {
    return kWrongStatus;
  }
  lane->set_position_spline(spline);
  if (!lane->IsValid()) return kWrongStatus;
  return kSuccess;
}

/*
 * GetLaneBySamplePoints - 从采样点生成车道（自动参数化）
 *
 * 不需要预先提供弧长参数，自动根据采样点之间的欧氏距离
 * 累积计算弧长参数序列，然后调用 GetLaneBySampleInterpolation。
 *
 * 参数化方法：para[0] = 0, para[i] = para[i-1] + hypot(dx, dy)
 *
 * @param samples 车道采样点序列
 * @param lane 输出参数，生成的车道对象
 * @return kSuccess 生成成功；kWrongStatus 生成失败
 */
ErrorType LaneGenerator::GetLaneBySamplePoints(const vec_Vecf<LaneDim>& samples,
                                               Lane* lane) {
  std::vector<decimal_t> para;
  double d = 0;
  para.push_back(d);
  // 累积欧氏距离作为弧长参数的近似值
  for (int i = 1; i < (int)samples.size(); ++i) {
    double dx = samples[i](0) - samples[i - 1](0);
    double dy = samples[i](1) - samples[i - 1](1);
    d += std::hypot(dx, dy);  // sqrt(dx^2 + dy^2) 弦长近似弧长
    para.push_back(d);
  }
  if (common::LaneGenerator::GetLaneBySampleInterpolation(samples, para,
                                                          lane) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

/*
 * GetLaneBySampleFitting - 通过五次样条拟合生成车道（带正则化）
 *
 * 使用分段五次多项式在指定的断点（breaks）处拟合采样点，
 * 通过 regulator 参数控制拟合精度与光滑度之间的权衡。
 *
 * 内部使用 SplineGenerator::GetQuinticSplineBySampleFitting，
 * 该方法求解一个带平滑正则化项的最小二乘问题。
 *
 * @param samples 车道采样点序列
 * @param para 对应的弧长参数序列
 * @param breaks 分段多项式的断点位置（定义分段区间）
 * @param regulator 光滑度正则化系数（越大越光滑，但可能牺牲拟合精度）
 * @param lane 输出参数，生成的车道对象
 * @return kSuccess 生成成功；kWrongStatus 拟合失败
 */
ErrorType LaneGenerator::GetLaneBySampleFitting(
    const vec_Vecf<LaneDim>& samples, const std::vector<decimal_t>& para,
    const Eigen::ArrayXf& breaks, const decimal_t regulator, Lane* lane) {
  Spline<LaneDegree, LaneDim> spline;
  SplineGenerator<LaneDegree, LaneDim> spline_generator;

  if (spline_generator.GetQuinticSplineBySampleFitting(
          samples, para, breaks, regulator, &spline) != kSuccess) {
    return kWrongStatus;
  }
  lane->set_position_spline(spline);
  if (!lane->IsValid()) return kWrongStatus;
  return kSuccess;
}

}  // namespace common
