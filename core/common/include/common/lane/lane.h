/**
 * @file lane.h
 * @brief 基于样条曲线的车道线表示
 *
 * 本文件定义了 EPSILON 自动驾驶规划系统中的 Lane 类，它通过样条曲线
 * 来表示结构化道路中的车道线（通常是车道中心线）。Lane 类是 Frenet 坐标系
 * 的基础，因为在 Frenet 框架中，s 坐标表示沿车道线的弧长，d 坐标表示
 * 偏离车道线的横向距离。
 *
 * 车道线的数学表示：
 *   - 使用 Spline<LaneDegree, LaneDim> 类型（由 config.h 中的 LaneDegree
 *     和 LaneDim 常量控制多项式的次数和维度）作为车道线的参数化曲线
 *   - 参数为弧长 s，位置函数 P(s) = (x(s), y(s))
 *   - 通过样条的解析导数可以直接获得各阶几何量（切向量、法向量、曲率等）
 *
 * 该类提供的主要功能：
 *   - 按弧长求值：位置、切向量、法向量、朝向角、曲率及其导数
 *   - 反向投影：根据笛卡尔坐标点查找最近弧长（即找到点在车道线上的投影）
 *   - 边界查询：获取车道线的起止弧长范围 [begin, end]
 *
 * Lane 类不负责车道线的生成，相关生成功能由 LaneGenerator 提供。
 */

#ifndef _CORE_COMMON_INC_COMMON_LANE_LANE_H__
#define _CORE_COMMON_INC_COMMON_LANE_LANE_H__

#include "common/basics/config.h"
#include "common/spline/spline.h"
#include "common/state/state.h"

namespace common {

/**
 * @class Lane
 * @brief 样条曲线表示的车道线类
 *
 * 该类封装了一条由样条参数化曲线表示的车道线。车道线的参数是弧长 s，
 * 曲线上的点 P(s) 表示距离车道线起点 s 米处的坐标 (x(s), y(s))。
 * 样条曲线的多项式次数和维度由配置常量 LaneDegree 和 LaneDim 控制。
 *
 * 由于样条是解析的数学表示，该车道线类可以精确高效地计算：
 *   - 任意弧长处的几何属性（位置、切向、法向、曲率）
 *   - 笛卡尔点的弧长投影（最近点搜索）
 *
 * 这些功能是 Frenet 坐标系转换和轨迹规划的关键支撑。
 */
class Lane {
 public:
  /// 车道线的样条类型别名：次数为 LaneDegree，维度为 LaneDim
  typedef Spline<LaneDegree, LaneDim> SplineType;

  /// 默认构造函数，创建无效的车道线（is_valid_ = false）
  Lane() {}

  /**
   * @brief 通过指定的位置样条构造有效的车道线
   * @param position_spline 车道线的位置样条曲线 P(s) = (x(s), y(s))
   */
  Lane(const SplineType& position_spline)
      : position_spline_(position_spline), is_valid_(true) {}

  /**
   * @brief 检查车道线是否有效（已正确初始化样条曲线）
   * @return true 表示车道线可用
   */
  bool IsValid() const { return is_valid_; }

  /**
   * @brief 设置车道线的样条曲线参数化
   * @param position_spline 新的位置样条曲线，替换当前车道线的几何表示
   * @note 如果传入样条的域为空，操作将被忽略
   */
  void set_position_spline(const SplineType& position_spline) {
    if (position_spline.vec_domain().empty()) return;
    position_spline_ = position_spline;
    is_valid_ = true;
  }

  /**
   * @brief 根据弧长计算曲率及其导数
   *
   * 通过样条曲线的一阶和二阶解析导数，利用公式：
   *   kappa = |P' x P''| / |P'|^3
   * 计算弧长 s 处车道线的曲率。同时可通过更高阶导数计算曲率对弧长的导数。
   *
   * @param arc_length 查询弧长值（必须位于 [begin(), end()] 区间内）
   * @param curvature 输出参数，弧长处的曲率值，单位 1/m
   * @param curvature_derivative 输出参数，曲率对弧长的导数，单位 1/m^2
   * @return ErrorType 操作是否成功
   */
  ErrorType GetCurvatureByArcLength(const decimal_t& arc_length,
                                    decimal_t* curvature,
                                    decimal_t* curvature_derivative) const;

  /**
   * @brief 根据弧长计算曲率（不含导数）
   * @param arc_length 查询弧长值
   * @param curvature 输出参数，弧长处的曲率值，单位 1/m
   * @return ErrorType 操作是否成功
   */
  ErrorType GetCurvatureByArcLength(const decimal_t& arc_length,
                                    decimal_t* curvature) const;

  /**
   * @brief 根据弧长获取指定阶数的导数
   *
   * @param arc_length 查询弧长值
   * @param d 求导阶数：0 表示位置 P(s)，1 表示切向量 dP/ds，2 表示二阶导数 d^2P/ds^2
   * @param derivative 输出参数，d 阶导数值
   * @return ErrorType 操作是否成功
   */
  ErrorType GetDerivativeByArcLength(const decimal_t arc_length, const int d,
                                     Vecf<LaneDim>* derivative) const;

  /**
   * @brief 根据弧长获取车道线上的坐标位置
   * @param arc_length 查询弧长值
   * @param derivative 输出参数，该弧长处的坐标位置 P(s) = (x(s), y(s))
   * @return ErrorType 操作是否成功
   */
  ErrorType GetPositionByArcLength(const decimal_t arc_length,
                                   Vecf<LaneDim>* derivative) const;

  /**
   * @brief 根据弧长获取车道线的单位切向量
   *
   * 切向量指向弧长增大的方向（即车道线的前进方向）。
   *
   * @param arc_length 查询弧长值
   * @param tangent_vector 输出参数，弧长处的单位切向量 T(s) = P'(s) / |P'(s)|
   * @return ErrorType 操作是否成功
   */
  ErrorType GetTangentVectorByArcLength(const decimal_t arc_length,
                                        Vecf<LaneDim>* tangent_vector) const;

  /**
   * @brief 根据弧长获取车道线的单位法向量
   *
   * 法向量定义为切向量顺时针旋转 90 度（即指向车道线左侧）。
   *
   * @param arc_length 查询弧长值
   * @param normal_vector 输出参数，弧长处的单位法向量
   * @return ErrorType 操作是否成功
   */
  ErrorType GetNormalVectorByArcLength(const decimal_t arc_length,
                                       Vecf<LaneDim>* normal_vector) const;

  /**
   * @brief 根据弧长获取车道线的朝向角
   *
   * 朝向角定义为切向量方向与 X 轴正方向的夹角，范围 [-pi, pi]。
   *
   * @param arc_length 查询弧长值
   * @param angle 输出参数，该弧长处车道线的切向角，单位 rad
   * @return ErrorType 操作是否成功
   */
  ErrorType GetOrientationByArcLength(const decimal_t arc_length,
                                      decimal_t* angle) const;

  /**
   * @brief 根据笛卡尔坐标在车道线上寻找对应的最近弧长（投影问题）
   *
   * 在车道线上搜索距离给定点 (vec_position) 最近的弧长 s。
   * 这是笛卡尔坐标到 Frenet 坐标转换中的关键步骤（s 投影）。
   *
   * @param vec_position 笛卡尔坐标系中的点 (x, y)
   * @param arc_length 输出参数，该点在车道线上的投影弧长
   * @return ErrorType 操作是否成功
   */
  ErrorType GetArcLengthByVecPosition(const Vecf<LaneDim>& vec_position,
                                      decimal_t* arc_length) const;

  /**
   * @brief 带初始猜测的笛卡尔坐标弧长投影
   *
   * 通过提供上一个点的弧长作为初始猜测，可以显著加速投影过程，
   * 特别适用于对一系列连续点进行投影的场景。
   *
   * @param vec_position 笛卡尔坐标系中的点 (x, y)
   * @param initial_guess 投影弧长的初始猜测值（如前一个点的投影结果）
   * @param arc_length 输出参数，该点在车道线上的投影弧长
   * @return ErrorType 操作是否成功
   */
  ErrorType GetArcLengthByVecPositionWithInitialGuess(
      const Vecf<LaneDim>& vec_position, const decimal_t& initial_guess,
      decimal_t* arc_length) const;

  /**
   * @brief 检查输入的弧长值是否在车道线的有效范围内
   * @param arc_length 待检查的弧长值
   * @return ErrorType 若弧长位于 [begin(), end()] 内则返回 kSuccess
   */
  ErrorType CheckInputArcLength(const decimal_t arc_length) const;

  /**
   * @brief 获取车道线内部的位置样条曲线
   * @return 车道线当前使用的 SplineType 样条曲线
   */
  SplineType position_spline() const { return position_spline_; }

  /**
   * @brief 获取车道线参数域的起点弧长
   * @return 车道线起点的弧长值
   */
  decimal_t begin() const { return position_spline_.begin(); }

  /**
   * @brief 获取车道线参数域的终点弧长
   * @return 车道线终点的弧长值
   */
  decimal_t end() const { return position_spline_.end(); }

  /**
   * @brief 打印车道线样条信息（调试用）
   */
  void print() const { position_spline_.print(); }

 private:
  /// 车道线的位置样条曲线 P(s) = (x(s), y(s))
  SplineType position_spline_;

  /// 车道线有效性标志，false 表示未初始化或无效状态
  bool is_valid_ = false;
};

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_LANE_LANE_H__
