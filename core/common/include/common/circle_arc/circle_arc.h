/**
 * @file circle_arc.h
 * @brief 圆弧几何基元（Circle Arc Geometric Primitive）
 *
 * 圆弧是自动驾驶轨迹规划和运动预测中最基础的几何基元之一。
 * 它以固定曲率描述车辆的运动弧段，是车辆运动学模型（自行车模型）
 * 在恒定转向角下的精确解。
 *
 * 圆弧的数学描述：
 *   - 由起始状态 (x0, y0, theta0)、曲率 κ 和弧长 L 唯一确定
 *   - 位置参数化：θ(s) = θ0 + κ*s
 *               x(s) = x0 + ∫ cos(θ(t)) dt  （0 到 s）
 *               y(s) = y0 + ∫ sin(θ(t)) dt  （0 到 s）
 *   - 当 κ = 0 时退化为直线段
 *   - 当 κ != 0 时，圆心在起始方向左侧（κ > 0）或右侧（κ < 0）
 *
 * 在本项目中的应用：
 *   - EUDM 规划器使用圆弧基元进行前向仿真（forward simulation）
 *   - 运动预测中用于估计车辆的未来可能位置
 *   - 可视化中用于绘制车辆的预测轨迹弧
 *   - CircleArcBranch 的基础构建单元
 *
 * @author ZHANG Lu (lzhangbz@connect.ust.hk)
 * @date
 */

#ifndef _CORE_COMMON_INC_COMMON_CIRCLE_ARC_H_
#define _CORE_COMMON_INC_COMMON_CIRCLE_ARC_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include "common/basics/basics.h"

namespace common {

/**
 * @class CircleArc
 * @brief 圆弧几何基元
 *
 * 以固定曲率描述从起始状态出发的一段圆弧运动。
 * 支持位置、一阶导数（切线方向）、二阶导数（法线方向/曲率）的计算，
 * 以及带横向偏移的偏移曲线计算。
 *
 * @note 当曲率为 0 时，弧退化为直线段；当弧长为 0 时，弧退化为一个点
 */
class CircleArc {
 public:
  /// 默认构造函数
  CircleArc() {}

  /**
   * @brief 根据起始状态、曲率和弧长构造圆弧
   *
   * @param start_state 起始状态 (x0, y0, theta0)，其中 theta0 = 0 表示沿 x 轴正方向
   * @param curvature 恒定曲率 κ [1/m]（正值表示左转，负值表示右转，0 表示直线）
   * @param arc_length 弧长 L [m]（正值表示前进方向）
   */
  CircleArc(const Vec3f &start_state, const double &curvature,
            const double &arc_length);

  /// 析构函数
  ~CircleArc() {}

  /// @name 属性访问器
  /// @{
  inline double curvature() const { return curvature_; }          ///< 曲率 κ [1/m]
  inline double arc_length() const { return arc_length_; }        ///< 弧长 L [m]
  inline double central_angle() const { return central_angle_; }  ///< 圆心角 = κ * L [rad]
  inline Vec3f start_state() const { return start_state_; }       ///< 起始状态 (x, y, theta)
  inline Vec3f final_state() const { return final_state_; }       ///< 终止状态 (x, y, theta)
  inline Vec2f center() const { return center_; }                 ///< 圆心坐标（κ = 0 时无定义）
  /// @}

  // ========== 位置及其导数（0 阶、1 阶、2 阶） ==========

  /// @name 纵向位置 x(s) 及其导数
  /// @{
  double x_d0(const double s) const;  ///< x 坐标（位置）在弧长 s 处的值
  double x_d1(const double s) const;  ///< dx/ds = cos(theta(s))，x 对弧长的 1 阶导数
  double x_d2(const double s) const;  ///< d2x/ds2 = -κ*sin(theta(s))，x 对弧长的 2 阶导数
  /// @}

  /// @name 横向位置 y(s) 及其导数
  /// @{
  double y_d0(const double s) const;  ///< y 坐标（位置）在弧长 s 处的值
  double y_d1(const double s) const;  ///< dy/ds = sin(theta(s))，y 对弧长的 1 阶导数
  double y_d2(const double s) const;  ///< d2y/ds2 = κ*cos(theta(s))，y 对弧长的 2 阶导数
  /// @}

  /// @name 航向角 theta(s) 及其导数
  /// @{
  double theta_d0(const double s) const;  ///< θ(s) = θ0 + κ*s，航向角在弧长 s 处的值
  double theta_d1(const double s) const;  ///< dθ/ds = κ，航向角对弧长的 1 阶导数（即曲率）
  /// @}

  // ========== 切线向量 ==========

  /// @name 单位切线向量 t(s) = (cos(θ), sin(θ))
  /// @{
  double tx_d0(const double s) const { return x_d1(s); }   ///< 切向量的 x 分量
  double ty_d0(const double s) const { return y_d1(s); }   ///< 切向量的 y 分量
  /// @}

  // ========== 法线向量（左侧方向） ==========

  /// @name 单位法线向量（左侧）n(s) = (-sin(θ), cos(θ))
  /// @{
  double nx_d0(const double s) const { return -this->ty_d0(s); }   ///< 法线向量的 x 分量
  double ny_d0(const double s) const { return this->tx_d0(s); }    ///< 法线向量的 y 分量
  /// @}

  // ========== 带横向偏移的曲线 ==========

  /**
   * @brief 带横向偏移的 x 坐标：x_offs(s, d) = x(s) - d*sin(θ(s))
   *
   * @param s 弧长参数 [m]
   * @param offs 横向偏移 d [m]（正值向左，负值向右）
   * @return 偏移曲线的 x 坐标
   */
  double x_offs_d0(const double s, const double offs) const;

  /**
   * @brief 带横向偏移的 x 坐标对弧长的 1 阶导数
   */
  double x_offs_d1(const double s, const double offs) const;

  /**
   * @brief 带横向偏移的 y 坐标：y_offs(s, d) = y(s) + d*cos(θ(s))
   *
   * @param s 弧长参数 [m]
   * @param offs 横向偏移 d [m]
   * @return 偏移曲线的 y 坐标
   */
  double y_offs_d0(const double s, const double offs) const;

  /**
   * @brief 带横向偏移的 y 坐标对弧长的 1 阶导数
   */
  double y_offs_d1(const double s, const double offs) const;

  /**
   * @brief 带横向偏移的航向角（与原弧相同，因为偏移不改变切线方向）
   */
  double theta_offs_d0(const double s, const double offs) const;

  /**
   * @brief 对圆弧进行均匀采样，获取离散的状态序列
   *
   * @param s_step 采样步长 [m]
   * @param[out] p_sampled_states 输出采样结果的指针（Vec3f 向量，每个元素为 (x, y, theta)）
   *
   * @note 采样范围：从 0 到 arc_length_，步长为 s_step
   */
  void GetSampledStates(const double s_step,
                        std::vector<Vec3f> *p_sampled_states) const;

 private:
  Vec3f start_state_;   ///< 起始状态 (x0, y0, theta0)
  Vec3f final_state_;   ///< 终止状态（由起始状态+曲率+弧长计算得出）

  double curvature_;    ///< 恒定曲率 κ [1/m]
  double arc_length_;   ///< 弧长 L [m]

  bool is_arc_ = true;  ///< 是否为真正的圆弧（κ != 0），若 κ = 0 则为直线段
  double central_angle_; ///< 圆心角 = κ * L [rad]
  Vec2f center_;         ///< 圆心坐标（仅当 κ != 0 时有效）
};

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_CIRCLE_ARC_H_
