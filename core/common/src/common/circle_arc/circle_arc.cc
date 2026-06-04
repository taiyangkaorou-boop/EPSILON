/**
 * @file circle_arc.cc
 * @brief 圆弧轨迹类的实现文件
 *
 * 本文件实现了EPSILON自动驾驶规划系统中的圆弧运动原语。
 * CircleArc类描述了一条由起始状态、恒定曲率和弧长参数定义的
 * 圆弧（或直线）轨迹。
 *
 * ==================== 圆弧运动学 ====================
 *
 * 给定起始状态 (x0, y0, theta0) 和恒定曲率 kappa:
 *
 * 如果 kappa != 0 (圆弧):
 *   半径: r = 1 / kappa
 *   圆心: (cx, cy) = (x0 - r*sin(theta0), y0 + r*cos(theta0))
 *   圆心角: delta = kappa * arc_length
 *   轨迹: (x(s), y(s)) = (cx + r*sin(theta0 + s/r), cy - r*cos(theta0 + s/r))
 *   朝向: theta(s) = theta0 + s * kappa
 *   其中 s ∈ [0, arc_length]
 *
 * 如果 kappa == 0 (直线):
 *   轨迹: (x(s), y(s)) = (x0 + s*cos(theta0), y0 + s*sin(theta0))
 *   朝向: theta(s) = theta0
 *
 * ==================== 导数计算 ====================
 *
 * x_d0(s): x坐标关于弧长s的零阶导数（即位置）
 * x_d1(s): x坐标关于弧长s的一阶导数（dx/ds = cos(theta)）
 * x_d2(s): x坐标关于弧长s的二阶导数（d²x/ds² = -kappa * sin(theta)）
 * y_d0(s) ~ y_d2(s): 同上，y坐标的各阶导数
 * theta_d0(s): 朝向的零阶导数（即朝向角）
 * theta_d1(s): 朝向的一阶导数（dtheta/ds = kappa）
 *
 * ==================== 带偏移量的计算 ====================
 *
 * x_offs_d0(s, offs): 在弧长s处，沿法线方向偏移offs后的x坐标
 * 用于描述与圆弧等距的平行曲线（如车道的左右边界）。
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/circle_arc/circle_arc.h"

namespace common {

/*
 * CircleArc - 圆弧构造函数
 *
 * 根据起始状态、曲率和弧长计算圆弧的几何属性，包括：
 *   - 圆心位置（曲率不为零时）
 *   - 终点状态
 *   - 是否为真圆弧（is_arc_）
 *
 * @param start_state 起始状态 (x, y, theta)
 * @param curvature 恒定曲率 kappa（0表示直线）
 * @param arc_length 弧长（可为负值表示反向运动）
 */
CircleArc::CircleArc(const Vec3f &start_state, const double &curvature,
                     const double &arc_length)
    : start_state_(start_state),
      curvature_(curvature),
      arc_length_(arc_length) {
  if (curvature_ != 0.0) {
    is_arc_ = true;  // 曲率非零，这是一段圆弧

    central_angle_ = curvature_ * arc_length_;  // 圆心角 = kappa * s
    double r = 1.0 / curvature_;                // 半径 = 1 / kappa

    // 圆心坐标：(x0 - r*sin(theta0), y0 + r*cos(theta0))
    // 注意：弧从起始点开始，初始朝向垂直于起始点到圆心的连线
    center_(0) = start_state_(0) - r * sin(start_state_(2));
    center_(1) = start_state_(1) + r * cos(start_state_(2));

    // 终点状态：(cx + r*sin(theta0+delta), cy - r*cos(theta0+delta), theta0+delta)
    final_state_(0) = center_(0) + r * sin(start_state_(2) + central_angle_);
    final_state_(1) = center_(1) - r * cos(start_state_(2) + central_angle_);
    final_state_(2) = start_state_(2) + central_angle_;

  } else {
    is_arc_ = false;  // 曲率为零，这是一条直线

    // 直线终点：(x0 + s*cos(theta0), y0 + s*sin(theta0), theta0)
    final_state_(0) = start_state_(0) + arc_length_ * cos(start_state_(2));
    final_state_(1) = start_state_(1) + arc_length_ * sin(start_state_(2));
    final_state_(2) = start_state_(2);
  }
}

/*
 * x_d0 - 弧长s处的x坐标（零阶导数）
 *
 * 圆弧: x = cx + r * sin(theta0 + s/r)
 * 直线: x = x0 + s * cos(theta0)
 *
 * @param s 弧长参数
 * @return x坐标
 */
double CircleArc::x_d0(const double s) const {
  if (is_arc_) {
    double r = 1 / curvature_;
    return center_(0) + r * sin(start_state_(2) + s / r);
  } else {
    return start_state_(0) + s * cos(start_state_(2));
  }
}

/*
 * x_d1 - 弧长s处x坐标的一阶导数 dx/ds
 *
 * 圆弧: dx/ds = cos(theta0 + s*kappa)
 * 直线: dx/ds = cos(theta0)
 *
 * @param s 弧长参数
 * @return dx/ds
 */
double CircleArc::x_d1(const double s) const {
  if (is_arc_) {
    return cos(start_state_(2) + s * curvature_);
  } else {
    return cos(start_state_(2));
  }
}

/*
 * x_d2 - 弧长s处x坐标的二阶导数 d²x/ds²
 *
 * 圆弧: d²x/ds² = -kappa * sin(theta0 + s*kappa)
 * 直线: d²x/ds² = 0
 *
 * @param s 弧长参数
 * @return d²x/ds²
 */
double CircleArc::x_d2(const double s) const {
  if (is_arc_) {
    return -curvature_ * sin(start_state_(2) + s * curvature_);
  } else {
    return 0;
  }
}

/*
 * y_d0 - 弧长s处的y坐标（零阶导数）
 *
 * 圆弧: y = cy - r * cos(theta0 + s/r)
 * 直线: y = y0 + s * sin(theta0)
 */
double CircleArc::y_d0(const double s) const {
  if (is_arc_) {
    double r = 1 / curvature_;
    return center_(1) - r * cos(start_state_(2) + s / r);
  } else {
    return start_state_(1) + s * sin(start_state_(2));
  }
}

/*
 * y_d1 - 弧长s处y坐标的一阶导数 dy/ds
 *
 * 圆弧: dy/ds = sin(theta0 + s*kappa)
 * 直线: dy/ds = sin(theta0)
 */
double CircleArc::y_d1(const double s) const {
  if (is_arc_) {
    return sin(start_state_(2) + s * curvature_);
  } else {
    return sin(start_state_(2));
  }
}

/*
 * y_d2 - 弧长s处y坐标的二阶导数 d²y/ds²
 *
 * 圆弧: d²y/ds² = kappa * cos(theta0 + s*kappa)
 * 直线: d²y/ds² = 0
 */
double CircleArc::y_d2(const double s) const {
  if (is_arc_) {
    return curvature_ * cos(start_state_(2) + s * curvature_);
  } else {
    return 0;
  }
}

/*
 * theta_d0 - 弧长s处的朝向角
 *
 * 圆弧: theta = theta0 + s * kappa
 * 直线: theta = theta0
 */
double CircleArc::theta_d0(const double s) const {
  if (is_arc_) {
    return start_state_(2) + s * curvature_;
  } else {
    return start_state_(2);
  }
}

/*
 * theta_d1 - 弧长s处朝向角的一阶导数 dtheta/ds
 *
 * 圆弧: dtheta/ds = kappa（恒定曲率）
 * 直线: dtheta/ds = 0
 */
double CircleArc::theta_d1(const double s) const {
  if (is_arc_) {
    return curvature_;
  } else {
    return 0;
  }
}

/*
 * x_offs_d0 - 弧长s处沿法线偏移offs后的x坐标
 *
 * 在弧线的基础位置上叠加法线方向的偏移量：
 *   x_offs = x(s) + offs * nx(s)
 * 其中 nx(s) 是法线方向的x分量：nx = -sin(theta(s))
 *
 * @param s 弧长参数
 * @param offs 沿法线方向的偏移量（正值向左，负值向右）
 * @return 偏移后的x坐标
 */
double CircleArc::x_offs_d0(const double s, const double offs) const {
  return x_d0(s) + offs * nx_d0(s);
}

/*
 * y_offs_d0 - 弧长s处沿法线偏移offs后的y坐标
 *
 * y_offs = y(s) + offs * ny(s)
 * 其中 ny(s) 是法线方向的y分量：ny = cos(theta(s))
 */
double CircleArc::y_offs_d0(const double s, const double offs) const {
  return y_d0(s) + offs * ny_d0(s);
}

/*
 * theta_offs_d0 - 弧长s处沿法线偏移后的朝向角
 *
 * 沿法线方向偏移不改变朝向角（因为偏移是沿等距平行曲线的法线方向）。
 * 等距曲线与原始曲线在对应点处有相同的切线方向。
 */
double CircleArc::theta_offs_d0(const double s, const double offs) const {
  return theta_d0(s);
}

/*
 * GetSampledStates - 按步长对圆弧进行均匀采样
 *
 * 从弧长0开始，以 s_step 为步长，直到弧长终点，
 * 在每个采样点处记录 (x, y, theta) 三自由度状态。
 *
 * @param s_step 采样步长（沿弧长方向）
 * @param p_sampled_states 输出参数，采样状态序列
 */
void CircleArc::GetSampledStates(const double s_step,
                                 std::vector<Vec3f> *p_sampled_states) const {
  for (double length = 0.0; fabs(length) < fabs(arc_length_);
       length += s_step) {
    Vec3f sampled_state;
    sampled_state(0) = x_d0(length);    // 采样点的x坐标
    sampled_state(1) = y_d0(length);    // 采样点的y坐标
    sampled_state(2) = theta_d0(length); // 采样点的朝向角
    p_sampled_states->emplace_back(sampled_state);
  }
}

}  // namespace common
