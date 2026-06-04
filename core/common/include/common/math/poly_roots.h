/**
 * @file poly_roots.h
 * @brief 多项式实根求解器（Polynomial Roots Solver）
 *
 * 提供 n 阶多项式（最高 6 阶）实根的求解函数集合。
 * 求解策略根据多项式的阶数自适应选择：
 *   - n = 2（二次）：使用求根公式（quadratic formula）
 *   - n = 3（三次）：使用 Cardano 公式（解析解）
 *   - n = 4（四次）：使用 Ferrari 降阶法（通过解辅助三次方程降阶）
 *   - n = 5, 6：使用 Eigen 多项式求解器（数值方法，使用 companion matrix）
 *
 * 多项式根求解在自动驾驶中的典型应用：
 *   - 轨迹碰撞检测：求解自车轨迹与障碍物轨迹的交点
 *   - 曲率极值搜索：通过求导数多项式的根定位曲率峰值点
 *   - 到达时间估计：求解 s(t) - s_target = 0 的根
 *   - 控制稳定性分析：求解特征多项式的根
 *
 * 参考代码：
 *   https://github.com/sikang/motion_primitive_library
 *
 * @author EPSILON Autonomous Driving Team (adapted from Sikang Liu)
 * @date 2026
 */

#ifndef _CORE_COMMON_INC_COMMON_MATH_POLY_ROOTS_H__
#define _CORE_COMMON_INC_COMMON_MATH_POLY_ROOTS_H__

#include "common/basics/basics.h"
#include <unsupported/Eigen/Polynomials>
#include <iostream>

/**
 * @brief 求解二次方程 b*t^2 + c*t + d = 0 的实根
 *
 * 使用标准二次求根公式：
 *   t = (-c +/- sqrt(c^2 - 4bd)) / (2b)
 *
 * @param b 二次项系数
 * @param c 一次项系数
 * @param d 常数项
 * @return std::vector<decimal_t> 实根列表（0、1 或 2 个）
 *
 * @note 仅返回实根（判别式 >= 0 的情况），复数根被忽略
 */
inline std::vector<decimal_t> quad(decimal_t b, decimal_t c, decimal_t d) {
  std::vector<decimal_t> dts;
  decimal_t p = c * c - 4 * b * d;  // 判别式 Δ = c^2 - 4bd
  if (p < 0)
    return dts;  // 无实根
  else {
    dts.push_back((-c - sqrt(p)) / (2 * b));
    dts.push_back((-c + sqrt(p)) / (2 * b));
    return dts;
  }
}

/**
 * @brief 求解三次方程 a*t^3 + b*t^2 + c*t + d = 0 的实根
 *
 * 使用 Cardano 公式：
 *   1. 首先归一化：t^3 + a2*t^2 + a1*t + a0 = 0
 *   2. 通过代换 t = y - a2/3 消去二次项
 *   3. 解降阶方程 y^3 + 3Qy + 2R = 0
 *   4. 判别式 D = Q^3 + R^2 决定根的类型：
 *      - D > 0：一个实根，两个共轭复根（仅返回实根）
 *      - D = 0：一个单根和一个二重根
 *      - D < 0：三个不同实根（三角函数解法）
 *
 * @param a 三次项系数
 * @param b 二次项系数
 * @param c 一次项系数
 * @param d 常数项
 * @return std::vector<decimal_t> 实根列表（1、2 或 3 个）
 */
inline std::vector<decimal_t> cubic(decimal_t a, decimal_t b, decimal_t c,
                                    decimal_t d) {
  std::vector<decimal_t> dts;

  decimal_t a2 = b / a;   // 归一化：a*t^3 -> t^3
  decimal_t a1 = c / a;
  decimal_t a0 = d / a;

  // Cardano 参数计算
  decimal_t Q = (3 * a1 - a2 * a2) / 9;        // Q 参数
  decimal_t R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;  // R 参数
  decimal_t D = Q * Q * Q + R * R;              // 判别式 D
  if (D > 0) {
    // 一个实根的情况
    decimal_t S = std::cbrt(R + sqrt(D));
    decimal_t T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  } else if (D == 0) {
    // 重根情况：一个二重根和一个单根
    decimal_t S = std::cbrt(R);
    dts.push_back(-a2/3+S+S);     // 二重根（只记录一次）
    dts.push_back(-a2/3-S);       // 单根
    return dts;
  }
  else {
    // 三个不同实根：使用三角函数解法（casus irreducibilis）
    decimal_t theta = acos(R/sqrt(-Q*Q*Q));
    dts.push_back(2*sqrt(-Q)*cos(theta/3)-a2/3);
    dts.push_back(2*sqrt(-Q)*cos((theta+2*M_PI)/3)-a2/3);
    dts.push_back(2*sqrt(-Q)*cos((theta+4*M_PI)/3)-a2/3);
    return dts;
  }
}

/**
 * @brief 求解四次方程 a*t^4 + b*t^3 + c*t^2 + d*t + e = 0 的实根
 *
 * 使用 Ferrari 降阶法：
 *   1. 归一化：t^4 + a3*t^3 + a2*t^2 + a1*t + a0 = 0
 *   2. 求解辅助三次方程以获取降阶参数 y1
 *   3. 将四次方程分解为两个二次方程
 *   4. 分别求解两个二次方程
 *
 * @param a 四次项系数
 * @param b 三次项系数
 * @param c 二次项系数
 * @param d 一次项系数
 * @param e 常数项
 * @return std::vector<decimal_t> 实根列表（0、1、2、3 或 4 个）
 */
inline std::vector<decimal_t> quartic(decimal_t a, decimal_t b, decimal_t c,
                                      decimal_t d, decimal_t e) {
  std::vector<decimal_t> dts;

  decimal_t a3 = b / a;   // 归一化各项系数
  decimal_t a2 = c / a;
  decimal_t a1 = d / a;
  decimal_t a0 = e / a;

  // 求解辅助三次方程以获取 y1
  std::vector<decimal_t> ys = cubic(1, -a2, a1*a3-4*a0, 4*a2*a0-a1*a1-a3*a3*a0);
  decimal_t y1 = ys.front();
  decimal_t r = a3*a3/4-a2+y1;

  if(r < 0)
    return dts;  // 无实根（r < 0 意味着无法分解为实二次方程）

  decimal_t R = sqrt(r);
  decimal_t D, E;
  if(R != 0) {
    // 标准分解情况
    D = sqrt(0.75*a3*a3-R*R-2*a2+0.25*(4*a3*a2-8*a1-a3*a3*a3)/R);
    E = sqrt(0.75*a3*a3-R*R-2*a2-0.25*(4*a3*a2-8*a1-a3*a3*a3)/R);
  }
  else {
    // R = 0 的退化情况
    D = sqrt(0.75*a3*a3-2*a2+2*sqrt(y1*y1-4*a0));
    E = sqrt(0.75*a3*a3-2*a2-2*sqrt(y1*y1-4*a0));
  }

  // 收集四个根（仅有效根，忽略 NaN）
  if(!std::isnan(D)) {
    dts.push_back(-a3/4+R/2+D/2);
    dts.push_back(-a3/4+R/2-D/2);
  }
  if(!std::isnan(E)) {
    dts.push_back(-a3/4-R/2+E/2);
    dts.push_back(-a3/4-R/2-E/2);
  }

  return dts;
}

/**
 * @brief 通用四次多项式求解器
 *
 * 自动检测多项式的最高阶数，路由到相应的求解函数。
 * 支持最高次数为 0、1、2、3、4 的退化情况。
 *
 * @param a 四次项系数（可以为 0）
 * @param b 三次项系数（可以为 0）
 * @param c 二次项系数（可以为 0）
 * @param d 一次项系数
 * @param e 常数项
 * @return std::vector<decimal_t> 实根列表
 */
inline std::vector<decimal_t> solve(decimal_t a, decimal_t b, decimal_t c,
                                    decimal_t d, decimal_t e) {
  // 自适应路由：根据最高非零系数选择求解方法
  std::vector<decimal_t> ts;
  if (a != 0)
    return quartic(a, b, c, d, e);   // 四次
  else if (b != 0)
    return cubic(b, c, d, e);        // 三次
  else if (c != 0)
    return quad(c, d, e);            // 二次
  else if (d != 0) {
    ts.push_back(-e / d);            // 一元一次：d*t + e = 0
    return ts;
  } else
    return ts;  // 常数方程：无解
}

/**
 * @brief 求解五次多项式 a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t + f = 0 的实根
 *
 * 五次及以上多项式无解析解，使用 Eigen 多项式求解器（Companion Matrix 特征值法）。
 * 如果最高次系数 a = 0，自动降阶为四次求解。
 *
 * @param a 五次项系数
 * @param b 四次项系数
 * @param c 三次项系数
 * @param d 二次项系数
 * @param e 一次项系数
 * @param f 常数项
 * @return std::vector<decimal_t> 实根列表
 *
 * @note Eigen 多项式求解器计算 Companion Matrix 的所有特征值，
 *       时间复杂度为 O(n^3)，其中 n 为矩阵维度
 */
inline std::vector<decimal_t> solve(decimal_t a, decimal_t b, decimal_t c,
                                    decimal_t d, decimal_t e, decimal_t f) {
  std::vector<decimal_t> ts;
  if (a == 0)
    return solve(b, c, d, e, f);  // 降阶为四次
  else {
    Eigen::VectorXd coeff(6);
    coeff << f, e, d, c, b, a;    // Eigen 期望从常数项开始排列
    Eigen::PolynomialSolver<double, 5> solver;
    solver.compute(coeff);

    const Eigen::PolynomialSolver<double, 5>::RootsType &r = solver.roots();
    std::vector<decimal_t> ts;
    // 过滤复数根，仅保留实根
    for (int i = 0; i < r.rows(); ++i) {
      if (r[i].imag() == 0) {
        ts.push_back(r[i].real());
      }
    }
    return ts;
  }
}

/**
 * @brief 求解六次多项式 a*t^6 + b*t^5 + ... + f*t + g = 0 的实根
 *
 * 使用 Eigen 多项式求解器。如果最高次系数 a = 0 且 b = 0，自动降阶。
 *
 * @param a 六次项系数
 * @param b 五次项系数
 * @param c 四次项系数
 * @param d 三次项系数
 * @param e 二次项系数
 * @param f 一次项系数
 * @param g 常数项
 * @return std::vector<decimal_t> 实根列表
 */
inline std::vector<decimal_t> solve(decimal_t a, decimal_t b, decimal_t c,
                                    decimal_t d, decimal_t e, decimal_t f,
                                    decimal_t g) {
  std::vector<decimal_t> ts;
  if (a == 0 && b == 0)
    return solve(c, d, e, f, g);  // 降阶为五次
  else {
    Eigen::VectorXd coeff(7);
    coeff << g, f, e, d, c, b, a;
    Eigen::PolynomialSolver<double, 6> solver;
    solver.compute(coeff);

    const Eigen::PolynomialSolver<double, 6>::RootsType &r = solver.roots();
    std::vector<decimal_t> ts;
    for (int i = 0; i < r.rows(); ++i) {
      if (r[i].imag() == 0) {
        ts.push_back(r[i].real());
      }
    }

    return ts;
  }
}

#endif
