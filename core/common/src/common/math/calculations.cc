/**
 * @file calculations.cc
 * @brief 数学计算工具函数的实现文件
 *
 * 本文件实现了EPSILON自动驾驶规划系统中常用的数学计算工具函数，
 * 是系统中所有数学相关基础运算的集合。
 *
 * 核心功能包括：
 * - fac(n): 阶乘计算，对n<=5进行查表优化
 * - nchoosek(n,k): 二项式系数（组合数）C(n,k) = n!/(k!(n-k)!)
 * - normalize_angle(theta): 角度归一化到(-pi, pi]范围
 * - rotate_vector_2d(v, angle): 二维向量逆时针旋转
 * - vec2d_to_angle(v): 将二维向量转换为角度
 * - truncate(val, lower, upper): 值钳位函数
 * - normalize_with_bound(val, l1, u1, l2, u2): 区间线性映射
 *   (带钳位功能的仿射变换)
 * - RemapUsingQuadraticFuncAroundSmallValue: 小值附近的二次重映射
 *   使小值区域变化更敏感
 *
 * ==================== normalize_with_bound 详解 ====================
 *
 * 将值从源区间 [lower, upper] 钳位后线性映射到目标区间 [new_lower, new_upper]:
 *   1. val_bounded = truncate(val, lower, upper)
 *   2. ratio = (val_bounded - lower) / (upper - lower)
 *   3. result = new_lower + ratio * (new_upper - new_lower)
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/math/calculations.h"

/*
 * fac - 计算整数n的阶乘 (n!)
 *
 * 对n<=5使用查表法加速（这些是样条计算中最常用的值），
 * 超过5的使用循环计算。
 *
 * @param n 非负整数
 * @return n! (n的阶乘)
 */
long long fac(int n) {
  if (n == 0) return 1;
  if (n == 1) return 1;
  if (n == 2) return 2;
  if (n == 3) return 6;
  if (n == 4) return 24;
  if (n == 5) return 120;

  long long ans = 1;
  for (int i = 1; i <= n; i++) ans *= i;
  return ans;
}

/*
 * nchoosek - 计算二项式系数（组合数）C(n, k)
 *
 * 公式: C(n,k) = n! / (k! * (n-k)!)
 *
 * 用于多项式系数的Bernstein基函数计算、贝塞尔曲线控制点组合等场景。
 *
 * @param n 总元素数
 * @param k 选择元素数
 * @return C(n, k) = n! / (k! * (n-k)!)
 */
long long nchoosek(int n, int k) { return fac(n) / fac(k) / fac(n - k); }

/*
 * normalize_angle - 将角度归一化到 (-pi, pi] 范围内
 *
 * 使用带符号的2pi取模策略（避免循环）：
 *   - 如果 theta >= pi:  theta -= 2*pi
 *   - 如果 theta < -pi:  theta += 2*pi
 *
 * 使用布尔表达式转为整数0/1实现无分支计算：
 *   (theta >= kPi) 结果为1时减去2*pi
 *   (theta < -kPi) 结果为1时加上2*pi
 *
 * @param theta 输入角度（弧度）
 * @return 归一化到(-pi, pi]的角度
 */
decimal_t normalize_angle(const decimal_t& theta) {
  decimal_t theta_tmp = theta;
  // 如果 >= pi，减去 2*pi
  theta_tmp -= (theta >= kPi) * 2 * kPi;
  // 如果 < -pi，加上 2*pi
  theta_tmp += (theta < -kPi) * 2 * kPi;
  return theta_tmp;
}

/*
 * rotate_vector_2d - 将二维向量逆时针旋转指定角度
 *
 * 旋转矩阵: [cos -sin; sin cos]
 * 旋转后: (x*cos(angle) - y*sin(angle), x*sin(angle) + y*cos(angle))
 *
 * @param v 输入的二维向量
 * @param angle 旋转角度（弧度，逆时针为正）
 * @return 旋转后的向量
 */
Vecf<2> rotate_vector_2d(const Vecf<2>& v, const decimal_t angle) {
  return Vecf<2>(v[0] * cos(angle) - v[1] * sin(angle),
                 v[0] * sin(angle) + v[1] * cos(angle));
}

/*
 * vec2d_to_angle - 将二维向量转换为角度（弧度）
 *
 * 使用 atan2(y, x) 计算向量与x轴正方向的夹角。
 * 返回值范围: (-pi, pi]
 *
 * @param v 输入的二维向量 (x, y)
 * @return 向量方向角（弧度）
 */
decimal_t vec2d_to_angle(const Vecf<2>& v) { return atan2(v[1], v[0]); }

/*
 * truncate - 将值钳位到指定范围内
 *
 * 如果 lower > upper，触发断言错误。
 * 否则将 val_in 限制在 [lower, upper] 区间内。
 *
 * @param val_in 输入值
 * @param lower 下界
 * @param upper 上界
 * @return 钳位后的值
 */
decimal_t truncate(const decimal_t& val_in, const decimal_t& lower,
                   const decimal_t& upper) {
  if (lower > upper) {
    printf("[Calculations]Invalid input!\n");
    assert(false);
  }
  decimal_t res = val_in;
  res = std::max(res, lower);
  res = std::min(res, upper);
  return res;
}

/*
 * normalize_with_bound - 将值从一个区间线性映射到另一个区间
 *
 * 功能：将 [lower, upper] 中的值线性映射到 [new_lower, new_upper]。
 * 输入值超出 [lower, upper] 时会被先钳位到该区间。
 *
 * 映射公式:
 *   ratio = (val_bounded - lower) / (upper - lower)
 *   result = new_lower + ratio * (new_upper - new_lower)
 *
 * 使用场景示例：
 *   将MOBIL增益值从 [-1, 6] 标准化到 [0, 1] 作为概率值
 *
 * @param val_in 输入值
 * @param lower 源区间下界
 * @param upper 源区间上界
 * @param new_lower 目标区间下界
 * @param new_upper 目标区间上界
 * @return 映射后的值
 */
decimal_t normalize_with_bound(const decimal_t& val_in, const decimal_t& lower,
                               const decimal_t& upper,
                               const decimal_t& new_lower,
                               const decimal_t& new_upper) {
  if (new_lower > new_upper) {
    printf("[Calculations]Invalid input!\n");
    assert(false);
  }
  decimal_t val_bounded = truncate(val_in, lower, upper);  // 钳位
  decimal_t ratio = (val_bounded - lower) / (upper - lower);  // 归一化到[0,1]
  decimal_t res = new_lower + (new_upper - new_lower) * ratio;  // 映射
  return res;
}

/*
 * RemapUsingQuadraticFuncAroundSmallValue - 小值附近的二次函数重映射
 *
 * 目的：使小值区域对变化更敏感（放大微小变化），大值区域保持线性。
 *
 * 重映射策略：
 *   如果 |val_in| <= |th|:  使用二次函数 y = sign(val) * c * x^2
 *      其中 c = 1/th，保证在 x=th 处连续
 *   如果 |val_in| > |th|:   保持线性 y = x
 *
 * 分段函数在 |th| 处连续: c * th^2 = (1/th) * th^2 = th
 *
 * 典型应用：在运动规划中，对于小的横向偏差（如<0.1m），
 * 通过二次映射将该偏差放大，使优化器更加重视
 *
 * @param th 阈值（重映射的分界点）
 * @param val_in 输入值
 * @param val_out 输出参数，重映射后的值
 * @return kSuccess
 */
ErrorType RemapUsingQuadraticFuncAroundSmallValue(const decimal_t& th,
                                                  const decimal_t& val_in,
                                                  decimal_t* val_out) {
  decimal_t c = 1.0 / th;  // 二次项系数，保证在 th 处连续
  if (fabs(val_in) <= fabs(th)) {
    // 小值区域：二次映射 y = sign(x) * (1/th) * x^2
    *val_out = sgn(val_in) * c * val_in * val_in;
  } else {
    // 大值区域：直接返回
    *val_out = val_in;
  }
  return kSuccess;
}
