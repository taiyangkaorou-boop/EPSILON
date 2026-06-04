/**
 * @file calculations.h
 * @brief 数学计算工具函数集合
 *
 * 提供 EPSILON 项目中广泛使用的基础数学工具函数，涵盖：
 *   - 符号函数（sgn）
 *   - 组合数学（阶乘 fac、组合数 nchoosek）
 *   - 角度归一化（normalize_angle）
 *   - 二维向量旋转（rotate_vector_2d）
 *   - 向量到角度的转换（vec2d_to_angle）
 *   - 数值截断与归一化（truncate、normalize_with_bound）
 *   - 小值区域的二次函数重映射（RemapUsingQuadraticFuncAroundSmallValue）
 *
 * 这些工具函数被规划器（BehaviorPlanner、EudmPlanner、SscPlanner）
 * 及其依赖模块（Chassis、StateTransformer 等）广泛调用。
 *
 * @author EPSILON Autonomous Driving Team
 * @date 2026
 */

#ifndef _CORE_COMMON_INC_COMMON_MATH_CALCULATIONS_H__
#define _CORE_COMMON_INC_COMMON_MATH_CALCULATIONS_H__

#include "common/basics/basics.h"

/**
 * @brief 符号函数（Sign Function）
 *
 * 返回输入值的符号：
 *   - val > 0 时返回 +1
 *   - val = 0 时返回 0
 *   - val < 0 时返回 -1
 *
 * @tparam T 数值类型（int, float, double 等）
 * @param val 输入值
 * @return int 符号值（-1, 0, +1）
 *
 * @note 该实现使用无分支的比较技巧，避免条件跳转，提高性能
 */
template <typename T>
int sgn(const T val) {
  return (T(0) < val) - (val < T(0));
}

/**
 * @brief 计算阶乘（Factorial）
 *
 * 计算 n! = 1 * 2 * 3 * ... * n
 *
 * @param n 输入值（非负整数）
 * @return long long 阶乘结果 n!
 *
 * @note n 较大时结果可能溢出，请确保 n <= 20（对 64 位整数而言）
 */
long long fac(int n);

/**
 * @brief 计算组合数（Combinations）
 *
 * 计算 C(n, k) = n! / (k! * (n-k)!)，即从 n 个元素中选 k 个的方案数。
 *
 * @param n 总元素数
 * @param k 选择的元素数
 * @return long long 组合数 C(n, k)
 *
 * @note 当 k > n 或 k < 0 时，返回 0
 * @note 常用于贝塞尔曲线和二项式展开计算
 */
long long nchoosek(int n, int k);

/**
 * @brief 角度归一化到 [-pi, pi) 区间
 *
 * 将任意角度值归一化到 [-pi, pi) 的半开区间内。
 * 这是处理角度相关运算的标准预处理步骤。
 *
 * @param theta 输入角度 [rad]
 * @return decimal_t 归一化后的角度 [rad]
 *
 * @note 对 pi 的整倍数边界情况，返回 -pi（而非 +pi）
 */
decimal_t normalize_angle(const decimal_t& theta);

/**
 * @brief 二维向量逆时针旋转
 *
 * 将二维向量按给定角度进行逆时针旋转。
 * 旋转矩阵：R(angle) = [[cos, -sin], [sin, cos]]
 *
 * @param v 输入二维向量 (x, y)
 * @param angle 逆时针旋转角度 [rad]
 * @return Vecf<2> 旋转后的二维向量
 *
 * @note 用于 Frenet 坐标与笛卡尔坐标之间的法线/切线方向转换
 */
Vecf<2> rotate_vector_2d(const Vecf<2>& v, const decimal_t angle);

/**
 * @brief 二维向量方向角计算
 *
 * 计算向量与 x 轴正方向之间的夹角（使用 atan2）。
 *
 * @param v 输入二维向量 (x, y)
 * @return decimal_t 向量方向角 [rad]，范围 (-pi, pi]
 */
decimal_t vec2d_to_angle(const Vecf<2>& v);

/**
 * @brief 数值截断（Clamp）
 *
 * 将输入值限制在 [lower, upper] 范围内。
 *   - val < lower 时返回 lower
 *   - val > upper 时返回 upper
 *   - 否则返回 val
 *
 * @param val_in 输入值
 * @param lower 下界
 * @param upper 上界
 * @return decimal_t 截断后的值
 *
 * @note 如果 lower > upper，行为未定义
 */
decimal_t truncate(const decimal_t& val_in, const decimal_t& lower,
                   const decimal_t& upper);

/**
 * @brief 带边界映射的数值归一化
 *
 * 将 val_in 从 [lower, upper] 线性映射到 [new_lower, new_upper]，
 * 超出范围的值先被截断。
 *
 * @param val_in 输入值
 * @param lower 原始下界
 * @param upper 原始上界
 * @param new_lower 目标下界
 * @param new_upper 目标上界
 * @return decimal_t 归一化后的值
 *
 * @note 常用于将物理量映射到 [0, 1] 区间用于代价评估
 */
decimal_t normalize_with_bound(const decimal_t& val_in, const decimal_t& lower,
                               const decimal_t& upper,
                               const decimal_t& new_lower,
                               const decimal_t& new_upper);

/**
 * @brief 小值区域的二次函数重映射
 *
 * 使用二次函数在阈值附近的狭窄区间内进行平滑过渡，避免在
 * 小值区域出现不连续的梯度跳变。常用于代价函数中的平滑处理。
 *
 * @param th 过渡区域的半宽阈值
 * @param val_in 输入值
 * @param[out] val_out 重映射后的输出值
 * @return ErrorType
 *
 * @note 当 |val_in| <= th 时，使用二次函数插值；
 *       当 |val_in| > th 时，输出等于输入（恒等映射）
 */
ErrorType RemapUsingQuadraticFuncAroundSmallValue(const decimal_t& th,
                                                  const decimal_t& val_in,
                                                  decimal_t* val_out);

#endif  // _CORE_COMMON_INC_COMMON_MATH_CALCULATIONS_H__
