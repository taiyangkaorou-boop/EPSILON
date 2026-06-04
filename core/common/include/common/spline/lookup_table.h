/**
 * @file lookup_table.h
 * @brief 系数求解矩阵逆的查表工具
 *
 * 本文件定义了 EPSILON 自动驾驶规划系统中用于加速 Jerk 最优多项式
 * 求解的查表工具。
 *
 * 核心问题背景：
 *   在轨迹规划中，经常需要求解"最小 jerk 五次多项式"连接问题：
 *   给定起始和终止的位置、速度、加速度以及时间 S，求解一个五次多项式
 *   使得 jerk 的平方积分最小。这个问题等价于求解线性方程组：
 *     A(S) * c = b
 *   其中 A(S) 是一个 6x6 矩阵，其元素是时间 S 的多项式函数，
 *   b 是边界条件向量。
 *
 *   由于 A(S) 只依赖于时间 S，因此：
 *     - 对于离散的、重复出现的 S 值，可以预计算 A(S) 的逆矩阵并缓存
 *     - 在运行时直接查表读取，避免重复求解 6x6 线性系统
 *     - 轨迹规划中 S 值通常取离散值（如固定的采样时间步长），查表命中率很高
 *
 * 查表数据结构：
 *   kTableAInverse 是一个 std::map<decimal_t, MatNf<6>>，
 *   以时间 S 为键，以 A 矩阵的逆（6x6 矩阵）为值。
 *   使用 Eigen 的对齐分配器 (aligned_allocator) 保证 Eigen 内存对齐。
 *
 * GetAInverse(t):
 *   对于任意时间 t（不必是离散值），实时计算 A 矩阵的逆。
 *   在查表未命中时作为回退方案。
 */

#ifndef _COMMON_INC_COMMON_SPLINE_LOOKUP_TABLE_H__
#define _COMMON_INC_COMMON_SPLINE_LOOKUP_TABLE_H__

#include <map>
#include "common/basics/basics.h"
#include "common/math/calculations.h"

namespace common {

/**
 * @brief 计算系数空间到导数空间的映射矩阵 A 的逆矩阵
 *
 * 对于五次多项式的 jerk 最优连接问题（给定 p1, v1, a1, p2, v2, a2, S）：
 *
 * 多项式表示（反向系数形式）：
 *   f(s) = c0/5! * s^5 + c1/4! * s^4 + c2/3! * s^3 + c3/2! * s^2 + c4 * s + c5
 *
 * 边界条件矩阵方程 A(t) * c = b：
 *   A(t) 建立了系数向量 c = [c0, c1, c2, c3, c4, c5]^T 与
 *   边界条件向量 b = [f(0), f'(0), f''(0), f(t), f'(t), f''(t)]^T 之间的映射。
 *
 * GetAInverse(t) 返回 A(t) 的逆矩阵，使得 c = A^{-1} * b。
 *
 * 该函数在查表不命中时被调用，对于频繁使用的 t 值，建议将其加入
 * kTableAInverse 查表以提升运行效率。
 *
 * @param t 时间长度 S
 * @return 6x6 矩阵 A(t) 的逆矩阵
 */
MatNf<6> GetAInverse(decimal_t t);

/**
 * @brief A 逆矩阵的全局查表
 *
 * 键：时间长度 S (decimal_t)
 * 值：6x6 矩阵 A(S) 的逆 (MatNf<6>)
 *
 * 使用 std::map 存储（基于红黑树的有序映射），配合 std::less<decimal_t>
 * 比较器和 Eigen 的对齐分配器 (aligned_allocator)，确保 Eigen 矩阵在
 * 容器中的内存对齐。
 *
 * 在系统初始化或首次运行时，应预填充该表格以包含常用的采样时间值。
 */
extern std::map<decimal_t, MatNf<6>, std::less<decimal_t>,
                Eigen::aligned_allocator<std::pair<const decimal_t, MatNf<6>>>>
    kTableAInverse;

}  // namespace common

#endif
