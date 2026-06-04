/**
 * @file config.h
 * @author HKUST Aerial Robotics Group
 * @brief EPSILON 自动驾驶决策规划系统 — 全局编译期配置宏定义
 *
 * @details
 * 本文件定义了 EPSILON 系统中在编译期就确定的全局配置常量。
 * 这些宏定义了车道和轨迹的多项式拟合参数，直接决定了曲线的表达能力和计算复杂度。
 *
 * 参数含义：
 *   - LaneDegree：车道中心线多项式拟合的次数（5次多项式可表达 S 弯等复杂曲线）
 *   - LaneDim：车道曲线参数化空间的维度（2维 = x-y 平面）
 *   - TrajectoryDegree：轨迹多项式拟合的次数（5次多项式平衡灵活性与数值稳定性）
 *   - TrajectoryDim：轨迹参数化空间的维度（2维 = x-y 平面）
 *
 * @note 修改这些宏会导致所有依赖曲率计算的代码重新编译，且会影响运行时行为。
 *       增大 Degree 会增加曲线表达能力但可能引入过拟合（Runge 现象）；
 *       减小 Degree 会降低灵活性但更稳定。
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_COMMON_INC_BASICS_CONFIG_H__
#define _CORE_COMMON_INC_BASICS_CONFIG_H__

/// @brief 车道中心线的多项式拟合次数（5次多项式可表达 S 弯等复杂道路曲率变化）
#define LaneDegree 5
/// @brief 车道曲线的参数空间维度（2 表示二维平面 x-y）
#define LaneDim 2
/// @brief 轨迹的多项式拟合次数（5次多项式在灵活性和数值稳定性之间取得平衡）
#define TrajectoryDegree 5
/// @brief 轨迹的参数空间维度（2 表示二维平面 x-y）
#define TrajectoryDim 2

#endif  // _CORE_COMMON_INC_BASICS_CONFIG_H__
