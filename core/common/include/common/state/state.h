/**
 * @file state.h
 * @brief 车辆笛卡尔坐标系下的状态表示
 *
 * 本文件定义了 EPSILON 自动驾驶规划系统中车辆在笛卡尔坐标系下的状态结构体。
 * 笛卡尔状态是系统中最基础的状态表示形式，描述了车辆在世界坐标系（XY平面）中
 * 的位姿（position、angle）、运动学量（velocity、acceleration、steer）以及
 * 几何量（curvature）。
 *
 * 在规划系统中，State 作为核心数据结构之一，与 FrenetState（Frenet坐标系状态）
 * 和 FreeState（自由空间状态）相互转换，共同支撑轨迹规划、碰撞检测和控制等
 * 核心模块的运作。
 *
 * 坐标系约定：
 *   - 位置 (vec_position): 车辆后轴中心在 XY 平面中的坐标，单位：m
 *   - 朝向角 (angle): 车辆朝向与 X 轴正方向的夹角，单位：rad
 *   - 曲率 (curvature): 车辆行驶轨迹的瞬时曲率，单位：1/m，正值表示左转
 */

#ifndef _COMMON_INC_COMMON_STATE_STATE_H__
#define _COMMON_INC_COMMON_STATE_STATE_H__

#include "common/basics/basics.h"

namespace common {

/**
 * @struct State
 * @brief 笛卡尔坐标系下的车辆状态结构体
 *
 * 该结构体封装了车辆在二维笛卡尔空间中的完整运动学状态信息。
 * 所有状态量均在一个确定的时间戳上定义，支持从不同时间尺度进行
 * 状态追踪与插值。
 *
 * 各字段的物理含义与默认值：
 *   - time_stamp: 状态对应的时间戳（秒），默认 0.0
 *   - vec_position: 二维位置向量 (x, y)，单位 m，默认 (0, 0)
 *   - angle: 航向角，单位 rad，默认 0.0
 *   - curvature: 路径曲率，单位 1/m，默认 0.0
 *   - velocity: 线速度（沿车辆朝向方向），单位 m/s，默认 0.0
 *   - acceleration: 线加速度，单位 m/s^2，默认 0.0
 *   - steer: 前轮转向角，单位 rad，默认 0.0
 */
struct State {
  /// 时间戳，记录该状态对应的时刻（秒）
  decimal_t time_stamp{0.0};

  /// 二维位置向量，表示车辆后轴中心在全局坐标系中的 (x, y) 坐标，单位：m
  Vecf<2> vec_position{Vecf<2>::Zero()};

  /// 航向角，车辆朝向与 X 轴正方向之间的夹角，单位：rad
  decimal_t angle{0.0};

  /// 路径曲率，车辆当前行驶轨迹的瞬时曲率值，单位：1/m（0 表示直行）
  decimal_t curvature{0.0};

  /// 线速度，沿车辆朝向方向的标量速度，单位：m/s
  decimal_t velocity{0.0};

  /// 线加速度，沿车辆朝向方向的标量加速度，单位：m/s^2
  decimal_t acceleration{0.0};

  /// 前轮转向角，单位：rad（正值表示左转，负值表示右转）
  decimal_t steer{0.0};

  /**
   * @brief 打印状态信息到标准输出
   * @note 主要用于调试目的，将所有字段值格式化输出
   */
  void print() const {
    printf("State:\n");
    printf(" -- time_stamp: %lf.\n", time_stamp);
    printf(" -- vec_position: (%lf, %lf).\n", vec_position[0], vec_position[1]);
    printf(" -- angle: %lf.\n", angle);
    printf(" -- curvature: %lf.\n", curvature);
    printf(" -- velocity: %lf.\n", velocity);
    printf(" -- acceleration: %lf.\n", acceleration);
    printf(" -- steer: %lf.\n", steer);
  }

  /**
   * @brief 将状态转换为 (x, y, theta) 三元组向量
   * @return Vec3f 包含 (pos_x, pos_y, angle) 的三维向量
   * @note 该函数提供了一种便捷的格式转换，便于与其他仅需要位置和朝向的模块对接
   */
  Vec3f ToXYTheta() const {
    return Vec3f(vec_position(0), vec_position(1), angle);
  }

  /// Eigen 库内存对齐宏，确保在 STL 容器中安全使用
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace common

#endif  // _COMMON_INC_COMMON_STATE_STATE_H__
