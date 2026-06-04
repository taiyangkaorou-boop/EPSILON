/**
 * @file free_state.h
 * @brief 自由空间下的车辆运动状态表示
 *
 * 本文件定义了 EPSILON 自动驾驶规划系统中自由空间（Free Space）下的
 * 车辆状态结构体 FreeState。自由空间状态与笛卡尔状态 State 的区别在于：
 *
 *   - FreeState 使用二维向量（Vec2f）表示速度（velocity）和加速度（acceleration），
 *     而 State 使用标量表示沿车辆朝向方向的速度和加速度。
 *   - FreeState 不包含 curvature（曲率）和 steer（转向角）信息，仅关注
 *     车辆质心在二维平面中的位置、速度和加速度矢量。
 *
 * FreeState 主要用于非结构化场景（如停车场、交叉路口等无车道线参考的场景）
 * 中的路径规划和轨迹生成。在此类场景中，车辆的运动学约束较为宽松，
 * 不需要严格沿车道线行驶，因此用二维向量描述运动状态更为合适。
 *
 * 本文件还提供了 FreeState 与 State 之间的双向转换函数，转换过程中
 * 将标量速度/加速度分解为二维分量（或反向合成）。
 */

#ifndef _COMMON_INC_COMMON_STATE_FREE_STATE_H__
#define _COMMON_INC_COMMON_STATE_FREE_STATE_H__

#include "common/basics/basics.h"
#include "common/state/state.h"

#include <math.h>

namespace common {

/**
 * @struct FreeState
 * @brief 自由空间下的车辆运动状态结构体
 *
 * 与 State 结构体相比，FreeState 使用二维向量表示速度和加速度，
 * 不包含曲率和转向角信息。这种表示方式适用于无车道线约束的
 * 非结构化环境（如停车场、开放区域等），此时车辆的运动方向
 * 不再被限制在沿车体朝向的方向上。
 *
 * 字段说明：
 *   - position: 二维位置向量 (x, y)，单位：m
 *   - velocity: 二维速度向量 (vx, vy)，单位：m/s
 *   - acceleration: 二维加速度向量 (ax, ay)，单位：m/s^2
 *   - angle: 车辆朝向角，单位：rad
 */
struct FreeState {
  /// 时间戳，记录该状态对应的时刻（秒）
  decimal_t time_stamp{0.0};

  /// 二维位置向量，表示车辆在全局坐标系中的 (x, y) 坐标，单位：m
  Vecf<2> position{Vecf<2>::Zero()};

  /// 二维速度向量 (vx, vy)，表示车辆在 x 和 y 方向的速度分量，单位：m/s
  Vecf<2> velocity{Vecf<2>::Zero()};

  /// 二维加速度向量 (ax, ay)，表示车辆在 x 和 y 方向的加速度分量，单位：m/s^2
  Vecf<2> acceleration{Vecf<2>::Zero()};

  /// 车辆朝向角，单位：rad
  decimal_t angle{0.0};

  /**
   * @brief 打印自由空间状态详情到标准输出
   */
  void print() const {
    printf("position: (%lf, %lf).\n", position[0], position[1]);
    printf("velocity: (%lf, %lf).\n", velocity[0], velocity[1]);
    printf("acceleration: (%lf, %lf).\n", acceleration[0], acceleration[1]);
    printf("angle: %lf.\n", angle);
  }
};

/**
 * @brief 将笛卡尔状态（State）转换为自由空间状态（FreeState）
 *
 * 转换过程中，State 的标量速度沿 angle 方向分解为 velocity 的二维分量，
 * 标量加速度沿 angle 方向分解为 acceleration 的二维分量。
 *
 * @param state 输入的笛卡尔坐标系状态（标量速度/加速度）
 * @param free_state 输出的自由空间状态（矢量速度/加速度）
 */
void GetFreeStateFromState(const State& state, FreeState* free_state);

/**
 * @brief 将自由空间状态（FreeState）转换为笛卡尔状态（State）
 *
 * 转换过程中，FreeState 的二维速度向量投影到 angle 方向得到标量速度，
 * 二维加速度向量投影到 angle 方向得到标量加速度。
 *
 * @param free_state 输入的自由空间状态（矢量速度/加速度）
 * @param state 输出的笛卡尔坐标系状态（标量速度/加速度）
 */
void GetStateFromFreeState(const FreeState& free_state, State* state);

}  // namespace common

#endif  // _COMMON_INC_COMMON_STATE_FREE_STATE_H__
