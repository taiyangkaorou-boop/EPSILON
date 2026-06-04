/**
 * @file free_state.cc
 * @brief 自由状态（FreeState）与通用状态（State）之间的双向转换实现
 *
 * 本文件实现了EPSILON自动驾驶规划系统中两种状态表示之间的转换：
 * State（极坐标风格表示）和 FreeState（笛卡尔分量表示）。
 *
 * ==================== 两种状态表示的区别 ====================
 *
 * State 表示法（适用于自行车模型）:
 *   - position: (x, y) 位置坐标（通常以后轴中心为参考点）
 *   - angle: 朝向角（弧度）
 *   - velocity: 标量速度大小（沿朝向方向）
 *   - curvature: 路径曲率 = 1/转弯半径
 *   - acceleration: 标量加速度（沿朝向方向）
 *
 * FreeState 表示法（适用于自由二维运动）:
 *   - position: (x, y) 位置坐标
 *   - velocity: (vx, vy) 速度的分量形式
 *   - acceleration: (ax, ay) 加速度的分量形式
 *   - angle: 朝向角
 *
 * ==================== 转换公式 ====================
 *
 * State → FreeState:
 *   vx = v * cos(theta)
 *   vy = v * sin(theta)
 *   a_normal = v^2 * kappa              （向心加速度）
 *   ax = a_long * cos(theta) - a_normal * sin(theta)
 *   ay = a_long * sin(theta) + a_normal * cos(theta)
 *
 * FreeState → State:
 *   v = sqrt(vx^2 + vy^2)
 *   a_long = ax * cos(theta) + ay * sin(theta)  （切向加速度分量）
 *   a_normal = -ax * sin(theta) + ay * cos(theta) （法向加速度分量）
 *   kappa = a_normal / v^2  （由法向加速度反推曲率）
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/state/free_state.h"

#include <math.h>
namespace common {

/*
 * GetFreeStateFromState - 将自行车模型状态转换为自由运动状态
 *
 * State 使用极坐标风格的描述（速度幅值+朝向），适用于自行车模型。
 * FreeState 使用笛卡尔分量描述，适用于无约束的二维运动。
 *
 * 转换过程：
 * 1. 位置直接复制
 * 2. 速度：将标量速度沿朝向方向分解为 (v*cos(theta), v*sin(theta))
 * 3. 加速度：标量加速度分解为切向和法向分量
 *    - 切向加速度(沿朝向): a_long * (cos(theta), sin(theta))
 *    - 法向加速度(垂直朝向): v^2 * kappa * (-sin(theta), cos(theta))
 *    - 总加速度 = 切向 + 法向
 *
 * @param state 输入的自行车模型状态（含标量速度和曲率）
 * @param free_state 输出参数，自由运动状态（含速度/加速度的分量形式）
 */
void GetFreeStateFromState(const State& state, FreeState* free_state) {
  free_state->position = state.vec_position;
  decimal_t cn = cos(state.angle);
  decimal_t sn = sin(state.angle);
  // 速度在全局坐标系中的分量
  free_state->velocity[0] = state.velocity * cn;
  free_state->velocity[1] = state.velocity * sn;
  // 向心加速度: a_normal = v * v * kappa（沿法线方向）
  decimal_t normal_acc = state.velocity * state.velocity * state.curvature;
  // 总加速度 = 切向分量 + 法向分量的矢量和
  free_state->acceleration[0] = state.acceleration * cn - normal_acc * sn;
  free_state->acceleration[1] = state.acceleration * sn + normal_acc * cn;
  free_state->angle = state.angle;
  free_state->time_stamp = state.time_stamp;
}

/*
 * GetStateFromFreeState - 将自由运动状态转换回自行车模型状态
 *
 * 从速度/加速度的笛卡尔分量形式恢复为标量形式。
 *
 * 转换过程：
 * 1. 角度和位置直接复制
 * 2. 速度：取速度向量的模（sqrt(vx^2 + vy^2)）
 * 3. 加速度：
 *    - 切向加速度 = a dot t（加速度在朝向方向上的投影）
 *    - 法向加速度 = a dot n（加速度在法向上的投影）
 *    - 曲率 = 法向加速度 / 速度²（当速度非零时）
 *      * 当速度接近零时，曲率设为0（避免除零错误）
 *
 * @param free_state 输入的自由运动状态
 * @param state 输出参数，自行车模型状态
 */
void GetStateFromFreeState(const FreeState& free_state, State* state) {
  state->angle = free_state.angle;
  state->vec_position = free_state.position;
  state->velocity = free_state.velocity.norm();
  decimal_t cn = cos(state->angle);
  decimal_t sn = sin(state->angle);
  // 朝向方向的单位切向量和单位法向量
  Vecf<2> tangent_vec{Vecf<2>(cn, sn)};      // 沿朝向方向
  Vecf<2> normal_vec{Vecf<2>(-sn, cn)};      // 垂直于朝向方向（左侧）
  auto a_tangent = free_state.acceleration.dot(tangent_vec);  // 切向加速度
  auto a_normal = free_state.acceleration.dot(normal_vec);    // 法向加速度
  state->acceleration = a_tangent;
  if (fabs(state->velocity) > kBigEPS) {
    // 由法向加速度反推曲率: kappa = a_normal / v^2
    state->curvature = a_normal / pow(state->velocity, 2);
  } else {
    state->curvature = 0.0;  // 速度为零时曲率无定义，设为0
  }
  state->time_stamp = free_state.time_stamp;
}

}  // namespace common
