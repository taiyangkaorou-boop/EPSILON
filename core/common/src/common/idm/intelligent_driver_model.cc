/**
 * @file intelligent_driver_model.cc
 * @brief 智能驾驶员模型（IDM, Intelligent Driver Model）及改进版本的实现文件
 *
 * 本文件实现了EPSILON自动驾驶规划系统中的车辆跟驰（car-following）模型，
 * 包括原始IDM模型、改进版IIDM模型，以及结合了CAH策略的ACC模型。
 *
 * ==================== IDM模型简介 ====================
 *
 * 智能驾驶员模型（Intelligent Driver Model, IDM）是由Martin Treiber等人
 * 提出的车辆跟驰模型，能描述人类驾驶员的加速度行为。
 *
 * IDM的加速度公式：
 *   a = a_max * (1 - (v/v0)^delta - (s*(v, dv)/s_alpha)^2)
 *
 * 其中：
 *   - a_max: 最大加速度
 *   - v0: 期望速度
 *   - delta: 加速度指数（通常取4）
 *   - s*(v, dv): 期望安全跟随距离
 *     = s0 + max(0, v*T + v*dv/(2*sqrt(a_max*b)))
 *     * s0: 最小停车距离
 *     * T: 期望车头时距
 *     * b: 舒适制动减速度
 *   - s_alpha: 实际净距（前车距离 - 前车长度 - 后车长度）
 *
 * ==================== IIDM改进 ====================
 *
 * IIDM（Improved IDM）修正了原始IDM的两个缺陷：
 * 1. 超速时的减速度过大（特别是delta较大时）
 * 2. 低接近期望速度时稳态车间距过大
 *
 * IIDM的策略：
 *   - 自加速项 a_free：v <= v0时用IDM公式，v > v0时独立处理
 *   - 制动项：当z>=1时用IDM形式，否则用 a_free * (1 - z^(2*amax/a_free))
 *   - 最终加速度限制在 [amax, -hard_brake] 范围内
 *
 * ==================== ACC（自适应巡航控制）策略 ====================
 *
 * 结合了 IIDM 和 CAH（Constant-Acceleration Heuristic）的混合策略：
 *   - IIDM 提供平滑的跟驰加速度
 *   - CAH 提供基于恒定加速度假设的碰撞避免加速度
 *   - 最终输出 = min(IIDM, CAH - b * tanh((IIDM-CAH)/b))
 *     平滑地过渡到更保守的CAH加速度
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/idm/intelligent_driver_model.h"

namespace common {

/*
 * GetIdmDesiredAcceleration - 计算原始IDM模型下的期望加速度
 *
 * IDM加速度公式（原始版本）:
 *   a = a_max * (1 - (v/v0)^delta - (s* / s_alpha)^2)
 *
 * 期望安全距离 s*:
 *   s* = s0 + max(0, v*T + v*(v-v_front)/(2*sqrt(a_max*b)))
 *
 * 参数含义：
 *   - v/v0 项: 加速项，当前速度越接近期望速度，加速度越小（趋于0）
 *   - s*/s_alpha 项: 制动项，当前车间距越小于期望距离，减速越大
 *   - delta 指数(通常4): 控制加速度衰减的陡峭程度
 *
 * @param param IDM模型参数（期望速度、最大加速度、安全距离等）
 * @param cur_state 当前跟驰状态（本车位置/速度，前车位置/速度）
 * @param acc 输出参数，IDM期望加速度 (m/s^2)
 * @return kSuccess
 */
ErrorType IntelligentDriverModel::GetIdmDesiredAcceleration(
    const IntelligentDriverModel::Param &param,
    const IntelligentDriverModel::State &cur_state, decimal_t *acc) {
  // 计算期望安全间距 s*
  // s* = s0 + v*T + v*(v-vf)/(2*sqrt(a*b))
  decimal_t s_star =
      param.kMinimumSpacing +
      std::max(0.0,
               cur_state.v * param.kDesiredHeadwayTime +
                   cur_state.v * (cur_state.v - cur_state.v_front) /
                       (2.0 * sqrt(param.kAcceleration *
                                   param.kComfortableBrakingDeceleration)));
  // 计算实际净距 s_alpha = max(0, sf - s - L)
  decimal_t s_alpha =
      std::max(0.0, cur_state.s_front - cur_state.s - param.kVehicleLength);
  // IDM加速度公式
  *acc = param.kAcceleration *
         (1.0 - pow(cur_state.v / param.kDesiredVelocity, param.kExponent) -
          pow(s_star / s_alpha, 2));
  return kSuccess;
}

/*
 * GetIIdmDesiredAcceleration - 计算改进版IDM（IIDM）模型的期望加速度
 *
 * IIDM相对于原始IDM的改进：
 *
 * 1. 自由加速项 a_free 的分段处理:
 *    - 如果 v <= v0: a_free = a_max * (1 - (v/v0)^delta)     （和IDM相同）
 *    - 如果 v > v0:  a_free = -b * (1 - (v0/v)^(a_max*delta/b))
 *      避免超速时减速度过大的问题
 *
 * 2. 跟驰制动项的改进:
 *    定义 z = s* / s_alpha（无量纲的跟车紧密程度）
 *    - 如果 z >= 1 (实际间距小于期望间距):
 *      - v <= v0: a = a_max * (1 - z^2)
 *      - v > v0:  a = a_free + a_max * (1 - z^2)
 *    - 如果 z < 1 (实际间距大于期望间距):
 *      - v <= v0: a = a_free * (1 - z^(2*a_max/a_free))
 *      - v > v0:  a = a_free   （不需要额外制动）
 *
 * 3. 最终加速度限制在 [a_max, -hard_brake] 范围内
 *
 * @param param IDM模型参数
 * @param cur_state 当前跟驰状态
 * @param acc 输出参数，IIDM期望加速度
 * @return kSuccess
 */
ErrorType IntelligentDriverModel::GetIIdmDesiredAcceleration(
    const IntelligentDriverModel::Param &param,
    const IntelligentDriverModel::State &cur_state, decimal_t *acc) {
  /*
   * 自由加速项 a_free:
   * - 速度未超期望速度时，使用标准IDM加速公式
   * - 速度超过期望速度时，使用修正的减速公式
   */
  decimal_t a_free =
      cur_state.v <= param.kDesiredVelocity
          ? param.kAcceleration *
                (1 - pow(cur_state.v / param.kDesiredVelocity, param.kExponent))
          : -param.kComfortableBrakingDeceleration *
                (1 - pow(param.kDesiredVelocity / cur_state.v,
                         param.kAcceleration * param.kExponent /
                             param.kComfortableBrakingDeceleration));
  // 计算实际净距
  decimal_t s_alpha =
      std::max(0.0, cur_state.s_front - cur_state.s - param.kVehicleLength);
  // 计算无量纲跟车紧密程度 z = s* / s_alpha
  decimal_t z =
      (param.kMinimumSpacing +
       std::max(0.0,
                cur_state.v * param.kDesiredHeadwayTime +
                    cur_state.v * (cur_state.v - cur_state.v_front) /
                        (2.0 * sqrt(param.kAcceleration *
                                    param.kComfortableBrakingDeceleration)))) /
      s_alpha;
  // 根据z值和期望速度情况计算输出加速度
  decimal_t a_out =
      cur_state.v <= param.kDesiredVelocity
          ? (z >= 1.0
                 ? param.kAcceleration * (1 - pow(z, 2))
                 : a_free * (1 - pow(z, 2.0 * param.kAcceleration / a_free)))
          : (z >= 1.0 ? a_free + param.kAcceleration * (1 - pow(z, 2))
                      : a_free);
  // 将加速度钳位到 [a_max, -hard_brake] 范围内
  a_out = std::max(std::min(param.kAcceleration, a_out),
                   -param.kHardBrakingDeceleration);
  *acc = a_out;
  return kSuccess;
}

/*
 * GetAccDesiredAcceleration - 计算ACC模式下的期望加速度
 *
 * ACC（自适应巡航控制）策略结合了IIDM和CAH两种方法：
 *
 * IIDM贡献：
 *   - 提供符合人类驾驶习惯的平滑跟驰加速度
 *
 * CAH（Constant-Acceleration Heuristic）贡献：
 *   - 基于前车以恒定减速度制动的假设，计算安全加速度
 *   - 公式: acc_cah = v^2 * (-b) / (vf^2 - 2*ds*(-b))
 *     其中 -b 假设前车以舒适制动减速度减速
 *
 * 平滑过渡策略（使用 coolness 系数 0.99）：
 *   如果 acc_iidm >= acc_cah:
 *     acc = acc_iidm    （IIDM的加速度就已经足够安全）
 *   否则:
 *     acc = (1-c) * acc_iidm + c * (acc_cah - b * tanh((acc_iidm - acc_cah)/-b))
 *     这个混合形式确保在需要更强制动时平滑过渡到CAH加速度
 *
 * @param param IDM模型参数
 * @param cur_state 当前跟驰状态
 * @param acc 输出参数，ACC期望加速度
 * @return kSuccess
 */
ErrorType IntelligentDriverModel::GetAccDesiredAcceleration(
    const IntelligentDriverModel::Param &param,
    const IntelligentDriverModel::State &cur_state, decimal_t *acc) {
  decimal_t acc_iidm;
  GetIIdmDesiredAcceleration(param, cur_state, &acc_iidm);

  /*
   * CAH加速度计算：
   * 假设前车以 -kComfortableBrakingDeceleration 的减速度恒减速，
   * 计算当前速度下不与前车碰撞的最大安全加速度。
   */
  decimal_t ds = std::max(0.0, cur_state.s_front - cur_state.s);
  decimal_t acc_cah =
      (cur_state.v * cur_state.v * -param.kComfortableBrakingDeceleration) /
      (cur_state.v_front * cur_state.v_front -
       2 * ds * -param.kComfortableBrakingDeceleration);

  // coolness=0.99 表示混合时强烈偏向CAH（仅0.01保留IIDM分量）
  decimal_t coolness = 0.99;

  if (acc_iidm >= acc_cah) {
    // IIDM加速度已经足够（不小于CAH），直接使用IIDM
    *acc = acc_iidm;
  } else {
    // IIDM加速度过于激进（小于CAH即需要更大减速），混合IIDM和CAH
    // 用 tanh 实现平滑过渡
    *acc =
        (1 - coolness) * acc_iidm +
        coolness * (acc_cah - param.kComfortableBrakingDeceleration *
                                  tanh((acc_iidm - acc_cah) /
                                       -param.kComfortableBrakingDeceleration));
  }
  return kSuccess;
}

}  // namespace common
