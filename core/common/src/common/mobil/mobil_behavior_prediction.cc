/**
 * @file mobil_behavior_prediction.cc
 * @brief 基于MOBIL模型的车道变更行为预测实现文件
 *
 * 本文件实现了EPSILON自动驾驶规划系统中的MobilBehaviorPrediction类，
 * 它使用MOBIL模型预测其他车辆在下一时刻的车道变更行为，
 * 将MOBIL的加速度增益转换为车道变更的概率分布。
 *
 * ==================== MOBIL行为预测流程 ====================
 *
 * 1. 对当前车道(t)，计算本车加速度 acc_c 以及后车状态
 * 2. 对左车道(l)，计算MOBIL增益 mobil_gain_left (如果安全)
 * 3. 对右车道(r)，计算MOBIL增益 mobil_gain_right (如果安全)
 * 4. 将增益映射为概率分布（使用 RemapGainsToProb）
 *
 * ==================== 增益 → 概率映射策略 ====================
 *
 * MOBIL的增益值范围约为 [-1.0, 6.0]（通过阈值截断）
 *
 * 情况一：左右都安全（is_lcl_safe && is_lcr_safe）：
 *   将三个增益值归一化到 [0, 1]，然后按比例分配概率：
 *     P(L) = gain_left / (gain_left + gain_keep + gain_right)
 *     P(K) = gain_keep / sum  （gain_keep 固定为 1.0 归一化后的值）
 *     P(R) = gain_right / sum
 *
 * 情况二：仅左侧安全：
 *   gain = normalize(gain_left, [-1, 6], [0, 1])
 *   P(L) = gain,  P(K) = 1 - gain,  P(R) = 0
 *
 * 情况三：仅右侧安全：
 *   对称于情况二
 *
 * 情况四：都不安全：
 *   P(L) = 0,  P(R) = 0,  P(K) = 1.0  （强制保持车道）
 *
 * 速度为零的车辆强制保持车道（速度低于阈值时）
 *
 * ==================== 车道索引约定 ====================
 *
 * 输入的lanes向量必须包含3个元素:
 *   lanes[0]: 当前车道（参考车道）
 *   lanes[1]: 左侧车道（向左变道的目标车道）
 *   lanes[2]: 右侧车道（向右变道的目标车道）
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/mobil/mobil_behavior_prediction.h"

namespace common {

/*
 * RemapGainsToProb - 将MOBIL增益映射为变道行为的概率分布
 *
 * 根据左右两侧是否安全、以及MOBIL增益值，计算保持车道、
 * 向左变道和向右变道的概率。
 *
 * 增益范围：lower_bound = -1.0, upper_bound = 6.0
 * 归一化函数将 [lower, upper] → [0, 1]
 *
 * @param is_lcl_safe 向左变道是否安全（RSS检查结果）
 * @param mobil_gain_left 向左变道的MOBIL增益
 * @param is_lcr_safe 向右变道是否安全（RSS检查结果）
 * @param mobil_gain_right 向右变道的MOBIL增益
 * @param res 输出参数，横向行为概率分布
 * @return kSuccess
 */
ErrorType MobilBehaviorPrediction::RemapGainsToProb(
    const bool is_lcl_safe, const decimal_t mobil_gain_left,
    const bool is_lcr_safe, const decimal_t mobil_gain_right,
    ProbDistOfLatBehaviors *res) {
  decimal_t lower_bound = -1.0;
  decimal_t upper_bound = 6.0;

  if (is_lcl_safe && is_lcr_safe) {
    /*
     * 两侧都安全：按MOBIL增益的比例分配概率
     * 保持车道的基准增益设为1.0（归一化后的值）
     */
    decimal_t gain_l = normalize_with_bound(mobil_gain_left, lower_bound,
                                            upper_bound, 0.0, 1.0);
    decimal_t gain_k =
        normalize_with_bound(1.0, lower_bound, upper_bound, 0.0, 1.0);
    decimal_t gain_r = normalize_with_bound(mobil_gain_right, lower_bound,
                                            upper_bound, 0.0, 1.0);

    decimal_t p_l = gain_l / (gain_k + gain_l + gain_r);
    decimal_t p_k = gain_k / (gain_k + gain_l + gain_r);
    decimal_t p_r = gain_r / (gain_k + gain_l + gain_r);

    res->SetEntry(common::LateralBehavior::kLaneChangeLeft, p_l);
    res->SetEntry(common::LateralBehavior::kLaneChangeRight, p_r);
    res->SetEntry(common::LateralBehavior::kLaneKeeping, p_k);
    res->is_valid = true;

  } else if (is_lcl_safe) {
    // 仅左侧安全：概率在左变道和保持之间分配
    decimal_t gain = normalize_with_bound(mobil_gain_left, lower_bound,
                                          upper_bound, 0.0, 1.0);
    res->SetEntry(common::LateralBehavior::kLaneChangeLeft, gain);
    res->SetEntry(common::LateralBehavior::kLaneChangeRight, 0.0);
    res->SetEntry(common::LateralBehavior::kLaneKeeping, 1.0 - gain);
    res->is_valid = true;
  } else if (is_lcr_safe) {
    // 仅右侧安全：概率在右变道和保持之间分配
    decimal_t gain = normalize_with_bound(mobil_gain_right, lower_bound,
                                          upper_bound, 0.0, 1.0);
    res->SetEntry(common::LateralBehavior::kLaneChangeLeft, 0.0);
    res->SetEntry(common::LateralBehavior::kLaneChangeRight, gain);
    res->SetEntry(common::LateralBehavior::kLaneKeeping, 1.0 - gain);
    res->is_valid = true;
  } else {
    // 都不安全：强制保持车道
    res->SetEntry(common::LateralBehavior::kLaneChangeLeft, 0.0);
    res->SetEntry(common::LateralBehavior::kLaneChangeRight, 0.0);
    res->SetEntry(common::LateralBehavior::kLaneKeeping, 1.0);
    res->is_valid = true;
  }
  return kSuccess;
}

/*
 * LateralBehaviorPrediction - 预测指定车辆的横向行为
 *
 * 完整的三车道MOBIL行为预测流程：
 *
 * 步骤1: 输入验证
 *   确保车道数为3，车辆速度不为零（静止车辆默认保持车道）
 *
 * 步骤2: 当前车道分析
 *   计算本车在当前车道的加速度 acc_c
 *   以及后车的加速度变化 acc_o, acc_o_tilda
 *
 * 步骤3: 左变道分析
 *   将本车状态投影到左车道，计算MOBIL增益
 *   增益公式: d_a_c + p * (d_a_n + d_a_o)
 *   其中 d_a_c = acc_c_tilda - acc_c (本车变道后的加速度变化)
 *         d_a_n = acc_n_tilda - acc_n (左车道后车的加速度变化)
 *         d_a_o = acc_o_tilda - acc_o (当前车道后车的加速度变化)
 *         p = 0.0 (礼貌系数为0，完全自私策略)
 *
 * 步骤4: 右变道分析（对称于步骤3）
 *
 * 步骤5: 增益→概率映射
 *
 * @param vehicle 待预测行为的车辆
 * @param lanes 三个车道的引用 [当前, 左边, 右边]
 * @param leading_vehicles 各车道前车引用
 * @param leading_frenet_states 各车道前车的Frenet状态
 * @param following_vehicles 各车道后车引用
 * @param follow_frenet_states 各车道后车的Frenet状态
 * @param nearby_vehicles 附近所有车辆的集合（当前未使用，预留接口）
 * @param res 输出参数，横向行为的概率分布
 * @return kSuccess；kWrongStatus 输入不合法或车道无效
 */
ErrorType MobilBehaviorPrediction::LateralBehaviorPrediction(
    const Vehicle &vehicle, const vec_E<Lane> &lanes,
    const vec_E<common::Vehicle> &leading_vehicles,
    const vec_E<common::FrenetState> &leading_frenet_states,
    const vec_E<common::Vehicle> &following_vehicles,
    const vec_E<common::FrenetState> &follow_frenet_states,
    const common::VehicleSet &nearby_vehicles, ProbDistOfLatBehaviors *res) {
  if (lanes.size() != 3) {
    printf(
        "[MobilBehaviorPrediction]Must have three lanes (invalid also "
        "acceptable).\n");
    return kWrongStatus;
  }

  decimal_t acc_c = 0.0;
  decimal_t acc_o = 0.0, acc_o_tilda = 0.0;
  decimal_t politeness_coeff = 0.0;  // 礼貌系数 = 0 (自私策略)

  bool is_lcl_safe = false;
  bool is_lcr_safe = false;

  decimal_t mobil_gain_left = -kInf;
  decimal_t mobil_gain_right = -kInf;

  // 步骤1: 静止车辆强制保持车道
  decimal_t desired_vel = vehicle.state().velocity;
  if (fabs(desired_vel) < kBigEPS) {
    res->SetEntry(common::LateralBehavior::kLaneChangeLeft, 0.0);
    res->SetEntry(common::LateralBehavior::kLaneChangeRight, 0.0);
    res->SetEntry(common::LateralBehavior::kLaneKeeping, 1.0);
    res->is_valid = true;
    return kSuccess;
  }

  // 步骤2: 当前车道分析
  common::Lane lk_lane = lanes[0];
  if (lk_lane.IsValid()) {
    common::Vehicle leading_vehicle = leading_vehicles[0];
    common::FrenetState leading_fs = leading_frenet_states[0];
    common::Vehicle following_vehicle = following_vehicles[0];
    common::FrenetState following_fs = follow_frenet_states[0];

    common::StateTransformer stf(lk_lane);
    common::FrenetState ego_frenet_state;
    stf.GetFrenetStateFromState(vehicle.state(), &ego_frenet_state);

    if (MobilLaneChangingModel::GetMobilAccChangesOnCurrentLane(
            ego_frenet_state, leading_vehicle, leading_fs, following_vehicle,
            following_fs, &acc_o, &acc_o_tilda, &acc_c) != kSuccess) {
      printf("[MobilBehaviorPrediction]lane-keep lane not valid.\n");
      return kWrongStatus;
    }
  } else {
    return kWrongStatus;
  }

  // 步骤3: 左变道分析
  common::Lane lcl_lane = lanes[1];
  if (lcl_lane.IsValid()) {
    common::Vehicle leading_vehicle = leading_vehicles[1];
    common::FrenetState leading_fs = leading_frenet_states[1];
    common::Vehicle following_vehicle = following_vehicles[1];
    common::FrenetState following_fs = follow_frenet_states[1];
    decimal_t acc_n = 0.0, acc_n_tilda = 0.0, acc_c_tilda = 0.0;

    // 将本车状态投影到左车道
    common::StateTransformer stf(lcl_lane);
    common::FrenetState projected_frenet_state;
    stf.GetFrenetStateFromState(vehicle.state(), &projected_frenet_state);

    MobilLaneChangingModel::GetMobilAccChangesOnTargetLane(
        projected_frenet_state, leading_vehicle, leading_fs, following_vehicle,
        following_fs, &is_lcl_safe, &acc_n, &acc_n_tilda, &acc_c_tilda);
    if (is_lcl_safe) {
      decimal_t d_a_c = acc_c_tilda - acc_c;
      decimal_t d_a_n = acc_n_tilda - acc_n;
      decimal_t d_a_o = acc_o_tilda - acc_o;
      // MOBIL增益 = 本车增益 + 礼貌系数 * (他人增益之和)
      mobil_gain_left = d_a_c + politeness_coeff * (d_a_n + d_a_o);
    }
  }

  // 步骤4: 右变道分析（对称于左变道）
  common::Lane lcr_lane = lanes[2];
  if (lcr_lane.IsValid()) {
    common::Vehicle leading_vehicle = leading_vehicles[2];
    common::FrenetState leading_fs = leading_frenet_states[2];
    common::Vehicle following_vehicle = following_vehicles[2];
    common::FrenetState following_fs = follow_frenet_states[2];
    decimal_t acc_n = 0.0, acc_n_tilda = 0.0, acc_c_tilda = 0.0;

    common::StateTransformer stf(lcr_lane);
    common::FrenetState projected_frenet_state;
    stf.GetFrenetStateFromState(vehicle.state(), &projected_frenet_state);

    MobilLaneChangingModel::GetMobilAccChangesOnTargetLane(
        projected_frenet_state, leading_vehicle, leading_fs, following_vehicle,
        following_fs, &is_lcr_safe, &acc_n, &acc_n_tilda, &acc_c_tilda);
    if (is_lcr_safe) {
      decimal_t d_a_c = acc_c_tilda - acc_c;
      decimal_t d_a_n = acc_n_tilda - acc_n;
      decimal_t d_a_o = acc_o_tilda - acc_o;
      mobil_gain_right = d_a_c + politeness_coeff * (d_a_n + d_a_o);
    }
  }

  // 步骤5: 增益映射为概率分布
  RemapGainsToProb(is_lcl_safe, mobil_gain_left, is_lcr_safe, mobil_gain_right,
                   res);
  printf("[Mobil]%d, Left gain: %.3lf, Right gain: %.3lf\n", vehicle.id(),
         mobil_gain_left, mobil_gain_right);
  return kSuccess;
}

}  // namespace common
