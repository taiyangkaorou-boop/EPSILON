/**
 * @file mobil_model.cc
 * @brief MOBIL车道变更决策模型的实现文件
 *
 * 本文件实现了EPSILON自动驾驶规划系统中的MOBIL（Minimizing Overall
 * Braking Induced by Lane changes）车道变更决策模型。
 *
 * ==================== MOBIL模型简介 ====================
 *
 * MOBIL是由Arne Kesting等人提出的基于加速度的车道变更决策模型。
 * 核心思想：车道变更决策应基于变更前后相关车辆加速度变化的总和（效用增益）来判断。
 *
 * MOBIL的决策准则：
 *   变道是值得的，当且仅当：
 *   1. 安全条件满足（RSS安全检查通过）
 *   2. 加速度增益 = d_a_c + p * (d_a_n + d_a_o) > threshold
 *
 * 其中：
 *   - d_a_c: 本车变道后的加速度变化（target lane - current lane）
 *   - d_a_n: 目标车道后车的加速度变化（after - before）
 *   - d_a_o: 当前车道后车的加速度变化（after - before）
 *   - p: 礼貌系数（politeness factor），衡量对他人影响的重要程度
 *        p=0 表示完全自私（只考虑自己），p=1 表示完全无私（同等考虑他人）
 *   - threshold: 最小增益阈值，防止不必要的频繁变道
 *
 * ==================== IDM在MOBIL中的角色 ====================
 *
 * MOBIL使用IDM（智能驾驶员模型）来计算各相关车辆在变道前后的加速度：
 *   - acc_c: 本车在当前车道的加速度（跟前车）
 *   - acc_o: 当前车道后车在当前情况下的加速度（跟本车）
 *   - acc_o_tilda: 当前车道后车在本车离开后的加速度（跟再前车）
 *   - acc_n: 目标车道后车在当前情况下的加速度（跟它自己的前车）
 *   - acc_n_tilda: 目标车道后车在本车插入后的加速度（跟本车）
 *   - acc_c_tilda: 本车在目标车道的加速度（跟新前车）
 *
 * ==================== 安全条件（RSS检查） ====================
 *
 * 即使MOBIL增益为正，变道也必须在RSS安全约束下才能执行。
 * RSS（Responsibility-Sensitive Safety）模型确保：
 *   - 本车与目标车道前车的安全距离足够
 *   - 目标车道后车的安全距离足够
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/mobil/mobil_model.h"

namespace common {

/*
 * GetMobilAccChangesOnCurrentLane - 计算当前车道的加速度变化
 *
 * 计算在当前车道上与相关车辆的加速度：
 *   - acc_o: 当前车道后车跟本车的IDM加速度
 *   - acc_o_tilda: 当前车道后车在本车离开后（跟再前车）的IDM加速度
 *   - acc_c: 本车在当前车道的IDM加速度（跟前车）
 *
 * 考虑了多种场景：
 *   无后车或后车静止 → 只有本车的加速度计算
 *   无前车 → 使用虚拟前车（安全距离无穷大）
 *   有前有后 → 完整的IDM计算
 *
 * @param cur_fs 本车在当前车道的Frenet状态
 * @param leading_vehicle 本车的前车引用
 * @param leading_fs 前车的Frenet状态
 * @param following_vehicle 本车的后车引用
 * @param following_fs 后车的Frenet状态
 * @param acc_o 输出参数，当前车道后车跟本车的加速度
 * @param acc_o_tilda 输出参数，当前车道后车在本车离开后的加速度（无本车时跟再前车）
 * @param acc_c 输出参数，本车在当前车道的加速度
 * @return kSuccess
 */
ErrorType MobilLaneChangingModel::GetMobilAccChangesOnCurrentLane(
    const FrenetState &cur_fs, const Vehicle &leading_vehicle,
    const FrenetState &leading_fs, const Vehicle &following_vehicle,
    const FrenetState &following_fs, decimal_t *acc_o, decimal_t *acc_o_tilda,
    decimal_t *acc_c) {
  bool has_leading_vehicle =
      leading_vehicle.id() == kInvalidAgentId ? false : true;
  bool has_following_vehicle =
      following_vehicle.id() == kInvalidAgentId ? false : true;

  decimal_t acc_o_tmp = 0.0, acc_o_tilda_tmp = 0.0, acc_c_tmp = 0.0;

  // IDM模型参数：期望速度设为各自在当前车道上的速度
  IntelligentDriverModel::Param idm_param_o;
  idm_param_o.kDesiredVelocity = following_fs.vec_s(1);
  IntelligentDriverModel::Param idm_param_c;
  idm_param_c.kDesiredVelocity = cur_fs.vec_s(1);

  if ((!has_following_vehicle) || fabs(following_fs.vec_s(1)) < kEPS) {
    // 场景: 无后车或后车几乎静止 → 只需计算本车加速度
    if (!has_leading_vehicle) {
      // 无前车 → 使用虚拟前车
      GetDesiredAccelerationUsingIdm(idm_param_c, cur_fs, common::FrenetState(),
                                     true, &acc_c_tmp);
    } else {
      // 有前车 → 使用IDM计算本车加速度
      GetDesiredAccelerationUsingIdm(idm_param_c, cur_fs, leading_fs, false,
                                     &acc_c_tmp);
    }
  } else {
    // 场景: 有后车 → 需要计算后车的加速度变化
    if (!has_leading_vehicle) {
      // 无前车 → 后车跟本车(虚拟前车为无穷远)
      GetDesiredAccelerationUsingIdm(idm_param_o, following_fs, cur_fs, false,
                                     &acc_o_tmp);
      // 后车在本车离开后的加速度（无车在其前方）
      GetDesiredAccelerationUsingIdm(idm_param_o, following_fs,
                                     common::FrenetState(), true,
                                     &acc_o_tilda_tmp);
      // 本车无前车 → 虚拟前车
      GetDesiredAccelerationUsingIdm(idm_param_c, cur_fs, common::FrenetState(),
                                     true, &acc_c_tmp);
    } else {
      // 有前车 → 完整IDM计算
      // 后车跟本车
      GetDesiredAccelerationUsingIdm(idm_param_o, following_fs, cur_fs, false,
                                     &acc_o_tmp);
      // 后车在本车离开后跟再前车
      GetDesiredAccelerationUsingIdm(idm_param_o, following_fs, leading_fs,
                                     false, &acc_o_tilda_tmp);
      // 本车跟前车
      GetDesiredAccelerationUsingIdm(idm_param_c, cur_fs, leading_fs, false,
                                     &acc_c_tmp);
    }
  }
  *acc_o = acc_o_tmp;
  *acc_o_tilda = acc_o_tilda_tmp;
  *acc_c = acc_c_tmp;
  return kSuccess;
}

/*
 * GetMobilAccChangesOnTargetLane - 计算目标车道的加速度变化
 *
 * 计算变道到目标车道后的各项加速度，以及RSS安全检查。
 *
 * 关键步骤：
 * 1. RSS安全检查：确认本车与目标车道前后车的安全距离是否足够
 * 2. 如果安全，计算加速度：
 *    - acc_n: 目标车道后车在当前情况下的IDM加速度
 *    - acc_n_tilda: 目标车道后车在本车插入后（跟本车）的IDM加速度
 *    - acc_c_tilda: 本车在目标车道的IDM加速度（跟前车）
 *
 * @param projected_cur_fs 本车投影到目标车道的Frenet状态
 * @param leading_vehicle 目标车道的前车引用
 * @param leading_fs 目标车道前车的Frenet状态
 * @param following_vehicle 目标车道的后车引用
 * @param following_fs 目标车道后车的Frenet状态
 * @param is_lc_safe 输出参数，变道是否安全（RSS检查结果）
 * @param acc_n 输出参数，目标车道后车在当前情况下的加速度
 * @param acc_n_tilda 输出参数，目标车道后车在本车插入后的加速度
 * @param acc_c_tilda 输出参数，本车在目标车道的加速度
 * @return kSuccess
 */
ErrorType MobilLaneChangingModel::GetMobilAccChangesOnTargetLane(
    const FrenetState &projected_cur_fs, const Vehicle &leading_vehicle,
    const FrenetState &leading_fs, const Vehicle &following_vehicle,
    const FrenetState &following_fs, bool *is_lc_safe, decimal_t *acc_n,
    decimal_t *acc_n_tilda, decimal_t *acc_c_tilda) {
  bool has_leading_vehicle =
      leading_vehicle.id() == kInvalidAgentId ? false : true;
  bool has_following_vehicle =
      following_vehicle.id() == kInvalidAgentId ? false : true;

  decimal_t acc_n_tmp = 0.0, acc_n_tilda_tmp = 0.0, acc_c_tilda_tmp = 0.0;

  *is_lc_safe = false;
  /*
   * RSS安全检查：
   * 如果前车和后车是同一辆车（有车但两车ID相同），视为不安全。
   * 否则分别进行前向和后向RSS检查。
   */
  if (leading_vehicle.id() == following_vehicle.id() &&
      leading_vehicle.id() != kInvalidAgentId) {
    *is_lc_safe = false;
  } else {
    bool is_front_safe = true, is_rear_safe = true;
    RssChecker::RssCheck(projected_cur_fs, leading_fs,
                         common::RssChecker::RssConfig(), &is_front_safe);
    RssChecker::RssCheck(projected_cur_fs, following_fs,
                         common::RssChecker::RssConfig(), &is_rear_safe);
    *is_lc_safe = is_front_safe && is_rear_safe;
  }

  if (*is_lc_safe) {
    // 通过安全检查后，计算加速度变化
    IntelligentDriverModel::Param idm_param_n;
    idm_param_n.kDesiredVelocity = following_fs.vec_s(1);
    IntelligentDriverModel::Param idm_param_c;
    idm_param_c.kDesiredVelocity = projected_cur_fs.vec_s(1);

    if ((!has_following_vehicle) || fabs(following_fs.vec_s(1)) < kEPS) {
      // 无后车或后车静止 → 只算本车在目标车道的加速度
      if (!has_leading_vehicle) {
        GetDesiredAccelerationUsingIdm(idm_param_c, projected_cur_fs,
                                       common::FrenetState(), true,
                                       &acc_c_tilda_tmp);
      } else {
        GetDesiredAccelerationUsingIdm(idm_param_c, projected_cur_fs,
                                       leading_fs, false, &acc_c_tilda_tmp);
      }
    } else {
      // 有后车 → 计算完整加速度变化
      if (!has_leading_vehicle) {
        GetDesiredAccelerationUsingIdm(idm_param_n, following_fs,
                                       common::FrenetState(), true, &acc_n_tmp);
        GetDesiredAccelerationUsingIdm(idm_param_n, following_fs,
                                       projected_cur_fs, false,
                                       &acc_n_tilda_tmp);
        GetDesiredAccelerationUsingIdm(idm_param_c, projected_cur_fs,
                                       common::FrenetState(), true,
                                       &acc_c_tilda_tmp);
      } else {
        GetDesiredAccelerationUsingIdm(idm_param_n, following_fs, leading_fs,
                                       false, &acc_n_tmp);
        GetDesiredAccelerationUsingIdm(idm_param_n, following_fs,
                                       projected_cur_fs, false,
                                       &acc_n_tilda_tmp);
        GetDesiredAccelerationUsingIdm(idm_param_c, projected_cur_fs,
                                       leading_fs, false, &acc_c_tilda_tmp);
      }
    }
    *acc_n = acc_n_tmp;
    *acc_n_tilda = acc_n_tilda_tmp;
    *acc_c_tilda = acc_c_tilda_tmp;
  }
  return kSuccess;
}

/*
 * GetDesiredAccelerationUsingIdm - 使用IDM计算跟车场景下的加速度
 *
 * 这是MOBIL模型中的辅助函数，将Frenet状态转换为IDM输入格式，
 * 并调用IDM模型计算期望加速度。
 *
 * 虚拟前车模式 (use_virtual_front = true):
 *   当没有实际前车时，构造一个虚拟前车：
 *   - s_front = 100.0 + rear_vel * 10 (非常远的安全距离)
 *   - v_front = rear_vel (前车与后车速度相同)
 *   这模拟了"前方无车"的场景。
 *
 * @param param IDM模型参数
 * @param rear_fs 后方车辆（跟车者）的Frenet状态
 * @param front_fs 前方车辆（被跟车者）的Frenet状态（虚拟模式下忽略）
 * @param use_virtual_front 是否使用虚拟前车（true=无实际前车）
 * @param acc 输出参数，计算得到的期望加速度
 * @return kSuccess
 */
ErrorType MobilLaneChangingModel::GetDesiredAccelerationUsingIdm(
    const IntelligentDriverModel::Param &param, const FrenetState &rear_fs,
    const FrenetState &front_fs, const bool &use_virtual_front,
    decimal_t *acc) {
  IntelligentDriverModel::State idm_state;
  if (!use_virtual_front) {
    // 使用实际前车状态构造IDM状态
    idm_state =
        IntelligentDriverModel::State(rear_fs.vec_s(0), rear_fs.vec_s(1),
                                      front_fs.vec_s(0), front_fs.vec_s(1));
  } else {
    // 构造虚拟前车：位置为 100 + 10*v (非常远)，速度等于本车
    idm_state = IntelligentDriverModel::State(
        0.0, rear_fs.vec_s(1), 100.0 + rear_fs.vec_s(1) * 10, rear_fs.vec_s(1));
  }

  // 使用ACC策略（IIDM + CAH）计算加速度
  IntelligentDriverModel::GetAccDesiredAcceleration(param, idm_state, acc);
  return kSuccess;
}

}  // namespace common
