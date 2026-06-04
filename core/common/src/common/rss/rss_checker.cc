/**
 * @file rss_checker.cc
 * @brief RSS（Responsibility-Sensitive Safety）安全检查器的实现文件
 *
 * 本文件实现了EPSILON自动驾驶规划系统中的RSS安全检查功能。
 * RSS是由Mobileye提出的责任敏感安全模型，用于数学化地定义
 * 自动驾驶的安全约束。
 *
 * ==================== RSS模型核心原则 ====================
 *
 * RSS模型基于以下关键原则：
 *
 * 1. 谨慎性原则：每个交通参与者都需要保持最小安全距离
 * 2. 响应时间原则：假设感知-决策-执行存在延迟(response_time)
 * 3. 最坏情况假设：假设其他参与者可能采取最不利行为
 *
 * ==================== 纵向安全距离计算 ====================
 *
 * 纵向安全距离 d_safe 的计算取决于本车(ego)和它车(other)的相对位置和方向：
 *
 * 前向安全距离（本车在后，other在前）：
 *   ego在反应时间内的行驶距离 + ego以最小制动减速度减速到停止的距离
 *   - other以最大制动减速度减速到停止的距离
 *
 * 后向安全距离（本车在前，other在后）：
 *   other在反应时间内的行驶距离 + other以最小制动减速度减速到停止的距离
 *   - ego以最大制动减速度减速到停止的距离
 *
 * 如果两车相向而行（ego前进，other后退）：
 *   需要考虑两车的制动距离之和
 *
 * ==================== 横向安全距离计算 ====================
 *
 * 横向安全距离考虑了类似的因素：反应时间、最大加速度、最小/最大制动。
 * 额外增加了横向裕度 lateral_miu 作为安全缓冲。
 *
 * ==================== RSS检查流程 ====================
 *
 * RssCheck (Frenet状态版本)：
 * 1. 确定纵向方向（Front/Rear）：比较s坐标
 * 2. 确定横向方向（Left/Right）：比较d坐标
 * 3. 计算安全的纵向和横向距离
 * 4. 如果纵向和横向的实际距离都小于安全距离 → 不安全
 *
 * RssCheck (车辆版本)：
 * 1. 将车辆状态转换到Frenet坐标系
 * 2. 横向安全：包含车辆宽度附加项
 * 3. 纵向安全：考虑车长和d_cr计算实际纵向距离
 * 4. 如果横向距离足够大 → 安全（不同车道不考虑纵向冲突）
 * 5. 否则计算纵向安全速度，判断是否有违规
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/rss/rss_checker.h"

namespace common {

/*
 * CalculateSafeLongitudinalDistance - 计算纵向（沿车道方向）的安全距离
 *
 * 基于RSS模型的纵向安全距离计算，考虑反应时间和制动距离。
 *
 * 场景分析（Front方向，本车在后）：
 *   如果两车同向（ego速度>=0, other速度>=0）：
 *     ego_min_d = v*T + (v_max_at_T)^2 / (2*b_min)
 *     other_max_d = v_other^2 / (2*b_max_other)
 *     d_safe = ego_min_d - other_max_d
 *
 *   如果两车相向（ego前进，other后退）：
 *     d_safe = ego_passive_d + other_passive_d
 *
 * 场景分析（Rear方向，本车在前）：
 *   如果两车同向（ego速度>=0, other速度>=0）：
 *     other_min_d = other在反应时间内的最小制动距离（被动制动）
 *     ego_max_d = ego的最大制动距离（主动制动）
 *     d_safe = other_min_d - ego_max_d
 *
 *   如果本车前进，other后退：d_safe = 0（反向行驶不产生纵向冲突）
 *
 * @param ego_vel 本车纵向速度（沿道路方向）
 * @param other_vel 它车纵向速度
 * @param direction 纵向方向（本车在它车前面还是后面）
 * @param config RSS配置参数
 * @param distance 输出参数，计算的纵向安全距离
 * @return kSuccess
 */
ErrorType RssChecker::CalculateSafeLongitudinalDistance(
    const decimal_t ego_vel, const decimal_t other_vel,
    const LongitudinalDirection& direction, const RssConfig& config,
    decimal_t* distance) {
  decimal_t ret = 0.0;
  decimal_t ego_vel_abs = fabs(ego_vel);
  decimal_t other_vel_abs = fabs(other_vel);
  // 在反应时间结束后的速度（假设以最大加速度加速）
  decimal_t ego_vel_at_response_time =
      ego_vel_abs + config.longitudinal_acc_max * config.response_time;
  decimal_t other_vel_at_response_time =
      other_vel_abs + config.longitudinal_acc_max * config.response_time;

  decimal_t ego_distance_driven, other_distance_driven;
  if (direction == Front) {
    // 本车在后，other在前
    // "最坏情况"下本车的行驶距离
    ego_distance_driven =
        (ego_vel_abs + ego_vel_at_response_time) / 2.0 * config.response_time +
        ego_vel_at_response_time * ego_vel_at_response_time /
            (2 * config.longitudinal_brake_min);
    if (ego_vel >= 0.0 && other_vel >= 0.0) {
      // 两车同向 → 考虑other的最大制动距离
      other_distance_driven =
          (other_vel_abs * other_vel_abs) / (2 * config.longitudinal_brake_max);
      ret = ego_distance_driven - other_distance_driven;
    } else if (ego_vel >= 0.0 && other_vel <= 0.0) {
      // 两车相向 → 需要两者制动距离相加
      other_distance_driven = (other_vel_abs + other_vel_at_response_time) /
                                  2.0 * config.response_time +
                              other_vel_at_response_time *
                                  other_vel_at_response_time /
                                  (2 * config.longitudinal_brake_min);
      ret = ego_distance_driven + other_distance_driven;
    } else {
      ret = 0.0;  // 不支持倒车场景
    }
  } else if (direction == Rear) {
    // 本车在前，other在后
    ego_distance_driven =
        ego_vel_abs * ego_vel_abs / (2 * config.longitudinal_brake_max);
    if (ego_vel >= 0.0 && other_vel >= 0.0) {
      // 两车同向，other在后追本车
      other_distance_driven = (other_vel_abs + other_vel_at_response_time) /
                                  2.0 * config.response_time +
                              other_vel_at_response_time *
                                  other_vel_at_response_time /
                                  (2 * config.longitudinal_brake_min);
      ret = other_distance_driven - ego_distance_driven;
    } else if (ego_vel >= 0.0 && other_vel <= 0.0) {
      ret = 0.0;  // 反向行驶不产生纵向冲突
    } else {
      ret = 0.0;
    }
  }
  *distance = ret > 0.0 ? ret : 0.0;  // 安全距离不能为负
  return kSuccess;
}

/*
 * CalculateSafeLongitudinalVelocity - 计算满足RSS约束的纵向安全速度范围
 *
 * 给定与其他车辆的距离 lon_distance_abs，计算本车的安全速度上下界。
 *
 * Front方向（本车在后）：
 *   如果other同向（>=0）：
 *     other以最大制动距离减速 → 求解二次方程得到本车最大速度
 *   如果other相向（other_vel<0）：
 *     other以最小制动距离减速 → 调整lon_distance后求解最大速度
 *
 * Rear方向（本车在前）：
 *   如果other同向：
 *     如果other的最小制动距离 < lon_distance → 安全范围[0, inf]
 *     否则 → 计算最小安全速度（快速脱离）
 *   如果other相向 → 始终安全
 *
 * @param other_vel 它车的纵向速度
 * @param direction 纵向方向
 * @param lon_distance_abs 两车之间的实际纵向距离
 * @param config RSS配置参数
 * @param ego_vel_low 输出参数，本车的最小安全速度下界
 * @param ego_vel_upp 输出参数，本车的最大安全速度上界
 * @return kSuccess
 */
ErrorType RssChecker::CalculateSafeLongitudinalVelocity(
    const decimal_t other_vel, const LongitudinalDirection& direction,
    const decimal_t& lon_distance_abs, const RssConfig& config,
    decimal_t* ego_vel_low, decimal_t* ego_vel_upp) {
  decimal_t other_vel_abs = fabs(other_vel);
  decimal_t other_vel_at_response_time =
      other_vel_abs + config.longitudinal_acc_max * config.response_time;
  decimal_t other_distance_driven;
  if (direction == Front) {
    if (other_vel >= 0.0) {
      // other同向，在前
      other_distance_driven =
          (other_vel_abs * other_vel_abs) / (2 * config.longitudinal_brake_max);
      // 求解二次方程 a*v^2 + b*v + c = 0 得到本车最大速度
      decimal_t a = 1.0 / (2.0 * config.longitudinal_brake_min);
      decimal_t b = config.response_time +
                    (config.longitudinal_acc_max * config.response_time /
                     config.longitudinal_brake_min);
      decimal_t c = 0.5 *
                        (config.longitudinal_acc_max +
                         pow(config.longitudinal_acc_max, 2) /
                             config.longitudinal_brake_min) *
                        pow(config.response_time, 2) -
                    other_distance_driven - lon_distance_abs;
      *ego_vel_upp = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
      *ego_vel_low = 0.0;
    } else {
      // other相向，ego在前
      other_distance_driven = (other_vel_abs + other_vel_at_response_time) /
                                  2.0 * config.response_time +
                              other_vel_at_response_time *
                                  other_vel_at_response_time /
                                  (2 * config.longitudinal_brake_min);
      if (other_distance_driven > lon_distance_abs) {
        *ego_vel_upp = 0.0;  // 无法避免碰撞
        *ego_vel_low = 0.0;
      } else {
        // 用缩减后的距离求解最大速度
        decimal_t a = 1.0 / (2.0 * config.longitudinal_brake_min);
        decimal_t b = config.response_time +
                      (config.longitudinal_acc_max * config.response_time /
                       config.longitudinal_brake_min);
        decimal_t c = 0.5 *
                          (config.longitudinal_acc_max +
                           pow(config.longitudinal_acc_max, 2) /
                               config.longitudinal_brake_min) *
                          pow(config.response_time, 2) -
                      (lon_distance_abs - other_distance_driven);
        *ego_vel_upp = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
        *ego_vel_low = 0.0;
      }
    }
  } else {
    // Rear方向：本车在前，other在后
    if (other_vel >= 0.0) {
      // other同向，在后追本车
      other_distance_driven = (other_vel_abs + other_vel_at_response_time) /
                                  2.0 * config.response_time +
                              other_vel_at_response_time *
                                  other_vel_at_response_time /
                                  (2 * config.longitudinal_brake_min);
      if (other_distance_driven < lon_distance_abs) {
        *ego_vel_upp = kInf;   // 安全距离足够
        *ego_vel_low = 0.0;
      } else {
        *ego_vel_upp = kInf;
        *ego_vel_low = sqrt(2 * config.longitudinal_brake_max *
                            (other_distance_driven - lon_distance_abs));
      }
    } else {
      // other相向，远离本车 → 始终安全
      *ego_vel_upp = kInf;
      *ego_vel_low = 0.0;
    }
  }
  return kSuccess;
}

/*
 * CalculateSafeLateralDistance - 计算横向安全距离
 *
 * 横向安全距离的计算逻辑：
 * 根据两车的横向速度和方向（Right/Left），
 * 计算在反应时间和制动距离之后两车之间的最小安全间距。
 *
 * 场景分类（以Right方向为例，ego在other右侧）：
 *   ego右移(>=0) + other右移(>=0): other被动制动 - ego主动制动
 *   ego右移(>=0) + other左移(<0):  互相远离 = 0
 *   ego左移(<0)  + other左移(<0):  ego被动制动 - other主动制动
 *   ego左移(<0)  + other右移(>=0): 相向而行 = 两者被动制动之和
 *
 * 最后加上横向裕度 lateral_miu 作为安全缓冲。
 *
 * @param ego_vel 本车横向速度
 * @param other_vel 它车横向速度
 * @param direction 横向方向（本车在它车右侧还是左侧）
 * @param config RSS配置参数
 * @param distance 输出参数，横向安全距离
 * @return kSuccess
 */
ErrorType RssChecker::CalculateSafeLateralDistance(
    const decimal_t ego_vel, const decimal_t other_vel,
    const LateralDirection& direction, const RssConfig& config,
    decimal_t* distance) {
  decimal_t ret = 0.0;
  decimal_t ego_lat_vel_abs = fabs(ego_vel);
  decimal_t other_lat_vel_abs = fabs(other_vel);
  decimal_t distance_correction = config.lateral_miu;  // 横向安全裕度
  // 反应时间结束后的速度
  decimal_t ego_lat_vel_at_response_time =
      ego_lat_vel_abs + config.response_time * config.lateral_acc_max;
  decimal_t other_lat_vel_at_response_time =
      other_lat_vel_abs + config.response_time * config.lateral_acc_max;
  // 主动制动距离（最大制动力）
  decimal_t ego_active_brake_distance =
      ego_lat_vel_abs * ego_lat_vel_abs / (2 * config.lateral_brake_max);
  // 被动制动距离（最小制动力 + 反应时间）
  decimal_t ego_passive_brake_distance =
      (ego_lat_vel_abs + ego_lat_vel_at_response_time) / 2.0 *
          config.response_time +
      ego_lat_vel_at_response_time * ego_lat_vel_at_response_time /
          (2 * config.lateral_brake_min);
  decimal_t other_active_brake_distance =
      other_lat_vel_abs * other_lat_vel_abs / (2 * config.lateral_brake_max);
  decimal_t other_passive_brake_distance =
      (other_lat_vel_abs + other_lat_vel_at_response_time) / 2.0 *
          config.response_time +
      other_lat_vel_at_response_time * other_lat_vel_at_response_time /
          (2 * config.lateral_brake_min);

  if (direction == Right) {
    // 本车在other的右侧
    if (ego_vel >= 0.0 && other_vel >= 0.0) {
      // 同向向右 → other被动制动 - ego主动制动
      ret = other_passive_brake_distance - ego_active_brake_distance;
    } else if (ego_vel >= 0.0 && other_vel < 0.0) {
      // 背离 → 不需要额外横向距离
      ret = 0.0;
    } else if (ego_vel < 0.0 && other_vel < 0.0) {
      // 同向向左 → ego被动制动 - other主动制动
      ret = ego_passive_brake_distance - other_active_brake_distance;
    } else if (ego_vel < 0.0 && other_vel >= 0.0) {
      // 相向而行 → 两者被动制动之和
      ret = ego_passive_brake_distance + other_passive_brake_distance;
    } else {
      ret = 0.0;
    }
  } else if (direction == Left) {
    // 本车在other的左侧（对称于Right方向）
    if (ego_vel >= 0.0 && other_vel >= 0.0) {
      ret = ego_passive_brake_distance - other_active_brake_distance;
    } else if (ego_vel >= 0.0 && other_vel < 0.0) {
      ret = ego_passive_brake_distance + other_passive_brake_distance;
    } else if (ego_vel < 0.0 && other_vel < 0.0) {
      ret = other_passive_brake_distance - ego_active_brake_distance;
    } else if (ego_vel < 0.0 && other_vel >= 0.0) {
      ret = 0.0;
    } else {
      ret = 0.0;
    }
  }
  ret = ret > 0.0 ? ret : 0.0;
  ret += distance_correction;  // 加上横向裕度
  *distance = ret;
  return kSuccess;
}

/*
 * CalculateRssSafeDistances - 同时计算纵向和横向安全距离
 *
 * @param ego_vels 本车速度向量 [纵向速度, 横向速度]
 * @param other_vels 它车速度向量 [纵向速度, 横向速度]
 * @param lon_direct 纵向方向
 * @param lat_direct 横向方向
 * @param config RSS配置
 * @param safe_distances 输出参数 [纵向安全距离, 横向安全距离]
 * @return kSuccess
 */
ErrorType RssChecker::CalculateRssSafeDistances(
    const std::vector<decimal_t>& ego_vels,
    const std::vector<decimal_t>& other_vels,
    const LongitudinalDirection& lon_direct, const LateralDirection& lat_direct,
    const RssConfig& config, std::vector<decimal_t>* safe_distances) {
  safe_distances->clear();
  decimal_t safe_long_distance, safe_lat_distance;
  CalculateSafeLongitudinalDistance(ego_vels[0], other_vels[0], lon_direct,
                                    config, &safe_long_distance);
  CalculateSafeLateralDistance(ego_vels[1], other_vels[1], lat_direct, config,
                               &safe_lat_distance);
  safe_distances->push_back(safe_long_distance);
  safe_distances->push_back(safe_lat_distance);
  return kSuccess;
}

/*
 * RssCheck - 对两个Frenet状态进行RSS安全检查
 *
 * 根据Frenet坐标判断方向关系，计算安全距离，如果实际距离
 * 在纵向和横向上都小于安全距离，则判定为不安全。
 *
 * @param ego_fs 本车的Frenet状态 (s, s_dot, d, d_dot)
 * @param other_fs 它车的Frenet状态
 * @param config RSS配置参数
 * @param is_safe 输出参数，true=安全, false=不安全
 * @return kSuccess
 */
ErrorType RssChecker::RssCheck(const FrenetState& ego_fs,
                               const FrenetState& other_fs,
                               const RssConfig& config, bool* is_safe) {
  LongitudinalDirection lon_direct;
  LateralDirection lat_direct;
  // 根据s坐标确定纵向方向
  if (ego_fs.vec_s[0] >= other_fs.vec_s[0]) {
    lon_direct = Rear;    // 本车s坐标更大（在前面）
  } else {
    lon_direct = Front;   // 本车s坐标更小（在后面）
  }

  // 根据d坐标确定横向方向
  if (ego_fs.vec_dt[0] >= other_fs.vec_dt[0]) {
    lat_direct = Right;   // 本车在右侧
  } else {
    lat_direct = Left;    // 本车在左侧
  }

  std::vector<decimal_t> ego_vels{ego_fs.vec_s[1], ego_fs.vec_dt[1]};
  std::vector<decimal_t> other_vels{other_fs.vec_s[1], other_fs.vec_dt[1]};
  std::vector<decimal_t> safe_distances;
  CalculateRssSafeDistances(ego_vels, other_vels, lon_direct, lat_direct,
                            config, &safe_distances);

  // 纵向和横向都小于安全距离 → 不安全
  if (fabs(ego_fs.vec_s[0] - other_fs.vec_s[0]) < safe_distances[0] &&
      fabs(ego_fs.vec_dt[0] - other_fs.vec_dt[0]) < safe_distances[1]) {
    *is_safe = false;
  } else {
    *is_safe = true;
  }
  return kSuccess;
}

/*
 * RssCheck - 对两个车辆进行详细的RSS安全检查
 *
 * 此重载版本提供了更详细的RSS检查，包括：
 * 1. 将车辆状态转换到Frenet坐标系
 * 2. 如果本车速度为负（倒车），直接判为安全
 * 3. 横向安全检查：考虑车辆宽度后的横向安全距离
 * 4. 如果横向距离足够大 → 安全（不需要纵向检查）
 * 5. 纵向安全检查：考虑车辆长度和质心偏移后的实际纵向距离
 * 6. 计算安全速度范围，判断本车速度是否在范围内
 *
 * @param ego_vehicle 本车对象
 * @param other_vehicle 它车对象
 * @param stf 状态变换器（用于坐标转换）
 * @param config RSS配置参数
 * @param is_safe 输出参数，true=安全
 * @param lon_type 输出参数，纵向违规类型
 * @param rss_vel_low 输出参数，安全速度下界
 * @param rss_vel_up 输出参数，安全速度上界
 * @return kSuccess
 */
ErrorType RssChecker::RssCheck(const Vehicle& ego_vehicle,
                               const Vehicle& other_vehicle,
                               const StateTransformer& stf, const RssConfig& config,
                               bool* is_safe, LongitudinalViolateType* lon_type,
                               decimal_t* rss_vel_low, decimal_t* rss_vel_up) {
  FrenetState ego_fs, other_fs;
  // 将车辆状态转换到Frenet坐标系
  if (stf.GetFrenetStateFromState(ego_vehicle.state(), &ego_fs) != kSuccess) {
    printf("[RssChecker]ego not on ref lane.\n");
    return kWrongStatus;
  }
  if (stf.GetFrenetStateFromState(other_vehicle.state(), &other_fs) !=
      kSuccess) {
    printf("[RssChecker]other %d not on ref lane.\n", other_vehicle.id());
    return kWrongStatus;
  }

  LongitudinalDirection lon_direct;
  LateralDirection lat_direct;

  if (ego_fs.vec_s[0] >= other_fs.vec_s[0]) {
    lon_direct = Rear;
  } else {
    lon_direct = Front;
  }

  if (ego_fs.vec_dt[0] >= other_fs.vec_dt[0]) {
    lat_direct = Right;
  } else {
    lat_direct = Left;
  }

  // 倒车视为安全
  if (ego_fs.vec_s[1] < 0.0) {
    *is_safe = true;
    *lon_type = LongitudinalViolateType::Legal;
    *rss_vel_up = 0.0;
    *rss_vel_low = 0.0;
    return kSuccess;
  }

  // 横向安全距离（包含车辆宽度附加项）
  decimal_t safe_lat_distance;
  CalculateSafeLateralDistance(ego_fs.vec_dt[1], other_fs.vec_dt[1], lat_direct,
                               config, &safe_lat_distance);
  safe_lat_distance +=
      0.5 * (ego_vehicle.param().width() + other_vehicle.param().width());

  // 横向距离足够 → 安全（不在同一车道，不考虑纵向冲突）
  if (fabs(ego_fs.vec_dt[0] - other_fs.vec_dt[0]) > safe_lat_distance) {
    *is_safe = true;
    *lon_type = LongitudinalViolateType::Legal;
    *rss_vel_up = 0.0;
    *rss_vel_low = 0.0;
    return kSuccess;
  }

  // 计算纵向实际距离（扣除车辆长度和质心偏移）
  decimal_t lon_distance_abs, ego_vel_low, ego_vel_upp;
  if (lon_direct == Rear) {
    // 本车在前，计算本车后端到other前端的距离
    decimal_t other_rear_wheel_to_front_bump =
        0.5 * other_vehicle.param().length() + other_vehicle.param().d_cr();
    decimal_t ego_rear_wheel_to_back_bump =
        fabs(0.5 * ego_vehicle.param().length() - ego_vehicle.param().d_cr());
    lon_distance_abs = fabs(ego_fs.vec_s[0] - other_fs.vec_s[0]) -
                       other_rear_wheel_to_front_bump -
                       ego_rear_wheel_to_back_bump;
  } else if (lon_direct == Front) {
    // 本车在后，计算本车前端到other后端的距离
    decimal_t ego_rear_wheel_to_front_bump =
        0.5 * ego_vehicle.param().length() + ego_vehicle.param().d_cr();
    decimal_t other_rear_wheel_to_back_bump = fabs(
        0.5 * other_vehicle.param().length() - other_vehicle.param().d_cr());
    lon_distance_abs = fabs(ego_fs.vec_s[0] - other_fs.vec_s[0]) -
                       ego_rear_wheel_to_front_bump -
                       other_rear_wheel_to_back_bump;
  }

  // 如果实际纵向距离为负且本车在后 → 已经发生碰撞（太近）
  if (lon_distance_abs < 0.0 && lon_direct == Front) {
    *is_safe = false;
    *lon_type = LongitudinalViolateType::TooFast;
    *rss_vel_up = 0.0;
    *rss_vel_low = 0.0;
    return kSuccess;
  }

  // 计算安全速度范围
  CalculateSafeLongitudinalVelocity(other_fs.vec_s[1], lon_direct,
                                    lon_distance_abs, config, &ego_vel_low,
                                    &ego_vel_upp);
  if (ego_fs.vec_s[1] > ego_vel_upp + kEPS) {
    // 速度超过上界 → TooFast违规
    *is_safe = false;
    *lon_type = LongitudinalViolateType::TooFast;
    *rss_vel_up = ego_vel_upp;
    *rss_vel_low = ego_vel_low;
  } else if (ego_fs.vec_s[1] < ego_vel_low - kEPS) {
    // 速度低于下界 → TooSlow违规
    *is_safe = false;
    *lon_type = LongitudinalViolateType::TooSlow;
    *rss_vel_up = ego_vel_upp;
    *rss_vel_low = ego_vel_low;
  } else {
    // 速度在安全范围内
    *is_safe = true;
    *lon_type = LongitudinalViolateType::Legal;
    *rss_vel_up = 0.0;
    *rss_vel_low = 0.0;
  }
  return kSuccess;
}

}  // namespace common
