/**
 * @file mobil_behavior_prediction.h
 * @brief 基于 MOBIL 模型的周围车辆行为预测
 *
 * 该模块利用 MOBIL 换道决策模型对周围车辆的未来横向行为进行概率性预测。
 * 与直接用于自车决策不同，此模块从周围车辆的视角出发，模拟周围车辆
 * 是否会执行车道变换操作，从而为自车的轨迹规划提供更准确的环境预测。
 *
 * 预测流程：
 *   1. 对每辆周围车辆，收集其在不同车道上的前车和后车信息
 *   2. 使用 MOBIL 模型计算向左换道和向右换道的收益（mobil_gain_left/right）
 *   3. 使用 RSS 模型验证换道安全性（is_lcl_safe/is_lcr_safe）
 *   4. 通过 RemapGainsToProb 将 MOBIL 收益映射为横向行为概率分布
 *
 * 在本项目中，该预测结果用于 BehaviorPlanner 和 EudmPlanner 等规划器中，
 * 作为环境预测的一部分来指导自车的行为决策。
 *
 * @author EPSILON Autonomous Driving Team
 * @date 2026
 */

#ifndef _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_BEHAVIOR_PREDICTION_H__
#define _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_BEHAVIOR_PREDICTION_H__

#include <set>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/idm/intelligent_driver_model.h"
#include "common/mobil/mobil_model.h"
#include "common/rss/rss_checker.h"

namespace common {

/**
 * @class MobilBehaviorPrediction
 * @brief 基于 MOBIL 模型的周围车辆横向行为预测工具类
 *
 * 提供两个静态方法：
 *   - LateralBehaviorPrediction：综合计算周围车辆可能的横向行为概率
 *   - RemapGainsToProb：将 MOBIL 换道收益映射为概率分布
 *
 * @note 该类为纯静态工具类，不可实例化
 * @note 预测结果以 ProbDistOfLatBehaviors 形式输出，
 *       包含保持车道、左换道、右换道三种行为的概率
 */
class MobilBehaviorPrediction {
 public:
  /**
   * @brief 对指定周围车辆进行横向行为预测
   *
   * 综合 MOBIL 模型和 RSS 安全检查，计算该车辆执行左换道、右换道和保持车道
   * 三种横向行为的概率分布。预测过程考虑以下因素：
   *   - 车辆在各个车道上的前车和后车分布
   *   - MOBIL 换道收益（考虑利他系数和换道阈值）
   *   - RSS 安全约束（换道后目标车道的纵向安全性）
   *
   * @param vehicle 待预测的周围车辆对象（包含位置、速度、朝向等状态）
   * @param lanes 所有可用车道的集合（用于判断哪些车道可达）
   * @param leading_vehicles 各车道上该车辆的前车（引领车辆）列表
   * @param leading_frenet_states 各前车的 Frenet 状态列表
   * @param following_vehicles 各车道上该车辆的后车（跟随车辆）列表
   * @param follow_frenet_states 各后车的 Frenet 状态列表
   * @param nearby_vehicles 附近所有车辆的集合（用于构建环境上下文）
   * @param[out] res 预测结果：横向行为（保持/左换道/右换道）的概率分布
   * @return ErrorType 执行状态，kSuccess 表示预测成功
   *
   * @note 如果某方向车道不存在或不可达，对应的换道概率为零
   */
  static ErrorType LateralBehaviorPrediction(
      const Vehicle &vehicle, const vec_E<Lane> &lanes,
      const vec_E<common::Vehicle> &leading_vehicles,
      const vec_E<common::FrenetState> &leading_frenet_states,
      const vec_E<common::Vehicle> &following_vehicles,
      const vec_E<common::FrenetState> &follow_frenet_states,
      const common::VehicleSet &nearby_vehicles, ProbDistOfLatBehaviors *res);

  /**
   * @brief 将 MOBIL 换道收益映射为横向行为概率分布
   *
   * 映射策略：
   *   - 如果换道不安全（不满足 RSS 约束），对应的换道概率为 0
   *   - 如果换道收益为负，换道概率极低（但仍保留小概率）
   *   - 使用高斯或 Sigmoid 型函数将收益值映射为 [0, 1] 范围内的概率
   *   - 保持车道的概率 = 1 - P(左换道) - P(右换道)
   *
   * @param is_lcl_safe 向左换道是否满足 RSS 安全约束
   * @param mobil_gain_left 左换道的 MOBIL 收益值（正值表示有利，负值表示不利）
   * @param is_lcr_safe 向右换道是否满足 RSS 安全约束
   * @param mobil_gain_right 右换道的 MOBIL 收益值
   * @param[out] res 输出的横向行为概率分布
   * @return ErrorType 执行状态
   *
   * @note 该函数需要确保输出概率分布的和为 1，且各分量在 [0, 1] 范围内
   */
  static ErrorType RemapGainsToProb(const bool is_lcl_safe,
                                    const decimal_t mobil_gain_left,
                                    const bool is_lcr_safe,
                                    const decimal_t mobil_gain_right,
                                    ProbDistOfLatBehaviors *res);
};

}  // namespace common

#endif
