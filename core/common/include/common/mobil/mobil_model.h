/**
 * @file mobil_model.h
 * @brief MOBIL（Minimizing Overall Braking Induced by Lane changes）换道决策模型
 *
 * MOBIL 模型是自动驾驶系统中用于车道变换决策的核心数学模型。
 * 该模型通过比较换道前后自车及受影响车辆的加速度变化，评估换道行为
 * 的整体收益，从而决定是否执行换道操作。
 *
 * 模型原理：
 *   - 换道收益 = (acc_n - acc_c) + p * (acc_n_tilda - acc_o_tilda + acc_n' - acc_o')
 *   - 其中 acc_c/acc_n 分别为换道前后自车的加速度
 *   - acc_o_tilda/acc_n_tilda 分别为原车道跟随车在换道前后的加速度
 *   - acc_o'/acc_n' 分别为目标车道跟随车在换道前后的加速度
 *   - p 为利他系数（politeness factor），控制对周围车辆影响的权衡程度
 *   - 换道收益 > 换道阈值时，推荐执行换道
 *
 * 安全约束：
 *   - 换道后目标车道的跟随车加速度必须大于安全制动阈值（-b_safe）
 *   - 使用 RSS 模型验证换道过程中的纵向安全性
 *
 * 在本项目中，MOBIL 模型被用于行为规划层（BehaviorPlanner）的横向决策，
 * 结合 IDM 模型和 RSS 模型完成"是否换道"的判断。
 *
 * 参考文献：
 *   Kesting, A., Treiber, M., & Helbing, D. (2007). General lane-changing model
 *   MOBIL for car-following models. Transportation Research Record.
 *
 * @author EPSILON Autonomous Driving Team
 * @date 2026
 */

#ifndef _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_MODEL_H__
#define _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_MODEL_H__

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/idm/intelligent_driver_model.h"
#include "common/rss/rss_checker.h"

namespace common {

/**
 * @class MobilLaneChangingModel
 * @brief MOBIL 换道决策模型静态工具类
 *
 * 提供换道收益计算的核心方法，分别计算当前车道和目标车道的加速度变化。
 * 内部使用 IDM 模型来计算各车辆在换道前后的期望加速度。
 *
 * @note 该类为纯静态工具类，不可实例化
 */
class MobilLaneChangingModel {
 public:
  /**
   * @brief 计算在当前车道上（未换道时）各车辆的加速度变化
   *
   * 评估如果不换道，自车在当前车道上跟随前车的行为对原车道跟随车的影响。
   * 计算三个加速度值：
   *   - acc_o：原车道跟随车（新跟随车，使用自车作为前车）的加速度
   *   - acc_o_tilda：原车道跟随车（原跟随车，使用原前车作为前车）的加速度
   *   - acc_c：自车在当前车道的加速度
   *
   * @param cur_fs 自车在当前车道上的 Frenet 状态（位置、速度、加速度）
   * @param leading_vehicle 自车在当前车道上的前车（引领车辆）
   * @param leading_fs 前车在当前车道上的 Frenet 状态
   * @param following_vehicle 自车在当前车道上的后车（跟随车辆）
   * @param following_fs 后车在当前车道上的 Frenet 状态
   * @param[out] acc_o 原车道跟随车以自车为前车时的加速度 [m/s^2]
   * @param[out] acc_o_tilda 原车道跟随车以原前车为前车时的加速度 [m/s^2]
   * @param[out] acc_c 自车在当前车道的加速度 [m/s^2]
   * @return ErrorType 执行状态
   */
  static ErrorType GetMobilAccChangesOnCurrentLane(
      const FrenetState &cur_fs, const Vehicle &leading_vehicle,
      const FrenetState &leading_fs, const Vehicle &following_vehicle,
      const FrenetState &following_fs, decimal_t *acc_o, decimal_t *acc_o_tilda,
      decimal_t *acc_c);

  /**
   * @brief 计算在目标车道上（换道后）各车辆的加速度变化
   *
   * 评估如果执行换道，自车在目标车道上对目标车道跟随车的影响，并进行 RSS 安全检查。
   * 计算三个值：
   *   - is_lc_safe：换道是否满足 RSS 安全约束（即目标车道不会发生碰撞风险）
   *   - acc_n：自车在目标车道的加速度
   *   - acc_n_tilda：目标车道跟随车以自车为前车时的加速度
   *   - acc_c_tilda：目标车道跟随车以原前车为前车时的加速度
   *
   * @param projected_cur_fs 自车投影到目标车道后的 Frenet 状态
   * @param leading_vehicle 目标车道上的前车
   * @param leading_fs 目标车道前车的 Frenet 状态
   * @param following_vehicle 目标车道上的后车（跟随车辆）
   * @param following_fs 目标车道后车的 Frenet 状态
   * @param[out] is_lc_safe 输出：换道是否满足 RSS 安全约束
   * @param[out] acc_n 自车在目标车道的加速度 [m/s^2]
   * @param[out] acc_n_tilda 目标车道跟随车以自车为前车的加速度 [m/s^2]
   * @param[out] acc_c_tilda 目标车道跟随车以原前车为前车的加速度 [m/s^2]
   * @return ErrorType 执行状态
   */
  static ErrorType GetMobilAccChangesOnTargetLane(
      const FrenetState &projected_cur_fs, const Vehicle &leading_vehicle,
      const FrenetState &leading_fs, const Vehicle &following_vehicle,
      const FrenetState &following_fs, bool *is_lc_safe, decimal_t *acc_n,
      decimal_t *acc_n_tilda, decimal_t *acc_c_tilda);

 private:
  /**
   * @brief 使用 IDM 模型计算两车之间的期望加速度
   *
   * 内部辅助函数：根据前后车的 Frenet 状态，套用 IDM 模型计算后方车辆的期望加速度。
   *
   * @param param IDM 模型参数（期望速度、加速度、制动减速度等）
   * @param rear_fs 后方车辆（自车）的 Frenet 状态
   * @param front_fs 前方车辆的 Frenet 状态
   * @param use_virtual_front 是否使用虚拟前车（当无真实前车时，创造一个无限远的前车来模拟自由流）
   * @param[out] acc 计算得到的期望加速度 [m/s^2]
   * @return ErrorType 执行状态
   */
  static ErrorType GetDesiredAccelerationUsingIdm(
      const IntelligentDriverModel::Param &param, const FrenetState &rear_fs,
      const FrenetState &front_fs, const bool &use_virtual_front,
      decimal_t *acc);
};

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_MODEL_H__
