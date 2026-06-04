/**
 * @file rss_checker.h
 * @brief RSS（Responsibility-Sensitive Safety）责任敏感安全模型
 *
 * RSS 模型是 Mobileye 提出的形式化安全框架，用于确保自动驾驶车辆在所有
 * 驾驶场景下都能保持安全距离，避免碰撞。该模型将人类驾驶中的"合理注意义务"
 * 和"防御性驾驶"概念形式化为严格的数学约束。
 *
 * 核心概念：
 *   - 纵向安全距离：自车与前车/后车之间的最小安全距离，考虑了反应时间、
 *     最大加速度、最小制动力等参数
 *   - 横向安全距离：两车并排行驶或换道时的最小侧向安全距离
 *   - 安全包络（Safe Envelope）：在纵向和横向上定义的安全区域，自车必须
 *     保持在该区域内
 *
 * RSS 公式：
 *   - 纵向安全距离（自车在前方时）：
 *     d_min = v_r * ρ + 0.5 * a_acc_max * ρ^2 + (v_r + ρ*a_acc_max)^2/(2*b_brake_min)
 *            - v_f^2 / (2*b_brake_max)
 *     其中 ρ 为反应时间，v_r/v_f 为后方/前方车速
 *
 * 在本项目中，RSS 被用于：
 *   - 验证规划轨迹的安全性（碰撞检测）
 *   - MOBIL 换道决策中的安全约束检查
 *   - 轨迹优化中的约束条件构建
 *
 * 参考文献：
 *   Shalev-Shwartz, S., Shammah, S., & Shashua, A. (2017).
 *   On a Formal Model of Safe and Scalable Self-driving Cars. arXiv:1708.06374.
 *
 * @author EPSILON Autonomous Driving Team
 * @date 2026
 */

#ifndef _CORE_COMMON_INC_COMMON_RSS_CHECKER_H__
#define _CORE_COMMON_INC_COMMON_RSS_CHECKER_H__

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"

namespace common {

/**
 * @class RssChecker
 * @brief RSS 责任敏感安全检查器
 *
 * 提供完整的 RSS 安全验证功能，包括：
 *   - 纵向和横向安全距离计算
 *   - 两车之间的 RSS 安全检查（Frenet 坐标系和笛卡尔坐标系两种接口）
 *   - 安全速度范围计算（根据当前距离反推允许的速度上下限）
 *
 * @note 该类为纯静态工具类，不可实例化
 */
class RssChecker {
 public:
  /// @enum LongitudinalDirection 纵向方向枚举，指示被检查车辆相对于自车的位置
  enum LongitudinalDirection { Front = 0, Rear };

  /// @enum LateralDirection 横向方向枚举，指示被检查车辆相对于自车的侧向位置
  enum LateralDirection { Left = 0, Right };

  /**
   * @enum LongitudinalViolateType
   * @brief 纵向 RSS 违规类型枚举
   *
   * 用于描述 RSS 检查失败的详细原因，便于调试和决策调整
   */
  enum class LongitudinalViolateType {
    Legal = 0,   ///< 合法，满足 RSS 约束
    TooFast,     ///< 速度过快：超过 RSS 允许的安全速度上限
    TooSlow      ///< 速度过慢：低于 RSS 要求的安全速度下限
  };

  /**
   * @struct RssConfig
   * @brief RSS 模型的参数配置
   *
   * 所有参数对应于 RSS 论文中的标准参数定义，可根据自动驾驶系统的
   * 性能指标（响应速度、制动能力等）进行标定
   */
  struct RssConfig {
    decimal_t response_time = 0.1;               ///< 反应时间 ρ [s]，系统感知到执行制动的总延迟
    decimal_t longitudinal_acc_max = 2.0;        ///< 最大纵向加速度 [m/s^2]
    decimal_t longitudinal_brake_min = 4.0;      ///< 最小纵向制动减速度（舒适制动）[m/s^2]
    decimal_t longitudinal_brake_max = 5.0;      ///< 最大纵向制动减速度（紧急制动）[m/s^2]
    decimal_t lateral_acc_max = 1.0;             ///< 最大横向加速度（换道时的侧向机动能力）[m/s^2]
    decimal_t lateral_brake_min = 1.0;           ///< 最小横向制动减速度（舒适侧向制动）[m/s^2]
    decimal_t lateral_brake_max = 1.0;           ///< 最大横向制动减速度（紧急侧向制动）[m/s^2]
    decimal_t lateral_miu = 0.5;                 ///< 横向摩擦系数 μ，影响横向安全距离的计算

    /// 默认构造函数
    RssConfig() {}
    /// 带参构造函数，一次性配置所有 RSS 参数
    RssConfig(const decimal_t _response_time,
              const decimal_t _longitudinal_acc_max,
              const decimal_t _longitudinal_brake_min,
              const decimal_t _longitudinal_brake_max,
              const decimal_t _lateral_acc_max,
              const decimal_t _lateral_brake_min,
              const decimal_t _lateral_brake_max, const decimal_t _lateral_miu)
        : response_time(_response_time),
          longitudinal_acc_max(_longitudinal_acc_max),
          longitudinal_brake_min(_longitudinal_brake_min),
          longitudinal_brake_max(_longitudinal_brake_max),
          lateral_acc_max(_lateral_acc_max),
          lateral_brake_min(_lateral_brake_min),
          lateral_brake_max(_lateral_brake_max),
          lateral_miu(_lateral_miu) {}
  };

  /**
   * @brief 计算纵向安全距离
   *
   * 根据 RSS 公式计算自车与另一车辆之间所需的最小纵向安全距离。
   *
   * @param ego_vel 自车的纵向速度 [m/s]
   * @param other_vel 另一车辆的纵向速度 [m/s]
   * @param direction 方向指示：自车在前方（Front）还是后方（Rear）
   * @param config RSS 参数配置
   * @param[out] distance 计算得到的最小安全距离（绝对值）[m]
   * @return ErrorType 执行状态
   *
   * @note 当 direction == Front 时，计算后方来车需要保持的距离；
   *       当 direction == Rear 时，计算自车需要与前车保持的距离
   */
  static ErrorType CalculateSafeLongitudinalDistance(
      const decimal_t ego_vel, const decimal_t other_vel,
      const LongitudinalDirection& direction, const RssConfig& config,
      decimal_t* distance);

  /**
   * @brief 计算横向安全距离
   *
   * 根据 RSS 横向安全公式计算两车并排行驶所需的最小横向安全距离。
   *
   * @param ego_vel 自车的横向速度 [m/s]
   * @param other_vel 另一车辆的横向速度 [m/s]
   * @param direction 方向指示：另一车辆在左侧（Left）还是右侧（Right）
   * @param config RSS 参数配置
   * @param[out] distance 计算得到的最小横向安全距离 [m]
   * @return ErrorType 执行状态
   */
  static ErrorType CalculateSafeLateralDistance(
      const decimal_t ego_vel, const decimal_t other_vel,
      const LateralDirection& direction, const RssConfig& config,
      decimal_t* distance);

  /**
   * @brief 批量计算 RSS 安全距离
   *
   * 对一组速度和方向输入，批量计算对应的安全距离向量。
   *
   * @param ego_vels 自车速度序列
   * @param other_vels 对方车辆速度序列（必须与 ego_vels 长度相同）
   * @param long_direct 纵向方向
   * @param lat_direct 横向方向
   * @param config RSS 参数配置
   * @param[out] safe_distances 输出的安全距离向量（与输入序列一一对应）
   * @return ErrorType 执行状态
   *
   * @note 用于对一条轨迹上所有采样点进行批量安全检查
   */
  static ErrorType CalculateRssSafeDistances(
      const std::vector<decimal_t>& ego_vels,
      const std::vector<decimal_t>& other_vels,
      const LongitudinalDirection& long_direct,
      const LateralDirection& lat_direct, const RssConfig& config,
      std::vector<decimal_t>* safe_distances);

  /**
   * @brief RSS 安全检查（Frenet 坐标系版本）
   *
   * 在 Frenet 坐标系下，检查自车与另一车辆是否满足 RSS 安全约束。
   *
   * @param ego_fs 自车的 Frenet 状态（s, d 及其导数）
   * @param other_fs 对方车辆的 Frenet 状态
   * @param config RSS 参数配置
   * @param[out] is_safe 输出：是否满足安全约束
   * @return ErrorType 执行状态
   */
  static ErrorType RssCheck(const FrenetState& ego_fs,
                            const FrenetState& other_fs,
                            const RssConfig& config, bool* is_safe);

  /**
   * @brief RSS 安全检查（笛卡尔坐标系版本，更详细的输出）
   *
   * 在笛卡尔坐标系下，通过 StateTransformer 转换后进行 RSS 检查，
   * 并输出详细的违规类型和允许的速度范围。
   *
   * @param ego_vehicle 自车对象（包含完整的车辆状态）
   * @param other_vehicle 对方车辆对象
   * @param stf 状态转换器（用于笛卡尔坐标与 Frenet 坐标之间的转换）
   * @param config RSS 参数配置
   * @param[out] is_safe 输出：是否满足安全约束
   * @param[out] lon_type 输出：纵向违规类型（Legal/TooFast/TooSlow）
   * @param[out] rss_vel_low 输出：RSS 允许的最低速度 [m/s]
   * @param[out] rss_vel_up 输出：RSS 允许的最高速度 [m/s]
   * @return ErrorType 执行状态
   *
   * @note 该版本输出更详细的信息，适合用于调试和安全分析
   */
  static ErrorType RssCheck(const Vehicle& ego_vehicle,
                            const Vehicle& other_vehicle, const StateTransformer& stf,
                            const RssConfig& config, bool* is_safe,
                            LongitudinalViolateType* lon_type,
                            decimal_t* rss_vel_low, decimal_t* rss_vel_up);

  /**
   * @brief 根据当前距离反推允许的安全速度范围
   *
   * 给定当前纵向距离，通过逆解 RSS 公式计算自车的安全速度上下限。
   * 可用于轨迹优化中的速度约束构建。
   *
   * @param other_vel 对方车辆的纵向速度 [m/s]
   * @param direction 方向指示（自车在前方还是后方）
   * @param lon_distance_abs 两车之间的实际纵向距离（绝对值）[m]
   * @param config RSS 参数配置
   * @param[out] ego_vel_low 输出：自车允许的最低安全速度 [m/s]
   * @param[out] ego_vel_upp 输出：自车允许的最高安全速度 [m/s]
   * @return ErrorType 执行状态
   *
   * @note 如果当前距离已经小于安全距离，则速度范围可能为空或非常窄
   */
  static ErrorType CalculateSafeLongitudinalVelocity(
      const decimal_t other_vel, const LongitudinalDirection& direction,
      const decimal_t& lon_distance_abs, const RssConfig& config,
      decimal_t* ego_vel_low, decimal_t* ego_vel_upp);
};

}  // namespace common

#endif
