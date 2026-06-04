/**
 * @file intelligent_driver_model.h
 * @brief 智能驾驶员模型（Intelligent Driver Model，IDM）
 *
 * IDM 是自动驾驶系统中用于纵向跟车控制的核心数学模型。
 * 该模型根据自车与前方车辆之间的相对距离和相对速度，计算期望的纵向加速度，
 * 从而实现对前车的平滑跟车行为。
 *
 * 模型原理：
 *   - 自车加速度由两个竞争项组成：自由加速项（趋向于期望速度）和制动交互项（防止追尾）
 *   - 自由加速项：a * (1 - (v/v0)^δ)，其中 v0 为期望速度，δ 为加速度指数
 *   - 制动交互项：考虑与前车的实际距离与期望安全距离的比值
 *   - 期望安全距离：s0 + v*T + v*(v - v_front)/(2*sqrt(a*b))
 *
 * 在本项目中，IDM 被所有规划器（BehaviorPlanner、EudmPlanner、SscPlanner 等）
 * 用于计算自车及周围车辆的纵向加速度，是行为决策和轨迹规划的基础组件。
 *
 * 参考文献：
 *   Treiber, M., Hennecke, A., & Helbing, D. (2000).
 *   Congested traffic states in empirical observations and microscopic simulations.
 *
 * @author EPSILON Autonomous Driving Team
 * @date 2026
 */

#ifndef _CORE_COMMON_INC_COMMON_IDM_INTELLIGENT_DRIVER_MODEL_H__
#define _CORE_COMMON_INC_COMMON_IDM_INTELLIGENT_DRIVER_MODEL_H__

#include "common/basics/basics.h"

namespace common {

/**
 * @class IntelligentDriverModel
 * @brief 智能驾驶员模型（IDM）静态工具类
 *
 * 提供三种计算期望加速度的静态方法：
 *   - GetIdmDesiredAcceleration：标准 IDM 模型
 *   - GetIIdmDesiredAcceleration：改进型 IDM（IIDM），解决标准 IDM 在某些工况下的不稳定问题
 *   - GetAccDesiredAcceleration：ACC（自适应巡航控制）模式，仅考虑自由加速项
 *
 * @note 该类为纯静态工具类，不可实例化，所有方法均为 static
 */
class IntelligentDriverModel {
 public:
  /**
   * @struct State
   * @brief IDM 模型的状态输入
   *
   * 描述当前时刻自车与前车的纵向运动状态
   */
  struct State {
    decimal_t s{0.0};        ///< 自车的纵向位置（弧长坐标）[m]
    decimal_t v{0.0};        ///< 自车的纵向速度 [m/s]
    decimal_t s_front{0.0};  ///< 前车（引领车辆）的纵向位置 [m]
    decimal_t v_front{0.0};  ///< 前车（引领车辆）的纵向速度 [m/s]

    /// 默认构造函数，所有值初始化为 0
    State() {}
    /// 带参构造函数，直接初始化所有状态变量
    State(const decimal_t &s_, const decimal_t &v_, const decimal_t &s_front_,
          const decimal_t &v_front_)
        : s(s_), v(v_), s_front(s_front_), v_front(v_front_) {}
  };

  /**
   * @struct Param
   * @brief IDM 模型的参数配置
   *
   * 各参数对应 IDM 论文中的标准参数定义，可根据具体驾驶风格进行调节
   */
  struct Param {
    decimal_t kDesiredVelocity = 0.0;                     ///< 期望速度 v0 [m/s]，自车期望达到的自由行驶速度
    decimal_t kVehicleLength = 5.0;                       ///< 车辆有效长度 l_alpha-1 [m]，包含最小间距，用于计算净距
    decimal_t kMinimumSpacing = 2.0;                      ///< 最小停车间距 s0 [m]，静止状态下与前车的最小安全距离
    decimal_t kDesiredHeadwayTime = 1.0;                  ///< 期望安全时距 T [s]，与速度相关的安全跟车时间间隔
    decimal_t kAcceleration = 2.0;                        ///< 最大加速度 a [m/s^2]，自车的最大加速能力
    decimal_t kComfortableBrakingDeceleration = 3.0;      ///< 舒适制动减速度 b [m/s^2]，日常驾驶中的舒适刹车水平
    decimal_t kHardBrakingDeceleration = 5.0;             ///< 紧急制动减速度 [m/s^2]，紧急情况下的最大刹车能力
    int kExponent = 4;                                     ///< 加速度指数 δ，控制加速曲线的平滑度，典型值为 4
  };

  /**
   * @brief 标准 IDM 模型：计算自车的期望纵向加速度
   *
   * 核心公式：acc = a * (1 - (v/v0)^δ - (s*(v, Δv)/s_net)^2)
   * 其中 s*(v, Δv) = s0 + v*T + v*Δv/(2*sqrt(a*b)) 为期望安全距离
   *
   * @param param IDM 模型参数（期望速度、加速度、制动减速度等）
   * @param cur_state 当前状态（自车位置/速度，前车位置/速度）
   * @param[out] acc 计算得到的期望加速度 [m/s^2]
   * @return ErrorType 执行状态，kSuccess 表示计算成功
   *
   * @note 适用于所有速度范围的跟车场景
   * @note 当没有前车或前车距离很远时，该模型退化为自由加速模式
   */
  static ErrorType GetIdmDesiredAcceleration(const Param &param,
                                             const State &cur_state,
                                             decimal_t *acc);

  /**
   * @brief 改进型 IDM 模型（IIDM）：解决标准 IDM 的不稳定问题
   *
   * IIDM 在标准 IDM 的基础上，对制动项进行修正，使跟车行为更加平滑，
   * 特别是在低速跟车和启停场景中避免过度制动。该模型对参数变化更鲁棒，
   * 常用于对舒适性要求较高的场景。
   *
   * @param param IDM 模型参数
   * @param cur_state 当前状态（自车位置/速度，前车位置/速度）
   * @param[out] acc 计算得到的期望加速度 [m/s^2]
   * @return ErrorType 执行状态
   *
   * @note 推荐在低速城市道路及跟车场景中使用
   */
  static ErrorType GetIIdmDesiredAcceleration(const Param &param,
                                              const State &cur_state,
                                              decimal_t *acc);

  /**
   * @brief ACC（自适应巡航控制）模式：仅考虑自由加速项
   *
   * 该模型忽略前车的制动交互项，仅根据自车速度与期望速度的差值计算加速度。
   * 适用于前方无车或前车距离足够远的高速巡航场景。
   * 公式：acc = a * (1 - (v/v0)^δ)
   *
   * @param param IDM 模型参数（主要使用期望速度和加速度指数）
   * @param cur_state 当前状态（主要使用自车速度）
   * @param[out] acc 计算得到的期望加速度 [m/s^2]
   * @return ErrorType 执行状态
   *
   * @note 该模式下不保证对前车的碰撞安全，需配合其他安全模块（如 RSS）使用
   */
  static ErrorType GetAccDesiredAcceleration(const Param &param,
                                             const State &cur_state,
                                             decimal_t *acc);
};

}  // namespace common

#endif
