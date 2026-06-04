/**
 * @file frenet_primitive.h
 * @brief Frenet 运动基元（Frenet Motion Primitive）
 *
 * Frenet 运动基元是 EPSILON 轨迹规划系统中用于表示短时运动片段的核心数据结构。
 * 每个基元在 Frenet 坐标系下使用两个五次多项式分别描述纵向 (s) 和横向 (d) 的运动：
 *   - poly_s(t)：五次多项式，描述纵向弧长随时间的演化 s(t)
 *   - poly_d(t)：五次多项式，描述横向偏移随时间的演化 d(t)
 *
 * 为什么选择五次多项式？
 *   - 6 个自由度（系数）= 2 个边界条件 * 3 个约束（位置、速度、加速度）
 *   - 能够精确满足起止状态的完全匹配（C2 连续性）
 *   - 保证了轨迹在位置、速度和加速度层面的光滑过渡
 *   - jerk 是二次曲线，可以被量化评估
 *
 * 基元的两种构造模式：
 *   1. Connect 模式（两点边值问题）：
 *      给定起始 FrenetState (fs0) 和终止 FrenetState (fs1)，
 *      通过求解线性方程组确定五次多项式系数。
 *   2. Propagate 模式（前向仿真）：
 *      给定起始 FrenetState (fs0) 和控制输入 u = (u_lon, u_lat)，
 *      通过积分动力学方程生成未来状态。该模式用于前向仿真和
 *      行为预测中的轨迹生成。
 *
 * 横向独立模式（is_lateral_independent）：
 *   - true（高速模式）：横向偏移 d 与纵向弧长 s 独立参数化
 *     通过 s 的变化来间接反映横向速度约束
 *   - false（低速模式）：横向偏移 d 直接通过时间参数化
 *
 * 在本项目中，FrenetPrimitive 被 EUDM 和 SSC 等规划器广泛使用，
 * 是轨迹采样、评估和优化链条中的核心数据结构。
 *
 * @author EPSILON Autonomous Driving Team
 * @date 2026
 */

#ifndef _CORE_COMMON_INC_COMMON_FRENET_PRIMITIVE_H__
#define _CORE_COMMON_INC_COMMON_FRENET_PRIMITIVE_H__

#include "common/spline/polynomial.h"
#include "common/state/frenet_state.h"

namespace common {

/**
 * @class FrenetPrimitive
 * @brief Frenet 坐标系下的运动基元
 *
 * 封装一段由两个五次多项式描述的短时运动，支持两种构造模式（Connect 和 Propagate）
 * 以及后续的状态查询和采样。
 */
class FrenetPrimitive {
 public:
  /// 默认构造函数
  FrenetPrimitive() {}

  /**
   * @brief 两点连接模式：从起始状态连接到终止状态（Solve BVP）
   *
   * 给定起止 FrenetState，求解五次多项式系数使得轨迹精确通过起止点，
   * 并满足速度、加速度的边界条件。
   *
   * @param fs0 起始 Frenet 状态（位置、速度、加速度）
   * @param fs1 终止 Frenet 状态
   * @param stamp 起始时间戳 [s]
   * @param T 基元持续时间 [s]
   * @param is_lateral_independent 是否启用横向独立模式
   *        - true：d 与 s 独立参数化（高速模式）
   *        - false：d 直接通过时间 t 参数化（低速模式）
   * @return ErrorType kSuccess 表示构造成功
   */
  ErrorType Connect(const FrenetState& fs0, const FrenetState& fs1,
                    const decimal_t stamp, const decimal_t T,
                    bool is_lateral_independent);

  /**
   * @brief 前向传播模式：从起始状态施加控制输入（Forward Simulation）
   *
   * 给定起始状态和纵向/横向控制输入，通过积分动力学方程得到终止状态，
   * 然后求解五次多项式。
   *
   * @param fs0 起始 Frenet 状态
   * @param u 控制输入向量 (u_lon, u_lat)，
   *          u_lon 为纵向速度增量或加速度 [m/s^2]，
   *          u_lat 为横向控制量（与换道偏移相关）
   * @param stamp 起始时间戳 [s]
   * @param T 基元持续时间 [s]
   * @return ErrorType kSuccess 表示构造成功
   */
  ErrorType Propagate(const FrenetState& fs0, const Vecf<2>& u,
                      const decimal_t stamp, const decimal_t T);

  /// @name 时间范围
  /// @{
  decimal_t begin() const { return stamp_; }       ///< 起始时间戳
  decimal_t end() const { return stamp_ + duration_; }  ///< 结束时间戳 = stamp + T
  /// @}

  /**
   * @brief 获取指定全局时刻的 Frenet 状态
   *
   * @param[in] t_global 全局时间戳 [s]
   * @param[out] fs 输出的 Frenet 状态
   * @return ErrorType kSuccess 表示查询成功
   *
   * @note 当 t_global 超出参数化范围时，将应用外推（extrapolation）
   */
  ErrorType GetFrenetState(const decimal_t t_global, FrenetState* fs) const;

  /**
   * @brief 对基元进行均匀采样，获取离散的 Frenet 状态序列
   *
   * @param step 采样时间步长 [s]
   * @param offset 采样偏移时间 [s]
   * @param[out] fs_vec 输出的 FrenetState 向量
   * @return ErrorType
   *
   * @note 用于轨迹评估中的逐点碰撞检测和约束检查
   */
  ErrorType GetFrenetStateSamples(const decimal_t step, const decimal_t offset,
                                  vec_E<FrenetState>* fs_vec) const;

  /**
   * @brief 获取基元的 jerk（加加速度）
   *
   * jerk 是五次多项式曲线上的加速度变化率（三阶导数），
   * 在五次多项式上为非零常数。
   *
   * @param[out] c_s 纵向 jerk [m/s^3]
   * @param[out] c_d 横向 jerk [m/s^3]
   * @return ErrorType
   */
  ErrorType GetJ(decimal_t* c_s, decimal_t* c_d) const;

  /**
   * @brief 获取横向运动的持续时间
   *
   * 在横向独立模式下，横向运动可能在纵向运动结束前就已完成
   * （如换道完成后保持在新车道上）
   *
   * @return decimal_t 横向运动持续时间 [s]
   */
  decimal_t lateral_T() const;

  /**
   * @brief 获取纵向运动的持续时间
   *
   * @return decimal_t 纵向运动持续时间 [s]
   */
  decimal_t longitudial_T() const;

  /// @name 边界状态访问器
  /// @{
  FrenetState fs1() const;  ///< 终止 Frenet 状态
  FrenetState fs0() const;  ///< 起始 Frenet 状态
  /// @}

  /// @name 多项式访问器
  /// @{
  Polynomial<5> poly_s() const { return poly_s_; }  ///< 纵向五次多项式 s(t)
  Polynomial<5> poly_d() const { return poly_d_; }  ///< 横向五次多项式 d(t)
  /// @}

  /// @name 多项式设置器（用于优化后回写）
  /// @{
  void set_poly_s(const Polynomial<5>& poly) { poly_s_ = poly; }
  void set_poly_d(const Polynomial<5>& poly) { poly_d_ = poly; }
  /// @}

  /**
   * @brief 调试打印
   *
   * 打印基元的时间范围、多项式系数以及起止 Frenet 状态。
   */
  void print() const {
    printf("frenet primitive in duration [%lf, %lf].\n", begin(), end());
    poly_s_.print();
    poly_d_.print();
    fs0_.print();
    fs1_.print();
  }

  bool is_lateral_independent_ = false;  ///< 是否启用横向独立模式（高速模式）

 private:
  Polynomial<5> poly_s_;  ///< 纵向五次多项式 s(t)  — 描述沿道路弧长的行进
  Polynomial<5> poly_d_;  ///< 横向五次多项式 d(t)  — 描述相对参考线的横向偏移
  decimal_t stamp_{0.0};   ///< 起始时间戳 [s]
  decimal_t duration_{0.0}; ///< 基元持续时间 T [s]
  FrenetState fs0_;         ///< 起始 Frenet 状态
  FrenetState fs1_;         ///< 终止 Frenet 状态
  decimal_t kSmallDistanceThreshold_ = 2.0;  ///< 小距离阈值：距离小于此值时某些计算使用简化公式
};

}  // namespace common

#endif
