#ifndef _CORE_VEHICLE_MODEL_INC_CONTROLLERS_PID_CONTROLLER_H_
#define _CORE_VEHICLE_MODEL_INC_CONTROLLERS_PID_CONTROLLER_H_

/// @file        pid_controller.h
/// @brief       PID控制器（比例-积分-微分控制器）
/// @details     PID控制器是自动控制领域最经典、应用最广泛的控制算法之一。
///              在EPSILON中，该控制器可用于速度控制、转向控制等多种场景。
///
///              控制器的三个分量：
///              - 比例项（P）：输出与当前误差成正比，提供基本的纠正能力
///              - 积分项（I）：输出与误差累积量成正比，消除稳态误差
///              - 微分项（D）：输出与误差变化率成正比，抑制超调和振荡
///
///              实现细节：
///              - 使用std::deque存储历史误差值，支持积分项和微分项的计算
///              - 设定最大历史长度限制（1000个样本），防止积分器无限增长
///              - 误差过少时（少于2个样本）微分项为0，避免导数噪声

#include "common/basics/basics.h"

#include <deque>

namespace control {

/// @class PIDControl
/// @brief PID控制器类
/// @details 实现标准的位置式PID控制算法。
///
///          控制律公式：
///          u(t) = kP * e(t) + kI * integral(e) + kD * de(t)/dt
///
///          其中积分项采用矩形近似：integral(e) = sum(e_i * dt)对所有历史误差求和
///          微分项采用后向差分：de(t)/dt = (e(t) - e(t-1)) / dt
///
///          使用示例：
///          @code
///          PIDControl::ControlParam param(1.0, 0.1, 0.5);
///          PIDControl pid(param, 0.05);
///          decimal_t control = pid.CalculatePIDControl(desired, actual);
///          @endcode
class PIDControl {
 public:
  /// @struct ControlParam
  /// @brief PID控制器的三个增益参数
  struct ControlParam {
    decimal_t kP;  ///< 比例增益，影响响应速度和超调量
    decimal_t kI;  ///< 积分增益，消除稳态误差，过大会引起振荡
    decimal_t kD;  ///< 微分增益，抑制超调，过大会放大噪声

    /// @brief 默认构造函数，设置保守的参数值
    /// @details 默认参数：kP=1.0, kI=1.0, kD=0.5
    ControlParam() : kP(1.0), kI(1.0), kD(0.5) {}

    /// @brief 显式参数构造函数
    /// @param p 比例增益
    /// @param i 积分增益
    /// @param d 微分增益
    ControlParam(const decimal_t p, const decimal_t i, const decimal_t d)
        : kP(p), kI(i), kD(d) {}
  };

  /// @brief 使用指定参数构造PID控制器（默认时间步长0.05s）
  /// @param param PID增益参数
  PIDControl(const ControlParam& param);

  /// @brief 使用指定参数和自定义时间步长构造PID控制器
  /// @param param PID增益参数
  /// @param dt 时间步长（秒），用于积分和微分项的计算
  PIDControl(const ControlParam& param, const decimal_t dt);

  /// @brief 计算PID控制输出（核心算法）
  /// @param desired_state 期望值（目标状态）
  /// @param true_state 实际值（当前测量/估计状态）
  /// @return PID控制输出量，正值需要增加控制量，负值需要减少控制量
  /// @details 执行流程：
  ///          1. 计算当前误差 e(t) = desired - true
  ///          2. 将误差存入历史队列
  ///          3. 计算积分项：sum(e_i * dt)
  ///          4. 计算微分项：(e(t) - e(t-1)) / dt（至少需要2个样本）
  ///          5. 若历史长度超限，弹出最旧值
  ///          6. 返回 P + I + D 的加权和
  decimal_t CalculatePIDControl(const decimal_t desired_state, const decimal_t true_state);

 private:
  ControlParam param_;              ///< PID三个增益参数
  decimal_t dt_;                    ///< 控制时间步长（秒）
  int max_history_len_;             ///< 最大误差历史长度，防止积分器无限增长
  std::deque<decimal_t> error_hist_; ///< 误差历史记录队列，用于积分项和微分项计算
};

}  // namespace control

#endif
