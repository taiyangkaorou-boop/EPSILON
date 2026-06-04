/// @file        pid_controller.cc
/// @brief       PID控制器（比例-积分-微分控制器）的实现文件
/// @details     实现了标准的位置式PID控制算法，可用于EPSILON中的速度控制、
///              转向控制等多种闭环控制场景。
///
///              控制律：
///              u(t) = kP * e(t) + kI * integral_0^t e(tau)*dtau + kD * de(t)/dt
///
///              离散化实现：
///              - 比例项（P）：kP * e(t)
///              - 积分项（I）：kI * sum(e_i * dt) 对所有历史误差求和
///              - 微分项（D）：kD * (e(t) - e(t-1)) / dt（需要至少2个历史样本）
///
///              防积分饱和（Anti-windup）措施：
///              - 使用std::deque存储历史误差
///              - 最大历史长度限制为max_history_len_（默认1000），超限时弹出最旧值
///              - 当误差历史不足2个样本时，微分项置0（避免噪声放大）
///
///              使用场景：
///              - 速度闭环控制：期望速度 vs 实际速度
///              - 位置闭环控制：期望位置 vs 实际位置
///              - 转角闭环控制：期望转角 vs 实际转角

#include "vehicle_model/controllers/pid_controller.h"

namespace control {

// ============================================================================
// 构造函数
// ============================================================================

// 使用默认时间步长（0.05s）构造PID控制器
PIDControl::PIDControl(const ControlParam& param)
    : param_(param), dt_(0.05), max_history_len_(1000) {
  printf("default PID controller with dt: %lf ms.\n", dt_);
}

// 使用自定义时间步长构造PID控制器
PIDControl::PIDControl(const ControlParam& param, const decimal_t dt)
    : param_(param), dt_(dt), max_history_len_(1000) {}

// ============================================================================
// 计算PID控制输出（核心算法）
// ============================================================================

decimal_t PIDControl::CalculatePIDControl(const decimal_t desired_state,
                                          const decimal_t true_state) {
  // ---- 步骤1：计算当前误差 ----
  // e(t) = desired - true，正值表示真实状态小于期望（需要增加控制量）
  decimal_t et = desired_state - true_state;
  error_hist_.push_back(et);  // 将当前误差加入历史队列

  // ---- 步骤2：计算积分项（Integral） ----
  // I = sum(e_i * dt) 对所有历史误差求和
  // 矩形积分近似，每个误差项乘以时间间隔
  decimal_t int_e = 0.0;
  for (auto& err : error_hist_) {
    int_e += err * dt_;
  }

  // ---- 步骤3：计算微分项（Derivative） ----
  // D = (e(t) - e(t-1)) / dt
  // 需要至少2个历史样本才能计算有限差分近似
  decimal_t deriv_e = 0.0;
  int num_errors = static_cast<int>(error_hist_.size());
  if (num_errors > 2) {
    deriv_e =
        (error_hist_.at(num_errors - 1) - error_hist_.at(num_errors - 2)) / dt_;
  }
  // 注意：当 num_errors == 2 时，也能计算微分但需要仔细验证
  // 当前条件设置为 >2 而非 >=2，意味着需要至少3个样本才启用D项
  // 这可能是为了避免初始阶段的微分噪声

  // ---- 步骤4：防积分饱和 ----
  // 限制误差历史长度，防止积分器无限增长（Anti-windup）
  if (num_errors > max_history_len_) error_hist_.pop_front();

  // ---- 步骤5：计算PID输出 ----
  // u = kP * e + kI * sum(e*dt) + kD * delta_e/dt
  return param_.kP * et + param_.kI * int_e + param_.kD * deriv_e;
}

}  // namespace control
