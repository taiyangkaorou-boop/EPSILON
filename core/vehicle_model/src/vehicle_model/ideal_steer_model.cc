/// @file        ideal_steer_model.cc
/// @brief       理想转向车辆模型（IdealSteerModel）的实现文件
/// @details     实现了带物理约束限幅的增强型运动学自行车模型，
///              是前向仿真中最核心的运动学传播模块。
///
///              核心流程（Step方法）：
///              1. 从当前曲率和轴距反推当前前轮转角
///              2. 同步内部状态数组
///              3. 速度和转角进行初步限幅（非负、最大转角限制）
///              4. 调用TruncateControl()进行物理约束限幅：
///                 a. 纵向：限制加速度（max_lon_acc/max_lon_dec）
///                    和加加速度（max_lon_acc_jerk/max_lon_dec_jerk）
///                 b. 横向：从目标steer+velocity计算期望侧向加速度，
///                    限制侧向加速度（max_lat_acc）
///                    和侧向加加速度（max_lat_jerk）
///                 c. 转向：限制转向速率（max_steer_rate）
///              5. 计算限幅后期的纵向加速度和转向速率
///              6. odeint数值积分
///              7. 从积分结果恢复状态并更新曲率
///
///              与VehicleModel的关键区别：
///              - 控制量不是 steer_rate + acc_long 而是 steer + velocity
///                （值层而非导数层），更易被上层规划模块使用
///              - 内置TruncateControl进行物理约束，保证仿真结果在物理可行域内
///              - ODE函数中使用的是 desired_lon_acc_ 和 desired_steer_rate_
///                （限幅后的值），而非原始控制量

#include "vehicle_model/ideal_steer_model.h"

#include "common/math/calculations.h"

#include "odeint-v2/boost/numeric/odeint.hpp"

#include <boost/bind/placeholders.hpp> // Boost Placeholders 支持

using namespace boost::placeholders;   // 启用 _1, _2 等占位符
namespace odeint = boost::numeric::odeint;

namespace simulator {

// ============================================================================
// 构造函数
// ============================================================================

IdealSteerModel::IdealSteerModel(double wheelbase_len, double max_lon_acc,
                                 double max_lon_dec, double max_lon_acc_jerk,
                                 double max_lon_dec_jerk, double max_lat_acc,
                                 double max_lat_jerk, double max_steering_angle,
                                 double max_steer_rate, double max_curvature)
    : wheelbase_len_(wheelbase_len),
      max_lon_acc_(max_lon_acc),
      max_lon_dec_(max_lon_dec),
      max_lon_acc_jerk_(max_lon_acc_jerk),
      max_lon_dec_jerk_(max_lon_dec_jerk),
      max_lat_acc_(max_lat_acc),
      max_lat_jerk_(max_lat_jerk),
      max_steering_angle_(max_steering_angle),
      max_steer_rate_(max_steer_rate),
      max_curvature_(max_curvature) {
  UpdateInternalState();
}

IdealSteerModel::~IdealSteerModel() {}

// ============================================================================
// 控制限幅（核心方法）
// ============================================================================

void IdealSteerModel::TruncateControl(const decimal_t &dt) {
  // ===== 纵向限幅（加速度/减速度 + 加加速度 ≈ jerk） =====

  // 从目标速度反推期望纵向加速度
  // a_desired = (v_target - v_current) / dt
  desired_lon_acc_ = (control_.velocity - state_.velocity) / dt;

  // 纵向加加速度限幅（Jerk Limit）
  // j_desired = (a_desired - a_current) / dt
  decimal_t desired_lon_jerk = (desired_lon_acc_ - state_.acceleration) / dt;
  // 限幅在 [-max_lon_dec_jerk, max_lon_acc_jerk] 范围内
  // 注意：加速和减速使用不同的Jerk限制，因为人类的加速和减速响应不对称
  desired_lon_jerk =
      truncate(desired_lon_jerk, -max_lon_dec_jerk_, max_lon_acc_jerk_);

  // 用限幅后的Jerk重新计算可行加速度
  desired_lon_acc_ = desired_lon_jerk * dt + state_.acceleration;

  // 加速度/减速度限幅
  desired_lon_acc_ = truncate(desired_lon_acc_, -max_lon_dec_, max_lon_acc_);

  // 用限幅后的加速度重新计算可行速度（保证非负）
  control_.velocity = std::max(state_.velocity + desired_lon_acc_ * dt, 0.0);

  // ===== 横向限幅（侧向加速度 + 侧向加加速度） =====

  // 计算期望侧向加速度（向心加速度）
  // a_lat = v^2 * kappa = v^2 * tan(delta) / L
  desired_lat_acc_ =
      pow(control_.velocity, 2) * (tan(control_.steer) / wheelbase_len_);

  // 当前实际侧向加速度
  decimal_t lat_acc_ori = pow(state_.velocity, 2) * state_.curvature;

  // 侧向加加速度限幅（Lateral Jerk Limit）
  decimal_t lat_jerk_desired = (desired_lat_acc_ - lat_acc_ori) / dt;
  lat_jerk_desired = truncate(lat_jerk_desired, -max_lat_jerk_, max_lat_jerk_);

  // 用限幅后的侧向Jerk重新计算可行侧向加速度
  desired_lat_acc_ = lat_jerk_desired * dt + lat_acc_ori;

  // 侧向加速度限幅
  desired_lat_acc_ = truncate(desired_lat_acc_, -max_lat_acc_, max_lat_acc_);

  // 从限幅后的侧向加速度反推可行前轮转角
  // delta = atan(a_lat * L / v^2)
  // 分母加保护项防止除零（取 max(v^2, 0.1*kBigEPS)）
  control_.steer = atan(desired_lat_acc_ * wheelbase_len_ /
                        std::max(pow(control_.velocity, 2), 0.1 * kBigEPS));

  // 转向速率限幅（Steer Rate Limit）
  desired_steer_rate_ = normalize_angle(control_.steer - state_.steer) / dt;
  desired_steer_rate_ =
      truncate(desired_steer_rate_, -max_steer_rate_, max_steer_rate_);

  // 用限幅后的转向速率重新计算可行转角
  control_.steer = normalize_angle(state_.steer + desired_steer_rate_ * dt);
}

// ============================================================================
// 时间步进
// ============================================================================

void IdealSteerModel::Step(double dt) {
  // 步骤1：从当前曲率和轴距反推当前前轮转角（因为曲率是外部状态的一部分）
  state_.steer = atan(state_.curvature * wheelbase_len_);
  UpdateInternalState();

  // 步骤2：控制量初步限幅
  control_.velocity = std::max(0.0, control_.velocity);  // 速度非负
  control_.steer =
      truncate(control_.steer, -max_steering_angle_, max_steering_angle_);  // 转角机械限位

  // 步骤3：执行物理约束限幅（TruncateControl）
  TruncateControl(dt);

  // 步骤4：限幅后重新计算期望加速度和转向速率
  desired_lon_acc_ = (control_.velocity - state_.velocity) / dt;
  desired_steer_rate_ = normalize_angle(control_.steer - state_.steer) / dt;

  // 步骤5：使用odeint进行一步数值积分
  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);

  // 步骤6：从积分结果恢复状态
  state_.vec_position(0) = internal_state_[0];    // x坐标
  state_.vec_position(1) = internal_state_[1];    // y坐标
  state_.angle = normalize_angle(internal_state_[2]); // 航向角
  state_.velocity = internal_state_[3];            // 纵向速度
  state_.steer = normalize_angle(internal_state_[4]); // 前轮转角
  state_.curvature = tan(state_.steer) * 1.0 / wheelbase_len_; // 曲率
  state_.acceleration = desired_lon_acc_;         // 加速度（使用限幅后的值）
  UpdateInternalState();  // 同步到内部数组
}

// ============================================================================
// ODE右端函数（运动学方程）
// ============================================================================

void IdealSteerModel::operator()(const InternalState &x, InternalState &dxdt,
                                 const double /* t */) {
  // 从内部数组构建临时状态
  State cur_state;
  cur_state.vec_position(0) = x[0];
  cur_state.vec_position(1) = x[1];
  cur_state.angle = x[2];
  cur_state.velocity = x[3];
  cur_state.steer = x[4];

  // 标准运动学自行车模型方程
  dxdt[0] = cos(cur_state.angle) * cur_state.velocity;  // dx/dt
  dxdt[1] = sin(cur_state.angle) * cur_state.velocity;  // dy/dt
  dxdt[2] = tan(cur_state.steer) * cur_state.velocity / wheelbase_len_;  // dtheta/dt
  // 注意：此处使用限幅后的期望值而非原始控制量
  dxdt[3] = desired_lon_acc_;       // dv/dt（使用限幅后的期望加速度）
  dxdt[4] = desired_steer_rate_;    // ddelta/dt（使用限幅后的转向速率）
}

// ============================================================================
// 控制输入和状态存取
// ============================================================================

void IdealSteerModel::set_control(const Control &control) {
  control_ = control;
}

const IdealSteerModel::State &IdealSteerModel::state(void) const {
  return state_;
}

void IdealSteerModel::set_state(const IdealSteerModel::State &state) {
  state_ = state;
  UpdateInternalState();
}

// ============================================================================
// 内部状态同步
// ============================================================================

void IdealSteerModel::UpdateInternalState(void) {
  // 将外部状态结构体的各字段复制到odeint使用的内部数组中
  // 索引对应关系：
  // 0: vec_position(0) -- 全局x坐标
  // 1: vec_position(1) -- 全局y坐标
  // 2: angle           -- 航向角
  // 3: velocity        -- 纵向速度
  // 4: steer           -- 前轮转角
  internal_state_[0] = state_.vec_position(0);
  internal_state_[1] = state_.vec_position(1);
  internal_state_[2] = state_.angle;
  internal_state_[3] = state_.velocity;
  internal_state_[4] = state_.steer;
}

}  // namespace simulator
