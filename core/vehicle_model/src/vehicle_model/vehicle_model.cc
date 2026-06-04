/// @file        vehicle_model.cc
/// @brief       运动学自行车模型（VehicleModel）的实现文件
/// @details     实现了5状态运动学自行车模型的完整逻辑，包括：
///              - 构造函数（默认 + 带参数）
///              - 状态存取（set_state / state）
///              - 控制输入设置
///              - 时间步进（Step）：使用Boost.odeint进行Runge-Kutta Dormand-Prince积分
///              - ODE右端函数（operator()）：定义状态导数的运动学方程
///              - 内部状态同步（UpdateInternalState）
///
///              运动学方程：
///              dx/dt  = cos(theta) * v           -- 沿全局x轴的速度分量
///              dy/dt  = sin(theta) * v           -- 沿全局y轴的速度分量
///              dtheta/dt = tan(delta) * v / L    -- 航向角变化率（横摆角速度）
///              ddelta/dt = steer_rate            -- 前轮转角变化率
///              dv/dt  = acc_long                 -- 纵向加速度
///
///              积分方法：odeint::integrate + runge_kutta_dopri5 (自适应步长)

#include "vehicle_model/vehicle_model.h"

#include "common/math/calculations.h"

#include "odeint-v2/boost/numeric/odeint.hpp"

#include <boost/bind/placeholders.hpp> // Boost Placeholders 支持

using namespace boost::placeholders;   // 启用 _1, _2 等占位符
namespace odeint = boost::numeric::odeint;

namespace simulator {

// ============================================================================
// 构造函数
// ============================================================================

// 默认构造函数：使用默认轴距 2.5m
VehicleModel::VehicleModel() : wheelbase_len_(2.5) { UpdateInternalState(); }

// 带参数构造函数：指定轴距和最大前轮转角
VehicleModel::VehicleModel(double wheelbase_len, double max_steering_angle)
    : wheelbase_len_(wheelbase_len), max_steering_angle_(max_steering_angle) {
  UpdateInternalState();
}

// 析构函数（无特殊清理需求）
VehicleModel::~VehicleModel() {}

// ============================================================================
// 时间步进（核心方法）
// ============================================================================

void VehicleModel::Step(double dt) {
  // 使用Boost.odeint的integrate函数进行一步数值积分
  // 参数：functor(*this), 初始状态, 起始时间0, 终止时间dt, 初始步长dt
  // 内部使用自适应步长的Runge-Kutta Dormand-Prince 5阶方法
  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);

  // 从积分后的内部状态数组恢复到外部状态结构体
  state_.vec_position(0) = internal_state_[0];    // x坐标
  state_.vec_position(1) = internal_state_[1];    // y坐标
  state_.angle = normalize_angle(internal_state_[2]); // 航向角（归一化到[-pi, pi]）
  state_.steer = internal_state_[3];               // 前轮转角
  // 前轮转角限幅：最大转角由硬件/机械约束决定
  if (fabs(state_.steer) >= fabs(max_steering_angle_)) {
    if (state_.steer > 0)
      state_.steer = max_steering_angle_;
    else
      state_.steer = -max_steering_angle_;
  }
  state_.velocity = internal_state_[4];            // 纵向速度
  // 根据前轮转角和轴距计算当前行驶曲率：kappa = tan(delta) / L
  state_.curvature = tan(state_.steer) * 1.0 / wheelbase_len_;
  state_.acceleration = control_.acc_long;         // 加速度直接来自控制输入
  UpdateInternalState();  // 同步到内部数组，为下一步积分准备
}

// ============================================================================
// ODE右端函数（运动学方程）
// ============================================================================

void VehicleModel::operator()(const InternalState &x, InternalState &dxdt,
                              const double /* t */) {
  // 从内部数组构建临时状态（方便使用有名字段）
  State cur_state;
  cur_state.vec_position(0) = x[0];
  cur_state.vec_position(1) = x[1];
  cur_state.angle = x[2];
  cur_state.steer = x[3];
  cur_state.velocity = x[4];

  // 标准运动学自行车模型的状态空间方程：
  // 位置变化 = 速度 * 方向余弦（在全局x轴上的投影）
  dxdt[0] = cos(cur_state.angle) * cur_state.velocity;
  // 位置变化 = 速度 * 方向正弦（在全局y轴上的投影）
  dxdt[1] = sin(cur_state.angle) * cur_state.velocity;
  // 横摆角速度 = tan(前轮转角) * 速度 / 轴距
  dxdt[2] = tan(cur_state.steer) * cur_state.velocity / wheelbase_len_;
  // 前轮转角变化率 = 控制输入的转角速率
  dxdt[3] = control_.steer_rate;
  // 纵向加速度 = 控制输入的纵向加速度
  dxdt[4] = control_.acc_long;
}

// ============================================================================
// 控制输入和状态存取
// ============================================================================

void VehicleModel::set_control(const Control &control) {
  control_ = control;
  // TODO: (@denny.ding) 在此处添加控制限幅逻辑
}

const VehicleModel::State &VehicleModel::state(void) const { return state_; }

void VehicleModel::set_state(const VehicleModel::State &state) {
  state_ = state;
  UpdateInternalState();  // 设置状态后立即同步到内部数组
}

// ============================================================================
// 内部状态同步
// ============================================================================

void VehicleModel::UpdateInternalState(void) {
  // 将外部状态结构体的各字段复制到odeint使用的内部数组中
  // 索引对应关系：
  // 0: vec_position(0) -- 全局x坐标
  // 1: vec_position(1) -- 全局y坐标
  // 2: angle           -- 航向角
  // 3: steer           -- 前轮转角
  // 4: velocity        -- 纵向速度
  internal_state_[0] = state_.vec_position(0);
  internal_state_[1] = state_.vec_position(1);
  internal_state_[2] = state_.angle;
  internal_state_[3] = state_.steer;
  internal_state_[4] = state_.velocity;
}

}  // namespace simulator
