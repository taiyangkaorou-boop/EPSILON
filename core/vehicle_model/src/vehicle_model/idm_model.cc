/// @file        idm_model.cc
/// @brief       智能驾驶员模型（IntelligentDriverModel）的实现文件
/// @details     实现了IDM纵向跟车模型的完整逻辑，包括：
///              - 构造函数（默认 + 带参数）
///              - 状态存取（set_state / state）
///              - 时间步进（Step）：使用Boost.odeint进行数值积分
///              - Linear预测量（未启用，用于调试对比）
///              - ODE右端函数（operator()）：计算IDM加速度并定义状态导数
///              - 内部状态同步（UpdateInternalState）
///
///              IDM加速度公式（在common::IntelligentDriverModel中实现）：
///              a_desired = kAcc * (1 - (v/v0)^delta - (s_star/s_net)^2)
///              其中：
///              s_star = kMinSpacing + max(0, v*kT + v*dv/(2*sqrt(kAcc*kComfBrakeDec)))
///              s_net = s_front - s (净间距，需减去车长)
///              dv = v - v_front (速度差)
///
///              限幅保护：加速度不低于 -min(kHardBrakingDec, v/dt)
///              防止在极低速下出现不合理的急刹（制动距离超过车速本身）

#include "vehicle_model/idm_model.h"

#include "common/math/calculations.h"
#include "odeint-v2/boost/numeric/odeint.hpp"

#include <boost/bind/placeholders.hpp> // Boost Placeholders 支持

using namespace boost::placeholders;   // 启用 _1, _2 等占位符
namespace odeint = boost::numeric::odeint;

namespace simulator {

// ============================================================================
// 构造函数
// ============================================================================

IntelligentDriverModel::IntelligentDriverModel() { UpdateInternalState(); }

IntelligentDriverModel::IntelligentDriverModel(const Param &parm)
    : param_(parm) {
  UpdateInternalState();
}

IntelligentDriverModel::~IntelligentDriverModel() {}

// ============================================================================
// 时间步进（核心方法）
// ============================================================================

void IntelligentDriverModel::Step(double dt) {
  // 使用Boost.odeint进行自适应步长数值积分
  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);
  // 注释掉的Linear方法用于调试对比线性预测与非线性积分的差异
  // Linear(internal_state_, dt, &internal_state_);

  // 从积分后的内部状态数组恢复到外部状态结构体
  state_.s = internal_state_[0];        // 自车纵向位置
  state_.v = internal_state_[1];        // 自车纵向速度
  state_.s_front = internal_state_[2];  // 前车纵向位置
  state_.v_front = internal_state_[3];  // 前车纵向速度
  UpdateInternalState();  // 同步到内部数组，为下一步积分准备
}

// ============================================================================
// 线性预测方法（非主流程，用于调试/验证）
// ============================================================================

void IntelligentDriverModel::Linear(const InternalState &x, const double dt,
                                    InternalState *x_out) {
  // 从内部数组构建临时IDM状态
  State cur_state;
  cur_state.s = x[0];
  cur_state.v = x[1];
  cur_state.s_front = x[2];
  cur_state.v_front = x[3];

  // 计算IDM期望加速度
  decimal_t acc;
  common::IntelligentDriverModel::GetIIdmDesiredAcceleration(param_, cur_state,
                                                             &acc);

  // 加速度下限保护：不能低于硬制动减速度，也不能低于"车速/时间步"（即一帧内刹停所需减速度）
  acc = std::max(acc,
                 -std::min(param_.kHardBrakingDeceleration, cur_state.v / dt));

  // 线性预测：使用匀加速运动公式
  // s(t+dt) = s(t) + v(t)*dt + 0.5*a*dt^2
  (*x_out)[0] = x[0] + cur_state.v * dt + 0.5 * acc * dt * dt;
  // v(t+dt) = v(t) + a*dt
  (*x_out)[1] = cur_state.v + acc * dt;
  // 前车位置：匀速运动假设
  (*x_out)[2] = x[2] + x[3] * dt;
  // 前车速度：保持不变（匀速假设）
  (*x_out)[3] = x[3];
}

// ============================================================================
// ODE右端函数（IDM状态方程）
// ============================================================================

void IntelligentDriverModel::operator()(const InternalState &x,
                                        InternalState &dxdt, const double dt) {
  // 从内部数组构建当前IDM状态
  State cur_state;
  cur_state.s = x[0];
  cur_state.v = x[1];
  cur_state.s_front = x[2];
  cur_state.v_front = x[3];

  // 计算IDM期望加速度（在common库中实现核心公式）
  decimal_t acc;
  common::IntelligentDriverModel::GetAccDesiredAcceleration(param_, cur_state,
                                                            &acc);
  // 加速度下限保护：
  // acc >= -min(kHardBrakingDec, v/dt)
  // 确保即使在极低速下，制动减速度也不至于使速度在dt内变为负数
  acc = std::max(acc,
                 -std::min(param_.kHardBrakingDeceleration, cur_state.v / dt));

  // 状态导数定义
  dxdt[0] = cur_state.v;          // ds/dt = 当前位置变化率 = 自身速度
  dxdt[1] = acc;                  // dv/dt = IDM计算出的加速度
  dxdt[2] = cur_state.v_front;    // ds_front/dt = 前车位置变化率 = 前车速度
  dxdt[3] = 0.0;                  // dv_front/dt = 0（假设前车匀速行驶）
}

// ============================================================================
// 状态存取
// ============================================================================

const IntelligentDriverModel::State &IntelligentDriverModel::state(void) const {
  return state_;
}

void IntelligentDriverModel::set_state(
    const IntelligentDriverModel::State &state) {
  state_ = state;
  UpdateInternalState();
}

// ============================================================================
// 内部状态同步
// ============================================================================

void IntelligentDriverModel::UpdateInternalState(void) {
  // 将外部IDM状态结构体的各字段复制到内部状态数组
  // 索引对应关系：
  // 0: s        -- 自车纵向位置（Frenet s坐标）
  // 1: v        -- 自车纵向速度
  // 2: s_front  -- 前车纵向位置
  // 3: v_front  -- 前车纵向速度
  internal_state_[0] = state_.s;
  internal_state_[1] = state_.v;
  internal_state_[2] = state_.s_front;
  internal_state_[3] = state_.v_front;
}

}  // namespace simulator
