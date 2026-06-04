/// @file        ctx_idm_model.cc
/// @brief       上下文感知智能驾驶员模型（ContextIntelligentDriverModel）的实现文件
/// @details     实现了CtxIDM的完整逻辑，该模型在标准IDM的基础上增加了
///              目标车道参考状态的跟踪能力，专为换道场景设计。
///
///              核心加速度计算（operator()）：
///              1. 计算标准IDM加速度 acc_idm（基于前车安全跟车）
///              2. 计算目标跟踪加速度 acc_track：
///                 v_ref = v_target + k_s * (s_target - s)
///                 acc_track = k_v * (v_ref - v)
///                 其中 k_s 为位置误差增益，k_v 为速度误差增益
///              3. 对 acc_track 限幅在 [-1.0, 1.0] m/s^2
///              4. 最终使用 acc_track（当前版本不使用 acc_idm）
///
///              状态空间为 6 维：
///              (s, v, s_front, v_front, s_target, v_target)
///              假设前车和目标状态均保持匀速。

#include "vehicle_model/ctx_idm_model.h"

#include "common/math/calculations.h"
#include "odeint-v2/boost/numeric/odeint.hpp"
#include <boost/bind/placeholders.hpp> // Boost Placeholders 支持

using namespace boost::placeholders;   // 启用 _1, _2 等占位符
namespace odeint = boost::numeric::odeint;
using namespace boost::placeholders;

namespace simulator {

// ============================================================================
// 构造函数
// ============================================================================

// 默认构造函数：使用默认IDM参数和默认上下文参数
ContextIntelligentDriverModel::ContextIntelligentDriverModel() {
  UpdateInternalState();
}

// 带参数构造函数：指定IDM参数和上下文参数
ContextIntelligentDriverModel::ContextIntelligentDriverModel(
    const IdmParam &idm_parm, const CtxParam &ctx_param)
    : idm_param_(idm_parm), ctx_param_(ctx_param) {
  UpdateInternalState();
}

ContextIntelligentDriverModel::~ContextIntelligentDriverModel() {}

// ============================================================================
// 时间步进（核心方法）
// ============================================================================

void ContextIntelligentDriverModel::Step(double dt) {
  // 使用Boost.odeint进行自适应步长数值积分
  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);

  // 从积分后的内部状态数组恢复到外部CtxIdmState结构体
  state_.s = internal_state_[0];          // 自车纵向位置
  state_.v = internal_state_[1];          // 自车纵向速度
  state_.s_front = internal_state_[2];    // 前车纵向位置
  state_.v_front = internal_state_[3];    // 前车纵向速度
  state_.s_target = internal_state_[4];   // 目标车道参考位置
  state_.v_target = internal_state_[5];   // 目标车道参考速度
  UpdateInternalState();  // 同步到内部数组，为下一步积分准备
}

// ============================================================================
// ODE右端函数（CtxIDM状态方程）
// ============================================================================

void ContextIntelligentDriverModel::operator()(const InternalState &x,
                                               InternalState &dxdt,
                                               const double dt) {
  // 从内部数组构建当前CtxIDM状态
  CtxIdmState cur_state;
  cur_state.s = x[0];
  cur_state.v = x[1];
  cur_state.s_front = x[2];
  cur_state.v_front = x[3];
  cur_state.s_target = x[4];
  cur_state.v_target = x[5];

  // ---- 步骤1：计算标准IDM期望加速度（安全跟车）----
  IdmState idm_state;
  // 注意：idm_state 在此处使用默认值（全0），实际IDM计算需要前车状态
  // 但当前版本的 acc_track 为主要控制输出，acc_idm 被计算但未直接使用
  decimal_t acc_idm;
  common::IntelligentDriverModel::GetAccDesiredAcceleration(
      idm_param_, idm_state, &acc_idm);
  // 加速度下限保护（与标准IDM相同的限幅逻辑）
  acc_idm = std::max(acc_idm, -std::min(idm_param_.kHardBrakingDeceleration,
                                        cur_state.v / dt));

  // ---- 步骤2：计算目标跟踪加速度（向目标状态收敛）----
  // v_ref = v_target + k_s * (s_target - s)
  // 参考速度 = 目标速度 + 位置误差 * 位置增益
  // 物理意义：如果自车位置落后于目标位置(s_target>s)，参考速度应大于目标速度以追赶
  decimal_t v_ref =
      cur_state.v_target + ctx_param_.k_s * (cur_state.s_target - cur_state.s);

  // acc_track = k_v * (v_ref - v)
  // 跟踪加速度 = 速度增益 * (参考速度 - 当前速度)
  // 物理意义：PID比例控制，使自车速度向参考速度收敛
  decimal_t acc_track = ctx_param_.k_v * (v_ref - cur_state.v);

  // ---- 步骤3：加速度限幅 ----
  // 将跟踪加速度限制在 [-1.0, 1.0] m/s^2 范围内，确保换道过程平滑
  acc_track = std::min(std::max(acc_track, -1.0), 1.0);

  // ---- 步骤4：选择最终控制加速度 ----
  // 当前版本：直接使用跟踪加速度（目标收敛优先于安全跟车）
  // TODO: 考虑融合 acc_idm 和 acc_track，平衡安全性和换道效率
  decimal_t acc = acc_track;

  // ---- 状态导数定义 ----
  dxdt[0] = cur_state.v;          // ds/dt = 自车速度
  dxdt[1] = acc;                  // dv/dt = 跟踪加速度
  dxdt[2] = cur_state.v_front;    // ds_front/dt = 前车速度
  dxdt[3] = 0.0;                  // dv_front/dt = 0（假设前车匀速）
  dxdt[4] = cur_state.v_target;   // ds_target/dt = 目标状态速度
  dxdt[5] = 0.0;                  // dv_target/dt = 0（假设目标状态匀速）
}

// ============================================================================
// 状态存取
// ============================================================================

const ContextIntelligentDriverModel::CtxIdmState &
ContextIntelligentDriverModel::state(void) const {
  return state_;
}

void ContextIntelligentDriverModel::set_state(
    const ContextIntelligentDriverModel::CtxIdmState &state) {
  state_ = state;
  UpdateInternalState();
}

// ============================================================================
// 内部状态同步
// ============================================================================

void ContextIntelligentDriverModel::UpdateInternalState(void) {
  // 将外部CtxIdmState结构体的各字段复制到内部状态数组
  // 索引对应关系：
  // 0: s         -- 自车纵向位置
  // 1: v         -- 自车纵向速度
  // 2: s_front   -- 前车纵向位置
  // 3: v_front   -- 前车纵向速度
  // 4: s_target  -- 目标车道参考位置
  // 5: v_target  -- 目标车道参考速度
  internal_state_[0] = state_.s;
  internal_state_[1] = state_.v;
  internal_state_[2] = state_.s_front;
  internal_state_[3] = state_.v_front;
  internal_state_[4] = state_.s_target;
  internal_state_[5] = state_.v_target;
}

}  // namespace simulator
