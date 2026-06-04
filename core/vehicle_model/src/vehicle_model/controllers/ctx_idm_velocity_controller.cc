/// @file        ctx_idm_velocity_controller.cc
/// @brief       上下文感知IDM纵向速度控制器的实现文件
/// @details     实现了换道场景下通过CtxIDM模型计算期望速度的方法。
///              与标准IDM控制器相比，增加了目标车道参考状态参数
///              （s_target, v_target）和上下文控制参数（k_s, k_v），
///              使速度计算能同时考虑当前车道前车约束和目标车道汇入意图。
///
///              控制器使用流程：
///              1. 使用IDM参数和上下文参数构造ContextIntelligentDriverModel实例
///              2. 填充包含前车和目标状态的完整CtxIdmState
///              3. 执行一个时间步的odeint积分
///              4. 返回积分后的期望速度（保证非负）
///
///              该控制器主要用于换道前向仿真（Lane Change Forward Simulation），
///              评估车辆在换道过程中（同时考虑前车安全和目标车道位置）的速度变化。

#include "vehicle_model/controllers/ctx_idm_velocity_controller.h"

namespace control {

// ============================================================================
// 计算换道场景下的期望纵向速度
// ============================================================================

ErrorType ContextIntelligentVelocityControl::CalculateDesiredVelocity(
    const simulator::ContextIntelligentDriverModel::IdmParam& idm_param,
    const simulator::ContextIntelligentDriverModel::CtxParam& ctx_param,
    const decimal_t s, const decimal_t s_front, const decimal_t s_target,
    const decimal_t v, const decimal_t v_front, const decimal_t v_target,
    const decimal_t dt, decimal_t* velocity_at_dt) {
  // 步骤1：构造CtxIDM模型实例并设置IDM参数和上下文参数
  simulator::ContextIntelligentDriverModel model(idm_param, ctx_param);

  // 步骤2：填充完整的CtxIDM状态（6维）
  simulator::ContextIntelligentDriverModel::CtxIdmState state;
  state.s = s;
  state.s_front = s_front;
  state.s_target = s_target;
  state.v = std::max(0.0, v);      // 负速度保护
  state.v_front = v_front;
  state.v_target = v_target;

  // 步骤3：设置状态并执行一步积分
  model.set_state(state);
  model.Step(dt);

  // 步骤4：提取积分结果中的速度分量，保证非负
  auto desired_state = model.state();
  *velocity_at_dt = std::max(0.0, desired_state.v);

  return kSuccess;
}

}  // namespace control
