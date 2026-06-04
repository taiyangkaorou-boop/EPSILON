/// @file        idm_velocity_controller.cc
/// @brief       基于智能驾驶员模型（IDM）的纵向速度控制器实现
/// @details     实现了通过IDM模型计算一个时间步后期望速度的方法。
///              该文件是IDM模型的使用封装，将模型构造、状态设置、积分执行
///              和结果提取包装为一个静态方法调用。
///
///              控制器使用流程：
///              1. 使用给定参数构造IntelligentDriverModel实例
///              2. 填充当前状态（自车位置/速度 + 前车位置/速度）
///              3. 执行一个时间步的odeint积分
///              4. 返回积分后的期望速度（保证非负）
///
///              安全保护措施：
///              - 自车速度取max(0, v)：负速度可能导致积分无界
///              - 输出速度取max(0, desired_state.v)：确保结果非负

#include "vehicle_model/controllers/idm_velocity_controller.h"

#include "vehicle_model/idm_model.h"

namespace control {

// ============================================================================
// 计算期望纵向速度
// ============================================================================

ErrorType IntelligentVelocityControl::CalculateDesiredVelocity(
    const simulator::IntelligentDriverModel::Param& param, const decimal_t s,
    const decimal_t s_front, const decimal_t v, const decimal_t v_front,
    const decimal_t dt, decimal_t* velocity_at_dt) {
  using simulator::IntelligentDriverModel;

  // 步骤1：构造IDM模型实例并设置参数
  IntelligentDriverModel model(param);

  // 步骤2：填充当前IDM状态
  IntelligentDriverModel::State state;
  state.s = s;
  state.v =
      std::max(0.0, v);  // 负速度保护：odeint在导数不连续时可能发散
  state.s_front = s_front;
  state.v_front = v_front;

  // 步骤3：设置状态并执行一步积分
  model.set_state(state);
  model.Step(dt);

  // 步骤4：提取积分结果中的速度分量，保证非负
  auto desired_state = model.state();
  *velocity_at_dt = std::max(0.0, desired_state.v);

  return kSuccess;
}

}  // namespace control
