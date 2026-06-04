#ifndef _CORE_VEHICLE_MODE_INC_CONTROLLERS_CTX_IDM_VELOCITY_H_
#define _CORE_VEHICLE_MODE_INC_CONTROLLERS_CTX_IDM_VELOCITY_H_

/// @file        ctx_idm_velocity_controller.h
/// @brief       上下文感知的IDM纵向速度控制器
/// @details     该控制器是标准IDM速度控制器的增强版本，专为换道（lane change）场景设计。
///              与标准IDM仅考虑当前车道前车不同，上下文感知IDM额外考虑目标车道的参考状态，
///              使车辆能够在换道过程中同时兼顾当前车道安全性（跟车）和目标车道意图（汇入）。
///
///              控制器接收的参数包括：
///              - 标准IDM参数：用于计算基于前车的安全加速度
///              - 上下文参数（k_s, k_v）：控制向目标状态收敛的行为
///              - 当前车道前车状态 (s_front, v_front)：安全跟车参考
///              - 目标车道参考状态 (s_target, v_target)：换道目标参考
///
///              在EPSILON中，该控制器用于换道前向仿真（Lane Change Forward Simulation），
///              通过预测换道过程中车辆的纵向速度变化来评估换道方案的可行性。

#include "common/basics/basics.h"
#include "vehicle_model/ctx_idm_model.h"

namespace control {

/// @class ContextIntelligentVelocityControl
/// @brief 上下文感知IDM速度控制器
/// @details 通过实例化ContextIntelligentDriverModel并执行一个时间步积分，
///          返回在考虑当前前车和目标车道参考状态后的期望速度。
///          该方法对输入速度进行了非负截断（std::max(0.0, v)），
///          以防止负速度导致odeint积分发散。
class ContextIntelligentVelocityControl {
 public:
  /// @brief 计算换道场景下dt时刻后的期望纵向速度
  /// @param idm_param IDM参数集，用于安全跟车加速度计算
  /// @param ctx_param 上下文控制参数，包含位置增益k_s和速度增益k_v
  /// @param s 自车当前纵向位置（m），Frenet坐标系的s值
  /// @param s_front 当前车道前车的纵向位置（m）
  /// @param s_target 目标车道参考位置的纵向坐标（m），用于引导速度收敛
  /// @param v 自车当前纵向速度（m/s）
  /// @param v_front 当前车道前车的纵向速度（m/s）
  /// @param v_target 目标车道参考速度（m/s）
  /// @param dt 时间步长（秒）
  /// @param velocity_at_dt 输出参数：dt时刻后的期望纵向速度（m/s），保证非负
  /// @return kSuccess 表示计算成功
  /// @details 内部流程：
  ///          1. 构造ContextIntelligentDriverModel实例并设置IDM和上下文参数
  ///          2. 构造包含前车和目标状态的完整CtxIdmState
  ///          3. 执行一个时间步的odeint积分
  ///          4. 返回积分后的速度（保证非负）
  static ErrorType CalculateDesiredVelocity(
      const common::IntelligentDriverModel::Param& idm_param,
      const simulator::ContextIntelligentDriverModel::CtxParam& ctx_param,
      const decimal_t s, const decimal_t s_front, const decimal_t s_target,
      const decimal_t v, const decimal_t v_front, const decimal_t v_target,
      const decimal_t dt, decimal_t* velocity_at_dt);
};

}  // namespace control

#endif  //_CORE_VEHICLE_MODE_INC_CONTROLLERS_CTX_IDM_VELOCITY_H_
