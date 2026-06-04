#ifndef _CORE_VEHICLE_MODE_INC_CONTROLLERS_IDM_VELOCITY_H_
#define _CORE_VEHICLE_MODE_INC_CONTROLLERS_IDM_VELOCITY_H_

/// @file        idm_velocity_controller.h
/// @brief       基于智能驾驶员模型（IDM）的纵向速度控制器
/// @details     该控制器封装了IDM模型的使用接口，为上层模块（如前向仿真器）
///              提供统一的纵向速度计算服务。控制器接收当前自车与前车的状态
///              （纵向位置和速度），通过IDM模型计算一个时间步后的期望速度。
///
///              这是EPSILON中前向仿真（forward simulation）纵向预测的核心组件，
///              被OnLaneForwardSimulation内部调用以预测车辆在跟车场景下的速度变化。
///
///              参考：https://en.wikipedia.org/wiki/Intelligent_driver_model

#include "common/basics/basics.h"

#include "vehicle_model/idm_model.h"

namespace control {

/// @class IntelligentVelocityControl
/// @brief IDM纵向速度控制器
/// @details 提供静态方法CalculateDesiredVelocity，通过实例化IntelligentDriverModel、
///          设置状态、执行一个时间步的积分，返回积分后的期望速度。
///          该方法对输入速度进行了非负截断（std::max(0.0, v)），以防止
///          负速度导致积分发散（odeint在不连续导数下可能产生无界结果）。
class IntelligentVelocityControl {
 public:
  /// @brief 计算一个时间步后的期望纵向速度
  /// @param param IDM参数集，包括期望速度、加速度、舒适减速度、期望时距、最小间距等
  /// @param s 自车当前纵向位置（m），通常使用Frenet坐标系的s值（后轴中心）
  /// @param s_front 前方车辆（前车）的纵向位置（m）
  /// @param v 自车当前纵向速度（m/s）
  /// @param v_front 前车当前纵向速度（m/s）
  /// @param dt 时间步长（秒），仿真预测的时间增量
  /// @param velocity_at_dt 输出参数：dt时刻后的期望纵向速度（m/s），保证非负
  /// @return kSuccess 表示计算成功
  /// @details 内部流程：
  ///          1. 构造IntelligentDriverModel实例并设置参数
  ///          2. 构造IDM状态（对自车速度做非负截断保护）
  ///          3. 执行一个时间步的odeint积分
  ///          4. 取积分结果中的速度分量，确保非负后返回
  static ErrorType CalculateDesiredVelocity(
      const simulator::IntelligentDriverModel::Param& param, const decimal_t s,
      const decimal_t s_front, const decimal_t v, const decimal_t v_front,
      const decimal_t dt, decimal_t* velocity_at_dt);
};

}  // namespace control

#endif
