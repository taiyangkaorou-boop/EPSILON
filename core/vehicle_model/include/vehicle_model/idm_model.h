#ifndef _VEHICLE_MODEL_INC_VEHIDLE_MODEL_IDM_MODEL_H__
#define _VEHICLE_MODEL_INC_VEHIDLE_MODEL_IDM_MODEL_H__

/// @file        idm_model.h
/// @brief       智能驾驶员模型（Intelligent Driver Model, IDM）车辆跟随模型
/// @details     IDM是一种基于物理的纵向跟车模型，由Treiber等人于2000年提出。
///              该文件将IDM封装为可通过Boost.odeint进行数值积分的车辆模型。
///              模型根据自车与前方车辆的相对距离和相对速度，计算合理的纵向加速度。
///              IDM的核心特性包括：
///              - 期望速度趋近行为：车辆倾向于加速到期望速度
///              - 安全距离保持：与前方车辆保持基于速度的安全距离
///              - 平滑加减速：使用舒适制动减速度参数控制制动强度
///              模型状态包括自车纵向位置(s)、速度(v)以及前车的纵向位置(s_front)和速度(v_front)，
///              共四个状态量，假设前车保持匀速运动。

#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "common/basics/basics.h"
#include "common/idm/intelligent_driver_model.h"
#include "common/state/state.h"

using namespace boost::placeholders;

namespace simulator {

/// @class IntelligentDriverModel
/// @brief 智能驾驶员模型类，用于纵向跟车仿真
/// @details 该类封装了IDM的参数、状态和数值积分逻辑。
///          通过Boost.odeint进行时间步进，计算车辆在下一个时间步的纵向状态。
///          注意：该模型仅处理纵向运动（沿车道方向），不处理横向运动（转向）。
///          横向运动需要结合其他模型（如PurePursuitControl）共同使用。
///
///          IDM加速度公式：
///          a = kAcc * (1 - (v/v0)^δ - (s*(v,Δv)/Δs)^2)
///          其中 s*(v,Δv) = kMinSpacing + v*kDesiredHeadwayTime + v*Δv/(2*sqrt(kAcc*kComfBrakeDec))
///          这里 Δs 为前车间距，Δv = v - v_front 为速度差
class IntelligentDriverModel {
 public:
  /// @brief IDM参数类型别名
  using Param = common::IntelligentDriverModel::Param;
  /// @brief IDM状态类型别名
  using State = common::IntelligentDriverModel::State;

  /// @brief 默认构造函数，使用默认IDM参数
  IntelligentDriverModel();

  /// @brief 使用指定参数构造IDM模型
  /// @param parm IDM参数，包括期望速度、加速度、舒适减速度、期望时距、最小间距等
  IntelligentDriverModel(const Param &parm);

  ~IntelligentDriverModel();

  /// @brief 获取当前IDM状态（自车位置、速度 + 前车位置、速度）
  const State &state(void) const;

  /// @brief 设置IDM状态
  /// @param state 新的纵向状态，包含自车和前车的位置与速度
  void set_state(const State &state);

  /// @brief 执行一个时间步的IDM仿真
  /// @param dt 时间步长（秒）
  /// @details 使用odeint进行数值积分。积分完成后：
  ///          1. 从内部状态数组恢复各状态量
  ///          2. 重新同步外部状态，为下一次Step做好准备
  void Step(double dt);

  /// @brief 内部状态类型定义（4维数组），需设置为public供odeint访问
  /// @details 内部状态索引：
  ///          0: 自车纵向位置 s, 1: 自车纵向速度 v
  ///          2: 前车纵向位置 s_front, 3: 前车纵向速度 v_front
  typedef boost::array<double, 4> InternalState;

  /// @brief ODE系统函数对象（functor），供odeint库调用
  /// @param x 当前内部状态向量（4维）
  /// @param dxdt 输出：状态导数向量（4维）
  /// @param t 当前时间（本模型中不使用）
  /// @details 计算规则：
  ///          ds/dt = v（位置变化 = 当前速度）
  ///          dv/dt = IDM加速度公式计算值（速度变化 = 加速度）
  ///          ds_front/dt = v_front（前车位置变化 = 前车速度）
  ///          dv_front/dt = 0（假设前车保持匀速）
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

 private:
  /// @brief 将外部State同步到内部状态数组
  void UpdateInternalState(void);

  /// @brief 线性预测方法（当前未使用，保留用于调试/对比）
  /// @param x 当前状态
  /// @param dt 时间步长
  /// @param x_out 输出：下一时刻的状态（线性预测结果）
  void Linear(const InternalState &x, const double dt, InternalState *x_out);

  InternalState internal_state_;   ///< odeint内部使用的状态数组（4维）
  Param param_;                    ///< IDM参数集
  State state_;                    ///< 外部可见的IDM状态
};
}  // namespace simulator

#endif
