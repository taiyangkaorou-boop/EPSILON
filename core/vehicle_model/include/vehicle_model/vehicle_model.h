#ifndef _VEHICLE_MODEL_INC_VEHIDLE_MODEL_VEHICLE_MODEL_H__
#define _VEHICLE_MODEL_INC_VEHIDLE_MODEL_VEHICLE_MODEL_H__

/// @file        vehicle_model.h
/// @brief       运动学自行车模型（Kinematic Bicycle Model），基于Boost.odeint进行数值积分
/// @details     该文件定义了完整的前轮转向运动学自行车模型，是EPSILON自动驾驶仿真系统中
///              最基础、最精确的车辆动力学模型。模型包含5个状态量：
///              - 车辆位置 (x, y)
///              - 航向角 (angle)
///              - 前轮转角 (steer)
///              - 纵向速度 (velocity)
///              控制量为前轮转角速率和纵向加速度，系统通过Runge-Kutta Dormand-Prince方法
///              进行时间步进积分。该模型假设车辆运行在平面上，不考虑垂向运动和悬架系统。
///              与IdealSteerModel的区别在于，该模型不做输入限幅处理，直接使用原始控制输入。

#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "common/basics/basics.h"
#include "common/state/state.h"

using namespace boost::placeholders;

namespace simulator {

/// @class VehicleModel
/// @brief 运动学自行车模型类，描述车辆的平面运动学约束
/// @details 该模型基于以下运动学假设：
///          1. 车辆在平面上运动，忽略垂向和俯仰运动
///          2. 前后轮均为刚体，无侧向滑移
///          3. 前轮同时负责转向和驱动
///          4. 使用单轨（自行车）简化模型
///          状态方程如下：
///          - dx/dt = cos(θ) * v
///          - dy/dt = sin(θ) * v
///          - dθ/dt = tan(δ) * v / L      (L为轴距)
///          - dδ/dt = steer_rate           (前轮转角速率)
///          - dv/dt = acc_long             (纵向加速度)
class VehicleModel {
 public:
  /// @brief 状态类型别名，使用通用State结构体
  using State = common::State;

  /// @struct Control
  /// @brief 车辆控制输入结构体
  /// @details 包含两个控制量：前轮转角速率（影响转向）和纵向加速度（影响速度）
  struct Control {
    double steer_rate{0.0};  ///< 前轮转角速率（rad/s），正值表示向左打方向
    double acc_long{0.0};    ///< 纵向加速度（m/s^2），正值表示加速，负值表示减速
    Control() {}
    /// @brief 构造函数，直接初始化控制量
    /// @param s 前轮转角速率（rad/s）
    /// @param a 纵向加速度（m/s^2）
    Control(const double s, const double a) : steer_rate(s), acc_long(a) {}
  };

  /// @brief 默认构造函数，轴距默认值为2.5m
  VehicleModel();

  /// @brief 带参数的构造函数
  /// @param wheelbase_len 车辆轴距（m），影响转向灵敏度
  /// @param max_steering_angle 最大前轮转角（rad），用于输出限幅
  VehicleModel(double wheelbase_len, double max_steering_angle);

  ~VehicleModel();

  /// @brief 获取当前车辆状态（常量引用）
  /// @return 当前车辆状态的常量引用
  const State &state(void) const;

  /// @brief 设置车辆状态
  /// @param state 新的车辆状态，将同步更新内部状态数组
  void set_state(const State &state);

  /// @brief 设置控制输入
  /// @param control 包含前轮转角速率和纵向加速度的控制输入
  void set_control(const Control &control);

  /// @brief 执行一个时间步的仿真
  /// @param dt 时间步长（秒），通常为0.01~0.1s
  /// @details 使用Boost.odeint库的Runge-Kutta Dormand-Prince 5阶方法
  ///          对运动学方程进行数值积分，积分结果直接更新到外部state_中
  void Step(double dt);

  /// @brief 内部状态类型定义（5维数组），需设置为public供odeint访问
  /// @details 内部状态索引：
  ///          0: x位置, 1: y位置, 2: 航向角, 3: 前轮转角, 4: 纵向速度
  typedef boost::array<double, 5> InternalState;

  /// @brief ODE系统函数对象（functor），供odeint库调用
  /// @param x 当前内部状态向量（5维）
  /// @param dxdt 输出：状态的导数向量（5维），即运动学方程右端项
  /// @param t 当前时间（本模型中不使用，保留以兼容odeint接口）
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

 private:
  /// @brief 将外部State同步到内部状态数组
  /// @details odent通过内部状态数组进行积分，因此每次修改外部状态后
  ///          需要调用此函数将最新值写入内部状态数组
  void UpdateInternalState(void);

  State state_;                     ///< 外部可见的车辆状态
  Control control_;                 ///< 当前控制输入
  InternalState internal_state_;    ///< odeint内部使用的状态数组
  double wheelbase_len_;            ///< 车辆轴距（m）
  double max_steering_angle_;       ///< 最大前轮转角限制（rad）
};
}  // namespace simulator

#endif
