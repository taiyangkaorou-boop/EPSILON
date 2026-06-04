#ifndef _VEHICLE_MODEL_INC_VEHIDLE_MODEL_IDEAL_STEER_MODEL_H__
#define _VEHICLE_MODEL_INC_VEHIDLE_MODEL_IDEAL_STEER_MODEL_H__

/// @file        ideal_steer_model.h
/// @brief       理想转向车辆模型，带有物理约束限幅的运动学自行车模型
/// @details     该模型是运动学自行车模型（VehicleModel）的增强版本，在原始运动学模型基础上
///              增加了对控制输入的物理约束和限幅处理（TruncateControl），使仿真更接近真实车辆的
///              物理限制。与VehicleModel的关键区别：
///              - VehicleModel的直接控制输入是 steer_rate 和 acc_long（导数层控制）
///              - IdealSteerModel的控制输入是 steer 和 velocity（值层控制），内部再通过
///                TruncateControl限幅到满足加加速度（jerk）约束和安全侧向加速度约束
///
///              限幅约束包括：
///              - 纵向加速度/减速度限制（max_lon_acc / max_lon_dec）
///              - 纵向加加速度限制（max_lon_acc_jerk / max_lon_dec_jerk）
///              - 侧向加速度限制（max_lat_acc）
///              - 侧向加加速度限制（max_lat_jerk）
///              - 前轮转角限制（max_steering_angle）
///              - 前轮转角速率限制（max_steer_rate）
///              - 曲率限制（max_curvature）

#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>

#include "common/basics/basics.h"
#include "common/state/state.h"

using namespace boost::placeholders;

namespace simulator {

/// @class IdealSteerModel
/// @brief 理想转向模型类，带物理约束的增强型运动学自行车模型
/// @details 该类在标准运动学模型基础上加入了控制限幅机制。
///          核心流程：
///          1. 接收目标steer和velocity作为控制输入（而非导数形式）
///          2. 调用TruncateControl()对控制量进行物理约束限幅：
///             - 纵向：限制加速度和加加速度（jerk），防止急加速/急刹车
///             - 横向：限制侧向加速度和侧向加加速度，保证转向舒适性
///          3. 通过odeint积分运动学方程，得到下一时刻的车辆状态
///
///          该模型主要用于前向仿真（forward simulation），
///          当规划模块给出目标速度和目标转角后，该模型负责模拟车辆
///          在实际物理约束下的真实运动。
class IdealSteerModel {
 public:
  /// @brief 状态类型别名，使用通用State结构体
  using State = common::State;

  /// @struct Control
  /// @brief 简化的车辆控制输入结构体
  /// @details 与VehicleModel不同，这里直接指定目标steer和velocity（值层），
  ///          而非steer_rate和acc_long（导数层）。这样更易于高层次的规划模块使用。
  struct Control {
    double steer{0.0};     ///< 目标前轮转角（rad），正值表示左转
    double velocity{0.0};  ///< 目标车身纵向速度（m/s），后轴中心处
    Control() {}
    /// @brief 构造函数，直接初始化目标控制量
    /// @param s 目标前轮转角（rad）
    /// @param v 目标纵向速度（m/s）
    Control(const double s, const double v) : steer(s), velocity(v) {}
  };

  /// @brief 完整参数构造函数
  /// @param wheelbase_len 车辆轴距（m）
  /// @param max_lon_acc 最大纵向加速度（m/s^2），正值限制急加速
  /// @param max_lon_dec 最大纵向减速度（m/s^2），正值限制急减速
  /// @param max_lon_acc_jerk 最大纵向加速度变化率（m/s^3），限制加速度的突变
  /// @param max_lon_dec_jerk 最大纵向减速度变化率（m/s^3），限制减速度的突变
  /// @param max_lat_acc 最大侧向加速度绝对值（m/s^2），限制转向时离心力
  /// @param max_lat_jerk 最大侧向加加速度绝对值（m/s^3），限制侧向力的突变
  /// @param max_steering_angle 最大前轮转角绝对值（rad），机械限位
  /// @param max_steer_rate 最大前轮转角速率绝对值（rad/s），转向机速率限制
  /// @param max_curvature 最大曲率绝对值（1/m），限制最小转弯半径
  IdealSteerModel(double wheelbase_len, double max_lon_acc, double max_lon_dec,
                  double max_lon_acc_jerk, double max_lon_dec_jerk,
                  double max_lat_acc, double max_lat_jerk,
                  double max_steering_angle, double max_steer_rate,
                  double max_curvature);

  ~IdealSteerModel();

  /// @brief 获取当前车辆状态
  const State &state(void) const;

  /// @brief 设置车辆状态
  void set_state(const State &state);

  /// @brief 设置目标控制输入（steer + velocity）
  void set_control(const Control &control);

  /// @brief 执行一个时间步的仿真（包含控制限幅）
  /// @param dt 时间步长（秒）
  /// @details 执行顺序：
  ///          1. 从当前state计算曲率对应的steer
  ///          2. 更新内部状态数组
  ///          3. 对控制量进行物理限幅（TruncateControl）
  ///          4. 计算期望纵向加速度和转向速率
  ///          5. odeint数值积分
  ///          6. 从积分结果恢复状态
  void Step(double dt);

  /// @brief 对控制输入进行物理约束限幅
  /// @param dt 时间步长，用于计算加速度变化量和加加速度限幅
  /// @details 限幅处理分为纵向和横向两部分：
  ///          - 纵向：从目标velocity反推期望加速度desired_lon_acc_，
  ///            对加速度和加加速度（jerk）进行限幅，重新计算安全的velocity
  ///          - 横向：从目标steer和velocity计算期望侧向加速度，
  ///            对侧向加速度和侧向加加速度进行限幅，重新计算安全的steer
  ///          - 最后对转向速率进行限幅
  void TruncateControl(const decimal_t& dt);

  /// @brief 内部状态类型定义（5维数组），需设置为public供odeint访问
  /// @details 内部状态索引：
  ///          0: x位置, 1: y位置, 2: 航向角, 3: 纵向速度, 4: 前轮转角
  typedef boost::array<double, 5> InternalState;

  /// @brief ODE系统函数对象（functor），供odeint库调用
  /// @param x 当前内部状态向量（5维）
  /// @param dxdt 输出：状态导数向量（5维）
  /// @details 注意：与VehicleModel不同，dxdt[3]使用desired_lon_acc_而非control_.acc_long，
  ///          dxdt[4]使用desired_steer_rate_而非control_.steer_rate。
  ///          这是因为IdealSteerModel的控制限幅在积分之前已经完成。
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

 private:
  /// @brief 将外部State同步到内部状态数组
  void UpdateInternalState(void);

  State state_;                      ///< 外部可见的车辆状态
  Control control_;                  ///< 目标控制输入（限幅前）
  decimal_t desired_steer_rate_;     ///< 限幅后期望的转向速率（rad/s）
  decimal_t desired_lon_acc_;        ///< 限幅后期望的纵向加速度（m/s^2）
  decimal_t desired_lat_acc_;        ///< 限幅后期望的侧向加速度（m/s^2）
  InternalState internal_state_;     ///< odeint内部使用的状态数组（5维）

  // 物理约束参数
  double wheelbase_len_;             ///< 车辆轴距（m）
  double max_lon_acc_;               ///< 最大纵向加速度（m/s^2）
  double max_lon_dec_;               ///< 最大纵向减速度（m/s^2）
  double max_lon_acc_jerk_;          ///< 最大纵向加速度变化率（m/s^3）
  double max_lon_dec_jerk_;          ///< 最大纵向减速度变化率（m/s^3）
  double max_lat_acc_;               ///< 最大侧向加速度（m/s^2）
  double max_lat_jerk_;              ///< 最大侧向加加速度（m/s^3）
  double max_steering_angle_;        ///< 最大前轮转角限制（rad）
  double max_steer_rate_;            ///< 最大前轮转角速率限制（rad/s）
  double max_curvature_;             ///< 最大路径曲率限制（1/m）
};
}  // namespace simulator

#endif
