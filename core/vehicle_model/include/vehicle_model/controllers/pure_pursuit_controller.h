#ifndef _CORE_VEHICLE_MODE_INC_CONTROLLERS_PURE_PURSUIT_H_
#define _CORE_VEHICLE_MODE_INC_CONTROLLERS_PURE_PURSUIT_H_

/// @file        pure_pursuit_controller.h
/// @brief       纯追踪（Pure Pursuit）横向转向控制器
/// @details     纯追踪算法是一种几何路径跟踪方法，其核心思想是：
///              在参考路径前方选择一个预瞄点（look-ahead point），
///              计算车辆后轴中心到该预瞄点之间所需的圆弧曲率，
///              再根据运动学关系反推出前轮转角。
///
///              该方法最初由Wallace等人提出，后被广泛应用于自动驾驶车辆的
///              横向控制。在EPSILON中，纯追踪控制器用于根据给定的参考车道线
///              和预瞄距离，计算使车辆沿参考路径行驶所需的前轮转角。
///
///              参考论文：Snider et al., "Automatic steering methods for
///              autonomous automobile path tracking", CMU-RI-TR-09-08
///
///              核心公式（从自行车模型推导）：
///              steer = atan2(2 * L * sin(alpha), ld)
///              其中 L = 轴距, alpha = 航向角偏差(angle_diff), ld = 预瞄距离

#include "common/basics/basics.h"

namespace control {

/// @class PurePursuitControl
/// @brief 纯追踪控制器，用于计算使车辆沿参考路径行驶的前轮转角
/// @details 该类只包含一个静态计算方法，输入轴距、角度偏差和预瞄距离，
///          输出期望的前轮转角。控制器基于几何关系，无需迭代求解。
///          预瞄距离（look_ahead_dist）的选择对控制性能有重要影响：
///          - 较短的预瞄距离：路径跟踪精度高但可能引起振荡
///          - 较长的预瞄距离：跟踪平滑性好但弯道可能切入内侧
///          通常建议预瞄距离与当前车速成正比设置。
class PurePursuitControl {
 public:
  /// @brief 计算期望前轮转角（核心算法入口）
  /// @param wheelbase_len 车辆轴距（m），影响转向灵敏度
  /// @param angle_diff 车辆航向与预瞄点方向的夹角（rad），车辆坐标系下的相对角度
  /// @param look_ahead_dist 预瞄距离（m），当前车辆后轴中心到参考路径上预瞄点的欧氏距离
  /// @param steer 输出参数：期望前轮转角（rad），正值表示向左转
  /// @return kSuccess 表示计算成功
  /// @details 采用atan2计算以确保符号正确，公式为标准的纯追踪公式：
  ///          steer = atan2(2.0 * wheelbase_len * sin(angle_diff), look_ahead_dist)
  static ErrorType CalculateDesiredSteer(const decimal_t wheelbase_len,
                                         const decimal_t angle_diff,
                                         const decimal_t look_ahead_dist,
                                         decimal_t *steer);
};

}  // namespace control

#endif
