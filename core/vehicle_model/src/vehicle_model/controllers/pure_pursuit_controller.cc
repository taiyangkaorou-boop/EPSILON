/// @file        pure_pursuit_controller.cc
/// @brief       纯追踪（Pure Pursuit）横向控制器的实现文件
/// @details     实现了纯追踪算法的核心公式：从车辆轴距、角度偏差和预瞄距离
///              计算期望的前轮转角，用于横向路径跟踪控制。
///
///              纯追踪公式的几何推导：
///              设车辆后轴中心为P，预瞄点为Q，预瞄距离为Ld，角度偏差为alpha。
///              根据运动学自行车模型，车辆沿圆弧行驶时，有：
///              Ld / sin(2*alpha) = R / sin(pi/2 - alpha)
///              化简得曲率 kappa = 2*sin(alpha) / Ld
///              又根据自行车模型：kappa = tan(delta) / L
///              联立得：delta = atan2(2 * L * sin(alpha), Ld)
///
///              该公式具有以下优良性质：
///              - 角度偏差为0时转角为0（直线行驶）
///              - 转角随角度偏差增大而增大（大偏差需要大转角纠正）
///              - 转角随预瞄距离增大而减小（预瞄越远越平滑）
///              - 使用atan2确保符号正确且处理所有象限

#include "vehicle_model/controllers/pure_pursuit_controller.h"

namespace control {

// ============================================================================
// 计算期望前轮转角
// ============================================================================

ErrorType PurePursuitControl::CalculateDesiredSteer(
    const decimal_t wheelbase_len, const decimal_t angle_diff,
    const decimal_t look_ahead_dist, decimal_t *steer) {
  // 纯追踪核心公式：
  // delta = atan2(2 * L * sin(alpha), Ld)
  //
  // 其中：
  // L     = wheelbase_len   轴距，影响转向响应灵敏度
  // alpha = angle_diff      车辆航向与预瞄点方向的夹角
  // Ld    = look_ahead_dist 预瞄距离，越大转向越平滑但跟踪精度越低
  //
  // 使用 atan2(y, x) 而非 atan(y/x) 的原因：
  // atan2可以正确处理 x=0 或 x<0 的情况，且返回值在 [-pi, pi] 范围内
  *steer = atan2(2.0 * wheelbase_len * sin(angle_diff), look_ahead_dist);
  return kSuccess;
}

}  // namespace control
