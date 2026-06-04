/**
 * @file eudm_itf.h
 * @author EPSILON Autonomous Driving Team
 * @brief EUDM规划器接口类型定义（Task和LaneChangeInfo数据结构）
 * @version 0.1
 * @date 2019-07-07
 * @copyright Copyright (c) 2019
 *
 * 本文件定义了EUDM规划系统中使用的核心接口数据结构和任务定义。
 * 这些结构体用于在不同模块间传递规划约束和人机交互（HMI）指令。
 *
 * ## 主要内容
 * - LaneChangeInfo: 变道相关的约束和推荐信息，包括禁止变道标志、
 *   推荐变道标志、碰撞风险指示和实线检测
 * - Task: EUDM规划任务描述，聚合了用户指令（期望速度/拨杆信号）
 *   和变道约束信息
 *
 * @see eudm_planner.h
 * @see eudm_manager.h
 */

#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_ITF_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_ITF_H_

#include <string>
#include <unordered_map>
#include <vector>

namespace planning {
namespace eudm {

/// @struct LaneChangeInfo
/// @brief 变道相关信息结构体，传递变道约束和推荐信号
///
/// 该结构体由上游感知/地图模块填充，传达以下信息：
/// - **禁止变道**（forbid flags）：由于法规、安全或道路结构原因不能变道
/// - **碰撞风险**（unsafe_by_occu）：由于目标车道被占用而不能变道
/// - **实线检测**（solid_lane）：当前车道存在实线标记，不能跨越
/// - **推荐变道**（recommend）：系统主动推荐的变道方向
///
/// 这些标志在EUDM的代价函数中用于调整安全代价和导航代价。
struct LaneChangeInfo {
  bool forbid_lane_change_left = false;        ///< 禁止向左变道（法律/安全原因）
  bool forbid_lane_change_right = false;       ///< 禁止向右变道（法律/安全原因）
  bool lane_change_left_unsafe_by_occu = false; ///< 因目标车道被占用导致左变道不安全
  bool lane_change_right_unsafe_by_occu = false; ///< 因目标车道被占用导致右变道不安全
  bool left_solid_lane = false;                ///< 左侧存在实线标记
  bool right_solid_lane = false;               ///< 右侧存在实线标记
  bool recommend_lc_left = false;              ///< 系统主动推荐向左变道（导航需求）
  bool recommend_lc_right = false;             ///< 系统主动推荐向右变道（导航需求）
};

/// @struct Task
/// @brief EUDM规划任务描述结构体
///
/// 该结构体封装了一次EUDM规划周期所需的全部输入信息，
/// 由上游管理器（EudmManager）根据语义地图和人机交互信号组装。
///
/// 关键字段：
/// - is_under_ctrl: 自动驾驶系统是否处于激活状态
/// - user_desired_vel: 用户通过HMI设定的期望速度
/// - user_perferred_behavior: 用户通过拨杆（或按钮）指定的行为偏好
///   - 0: 无偏好（默认）
///   - 1: 请求右变道
///   - -1: 请求左变道
///   - 11/12: 特殊的取消信号（参见eudm_manager.h中的处理逻辑）
/// - lc_info: 变道约束和推荐信息
struct Task {
  bool is_under_ctrl = false;              ///< 自动驾驶系统是否激活控制
  double user_desired_vel;                 ///< 用户设定的期望速度（m/s）
  int user_perferred_behavior = 0;         ///< 用户偏好行为（0=无, 1=右变道, -1=左变道）
  LaneChangeInfo lc_info;                  ///< 当前变道约束和推荐信息
};

}  // namespace eudm
}  // namespace planning

#endif
