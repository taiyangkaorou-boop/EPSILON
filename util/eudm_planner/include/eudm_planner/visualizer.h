/**
 * @file visualizer.h
 * @author EPSILON Autonomous Driving Team
 * @brief EUDM规划可视工具——将规划结果发布为ROS2 MarkerArray
 * @version 0.1
 * @date 2019-07-07
 * @copyright Copyright (c) 2019
 *
 * ## 功能概述
 *
 * EudmPlannerVisualizer负责将EUDM行为规划器的内部状态可视化，
 * 特别关注**前向仿真轨迹**的RViz展示。这是调试和理解EUDM
 * 决策过程的关键工具。
 *
 * ## 可视化内容
 *
 * **前向轨迹（ForwardTrajectories）**：
 * 发布所有候选动作序列的前向仿真轨迹到ROS2 MarkerArray话题。
 * 每条轨迹由一系列圆柱标记点 + 一条连线表示。
 *
 * - **金色轨迹（gold）**：重选后的最终优胜序列（processed_winner_id）
 * - **春绿色轨迹（spring green）**：原始EUDM优胜序列（original_winner_id）
 * - **灰色半透明轨迹**：其他候选序列
 *
 * 通过对比原始优胜和重选后的轨迹，可以直观理解变道上下文
 * 如何改变了最终的行为选择。
 *
 * ## 发布话题
 *
 * 话题格式：/vis/agent_{ego_id}/forward_trajs
 * 消息类型：visualization_msgs::msg::MarkerArray
 *
 * @see eudm_planner.h EudmPlanner（规划结果来源）
 * @see eudm_manager.h EudmManager（提供原始和重选后的优胜ID）
 */

#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_ROS_ADAPTER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_ROS_ADAPTER_H_

#include <assert.h>
#include <functional>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"
#include "eudm_planner/eudm_manager.h"
#include "eudm_planner/eudm_planner.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace planning {

/// @class EudmPlannerVisualizer
/// @brief EUDM规划可视工具——将前向轨迹发布到ROS2话题
///
/// 该类专一负责前向仿真轨迹的可视化，将EUDM规划器的
/// 内部仿真结果发布为RViz可显示的MarkerArray格式。
///
/// ## 颜色编码
///
/// - **金色（gold）**：最终行为选择——经过EudmManager::ReselectByContext
///   处理后的优胜序列。此为最终执行的轨迹。
/// - **春绿（spring green）**：原始EUDM优胜——EudmPlanner::EvaluateMultiThreadSimResults
///   选出的代价最低序列。若与金色不同，说明变道上下文改变了选择。
/// - **灰色半透明**：其他候选序列——用于对比和调试参考。
///
/// ## 使用方法
///
/// 创建后调用Init()初始化ROS Publisher，然后通过
/// PublishDataWithStamp(stamp)在每个规划周期发布最新的轨迹。
class EudmPlannerVisualizer {
 public:
  /// @brief 构造函数
  /// @param node ROS2节点共享指针（用于创建publisher）
  /// @param p_bp_manager EudmManager指针（非空，用于获取规划结果数据）
  /// @param ego_id 自车ID（用于区分不同车辆的topic命名空间）
  EudmPlannerVisualizer(std::shared_ptr<rclcpp::Node> node, EudmManager* p_bp_manager, int ego_id)
      : node_(node), ego_id_(ego_id) {
    assert(p_bp_manager != nullptr);
    p_bp_manager_ = p_bp_manager;
  }

  /// @brief 初始化可视化工具
  ///
  /// 创建ROS2 Publisher，topic格式为 /vis/agent_{ego_id}/forward_trajs，
  /// 发布类型为 MarkerArray，队列深度为1。
  void Init() {
    std::string forward_traj_topic = std::string("/vis/agent_") +
                                     std::to_string(ego_id_) +
                                     std::string("/forward_trajs");

    forward_traj_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(forward_traj_topic, 1);
  }

  /// @brief 以指定时间戳发布可视化数据
  ///
  /// 当前仅发布前向轨迹。未来可能扩展更多可视化类型。
  ///
  /// @param stamp 消息时间戳（ROS Time类型）
  void PublishDataWithStamp(const rclcpp::Time& stamp) {
    VisualizeForwardTrajectories(stamp);
  }

  /// @brief 可视化前向仿真轨迹
  ///
  /// 将EudmManager中所有候选序列的前向轨迹发布为MarkerArray：
  /// 1. 获取所有前向轨迹（forward_trajs）
  /// 2. 获取原始优胜ID和重选后的优胜ID
  /// 3. 为每条轨迹分配颜色和Z轴高度：
  ///    - processed_winner: 金色, z=0.4
  ///    - original_winner: 春绿, z=0.4
  ///    - 其他: 灰色半透明, z=0.3
  /// 4. 每条轨迹创建圆柱点标记（半径0.5m）+ 线带标记
  /// 5. 使用FillHeaderIdInMarkerArray填充消息头并发布
  ///
  /// @param stamp ROS时间戳
  void VisualizeForwardTrajectories(const rclcpp::Time& stamp) {
    auto forward_trajs = p_bp_manager_->planner().forward_trajs();
    int processed_winner_id = p_bp_manager_->processed_winner_id();
    int original_winner_id = p_bp_manager_->original_winner_id();
    visualization_msgs::msg::MarkerArray traj_list_marker;
    common::ColorARGB traj_color(0.5, 0.5, 0.5, 0.5);  // 默认灰色半透明
    double traj_z = 0.3;
    for (int i = 0; i < static_cast<int>(forward_trajs.size()); ++i) {
      if (i == processed_winner_id) {
        traj_color = common::cmap.at("gold");           // 最终选择：金色
        traj_z = 0.4;
      } else if (i == original_winner_id) {
        traj_color = common::cmap.at("spring green");   // 原始优胜：春绿
        traj_z = 0.4;
      } else {
        traj_color = common::ColorARGB(0.5, 0.5, 0.5, 0.5);  // 其他：灰色半透明
        traj_z = 0.3;
      }
      std::vector<common::Point> points;
      for (const auto& v : forward_trajs[i]) {
        common::Point pt(v.state().vec_position(0), v.state().vec_position(1));
        pt.z = traj_z;
        points.push_back(pt);
        visualization_msgs::msg::Marker point_marker;
        // 创建圆柱标记表示轨迹点（半径0.5m, 高度0.1m）
        common::VisualizationUtil::GetRosMarkerCylinderUsingPoint(
            common::Point(pt), Vec3f(0.5, 0.5, 0.1), traj_color, 0,
            &point_marker);
        traj_list_marker.markers.push_back(point_marker);
      }
      visualization_msgs::msg::Marker line_marker;
      // 创建线带标记连接轨迹点（线宽0.1m）
      common::VisualizationUtil::GetRosMarkerLineStripUsingPoints(
          points, Vec3f(0.1, 0.1, 0.1), traj_color, 0, &line_marker);
      traj_list_marker.markers.push_back(line_marker);
    }
    int num_markers = static_cast<int>(traj_list_marker.markers.size());
    // 填充MarkerArray的消息头（时间戳、坐标系、递增ID）
    common::VisualizationUtil::FillHeaderIdInMarkerArray(
        stamp, std::string("map"), last_forward_trajs_marker_cnt_,
        &traj_list_marker);
    last_forward_trajs_marker_cnt_ = num_markers;
    forward_traj_vis_pub_->publish(traj_list_marker);
  }

  /// @brief 设置是否使用仿真状态的可视化模式
  void set_use_sim_state(bool use_sim_state) { use_sim_state_ = use_sim_state; }

 private:
  std::shared_ptr<rclcpp::Node> node_;        ///< ROS2节点指针
  int ego_id_;                                 ///< 自车ID
  bool use_sim_state_ = true;                  ///< 是否使用仿真状态

  int last_forward_trajs_marker_cnt_ = 0;      ///< 上一帧Marker数量（用于增量更新ID）

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr forward_traj_vis_pub_;  ///< 前向轨迹可视化话题发布者
  EudmManager* p_bp_manager_{nullptr};          ///< EudmManager指针（非所有权）

};

}  // namespace planning

#endif  // _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_ROS_ADAPTER_H_
