/**
 * @file visualizer.h
 * @brief MPDM 前向仿真轨迹可视化模块
 *
 * 本文件定义了 BehaviorPlannerVisualizer 类，负责将 MPDM 多策略决策过程中
 * 生成的所有候选行为前向仿真轨迹以 ROS2 MarkerArray 形式发布，供 RViz 等
 * 可视化工具进行调试和分析。
 *
 * ## 可视化内容
 *   1. 每条候选行为（车道保持/左变道/右变道）对应一条完整的仿真轨迹
 *   2. 轨迹上的每个时间步对应一个圆柱体 Marker（点位）
 *   3. 每条轨迹对应一条连线 Marker（LineStrip）
 *   4. 颜色使用 "gold"（金色），在 RViz 中易于辨识
 *   5. 发布话题: /vis/agent_{ego_id}/forward_trajs
 *
 * ## 使用方式
 * BehaviorPlannerServer 在构建时创建 BehaviorPlannerVisualizer 实例，
 * 每次 MPDM 决策完成后调用 PublishDataWithStamp 发布最新的轨迹数据。
 *
 * ## Marker 管理
 *   使用 FillHeaderIdInMarkerArray 进行增量 Marker 更新：
 *   - 每次发布时传入上次的 marker 数量计数
 *   - 框架自动管理新增/删除 Marker ID 的分配和回收
 */
#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_ROS_ADAPTER_H_
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_ROS_ADAPTER_H_

#include <assert.h>
#include <functional>
#include <iostream>
#include <vector>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "vehicle_msgs/decoder.h"

#include "behavior_planner/behavior_planner.h"
#include "common/basics/basics.h"
#include "common/basics/semantics.h"

namespace planning {

/**
 * @class BehaviorPlannerVisualizer
 * @brief MPDM 前向仿真轨迹的 ROS2 可视化发布器
 *
 * 从 BehaviorPlanner 获取 forward_trajs（所有候选行为的仿真轨迹），
 * 将其转换为可视化 MarkerArray 并发布。
 */
class BehaviorPlannerVisualizer {
 public:
  /**
   * @brief 构造函数
   * @param node ROS2 节点共享指针
   * @param ptr_bp BehaviorPlanner 指针（用于读取 forward_trajs）
   * @param ego_id 自车 ID（用于构造独立的可视化话题）
   */
  BehaviorPlannerVisualizer(std::shared_ptr<rclcpp::Node> node, BehaviorPlanner* ptr_bp, int ego_id)
      : node_(node), ego_id_(ego_id) {
    p_bp_ = ptr_bp;
  }

  /**
   * @brief 初始化可视化发布器
   *
   * 创建 MarkerArray 发布器，话题名为 /vis/agent_{ego_id}/forward_trajs。
   * 每个自车使用独立话题，支持多车场景调试。
   */
  void Init() {
    // 为每个自车构造独立的可视化话题，避免多车场景下话题冲突
    std::string forward_traj_topic = std::string("/vis/agent_") +
                                     std::to_string(ego_id_) +
                                     std::string("/forward_trajs");
    forward_traj_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(forward_traj_topic, 1);
  }

  /**
   * @brief 发布可视化数据（带时间戳）
   *
   * 转换逻辑：
   *   1. 从 BehaviorPlanner 获取所有候选行为的前向轨迹
   *   2. 对每条轨迹的每个状态点生成圆柱体 Marker（用于显示离散点位）
   *   3. 对每条轨迹生成 LineStrip Marker（用于显示连续轨迹线）
   *   4. 使用 FillHeaderIdInMarkerArray 管理增量 Marker ID
   *   5. 发布到 /vis/agent_{ego_id}/forward_trajs 话题
   *
   * @param stamp 时间戳（通常为当前 ROS 时间）
   */
  void PublishDataWithStamp(const rclcpp::Time& stamp) {
    if (p_bp_ == nullptr) return;

    // 从行为规划器获取所有候选行为的仿真轨迹
    auto forward_trajs = p_bp_->forward_trajs();

    visualization_msgs::msg::MarkerArray traj_list_marker;

    // 轨迹可视化颜色：金色
    common::ColorARGB traj_color = common::cmap.at("gold");

    for (const auto& traj : forward_trajs) {
      // 收集轨迹上所有点的 (x, y, z) 坐标
      std::vector<common::Point> points;
      for (const auto& v : traj) {
        common::Point pt(v.state().vec_position(0), v.state().vec_position(1));
        pt.z = 0.3;  // Z 坐标略微抬升，便于在 RViz 中辨识
        points.push_back(pt);

        // 为每个轨迹点生成圆柱体 Marker（表示车辆位置）
        visualization_msgs::msg::Marker point_marker;
        common::VisualizationUtil::GetRosMarkerCylinderUsingPoint(
            common::Point(pt), Vec3f(0.5, 0.5, 0.1), traj_color, 0,
            &point_marker);
        traj_list_marker.markers.push_back(point_marker);
      }

      // 为每条轨迹生成连线 Marker（表示运动路径）
      visualization_msgs::msg::Marker line_marker;
      common::VisualizationUtil::GetRosMarkerLineStripUsingPoints(
          points, Vec3f(0.1, 0.1, 0.1), traj_color, 0, &line_marker);
      traj_list_marker.markers.push_back(line_marker);
    }

    // 管理 Marker ID 的增量分配和回收
    int num_markers = static_cast<int>(traj_list_marker.markers.size());
    common::VisualizationUtil::FillHeaderIdInMarkerArray(
        stamp, std::string("map"), last_forward_trajs_marker_cnt_,
        &traj_list_marker);
    last_forward_trajs_marker_cnt_ = num_markers;

    forward_traj_vis_pub_->publish(traj_list_marker);
  }

 private:
  /// ROS2 节点共享指针
  std::shared_ptr<rclcpp::Node> node_;

  /// 自车 ID，用于区分不同车辆的可视化话题
  int ego_id_;

  /// 上次发布的 Marker 数量计数（用于 Marker ID 增量管理）
  int last_forward_trajs_marker_cnt_ = 0;

  /// 前向轨迹可视化 MarkerArray 发布器
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr forward_traj_vis_pub_;

  /// BehaviorPlanner 指针（用于读取仿真轨迹数据）
  BehaviorPlanner* p_bp_{nullptr};
};

}  // namespace planning

#endif  // _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_ROS_ADAPTER_H_
