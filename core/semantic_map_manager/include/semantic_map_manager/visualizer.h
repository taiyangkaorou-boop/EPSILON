/**
 * @file visualizer.h
 * @author EPSILON Autonomous Driving Group
 * @brief RViz可视化器——将语义地图管理器的内部数据转换为ROS可视化消息
 *
 * @details
 * Visualizer 将 SemanticMapManager 中存储的世界模型数据发布为 ROS
 * visualization_msgs/Marker 和 nav_msgs/OccupancyGrid 消息，供 RViz
 * 进行实时可视化调试。
 *
 * 可视化内容包括（每个智能体独立的命名空间 /vis/agent_{id}/）：
 *
 *   1. 自车（ego_vehicle_vis）：有向包围盒 + 速度向量 + 转向角可视化
 *   2. 障碍物地图（obstacle_map）：OccupancyGrid 灰度地图
 *   3. 局部车道网络（surrounding_lane_net_vis）：车道中心线 + 起止点标注 + 车道ID文本
 *   4. 局部拼接车道（local_lanes_vis）：按 LUT 拼接后的连续长车道可视化
 *   5. 自车行为（ego_behavior_vis）：当前横向行为的可视化表示
 *   6. 意图预测（pred_initial_intention_vis）：周车横向行为概率的箭头可视化
 *   7. 开环轨迹预测（pred_traj_openloop_vis）：周车预测轨迹的点+线表示
 *   8. 周围车辆（surrounding_vehicle_vis）：周围车辆OBB + 关键车辆高亮
 *   9. 限速标志（speed_limit）：六边形标牌 + 限速值/红绿灯文本
 *
 * 额外功能：
 *   - TF发布（SendTfWithStamp）：发布 map -> ego_vehicle 的坐标变换，
 *     使其他 RViz 显示可以参考自车坐标框架
 *   - Playback模式可视化（VisualizeDataWithStampForPlayback）：
 *     支持回放模式下过滤掉已删除的车道
 *
 * @version 0.1
 * @date 2019-03-20
 */

#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_VISUALIZER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_VISUALIZER_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"
#include "semantic_map_manager/semantic_map_manager.h"

namespace semantic_map_manager {

/// @class Visualizer
/// @brief 将语义地图管理器数据发布为 ROS 可视化消息（Marker Arrays + OccupancyGrid + TF）
///
/// 每个智能体实例化一个 Visualizer，发布到独立的 /vis/agent_{id}/ 命名空间下，
/// 支持多智能体同时可视化。使用 TF2 广播 map -> ego_vehicle 的坐标变换。
class Visualizer {
 public:
  using ObstacleMapType = uint8_t;

  /// @brief 构造函数——初始化所有ROS发布器
  /// @param node ROS2节点共享指针
  /// @param node_id 智能体ID（用于话题命名空间隔离）
  Visualizer(rclcpp::Node::SharedPtr node, int node_id);
  ~Visualizer() {}

  /// @brief 可视化数据（自动从 smm 获取时间戳）
  void VisualizeData(const SemanticMapManager &smm);

  /// @brief 可视化数据（使用显式时间戳）
  void VisualizeDataWithStamp(const rclcpp::Time &stamp,
                              const SemanticMapManager &smm);

  /// @brief 回放模式可视化（支持过滤已删除的车道ID）
  /// @param stamp 时间戳
  /// @param smm 语义地图管理器引用
  /// @param deleted_lane_ids 回放中已删除的车道ID列表（可视化时跳过）
  void VisualizeDataWithStampForPlayback(
      const rclcpp::Time &stamp, const SemanticMapManager &smm,
      const std::vector<int> &deleted_lane_ids);

  /// @brief 发送 map -> ego_vehicle 的 TF 坐标变换
  void SendTfWithStamp(const rclcpp::Time &stamp, const SemanticMapManager &smm);

 private:
  // ======================== 各类型数据的可视化函数 ========================

  /// @brief 可视化自车——OBB + 速度向量 + 转向角指示
  void VisualizeEgoVehicle(const rclcpp::Time &stamp,
                           const common::Vehicle &vehicle);

  /// @brief 可视化局部车道网络——中心线 + 起止点球体 + 车道ID文本
  void VisualizeSurroundingLaneNet(const rclcpp::Time &stamp,
                                   const common::LaneNet &lane_net,
                                   const std::vector<int> &deleted_lane_ids);

  /// @brief 可视化自车的横向行为决策
  void VisualizeBehavior(const rclcpp::Time &stamp,
                         const common::SemanticBehavior &behavior);

  /// @brief 可视化周围车辆——OBB + 速度 + 转向，关键车辆不透明度提高
  void VisualizeSurroundingVehicles(const rclcpp::Time &stamp,
                                    const common::VehicleSet &vehicle_set,
                                    const std::vector<int> &nearby_ids);

  /// @brief 可视化局部拼接后的大车道
  void VisualizeLocalLanes(
      const rclcpp::Time &stamp,
      const std::unordered_map<int, common::Lane> &local_lanes,
      const SemanticMapManager &smm,
      const std::vector<int> &deleted_lane_ids);

  /// @brief 可视化障碍物占据栅格地图（OccupancyGrid 格式）
  void VisualizeObstacleMap(
      const rclcpp::Time &stamp,
      const common::GridMapND<ObstacleMapType, 2> &obstacle_map);

  /// @brief 可视化周车意图预测——用箭头表示横向行为概率
  /// @note 箭头长度 = 概率值 * 2.0m；箭头方向：左/右换道(垂直车道方向)，车道保持(正前方)
  void VisualizeIntentionPrediction(
      const rclcpp::Time &stamp, const common::SemanticVehicleSet &s_vehicle_set);

  /// @brief 可视化开环轨迹预测——点标记 + 线段连接
  void VisualizeOpenloopTrajPrediction(
      const rclcpp::Time &stamp,
      const std::unordered_map<int, vec_E<common::State>> &openloop_pred_trajs);

  /// @brief 可视化限速标志——六边形标牌 + 限速值/红绿灯文本标注
  /// @note 若 vel_range(1) < kEPS 则视为"红灯禁止通行"（红色标牌），
  ///        否则为限速标志（白色标牌 + 限速值）
  void VisualizeSpeedLimit(const rclcpp::Time &stamp,
                           const vec_E<common::SpeedLimit> &speed_limits);

  // ======================== 可视化计数器——用于MarkerArray的id管理 ========================

  int last_traj_list_marker_cnt_ = 0;         ///< 上次轨迹可视化Marker数量
  int last_intention_marker_cnt_ = 0;         ///< 上次意图可视化Marker数量
  int last_surrounding_vehicle_marker_cnt_ = 0;  ///< 上次周车可视化Marker数量
  int last_speed_limit_marker_cnt_ = 0;       ///< 上次限速可视化Marker数量
  int last_surrounding_lanes_cnt_ = 0;        ///< 上次车道网络可视化Marker数量
  int last_behavior_marker_cnt_ = 0;          ///< 上次行为可视化Marker数量

  // ======================== TF相关 ========================

  std::string ego_tf_name_;  ///< 自车TF框架名称（格式：ego_vehicle_vis_{node_id_}）

  // ======================== ROS通信 ========================

  rclcpp::Node::SharedPtr node_;  ///< ROS2节点共享指针
  int node_id_;                   ///< 智能体ID

  // ROS发布器——每种可视化内容对应一个独立的话题发布器
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ego_vehicle_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr surrounding_lane_net_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_lanes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr behavior_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pred_traj_openloop_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pred_intention_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr surrounding_vehicle_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr speed_limit_vis_pub_;

  tf2_ros::TransformBroadcaster ego_to_map_tf_;  ///< TF广播器
  decimal_t marker_lifetime_{0.05};               ///< Marker生命周期（秒）
};  // Visualizer

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_VISUALIZER_H_
