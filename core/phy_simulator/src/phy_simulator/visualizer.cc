/**
 * @file visualizer.cc
 * @brief 仿真可视化实现 —— 将场景数据转换为 RViz Marker 消息并发布
 *
 * @version 0.1
 * @date 2019-03-18
 *
 * 本文件实现了 Visualizer 类的数据可视化方法。
 *
 * 可视化内容及实现：
 *   1. 车辆可视化（VisualizeVehicleSet）：
 *      - 每辆车一条包围盒（OBB）标记：半透明红色（color_obb = ARGB(1,0.8,0.8,0.8)）
 *      - 速度方向向量：黄色线段（color_vel_vec = ARGB(1,1,0,0)）
 *      - 前轮转向指示：白色线段（color_steer = ARGB(1,1,1,1)）
 *      - 每条 marker 的 namespace 使用车辆 ID * 7 来避免冲突
 *
 *   2. 车道网络可视化（VisualizeLaneNet）：
 *      - 车道中心线：天蓝色线带（color = "sky blue"）
 *      - 起点标记：紫色小球（RGBA: 0.2, 0.6, 1.0, alpha=1.0）
 *      - 终点标记：紫色小球（同起点颜色）
 *      - 车道 ID 文本标签：红色文字悬浮在起点上方 0.5m 处
 *
 *   3. 障碍物可视化（VisualizeObstacleSet）：
 *      - 委托 common::VisualizationUtil::GetRosMarkerUsingObstacleSet() 完成
 *
 * 所有可视化标记均以 "map" 坐标系为参考框架。
 *
 * 注意：SendTfWithStamp() 方法当前未实现（内部调用已被注释），
 * 预留用于未来通过 tf2 发布车辆的坐标系变换。
 */
#include "phy_simulator/visualizer.h"

#include <tf2_ros/transform_broadcaster.h>

namespace phy_simulator {

/// @brief 带 ROS 节点的构造函数
///
/// 创建三个可视化话题的发布者，用于在 RViz 中分别显示：
///   - 车辆集合、车道网络、障碍物集合
///
/// 话题均在 /phy_simulator_planning_node/vis/ 命名空间下，
/// 消息队列长度为 10。
Visualizer::Visualizer(rclcpp::Node::SharedPtr node) : node_(node) {
  vehicle_set_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/phy_simulator_planning_node/vis/vehicle_set_vis", 10);
  lane_net_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/phy_simulator_planning_node/vis/lane_net_vis", 10);
  obstacle_set_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/phy_simulator_planning_node/vis/obstacle_set_vis", 10);
}

/// @brief 使用当前时刻可视化所有数据
///
/// 调用 node_->get_clock()->now() 获取当前时间戳，然后调用
/// VisualizeDataWithStamp() 完成可视化发布。
void Visualizer::VisualizeData() {
  auto time_stamp = node_->get_clock()->now();
  VisualizeDataWithStamp(time_stamp);
  // SendTfWithStamp(time_stamp);  // tf 发布当前未启用
}

/// @brief 使用指定时间戳可视化数据
///
/// 按顺序发布车辆、车道网络、障碍物三部分的可视化标记。
/// 每个部分分别调用对应的可视化方法，生成 MarkerArray 消息并发布。
void Visualizer::VisualizeDataWithStamp(const rclcpp::Time &stamp) {
  VisualizeVehicleSet(stamp, p_phy_sim_->vehicle_set());
  VisualizeLaneNet(stamp, p_phy_sim_->lane_net());
  VisualizeObstacleSet(stamp, p_phy_sim_->obstacle_set());
}

/// @brief 可视化车辆集合
///
/// 遍历 vehicle_set 中的所有车辆，为每辆车生成三维可视化标记：
///   - color_obb：半透明的红色，用于显示车辆包围盒
///   - color_vel_vec：黄色，从车辆中心沿速度方向延伸
///   - color_steer：白色，显示当前前轮转向角
///
/// 每条 marker 使用车辆 ID * 7 作为 namespace 偏移量，
/// 这样每辆车有 7 个 namespace 可供使用（OBB 面、速度线、转向线等），
/// 避免不同车辆之间的 marker 冲突。
/// @param stamp 时间戳
/// @param vehicle_set 车辆集合
void Visualizer::VisualizeVehicleSet(const rclcpp::Time &stamp,
                                     const common::VehicleSet &vehicle_set) {
  visualization_msgs::msg::MarkerArray vehicle_marker;
  // 定义颜色方案
  common::ColorARGB color_obb(1.0, 0.8, 0.8, 0.8);    // 包围盒颜色：半透明白色
  common::ColorARGB color_vel_vec(1.0, 1.0, 0.0, 0.0); // 速度向量颜色：黄色
  common::ColorARGB color_steer(1.0, 1.0, 1.0, 1.0);   // 转向指示颜色：白色

  for (auto iter = vehicle_set.vehicles.begin();
       iter != vehicle_set.vehicles.end(); ++iter) {
    // 使用 VisualizationUtil 工具类生成车辆的包围盒、速度向量和转向指示标记
    common::VisualizationUtil::GetRosMarkerArrayUsingVehicle(
        iter->second, common::ColorARGB(1, 1, 0, 0), color_vel_vec, color_steer,
        7 * iter->first, &vehicle_marker);  // 7 * id 作为 namespace 偏移量
  }
  // 为所有标记统一填充时间戳信息
  common::VisualizationUtil::FillStampInMarkerArray(stamp, &vehicle_marker);
  vehicle_set_pub_->publish(vehicle_marker);
}

/// @brief 可视化车道网络
///
/// 对每条车道生成以下标记：
///   1. 车道中心线带（LineStrip）：天蓝色折线，沿车道采样点绘制
///   2. 起点标记球（Sphere）：紫色小球放置在车道第一个采样点处
///   3. 终点标记球（Sphere）：紫色小球放置在车道最后一个采样点处
///   4. 车道 ID 文本标签（Text）：红色文字，悬浮在车道起点上方 0.5m
///
/// 所有标记使用 map 坐标系。
/// @param stamp 时间戳
/// @param lane_net 车道网络
void Visualizer::VisualizeLaneNet(const rclcpp::Time &stamp,
                                  const common::LaneNet &lane_net) {
  visualization_msgs::msg::MarkerArray lane_net_marker;
  int id_cnt = 0;  // marker 序号计数器，确保每条 marker 有唯一的 id

  for (auto iter = lane_net.lane_set.begin(); iter != lane_net.lane_set.end();
       ++iter) {
    // ---- 1. 车道中心线带 ----
    visualization_msgs::msg::Marker lane_marker;
    common::VisualizationUtil::GetRosMarkerLineStripUsing2DofVec(
        iter->second.lane_points, common::cmap.at("sky blue"),
        Vec3f(0.1, 0.1, 0.1), iter->second.id, &lane_marker);
    lane_marker.header.stamp = stamp;
    lane_marker.header.frame_id = "map";
    lane_marker.id = id_cnt++;
    lane_net_marker.markers.push_back(lane_marker);

    // ---- 2. 起点标记球 ----
    visualization_msgs::msg::Marker start_point_marker, end_point_marker,
        lane_id_text_marker;
    {
      start_point_marker.header.stamp = stamp;
      start_point_marker.header.frame_id = "map";
      // 车道起点（第一个采样点）
      Vec2f pt = *(iter->second.lane_points.begin());
      common::VisualizationUtil::GetRosMarkerSphereUsingPoint(
          Vec3f(pt(0), pt(1), 0.0), common::ColorARGB(1.0, 0.2, 0.6, 1.0),
          Vec3f(0.5, 0.5, 0.5), id_cnt++, &start_point_marker);

      // ---- 3. 车道 ID 文本标签（悬浮在起点上方） ----
      lane_id_text_marker.header.stamp = stamp;
      lane_id_text_marker.header.frame_id = "map";
      common::VisualizationUtil::GetRosMarkerTextUsingPositionAndString(
          Vec3f(pt(0), pt(1), 0.5), std::to_string(iter->second.id),
          common::ColorARGB(1.0, 0.0, 0.0, 1.0), Vec3f(0.6, 0.6, 0.6), id_cnt++,
          &lane_id_text_marker);
    }

    // ---- 4. 终点标记球 ----
    {
      end_point_marker.header.stamp = stamp;
      end_point_marker.header.frame_id = "map";
      // 车道终点（最后一个采样点）
      Vec2f pt = *(iter->second.lane_points.rbegin());
      common::VisualizationUtil::GetRosMarkerSphereUsingPoint(
          Vec3f(pt(0), pt(1), 0.0), common::ColorARGB(1.0, 0.2, 0.6, 1.0),
          Vec3f(0.5, 0.5, 0.5), id_cnt++, &end_point_marker);
    }

    // 将起点、终点和 ID 标签加入 marker 数组
    lane_net_marker.markers.push_back(start_point_marker);
    lane_net_marker.markers.push_back(end_point_marker);
    lane_net_marker.markers.push_back(lane_id_text_marker);
  }
  lane_net_pub_->publish(lane_net_marker);
}

/// @brief 可视化障碍物集合
///
/// 委托 common::VisualizationUtil::GetRosMarkerUsingObstacleSet() 方法
/// 将障碍物多边形转换为 RViz MarkerArray，然后填充时间戳并发布。
/// @param stamp 时间戳
/// @param Obstacle_set 障碍物集合
void Visualizer::VisualizeObstacleSet(const rclcpp::Time &stamp,
                                      const common::ObstacleSet &Obstacle_set) {
  visualization_msgs::msg::MarkerArray Obstacles_marker;
  common::VisualizationUtil::GetRosMarkerUsingObstacleSet(Obstacle_set,
                                                          &Obstacles_marker);
  common::VisualizationUtil::FillStampInMarkerArray(stamp, &Obstacles_marker);
  obstacle_set_pub_->publish(Obstacles_marker);
}

}  // namespace phy_simulator
