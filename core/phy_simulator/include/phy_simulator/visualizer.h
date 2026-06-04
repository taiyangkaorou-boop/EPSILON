/**
 * @file visualizer.h
 * @brief 仿真可视化模块 - 将仿真场景数据发布为 RViz 可视化标记消息
 *
 * @version 0.1
 * @date 2019-03-18
 *
 * 本模块负责将 PhySimulation 中维护的仿真场景数据转换为
 * RViz 可视化消息（visualization_msgs::msg::MarkerArray），
 * 使用户能够在 RViz 中实时观察仿真运行状态。
 *
 * 可视化内容包括：
 *   - 车辆可视化（VehicleSet）：每辆车的包围盒（OBB）、速度方向向量、转向指示
 *   - 车道网络可视化（LaneNet）：车道中心线、起点/终点标记球、车道 ID 文本标签
 *   - 障碍物可视化（ObstacleSet）：障碍物多边形区域
 *
 * 所有可视化标记都以 "map" 为参考坐标系发布，时间戳与仿真时钟同步。
 * 各类型标记发布到独立的话题，便于在 RViz 中分别控制显隐。
 *
 * 发布的话题：
 *   - /phy_simulator_planning_node/vis/vehicle_set_vis
 *   - /phy_simulator_planning_node/vis/lane_net_vis
 *   - /phy_simulator_planning_node/vis/obstacle_set_vis
 */
#ifndef _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_VISUALIZER_H_
#define _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_VISUALIZER_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"

#include "phy_simulator/phy_simulator.h"

namespace phy_simulator {

/// @brief 仿真可视化器类
///
/// 将 PhySimulation 中的场景数据转换为 RViz 可视化消息并发布。
///
/// 可视化设计：
///   - 车辆包围盒：半透明红色，由 VehicleSet 中每辆车的位姿和尺寸生成
///   - 速度向量：黄色线段，从车辆中心沿当前速度方向延伸
///   - 转向指示：白色线段，显示当前前轮转向角
///   - 车道中心线：天蓝色线带，车道 ID 显示在车道起点上方
///   - 车道端点：紫色小球（起点和终点），用于明确车道方向
///   - 障碍物：根据类型显示不同颜色的多边形区域
///
/// 使用方式：
///   构造 Visualizer 时传入 ROS 节点 → 调用 set_phy_sim() 绑定仿真器 →
///   在仿真循环中以 ~20 Hz 频率调用 VisualizeDataWithStamp()。
class Visualizer {
 public:
  /// @brief 默认构造函数（不做任何初始化）
  Visualizer() {}

  /// @brief 带 ROS 节点的构造函数
  /// @param node ROS2 节点智能指针
  ///
  /// 在构造函数中创建三个可视化话题的发布者：
  ///   - vehicle_set_pub_：车辆集合可视化
  ///   - lane_net_pub_：车道网络可视化
  ///   - obstacle_set_pub_：障碍物集合可视化
  Visualizer(rclcpp::Node::SharedPtr node);

  /// @brief 析构函数（默认）
  ~Visualizer() {}

  /// @brief 绑定物理仿真器实例
  /// @param p_phy_sim PhySimulation 指针
  void set_phy_sim(PhySimulation *p_phy_sim) { p_phy_sim_ = p_phy_sim; }

  /// @brief 可视化当前数据（使用当前时钟时间戳）
  void VisualizeData();

  /// @brief 使用指定时间戳可视化数据
  /// @param stamp ROS 时间戳
  ///
  /// 依次调用 VisualizeVehicleSet()、VisualizeLaneNet()、
  /// VisualizeObstacleSet() 三个私有方法。
  void VisualizeDataWithStamp(const rclcpp::Time &stamp);

  /// @brief 发布 tf2 坐标变换（当前已注释，预留功能）
  /// @param stamp 时间戳
  void SendTfWithStamp(const rclcpp::Time &stamp);

 private:
  /// @brief 可视化车辆集合
  /// @param stamp 时间戳
  /// @param vehicle_set 车辆集合
  ///
  /// 遍历每辆车，使用 common::VisualizationUtil 生成包围盒、
  /// 速度向量和转向指示的三维标记。
  void VisualizeVehicleSet(const rclcpp::Time &stamp,
                           const common::VehicleSet &vehicle_set);

  /// @brief 可视化车道网络
  /// @param stamp 时间戳
  /// @param lane_net 车道网络
  ///
  /// 遍历每条车道，生成车道中心线带、起点/终点标记球和车道 ID 文本标签。
  /// 车道中心线使用 "sky blue"（天蓝色）颜色方案。
  void VisualizeLaneNet(const rclcpp::Time &stamp,
                        const common::LaneNet &lane_net);

  /// @brief 可视化障碍物集合
  /// @param stamp 时间戳
  /// @param Obstacle_set 障碍物集合
  void VisualizeObstacleSet(const rclcpp::Time &stamp,
                            const common::ObstacleSet &Obstacle_set);

  /// @brief ROS2 节点指针
  rclcpp::Node::SharedPtr node_;

  /// @brief 车辆集合可视化标记发布者
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vehicle_set_pub_;

  /// @brief 车道网络可视化标记发布者
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_net_pub_;

  /// @brief 障碍物集合可视化标记发布者
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_set_pub_;

  /// @brief 指向物理仿真器实例的裸指针
  PhySimulation *p_phy_sim_;
};  // Visualizer

}  // namespace phy_simulator

#endif  // _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_VISUALIZER_H_
