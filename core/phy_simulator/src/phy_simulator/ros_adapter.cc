/**
 * @file ros_adapter.cc
 * @brief ROS2 通信适配器实现 —— 仿真数据到 ROS 消息的编码与发布
 *
 * @version 0.1
 * @date 2019-03-18
 *
 * 本文件实现了 RosAdapter 类的三个数据发布方法。
 *
 * 数据转换流程：
 *   PhySimulation 内部数据结构
 *     ↓
 *   vehicle_msgs::Encoder::GetRos*FromSimulatorData()
 *     ↓
 *   vehicle_msgs::msg::ArenaInfo* ROS 消息
 *     ↓
 *   publisher->publish(msg)
 *     ↓
 *   ROS 网络中规划/感知等下游节点接收
 *
 * 三个发布方法分别对应三种不同粒度的场景数据：
 *   - PublishDataWithStamp()：全量数据（车道+车辆+障碍物）
 *   - PublishStaticDataWithStamp()：仅静态数据（车道+障碍物），低频发布
 *   - PublishDynamicDataWithStamp()：仅动态数据（车辆），高频发布
 *
 * 注意：实际的消息编码工作由 vehicle_msgs::Encoder 类的静态方法完成，
 * RosAdapter 仅负责调度（从 PhySimulation 获取数据 → 调用编码器 → 发布）。
 * 这种设计将"数据获取"、"消息编码"、"消息发布"三个关注点分离。
 */
#include "phy_simulator/ros_adapter.h"

namespace phy_simulator {

/// @brief 默认构造函数（不初始化发布者）
RosAdapter::RosAdapter() {}

/// @brief 带 ROS 节点的构造函数
///
/// 创建三个话题发布者：
///   - /arena_info：完整竞技场信息（静态+动态）
///   - /arena_info_static：静态信息（车道网络、障碍物）
///   - /arena_info_dynamic：动态信息（车辆状态）
///
/// 所有发布者的消息队列长度设为 10，足够容纳高频发布场景下的待发送消息。
RosAdapter::RosAdapter(std::shared_ptr<rclcpp::Node> node) : node_(node) {
  arena_info_pub_ = node_->create_publisher<vehicle_msgs::msg::ArenaInfo>("arena_info", 10);
  arena_info_static_pub_ =
      node_->create_publisher<vehicle_msgs::msg::ArenaInfoStatic>("arena_info_static", 10);
  arena_info_dynamic_pub_ =
      node_->create_publisher<vehicle_msgs::msg::ArenaInfoDynamic>("arena_info_dynamic", 10);
}

/// @brief 发布完整的竞技场数据
///
/// 调用编码器将 lane_net、vehicle_set、obstacle_set 三个数据集
/// 打包为一条 ArenaInfo 消息，发布到 /arena_info 话题。
///
/// 此方法为兼容旧版接口而保留，推荐使用下面两个分离的发布方法
/// 以减少带宽（静态数据不需要与动态数据同频率发布）。
/// @param stamp ROS 时间戳
void RosAdapter::PublishDataWithStamp(const rclcpp::Time &stamp) {
  vehicle_msgs::msg::ArenaInfo msg;
  vehicle_msgs::Encoder::GetRosArenaInfoFromSimulatorData(
      p_phy_sim_->lane_net(), p_phy_sim_->vehicle_set(),
      p_phy_sim_->obstacle_set(), stamp, std::string("map"), &msg);
  arena_info_pub_->publish(msg);
}

/// @brief 发布静态竞技场数据（车道网络 + 障碍物）
///
/// 仅编码和发布在仿真过程中不变的数据：
///   - lane_net：车道网络拓扑（车道 ID、中心线、连接关系、变道权限）
///   - obstacle_set：障碍物集合（多边形区域、类型）
///
/// 建议以低频（~10 Hz）发布，下游节点通常在收到第一条时缓存数据。
/// @param stamp ROS 时间戳
void RosAdapter::PublishStaticDataWithStamp(const rclcpp::Time &stamp) {
  vehicle_msgs::msg::ArenaInfoStatic msg;
  vehicle_msgs::Encoder::GetRosArenaInfoStaticFromSimulatorData(
      p_phy_sim_->lane_net(), p_phy_sim_->obstacle_set(), stamp,
      std::string("map"), &msg);
  arena_info_static_pub_->publish(msg);
}

/// @brief 发布动态竞技场数据（仅车辆状态）
///
/// 仅编码和发布在仿真过程中变化的数据：
///   - vehicle_set：所有车辆的实时状态（位置、速度、加速度、转向角等）
///
/// 建议以高频（~100 Hz）发布，以确保规划模块获得足够实时的车辆状态信息。
/// @param stamp ROS 时间戳
void RosAdapter::PublishDynamicDataWithStamp(const rclcpp::Time &stamp) {
  vehicle_msgs::msg::ArenaInfoDynamic msg;
  vehicle_msgs::Encoder::GetRosArenaInfoDynamicFromSimulatorData(
      p_phy_sim_->vehicle_set(), stamp, std::string("map"), &msg);
  arena_info_dynamic_pub_->publish(msg);
}

}  // namespace phy_simulator
