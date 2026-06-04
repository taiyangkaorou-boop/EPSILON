/**
 * @file ros_adapter.h
 * @brief ROS2 通信适配器 - 将物理仿真器的内部数据编码为 ROS 话题并发布
 *
 * @version 0.1
 * @date 2019-03-18
 *
 * 本模块是 phy_simulator 与 ROS2 通信系统之间的桥梁。
 * 其主要职责包括：
 *   - 将仿真器的静态数据（lane_net、obstacle_set）编码为 ROS 消息并发布
 *   - 将仿真器的动态数据（vehicle_set 中的车辆实时状态）编码为 ROS 消息并发布
 *   - 统一管理数据发布频率和时间戳
 *
 * 发布的话题：
 *   - /arena_info：完整的竞技场信息（车道网络 + 车辆集合 + 障碍物集合）
 *   - /arena_info_static：静态信息（车道网络 + 障碍物集合），低频发布
 *   - /arena_info_dynamic：动态信息（车辆集合），高频发布
 *
 * 数据转换由 vehicle_msgs::Encoder 的静态方法完成，该类封装了
 * 从 common 命名空间的 C++ 结构到 vehicle_msgs ROS 消息的编码逻辑。
 *
 * 典型使用方式：
 *   1. 构造 RosAdapter 时传入 ROS 节点指针，自动创建发布者
 *   2. 调用 set_phy_sim() 绑定到 PhySimulation 实例
 *   3. 在仿真循环中按频率调用 PublishDynamicDataWithStamp() 和
 *      PublishStaticDataWithStamp() 发布数据
 */
#ifndef _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ROS_ADAPTER_H_
#define _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ROS_ADAPTER_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "vehicle_msgs/encoder.h"

#include "common/basics/basics.h"
#include "common/basics/semantics.h"

#include "phy_simulator/phy_simulator.h"

namespace phy_simulator {

/// @brief ROS2 适配器类
///
/// 负责将 PhySimulation 中维护的仿真数据编码为 ROS 消息，
/// 并通过 ROS 话题发布给规划、感知等下游模块。
///
/// 发布策略：
///   - 静态数据（车道网络、障碍物）以较低频率发布（默认 10 Hz），
///     因为它们在仿真过程中通常不变化
///   - 动态数据（车辆位姿和状态）以较高频率发布（默认 100 Hz），
///     以保证下游模块获得足够实时的状态信息
///   - 同时保留 /arena_info 全量数据话题用于兼容旧版接口
class RosAdapter {
 public:
  /**
   * @brief Default constructor
   */
  /// @brief 默认构造函数，不初始化发布者
  RosAdapter();

  /**
   * @brief Construct a new RosAdapter object
   *
   * @param node node
   */
  /// @brief 带 ROS 节点的构造函数
  /// @param node ROS2 节点智能指针
  ///
  /// 在构造函数中创建三个话题发布者：
  ///   - arena_info_pub_：发布 /arena_info（完整竞技场信息）
  ///   - arena_info_static_pub_：发布 /arena_info_static（静态信息）
  ///   - arena_info_dynamic_pub_：发布 /arena_info_dynamic（动态信息）
  RosAdapter(std::shared_ptr<rclcpp::Node> node);

  /// @brief 绑定物理仿真器实例
  /// @param p_phy_sim PhySimulation 指针，不可为 nullptr
  void set_phy_sim(PhySimulation *p_phy_sim) { p_phy_sim_ = p_phy_sim; }

  /**
   * @brief Publish data of simulator with time stamp
   *
   * @param stamp ROS time stamp
   */
  /// @brief 发布完整的竞技场数据（静态 + 动态）
  /// @param stamp ROS 时间戳，用于消息头
  ///
  /// 将 lane_net、vehicle_set、obstacle_set 三个数据集合并为一条
  /// vehicle_msgs::msg::ArenaInfo 消息发布到 /arena_info 话题。
  void PublishDataWithStamp(const rclcpp::Time &stamp);

  /**
   * @brief Publish dynamic data of simulator with time stamp
   *
   * @param stamp ROS time stamp
   */
  /// @brief 发布动态数据（仅车辆状态）
  /// @param stamp ROS 时间戳
  ///
  /// 将 vehicle_set（所有车辆的位置、速度、加速度等实时状态）编码后
  /// 发布到 /arena_info_dynamic 话题。
  /// 此方法应在仿真循环中高频（~100 Hz）调用。
  void PublishDynamicDataWithStamp(const rclcpp::Time &stamp);

  /**
   * @brief Publish static data of simulator with time stamp
   *
   * @param stamp ROS time stamp
   */
  /// @brief 发布静态数据（车道网络和障碍物）
  /// @param stamp ROS 时间戳
  ///
  /// 将 lane_net 和 obstacle_set 编码后发布到 /arena_info_static 话题。
  /// 此方法可以低频（~10 Hz）调用，因为静态数据在仿真过程中不变。
  /// 下游节点通常缓存此消息，仅在接收到新消息时更新局部地图。
  void PublishStaticDataWithStamp(const rclcpp::Time &stamp);

 private:
  /// @brief ROS2 节点智能指针，用于创建发布者和获取时钟
  std::shared_ptr<rclcpp::Node> node_;

  /// @brief 完整竞技场信息发布者（兼容旧接口，topic: /arena_info）
  rclcpp::Publisher<vehicle_msgs::msg::ArenaInfo>::SharedPtr arena_info_pub_;

  /// @brief 静态竞技场信息发布者（topic: /arena_info_static）
  rclcpp::Publisher<vehicle_msgs::msg::ArenaInfoStatic>::SharedPtr arena_info_static_pub_;

  /// @brief 动态竞技场信息发布者（topic: /arena_info_dynamic）
  rclcpp::Publisher<vehicle_msgs::msg::ArenaInfoDynamic>::SharedPtr arena_info_dynamic_pub_;

  /// @brief 指向物理仿真器实例的裸指针（不拥有所有权）
  PhySimulation *p_phy_sim_;
};  // RosAdapter
}  // namespace phy_simulator

#endif  // _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ROS_ADAPTER_H_
