/**
 * @file phy_simulator_planning_node.cc
 * @author your name (you@domain.com)
 * @brief EPSILON 物理仿真器主入口节点 —— 仿真循环与 ROS 通信调度
 *
 * @version 0.1
 * @date 2024-06-16
 *
 * @copyright Copyright (c) 2024
 *
 * 本文件是 phy_simulator 模块的主入口（main 函数所在文件），
 * 负责启动完整的自动驾驶仿真环境。整个仿真系统以 ROS2 节点
 * "phy_simulator_planning_node" 的形式运行。
 *
 * 系统架构（main 函数中的组件层次）：
 *
 *   main()
 *   ├── rclcpp::init()              —— 初始化 ROS2 运行时
 *   ├── Node 创建                   —— "phy_simulator_planning_node"
 *   ├── 参数声明与获取              —— 通过 declare_parameter 获取三个 JSON 路径
 *   ├── PhySimulation()             —— 创建物理仿真器，加载场景并初始化车辆模型
 *   ├── RosAdapter(node)            —— 创建 ROS 通信适配器（发布话题）
 *   ├── Visualizer(node)            —— 创建可视化器（发布 RViz 标记）
 *   ├── 控制信号订阅                —— 为每辆车订阅 /ctrl/agent_{id} 话题
 *   ├── 初始位姿/目标位姿订阅      —— /initialpose 和 /move_base_simple/goal
 *   └── 主仿真循环 (while rclcpp::ok())
 *       ├── rclcpp::spin_some()     —— 处理 ROS 消息回调（接收控制信号）
 *       ├── UpdateSimulatorUsingSignalSet()  —— 物理仿真步进（500 Hz）
 *       ├── PublishDynamicDataWithStamp()     —— 发布动态数据（100 Hz）
 *       ├── PublishStaticDataWithStamp()      —— 发布静态数据（10 Hz）
 *       └── VisualizeDataWithStamp()          —— 发布可视化数据（20 Hz）
 *
 * 仿真频率设计：
 *   - simulation_rate = 500 Hz: 物理仿真步进频率（高精度运动学更新）
 *   - gt_msg_rate = 100 Hz: 动态数据（车辆状态）发布频率
 *   - gt_static_msg_rate = 10 Hz: 静态数据（地图、障碍物）发布频率
 *   - visualization_msg_rate = 20 Hz: RViz 可视化更新频率
 *
 * 各频率分离设计的原因：
 *   - 物理仿真需要高频以保证数值精度（500 Hz）
 *   - 动态数据给下游规划器，无需与仿真步进同频（100 Hz 已足够）
 *   - 静态数据几乎不变，极低频即可（10 Hz）
 *   - 可视化更新受限于渲染性能，不需要太高（20 Hz）
 */
#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "vehicle_msgs/msg/control_signal.hpp"
#include "vehicle_msgs/decoder.h"

#include "phy_simulator/basics.h"
#include "phy_simulator/phy_simulator.h"
#include "phy_simulator/ros_adapter.h"
#include "phy_simulator/visualizer.h"

using namespace phy_simulator;

DECLARE_BACKWARD;

/// @brief 物理仿真步进频率（Hz）—— 运动学模型更新频率
const double simulation_rate = 500.0;

/// @brief 动态场景信息（车辆状态）发布频率（Hz）
const double gt_msg_rate = 100.0;

/// @brief 静态场景信息（地图、障碍物）发布频率（Hz）
const double gt_static_msg_rate = 10.0;

/// @brief RViz 可视化数据更新频率（Hz）
const double visualization_msg_rate = 20.0;

/// @brief 全局控制信号集合 —— 各车辆的控制信号从 ROS 回调写入，仿真循环中读取
common::VehicleControlSignalSet _signal_set;

/// @brief 控制信号话题的 ROS 订阅者列表
/// 为每辆车创建一个独立的订阅者，topic 格式为 /ctrl/agent_{id}
std::vector<rclcpp::Subscription<vehicle_msgs::msg::ControlSignal>::SharedPtr> _ros_sub;

/// @brief 初始位姿（来自 /initialpose 话题）
Vec3f initial_state(0, 0, 0);

/// @brief 是否已接收到初始位姿
bool flag_rcv_initial_state = false;

/// @brief 导航目标位姿（来自 /move_base_simple/goal 话题）
Vec3f goal_state(0, 0, 0);

/// @brief 是否已接收到导航目标
bool flag_rcv_goal_state = false;

/// @brief 控制信号回调函数 —— 接收 ROS 控制信号消息并存储到全局信号集合
/// @param msg ROS 控制信号消息的共享指针
/// @param index 目标车辆的 ID（通过 lambda 捕获绑定）
///
/// 使用 vehicle_msgs::Decoder 将 ROS 消息解码为 common::VehicleControlSignal，
/// 然后按车辆 ID 索引存储到 _signal_set 中。
/// 这是一个异步回调：规划节点发布控制信号 → ROS 回调填充 _signal_set →
/// 仿真循环读取并应用。
void CtrlSignalCallback(const vehicle_msgs::msg::ControlSignal::SharedPtr msg, int index) {
  common::VehicleControlSignal ctrl;
  vehicle_msgs::Decoder::GetControlSignalFromRosControlSignal(*msg, &ctrl);
  _signal_set.signal_set[index] = ctrl;
}

/// @brief 初始位姿回调 —— 接收 RViz 2D Pose Estimate 工具发送的初始位姿
/// @param msg PoseWithCovarianceStamped 消息共享指针
///
/// 从 ROS 消息中提取三维自由度状态（x, y, yaw），更新全局 initial_state。
/// 主要供调试时手动指定仿真起始位置使用。
void InitialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  common::VisualizationUtil::Get3DofStateFromRosPose(msg->pose.pose, &initial_state);
  flag_rcv_initial_state = true;
}

/// @brief 导航目标回调 —— 接收 RViz 2D Nav Goal 工具发送的目标位姿
/// @param msg PoseStamped 消息共享指针
///
/// 从 ROS 消息中提取三维自由度状态（x, y, yaw），更新全局 goal_state。
/// 主要供调试时手动指定规划目标使用。
void NavGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  common::VisualizationUtil::Get3DofStateFromRosPose(msg->pose, &goal_state);
  flag_rcv_goal_state = true;
}

/// @brief EPSILON 物理仿真器主函数
///
/// 初始化 ROS2 节点，加载场景，进入主仿真循环。
/// 命令行参数由 rclcpp::init() 处理，参数通过 ROS 参数服务器获取。
///
/// @param argc 命令行参数个数
/// @param argv 命令行参数数组
/// @return 0 正常退出
int main(int argc, char** argv) {
  // ---- 1. ROS2 运行时初始化 ----
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("phy_simulator_planning_node");

  // ---- 2. 声明并读取参数：三个 JSON 配置文件路径 ----
  // 这些参数由 launch 文件传入，或者使用代码中指定的默认值
  node->declare_parameter<std::string>("vehicle_info_path",
      "/home/tao/Desktop/Autonomous-Motorsports-Motion-Planning-for-the-IAC/EPSILON/src/core/playgrounds/highway_v1.0/vehicle_set.json");
  node->declare_parameter<std::string>("map_path",
      "/home/tao/Desktop/Autonomous-Motorsports-Motion-Planning-for-the-IAC/EPSILON/src/core/playgrounds/highway_v1.0/obstacles_norm.json");
  node->declare_parameter<std::string>("lane_net_path",
      "/home/tao/Desktop/Autonomous-Motorsports-Motion-Planning-for-the-IAC/EPSILON/src/core/playgrounds/highway_v1.0/lane_net_norm.json");

  std::string vehicle_info_path;
  std::string map_path;
  std::string lane_net_path;

  // 获取车辆配置文件路径参数
  if (!node->get_parameter("vehicle_info_path", vehicle_info_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: vehicle_info_path");
  } else {
    RCLCPP_INFO(node->get_logger(), "vehicle_info_path: %s", vehicle_info_path.c_str());
  }

  // 获取障碍物地图配置文件路径参数
  if (!node->get_parameter("map_path", map_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: map_path");
  } else {
    RCLCPP_INFO(node->get_logger(), "map_path: %s", map_path.c_str());
  }

  // 获取车道网络配置文件路径参数
  if (!node->get_parameter("lane_net_path", lane_net_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter: lane_net_path");
  } else {
    RCLCPP_INFO(node->get_logger(), "lane_net_path: %s", lane_net_path.c_str());
  }

  // ---- 3. 创建核心仿真组件 ----
  // 创建物理仿真器：加载 JSON 场景数据，初始化所有车辆的运动学模型
  PhySimulation phy_sim(vehicle_info_path, map_path, lane_net_path);

  // 创建 ROS 通信适配器：负责将仿真数据发布为 ROS 话题
  RosAdapter ros_adapter(node);
  ros_adapter.set_phy_sim(&phy_sim);

  // 创建可视化器：负责发布 RViz 可视化标记
  Visualizer visualizer(node);
  visualizer.set_phy_sim(&phy_sim);

  // ---- 4. 创建控制信号订阅 ----
  // 为场景中的每辆车创建一个独立的控制信号订阅者
  auto vehicle_ids = phy_sim.vehicle_ids();
  int num_vehicles = static_cast<int>(vehicle_ids.size());
  _ros_sub.resize(num_vehicles);

  for (int i = 0; i < num_vehicles; i++) {
    auto vehicle_id = vehicle_ids[i];
    // 控制信号话题格式：/ctrl/agent_{车辆ID}
    std::string topic_name = std::string("/ctrl/agent_") + std::to_string(vehicle_id);
    printf("subscribing to %s\n", topic_name.c_str());
    // 使用 lambda 捕获 vehicle_id，将控制信号路由到正确的车辆
    _ros_sub[i] = node->create_subscription<vehicle_msgs::msg::ControlSignal>(
        topic_name, 10, [vehicle_id](const vehicle_msgs::msg::ControlSignal::SharedPtr msg) {
          CtrlSignalCallback(msg, vehicle_id);
        });
  }

  // 初始化所有车辆的默认控制信号（零信号：无加速、无转向）
  for (auto& vehicle_id : vehicle_ids) {
    common::VehicleControlSignal default_signal;
    _signal_set.signal_set.insert(std::make_pair(vehicle_id, default_signal));
  }

  // ---- 5. 创建调试用订阅 ----
  // 初始位姿订阅（来自 RViz 2D Pose Estimate 工具）
  auto ini_pos_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10, InitialPoseCallback);

  // 导航目标订阅（来自 RViz 2D Nav Goal 工具）
  auto goal_pos_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 10, NavGoalCallback);

  // ---- 6. 初始化发布定时器 ----
  rclcpp::Rate rate(simulation_rate);                         // 仿真步进频率控制器
  rclcpp::Time next_gt_pub_time = node->get_clock()->now();   // 下次动态数据发布时间
  rclcpp::Time next_gt_static_pub_time = next_gt_pub_time;    // 下次静态数据发布时间
  rclcpp::Time next_vis_pub_time = node->get_clock()->now();  // 下次可视化发布时间

  std::cout << "[PhySimulation] Initialization finished, waiting for callback" << std::endl;

  // ---- 7. 主仿真循环 ----
  // 循环持续运行，直到 ROS2 关闭（Ctrl+C 或 rclcpp::shutdown()）
  while (rclcpp::ok()) {
    // 7.1 处理 ROS 消息队列（包括接收控制信号回调）
    rclcpp::spin_some(node);

    // 7.2 物理仿真步进：使用当前控制信号集合更新所有车辆状态
    //     dt = 1/500 = 0.002 秒（2 ms 时间步长）
    phy_sim.UpdateSimulatorUsingSignalSet(_signal_set, 1.0 / simulation_rate);

    // 7.3 按频率分频发布消息
    rclcpp::Time tnow = node->get_clock()->now();

    // 100 Hz：发布动态数据（车辆实时状态）
    if (tnow >= next_gt_pub_time) {
      next_gt_pub_time += rclcpp::Duration::from_seconds(1.0 / gt_msg_rate);
      ros_adapter.PublishDynamicDataWithStamp(tnow);
    }

    // 10 Hz：发布静态数据（车道网络、障碍物）
    if (tnow >= next_gt_static_pub_time) {
      next_gt_static_pub_time += rclcpp::Duration::from_seconds(1.0 / gt_static_msg_rate);
      ros_adapter.PublishStaticDataWithStamp(tnow);
    }

    // 20 Hz：发布可视化数据（RViz 标记）
    if (tnow >= next_vis_pub_time) {
      next_vis_pub_time += rclcpp::Duration::from_seconds(1.0 / visualization_msg_rate);
      visualizer.VisualizeDataWithStamp(tnow);
    }

    // 7.4 休眠至下一个仿真周期
    rate.sleep();
  }

  // ---- 8. 清理与关闭 ----
  _ros_sub.clear();
  rclcpp::shutdown();
  return 0;
}
