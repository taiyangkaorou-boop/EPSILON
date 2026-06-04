/**
 * @file eudm_server_ros.h
 * @author EPSILON Autonomous Driving Team
 * @brief EUDM规划器的ROS2服务器包装器
 * @version 0.1
 * @date 2019-07-07
 * @copyright Copyright (c) 2019
 *
 * ## 功能概述
 *
 * EudmPlannerServer是EUDM行为规划系统与ROS2框架之间的桥梁。
 * 它负责：
 *
 * 1. **异步运行**：通过独立线程（MainThread）以固定频率（work_rate_）
 *    执行规划循环（PlanCycleCallback）
 * 2. **输入缓冲**：使用无锁队列（moodycamel::ReaderWriterQueue）
 *    缓冲来自上游的语义地图数据，避免阻塞publisher
 * 3. **HMI交互**：通过/custom_joy话题接收用户操作信号
 *    （变道请求、速度调节、功能开关），更新Task结构
 * 4. **输出发布**：将规划结果可视化为ROS MarkerArray话题
 * 5. **行为回调**：支持外部绑定回调函数，在每次规划后
 *    将行为结果注入语义地图
 *
 * ## 工作流程
 *
 * 1. Init(): 初始化管理器、订阅Joy话题、初始化可视化工具
 * 2. Start(): 启动独立规划线程（MainThread）
 * 3. PushSemanticMap(): 上游模块推送语义地图数据到缓冲区
 * 4. MainThread: 以work_rate频率执行PlanCycleCallback
 * 5. PlanCycleCallback: 从缓冲区取出最新地图 -> 运行EudmManager ->
 *    构建行为 -> 发布可视化 -> 调用回调
 *
 * @see eudm_manager.h EudmManager
 * @see visualizer.h EudmPlannerVisualizer
 */

#ifndef _CORE_EUDM_PLANNER_INC_EUDM_SERVER_ROS_H__
#define _CORE_EUDM_PLANNER_INC_EUDM_SERVER_ROS_H__

#include <sensor_msgs/msg/joy.hpp>
#include <chrono>
#include <functional>
#include <numeric>
#include <thread>

#include "common/basics/tic_toc.h"
#include "common/visualization/common_visualization_util.h"
#include "eudm_planner/dcp_tree.h"
#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/eudm_manager.h"
#include "eudm_planner/eudm_planner.h"
#include "eudm_planner/map_adapter.h"
#include "eudm_planner/visualizer.h"
#include "moodycamel/atomicops.h"
#include "moodycamel/readerwriterqueue.h"
#include <rclcpp/rclcpp.hpp>
#include "semantic_map_manager/semantic_map_manager.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vehicle_msgs/encoder.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace planning {

/// @class EudmPlannerServer
/// @brief EUDM规划器的ROS2服务器——提供异步规划循环和HMI交互
///
/// 该类将EUDM行为规划系统包装为ROS2节点中的一个服务组件。
/// 它使用独立的规划线程以固定频率运行，通过无锁缓冲区
/// 接收语义地图数据，支持用户通过Joy手柄进行交互式控制。
///
/// ## 架构设计
/// - **输入侧**：语义地图通过PushSemanticMap推送到无锁队列缓冲区
/// - **核心循环**：MainThread以work_rate_频率驱动PlanCycleCallback
/// - **控制器**：EudmManager负责规划生命周期和变道管理
/// - **输出侧**：通过可视化工具发布规划轨迹，通过回调注入行为到地图
class EudmPlannerServer {
 public:
  using SemanticMapManager = semantic_map_manager::SemanticMapManager;
  using DcpAction = DcpTree::DcpAction;
  using DcpLonAction = DcpTree::DcpLonAction;
  using DcpLatAction = DcpTree::DcpLatAction;

  /// @struct Config
  /// @brief 服务器配置结构
  struct Config {
    int kInputBufferSize{100};  ///< 输入语义地图缓冲队列大小
  };

  /// @brief 构造函数（默认20Hz工作频率）
  /// @param node ROS2节点共享指针
  /// @param ego_id 自车ID
  EudmPlannerServer(std::shared_ptr<rclcpp::Node> node, int ego_id);

  /// @brief 构造函数（自定义工作频率）
  /// @param node ROS2节点共享指针
  /// @param work_rate 规划工作频率（Hz）
  /// @param ego_id 自车ID
  EudmPlannerServer(std::shared_ptr<rclcpp::Node> node, double work_rate, int ego_id);

  /// @brief 推送语义地图数据到输入缓冲区（非阻塞）
  ///
  /// 上游模块在每个感知周期调用此函数。
  /// 使用try_enqueue确保非阻塞语义。
  ///
  /// @param smm 语义地图管理器副本
  void PushSemanticMap(const SemanticMapManager &smm);

  /// @brief 绑定行为更新回调函数
  ///
  /// 回调函数在每次规划完成后被调用，接收更新后的语义地图
  /// （包含新的ego行为），返回int状态码。
  ///
  /// @param fn 回调函数（签名：int(const SemanticMapManager&)）
  void BindBehaviorUpdateCallback(
      std::function<int(const SemanticMapManager &)> fn);

  /// @brief 设置用户期望速度
  void set_user_desired_velocity(const decimal_t desired_vel);

  /// @brief 获取用户期望速度
  decimal_t user_desired_velocity() const;

  /// @brief 初始化服务器
  ///
  /// 操作：
  /// 1. 初始化EudmManager
  /// 2. 订阅/custom_joy话题（用于接收用户操作信号）
  /// 3. 读取ROS参数"use_sim_state"
  /// 4. 初始化可视化工具
  ///
  /// @param bp_config_path EUDM配置文件路径
  void Init(const std::string &bp_config_path);

  /// @brief 启动异步规划线程
  ///
  /// 分离（detach）一个MainThread线程，开始以work_rate频率
  /// 执行规划循环。同时将is_under_ctrl设置为true。
  void Start();

 private:
  /// @brief 单次规划循环回调
  ///
  /// 执行流程：
  /// 1. 从输入缓冲区取出所有语义地图（try_dequeue循环直至队列为空）
  /// 2. 如果没有新地图数据，直接返回
  /// 3. 计算对齐后的时间戳
  /// 4. 调用bp_manager_.Run()执行完整的EUDM规划生命周期
  /// 5. 构建语义行为并注入回语义地图
  /// 6. 调用绑定的外部回调
  /// 7. 发布可视化数据
  void PlanCycleCallback();

  /// @brief Joy手柄消息回调——处理用户交互操作
  ///
  /// 按钮映射：
  /// - buttons[2] = 1: 触发左变道请求（user_perferred_behavior = -1）
  /// - buttons[1] = 1: 触发右变道请求（user_perferred_behavior = 1）
  /// - buttons[3] = 1: 增加期望速度 +1m/s
  /// - buttons[0] = 1: 减少期望速度 -1m/s
  /// - buttons[4] = 1: 切换禁止左变道标志
  /// - buttons[5] = 1: 切换禁止右变道标志
  /// - buttons[6] = 1: 切换自动驾驶控制状态
  ///
  /// @param msg Joy手柄消息
  void JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg);

  /// @brief 发布可视化数据（前向轨迹MarkerArray）
  void PublishData();

  /// @brief 主规划线程函数
  ///
  /// 以固定时间步长（1000/work_rate_ ms）循环调用PlanCycleCallback。
  /// 使用sleep_until确保严格的时间调度。
  void MainThread();

  /// @brief 从动作序列中获取当前时间点对应的动作
  ///
  /// 根据当前时间t和动作序列的时间累计，确定当前应该执行哪个动作。
  ///
  /// @param t 当前时间偏移
  /// @param action_seq 动作序列
  /// @param a [out] 对应的动作
  ErrorType GetCorrespondingActionInActionSequence(
      const decimal_t &t, const std::vector<DcpAction> &action_seq,
      DcpAction *a) const;

  Config config_;                                          ///< 服务器配置

  EudmManager bp_manager_;                                 ///< EUDM管理器
  std::unique_ptr<EudmPlannerVisualizer> p_visualizer_;    ///< 可视化工具

  SemanticMapManager smm_;                                 ///< 当前语义地图副本

  planning::eudm::Task task_;                              ///< 当前规划任务（用户交互动态更新）
  bool use_sim_state_ = true;                              ///< 是否使用仿真状态

  // ros related
  std::shared_ptr<rclcpp::Node> node_;                     ///< ROS2节点指针
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;  ///< Joy话题订阅

  double work_rate_{20.0};                                 ///< 规划工作频率（Hz）
  int ego_id_;                                              ///< 自车ID

  // 输入缓冲区
  /// @brief 无锁语义地图输入缓冲区（ReaderWriterQueue）
  ///
  /// 使用moodycamel::ReaderWriterQueue（无锁并发队列）实现，
  /// 避免上游publisher和规划线程之间的锁竞争。
  std::unique_ptr<moodycamel::ReaderWriterQueue<SemanticMapManager>> p_input_smm_buff_;

  bool has_callback_binded_ = false;                       ///< 是否已绑定行为更新回调
  std::function<int(const SemanticMapManager &)> private_callback_fn_;  ///< 外部行为更新回调
};

}  // namespace planning

#endif  // _CORE_EUDM_PLANNER_INC_EUDM_SERVER_ROS_H__
