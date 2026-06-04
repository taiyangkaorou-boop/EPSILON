/**
 * @file behavior_server_ros.h
 * @brief ROS2 行为规划器服务器封装层
 *
 * 本文件定义了 BehaviorPlannerServer 类，作为 MPDM 行为规划器在 ROS2 框架下的
 * 运行容器。负责管理以下核心职责：
 *
 * ## 模块角色
 * BehaviorPlannerServer 位于行为规划子系统和 ROS2 通信层之间，是一个适配器/调度器：
 *   - 封装 BehaviorPlanner 核心算法的生命周期
 *   - 管理无锁并发队列（ReaderWriterQueue），实现语义地图数据的高频异步输入
 *   - 提供 HMI（人机接口）交互，通过游戏手柄（Joy topic）接收变道和调速命令
 *   - 通过独立线程驱动规划循环，按固定频率调用 MPDM 决策
 *   - 集成可视化模块，发布前向仿真轨迹的 Marker 数据
 *
 * ## 架构模式
 *   外部生产者 ----PushSemanticMap----> [ReaderWriterQueue] ----PlanCycleCallback----> BehaviorPlanner
 *   游戏手柄 ----JoyCallback----> set_hmi_behavior / set_user_desired_velocity
 *
 * ## 线程模型
 *   - 主线程（ROS spinning）：接收 sensor_msgs/msg/Joy 和 SemanticMapManager 数据
 *   - 规划线程（MainThread）：以 work_rate_ 频率循环调用 PlanCycleCallback
 *   - 无锁队列确保两线程间数据安全传递，无需互斥锁
 *
 * ## HMI 接口说明
 *   - 按钮 [2]: 向左变道
 *   - 按钮 [1]: 向右变道
 *   - 按钮 [3]: 加速 +1 m/s
 *   - 按钮 [0]: 减速 -1 m/s
 *   仅在 L2+ 级别且 enable_hmi_interface() 已调用时生效
 */
#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_SERVER_ROS2_H__
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_SERVER_ROS2_H__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vehicle_msgs/encoder.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <chrono>
#include <functional>
#include <numeric>
#include <thread>

#include "behavior_planner/behavior_planner.h"
#include "behavior_planner/map_adapter.h"
#include "behavior_planner/visualizer.h"
#include "semantic_map_manager/semantic_map_manager.h"

#include "common/basics/tic_toc.h"
#include "common/visualization/common_visualization_util.h"

// 无锁并发队列库（moodycamel::ReaderWriterQueue）
// 用于异步传输语义地图数据，避免生产者-消费者间的锁竞争
#include "moodycamel/atomicops.h"
#include "moodycamel/readerwriterqueue.h"

namespace planning {

/**
 * @class BehaviorPlannerServer
 * @brief MPDM 行为规划器的 ROS2 服务器封装
 *
 * 提供完整的 ROS2 集成，包括：
 *   - 语义地图的异步接收与缓冲
 *   - 固定频率的规划循环调度
 *   - 游戏手柄 HMI 命令解析与转发
 *   - 前向轨迹可视化发布
 *   - 行为更新回调通知下游模块
 */
class BehaviorPlannerServer {
 public:
  /// 语义地图管理器类型别名
  using SemanticMapManager = semantic_map_manager::SemanticMapManager;

  /**
   * @struct Config
   * @brief 服务器配置参数
   */
  struct Config {
    /// 无锁输入队列的缓冲区大小，默认 100
    int kInputBufferSize{100};
  };

  /**
   * @brief 构造函数（默认工作频率 20Hz）
   * @param node ROS2 节点共享指针
   * @param ego_id 自车 ID
   */
  BehaviorPlannerServer(std::shared_ptr<rclcpp::Node> node, int ego_id);

  /**
   * @brief 构造函数（指定工作频率）
   * @param node ROS2 节点共享指针
   * @param work_rate 规划循环频率 [Hz]
   * @param ego_id 自车 ID
   */
  BehaviorPlannerServer(std::shared_ptr<rclcpp::Node> node, double work_rate, int ego_id);

  /**
   * @brief 向输入队列推送新的语义地图（由外部生产者线程调用）
   * @param smm 语义地图管理器副本
   * @note 使用 try_enqueue，队列满时直接丢弃，确保实时性
   */
  void PushSemanticMap(const SemanticMapManager &smm);

  /**
   * @brief 绑定行为更新回调函数
   * @param fn 回调函数，接收更新后的 SemanticMapManager（含决策行为）
   * @note 每次 MPDM 决策完成后将调用此回调通知下游
   */
  void BindBehaviorUpdateCallback(std::function<int(const SemanticMapManager &)> fn);

  /**
   * @brief 设置自动驾驶等级
   * @param level 自动驾驶等级（2 = L2 / 3 = L3）
   */
  void set_autonomous_level(int level);

  /**
   * @brief 设置用户期望速度
   * @param desired_vel 期望速度 [m/s]
   */
  void set_user_desired_velocity(const decimal_t desired_vel);

  /// 设置激进等级
  void set_aggressive_level(int level);

  /// 获取当前用户设定期望速度
  decimal_t user_desired_velocity() const;

  /// 获取参考期望速度（受曲率限制后的值）
  decimal_t reference_desired_velocity() const;

  /**
   * @brief 启用 HMI 人机接口
   * @note 调用后游戏手柄输入才会被处理
   */
  void enable_hmi_interface();

  /**
   * @brief 初始化服务器
   *
   * 执行步骤：
   *   1. 初始化 BehaviorPlanner 核心
   *   2. L2+ 级别订阅游戏手柄话题 /joy
   *   3. 从 ROS 参数服务器获取 use_sim_state 参数
   *   4. 初始化可视化模块
   */
  void Init();

  /**
   * @brief 启动规划循环
   *
   * 将地图适配器绑定到行为规划器，然后启动独立规划线程（detach 模式）
   */
  void Start();


 private:
  /**
   * @brief 单个规划循环回调（由规划线程以 work_rate_ 频率调用）
   *
   * 执行步骤：
   *   1. 从无锁队列中取出最新语义地图（丢弃中间过时数据）
   *   2. 将地图传给 map_adapter_ 更新数据源
   *   3. 调用 bp_.RunOnce() 执行 MPDM 决策
   *   4. 将决策结果写回 smm，调用外部回调通知下游
   *   5. 发布可视化数据
   */
  void PlanCycleCallback();

  /**
   * @brief 游戏手柄消息回调
   *
   * 按钮映射：
   *   - buttons[2] = 1: 向左变道
   *   - buttons[1] = 1: 向右变道
   *   - buttons[3] = 1: 加速 +1 m/s
   *   - buttons[0] = 1: 减速 -1 m/s
   *
   * @param msg Joy 消息（包含按键状态）
   */
  void JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg);

  /// 重新规划回调（预留接口）
  void Replan();

  /// 发布可视化数据（前向轨迹 MarkerArray）
  void PublishData();

  /**
   * @brief 规划主线程函数
   *
   * 以 work_rate_ 频率执行 PlanCycleCallback 循环。
   * 通过 sleep_until 确保精确的周期控制。
   */
  void MainThread();

  /// 服务器配置（队列大小等）
  Config config_;

  /// MPDM 行为规划器核心实例
  BehaviorPlanner bp_;

  /// 地图适配器（将 SemanticMapManager 适配为 BehaviorPlannerMapItf 接口）
  BehaviorPlannerMapAdapter map_adapter_;

  /// 可视化模块（发布前向仿真轨迹 MarkerArray）
  std::unique_ptr<BehaviorPlannerVisualizer> p_visualizer_;

  /// 计时工具，用于性能剖析
  TicToc time_profile_tool_;

  /// 全局初始化时间戳
  decimal_t global_init_stamp_{0.0};

  // ROS2 相关成员
  /// ROS2 节点共享指针
  std::shared_ptr<rclcpp::Node> node_;

  /// 游戏手柄话题订阅器
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  /// 自车 ID
  int ego_id_;

  /// 规划循环频率 [Hz]
  double work_rate_;

  // 输入缓冲与回调

  /// 无锁并发队列，用于异步接收语义地图数据
  /// ReaderWriterQueue 支持单生产者单消费者（SPSC）模式，无需互斥锁
  std::unique_ptr<moodycamel::ReaderWriterQueue<SemanticMapManager>> p_input_smm_buff_;

  /// 是否已绑定外部行为更新回调
  bool has_callback_binded_ = false;

  /// 外部行为更新回调函数对象
  std::function<int(const SemanticMapManager &)> private_callback_fn_;

  /// HMI 接口是否已启用
  bool is_hmi_enabled_ = false;
};

}  // namespace planning

#endif  // _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_SERVER_ROS2_H__
