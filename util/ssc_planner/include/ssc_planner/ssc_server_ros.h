/**
 * @file ssc_server_ros.h
 * @brief SSC 规划器的 ROS2 服务器封装层
 *
 * [概述]
 * SscPlannerServer 是 SscPlanner 在 ROS2 框架下的运行时封装。它负责：
 *   1. 从语义地图管理器（SemanticMapManager）异步接收环境数据
 *   2. 以固定频率（work_rate_, 默认 20Hz）调用 SscPlanner::RunOnce() 进行规划
 *   3. 管理正在执行的轨迹：根据时间推进从轨迹上提取期望状态
 *   4. 发布控制信号（ControlSignal）供车辆控制器使用
 *   5. 在轨迹即将耗尽时自动触发重新规划（Replan）
 *   6. 通过独立的可视化器发布调试信息
 *
 * [架构关系]
 *   外部系统 → PushSemanticMap(smm) → 输入缓冲队列 →
 *   PlanCycleCallback → Replan → SscPlanner::RunOnce →
 *   PublishData → ControlSignal + 可视化
 *
 * [线程模型]
 *   - MainThread: 独立的规划主线程
 *   - PlanCycleCallback: 每个规划周期执行一次
 *   - PushSemanticMap: 可由外部线程调用（使用 lock-free 队列）
 *
 * [重新规划触发条件]
 *   1. 首次规划或当前轨迹无效
 *   2. 当前时间超出执行轨迹的结束时间
 *   3. 下一轨迹为空或无效
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_SERVER_ROS_H_
#define _UTIL_SSC_PLANNER_INC_SSC_SERVER_ROS_H_

#include <chrono>
#include <memory>
#include <numeric>
#include <thread>

#include "common/basics/colormap.h"
#include "common/basics/tic_toc.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/trajectory/frenet_traj.h"
#include "common/visualization/common_visualization_util.h"
#include "moodycamel/atomicops.h"
#include "moodycamel/readerwriterqueue.h"
#include "rclcpp/rclcpp.hpp"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "ssc_planner/map_adapter.h"
#include "ssc_planner/ssc_planner.h"
#include "ssc_planner/ssc_visualizer.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vehicle_msgs/msg/control_signal.hpp"
#include "vehicle_msgs/encoder.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace planning {

/// @class SscPlannerServer
/// @brief SSC 规划器的 ROS2 服务器封装
///
/// 管理规划器的生命周期和运行时行为，负责：
///   - 接收环境地图更新
///   - 周期性地执行/重规划
///   - 从轨迹中提取控制目标并发布
///   - 所有 ROS 可视化输出
class SscPlannerServer {
 public:
  using SemanticMapManager = semantic_map_manager::SemanticMapManager;
  using FrenetTrajectory = common::FrenetTrajectory;

  /// @struct Config
  /// @brief 服务器配置
  struct Config {
    int kInputBufferSize{100};  ///< 输入缓冲队列容量
  };

  /// @brief 构造函数（使用默认工作频率 20Hz）
  /// @param node   ROS2 节点共享指针
  /// @param ego_id 自车ID
  SscPlannerServer(std::shared_ptr<rclcpp::Node> node, int ego_id);

  /// @brief 构造函数（指定工作频率）
  /// @param node      ROS2 节点共享指针
  /// @param work_rate 规划频率 (Hz)
  /// @param ego_id    自车ID
  SscPlannerServer(std::shared_ptr<rclcpp::Node> node, double work_rate, int ego_id);

  /// @brief 将语义地图压入输入缓冲队列（异步操作，线程安全）
  /// @param smm 语义地图管理器快照
  void PushSemanticMap(const SemanticMapManager &smm);

  /// @brief 初始化规划器并创建 ROS 发布者
  /// @param config_path SSC 规划器的 protobuf 配置文件路径
  void Init(const std::string &config_path);

  /// @brief 启动规划线程
  /// 设置地图接口后创建 MainThread 线程以固定频率运行
  void Start();

 private:
  /// @brief 每个规划周期的主回调
  ///
  /// 流程：
  ///   1. 从缓冲队列取出最新的语义地图
  ///   2. 发布可视化数据
  ///   3. 检查轨迹状态并决定是否重新规划
  ///   4. 若需要则调用 Replan()
  void PlanCycleCallback();

  /// @brief 执行重新规划
  ///
  /// 流程：
  ///   1. 根据当前时间从执行中轨迹提取期望状态
  ///   2. 对低速状态进行奇异点滤波（避免停走时的角度跳变）
  ///   3. 将期望状态设为规划器的初始状态
  ///   4. 调用 planner_.RunOnce() 生成新轨迹
  ///   5. 将新轨迹设为 next_traj_
  void Replan();

  /// @brief 发布全部 ROS 数据：语义地图可视化、SSC 可视化、控制信号
  void PublishData();

  /// @brief 主循环线程：以 work_rate_ 频率循环执行 PlanCycleCallback
  void MainThread();

  /// @brief 低速奇异状态滤波
  ///
  /// 问题：当车辆低速（近乎静止）时，航向角可能发生不合理的跳变。
  /// 检查条件：速度低于阈值时，若航向角变化超过运动学极限，则保持上一帧的角度。
  ///
  /// @param hist         历史控制状态序列
  /// @param filter_state 待过滤的状态 (in/out)
  /// @return 错误码
  ErrorType FilterSingularityState(const vec_E<common::State> &hist,
                                   common::State *filter_state);

  // =========================================================================
  // 成员变量
  // =========================================================================

  Config config_;                              ///< 服务器配置

  bool is_replan_on_ = false;                  ///< 重规划是否激活
  bool is_map_updated_ = false;                ///< 地图是否已更新
  bool use_sim_state_ = true;                  ///< 是否使用仿真状态 (vs 实际车辆反馈)
  std::unique_ptr<FrenetTrajectory> executing_traj_;  ///< 当前正在执行的轨迹
  std::unique_ptr<FrenetTrajectory> next_traj_;       ///< 下一帧将执行的轨迹（双缓冲）

  SscPlanner planner_;                         ///< SSC 规划器核心
  SscPlannerAdapter map_adapter_;              ///< 地图适配器（接口层）

  TicToc time_profile_tool_;                   ///< 耗时统计工具
  decimal_t global_init_stamp_{0.0};           ///< 全局初始化时间戳

  // --- ROS2 相关 ---
  std::shared_ptr<rclcpp::Node> node_;         ///< ROS2 节点指针
  decimal_t work_rate_ = 20.0;                 ///< 规划频率 (Hz)
  int ego_id_;                                  ///< 自车ID

  bool require_intervention_signal_ = false;   ///< 是否需要发布人工接管信号
  rclcpp::Publisher<vehicle_msgs::msg::ControlSignal>::SharedPtr ctrl_signal_pub_;  ///< 控制信号发布者
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_marker_pub_;      ///< 地图可视化发布者
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr executing_traj_vis_pub_; ///< 轨迹可视化发布者

  // --- 输入缓冲 ---
  /// Lock-free 单读单写队列，用于异步接收语义地图
  std::unique_ptr<moodycamel::ReaderWriterQueue<SemanticMapManager>> p_input_smm_buff_ {nullptr};

  SemanticMapManager last_smm_;                ///< 最近一次接收的语义地图
  std::unique_ptr<semantic_map_manager::Visualizer> p_smm_vis_ {nullptr};  ///< 语义地图可视化器
  std::unique_ptr<SscVisualizer> p_ssc_vis_;   ///< SSC 可视化器
  int last_trajmk_cnt_{0};                    ///< 上一帧轨迹可视化 marker 数量

  vec_E<common::State> desired_state_hist_;    ///< 期望状态历史（用于奇异点检测）
  vec_E<common::State> ctrl_state_hist_;       ///< 控制状态历史（用于奇异点检测）
};

}  // namespace planning

#endif  // _UTIL_SSC_PLANNER_INC_SSC_SERVER_ROS_H_
