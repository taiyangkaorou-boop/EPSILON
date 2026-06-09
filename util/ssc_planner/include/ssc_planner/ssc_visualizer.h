/**
 * @file ssc_visualizer.h
 * @brief SSC 规划器的 ROS2 可视化模块
 *
 * [概述]
 * SscVisualizer 负责将 SSC 规划器的内部数据以 ROS MarkerArray 形式
 * 发布到 RViz2 进行可视化调试。它将规划器内部的 Frenet 坐标系数据
 * 映射到 "ssc_map" 三维可视化坐标系中进行展示。
 *
 * [可视化内容]
 *   1. SSC 时空占据地图 (VisualizeSscMap)
 *      - 三维体素栅格的障碍物占据情况
 *      - 参考车道区域边界框
 *   2. 概率风险栅格 (VisualizeRiskGridInSscSpace)
 *      - 仅可视化 risk grid 中超过阈值的体素
 *      - 用颜色和透明度表达风险值，保持对规划链路只读
 *   3. 自车当前位置 (VisualizeEgoVehicleInSscSpace)
 *      - Frenet 坐标下的车辆轮廓
 *      - 状态位置球体标记
 *   4. 前向仿真轨迹 (VisualizeForwardTrajectoriesInSscSpace)
 *      - 各行为的多帧车辆轮廓 (渐变着色)
 *      - 各帧的 Frenet 状态位置标记
 *   5. 周围车辆预测轨迹 (VisualizeSurroundingVehicleTrajInSscSpace)
 *      - 周围车辆的多帧轮廓
 *   6. 时空走廊 (VisualizeCorridorsInSscSpace)
 *      - 走廊中的种子点 (球体)
 *      - 膨胀后的立方体边界 (半透明)
 *   7. QP 优化轨迹 (VisualizeQpTrajs)
 *      - (s, d, t) 空间中的 Bezier 样条曲线
 *
 * [坐标系映射]
 *   可视化使用 "ssc_map" 坐标系，其含义为：
 *     X = s (纵向距离, 米)
 *     Y = d (横向偏移, 米)
 *     Z = t - start_time_ (相对时间, 秒)
 */
#ifndef _UTIL_SSC_PLANNER_INC_VISUALIZER_H_
#define _UTIL_SSC_PLANNER_INC_VISUALIZER_H_

#include <assert.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/primitive/frenet_primitive.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"
#include "ssc_planner/ssc_planner.h"

namespace planning {

/// @class SscVisualizer
/// @brief SSC 规划器数据的 ROS2 可视化发布器
///
/// 所有可视化通过独立的话题发布，每个主题对应一类数据。
/// 使用 Marker.id 的递增基数来避免不同帧之间的 Marker 残留问题。
class SscVisualizer {
 public:
  /// @brief 构造函数：创建所有 ROS 可视化发布者
  /// @param node    ROS2 节点共享指针
  /// @param node_id 自车ID (用于构造话题名称: /vis/agent_{id}/ssc/...)
  SscVisualizer(rclcpp::Node::SharedPtr node, int node_id);
  ~SscVisualizer() {}

  /// @brief 发布全部可视化数据（主入口）
  /// 按顺序调用所有 Visualize* 方法
  /// @param stamp    ROS 时间戳
  /// @param planner  SSC 规划器引用（提供所有可视化源数据）
  void VisualizeDataWithStamp(const rclcpp::Time &stamp, const SscPlanner &planner);

 private:
  /// @brief 可视化 SSC 三维占据栅格地图
  void VisualizeSscMap(const rclcpp::Time &stamp, const SscMap *p_ssc_map);

  /// @brief 可视化概率风险栅格，仅作为论文实验和 RViz 调试侧通道
  /// @param risk_grid_snapshot 可选风险图快照；非空时优先显示 selected/baseline 候选快照
  /// @param source_label 风险图来源标签，用于日志区分 selected / live-map fallback
  void VisualizeRiskGridInSscSpace(const rclcpp::Time &stamp,
                                   const SscMap *p_ssc_map,
                                   const RiskGridMap3D *risk_grid_snapshot,
                                   const std::string &source_label);

  /// @brief 可视化自车在 SSC 空间 (s,d,t) 中的位置和轮廓
  void VisualizeEgoVehicleInSscSpace(const rclcpp::Time &stamp, const common::FsVehicle &fs_ego_vehicle);

  /// @brief 可视化各行为的前向仿真轨迹
  void VisualizeForwardTrajectoriesInSscSpace(
      const rclcpp::Time &stamp, const vec_E<vec_E<common::FsVehicle>> &trajs,
      const SscMap *p_ssc_map);

  /// @brief 可视化 QP 优化生成的 Bezier 样条轨迹 (s,d,t 空间中的曲线)
  void VisualizeQpTrajs(const rclcpp::Time &stamp, const vec_E<common::BezierSpline<5, 2>> &trajs);

  /// @brief 可视化周围车辆的 Frenet 预测轨迹
  void VisualizeSurroundingVehicleTrajInSscSpace(
      const rclcpp::Time &stamp,
      const vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>> &trajs_set,
      const SscMap *p_ssc_map);

  /// @brief 可视化时空走廊：半透明立方体 + 种子点
  void VisualizeCorridorsInSscSpace(
      const rclcpp::Time &stamp, const vec_E<common::DrivingCorridor> corridor_vec,
      const SscMap *p_ssc_map);

  // =========================================================================
  // 成员变量
  // =========================================================================

  int last_traj_list_marker_cnt_ = 0;              ///< 上一帧轨迹列表 Marker 数量
  int last_surrounding_vehicle_marker_cnt_ = 0;    ///< 上一帧周围车辆 Marker 数量

  rclcpp::Node::SharedPtr node_;                   ///< ROS2 节点指针
  int node_id_;                                     ///< 自车 ID

  decimal_t start_time_;                           ///< 起始时间（用于计算相对时间偏移）

  /// SSC 占据栅格地图可视化发布者
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ssc_map_pub_;
  /// 概率风险栅格可视化发布者
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr risk_grid_pub_;
  /// 自车 Frenet 位置可视化发布者
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ego_vehicle_pub_;
  /// 前向轨迹可视化发布者
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr forward_trajs_pub_;
  /// 周围车辆轨迹可视化发布者
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sur_vehicle_trajs_pub_;
  /// 时空走廊可视化发布者
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_pub_;
  /// QP 轨迹可视化发布者
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr qp_pub_;

  int last_corridor_mk_cnt = 0;       ///< 上一帧走廊 Marker 数量
  int last_qp_traj_mk_cnt = 0;        ///< 上一帧 QP 轨迹 Marker 数量
  int last_risk_grid_mk_cnt = 0;       ///< 上一帧风险栅格 Marker 数量
  int last_sur_vehicle_traj_mk_cnt = 0; ///< 上一帧周围车辆轨迹 Marker 数量
  int last_forward_traj_mk_cnt = 0;    ///< 上一帧前向轨迹 Marker 数量
};  // SscVisualizer
}  // namespace planning

#endif  // _UTIL_SSC_PLANNER_INC_VISUALIZER_H_
