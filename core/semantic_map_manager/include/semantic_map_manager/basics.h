/**
 * @file basics.h
 * @author HKUST Aerial Robotics Group
 * @brief 语义地图管理器的辅助类型定义——智能体配置信息结构体
 *
 * @details
 * 该文件定义了 AgentConfigInfo 结构体，它是整个语义地图管道的统一配置入口。
 * 融合了以下配置维度：
 *
 * - 感知配置：障碍物地图元信息（尺寸、分辨率）、周围搜索半径
 * - 功能开关：开环轨迹预测使能、跟踪噪声注入使能、日志记录使能、快速车道查找表使能
 * - 日志配置：日志文件输出路径
 *
 * 该结构体被 ConfigLoader 从 JSON 文件解析填充，并被 SemanticMapManager 和
 * DataRenderer 作为关键配置参数消费。
 *
 * @version 0.1
 * @date 2019-03-20
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_BASICS_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_BASICS_H_

#include <assert.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <vector>
#include <memory>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"

namespace semantic_map_manager {

/// @struct AgentConfigInfo
/// @brief 单个智能体的运行配置信息
///
/// 该结构体聚合了智能体的感知范围、功能开关和日志配置，
/// 由 ConfigLoader 从 JSON 配置文件解析填充，或由代码直接构造默认值。
///
/// 典型JSON配置字段映射：
///   - obstacle_map_meta_info <- agent["obstacle_map_meta_info"]
///   - surrounding_search_radius <- agent["surrounding_search_radius"]
///   - enable_openloop_prediction <- agent["enable_openloop_prediction"]
///   - enable_tracking_noise <- agent["enable_tracking_noise"]（可选）
///   - enable_log <- agent["enable_log"]（可选）
///   - log_file <- agent["log_file"]（可选）
struct AgentConfigInfo {
  /// @brief 障碍物栅格地图的元信息（宽度、高度、分辨率）
  /// @details GridMapMetaInfo 包含 width（栅格列数）、height（栅格行数）、
  ///          resolution（米/像素），决定 OccupancyGrid 的空间范围
  common::GridMapMetaInfo obstacle_map_meta_info;

  /// @brief 周围车辆/车道的搜索半径（米）
  /// @note 用于筛选 surrounding_vehicles 和 surrounding_lane_net，
  ///        以及控制关键车辆的搜索范围
  decimal_t surrounding_search_radius;

  /// @brief 是否启用开环轨迹预测
  /// @note 若为 true，则 SemanticMapManager::UpdateSemanticMap 会调用
  ///        OpenloopTrajectoryPrediction 预测所有周车的未来轨迹，
  ///        用于碰撞检测中的动态障碍物检查
  bool enable_openloop_prediction{false};

  /// @brief 是否启用跟踪噪声注入
  /// @note 若为 true，则 DataRenderer::Render 会调用 InjectObservationNoise
  ///        对部分周车注入位置和角度的高斯噪声，模拟真实感知的不确定性
  bool enable_tracking_noise{false};

  /// @brief 是否启用数据日志记录
  /// @note 若为 true，则 SemanticMapManager::UpdateSemanticMap 会将自车和
  ///        周围车辆的状态以CSV格式追加到 log_file 中
  bool enable_log{false};

  /// @brief 是否启用快速车道查找表
  /// @note 若为 true，则 SemanticMapManager 会构建 local_lanes_/LUT，
  ///        加速 GetRefLaneForStateByBehavior 等高频调用接口
  bool enable_fast_lane_lut{true};

  /// @brief 日志文件路径（仅在 enable_log 为 true 时有效）
  std::string log_file;

  /// @brief 打印配置信息到标准输出（用于调试确认配置正确加载）
  void PrintInfo() {
    obstacle_map_meta_info.print();
    printf("surrounding_search_radius: %f\n", surrounding_search_radius);
    printf("enable_openloop_prediction: %d\n", enable_openloop_prediction);
    printf("enable_tracking_noise: %d\n", enable_tracking_noise);
    printf("enable_log: %d\n", enable_log);
    printf("enable_fast_lane_lut: %d\n", enable_fast_lane_lut);
    printf("log_file: %s\n", log_file.c_str());
  }
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_BASICS_H_
