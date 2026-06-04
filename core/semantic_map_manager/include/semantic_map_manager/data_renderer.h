/**
 * @file data_renderer.h
 * @author EPSILON Autonomous Driving Group
 * @brief 数据渲染器——将ROS ArenaInfo消息渲染为 SemanticMapManager 的内部数据结构
 *
 * @details
 * DataRenderer 是数据管道的第一阶段处理器，负责：
 *
 * 1. 从原始 ROS 仿真数据中提取并组织为内部表示：
 *    - 自车提取（GetEgoVehicle）：从 vehicle_set 中按 ego_id_ 取出自车
 *    - 障碍物地图构建（GetObstacleMap）：将 ObstacleSet 中的圆形/多边形障碍物
 *      栅格化到 OccupancyGrid 中（使用 OpenCV 的 circle/fillPoly 算法）
 *    - 车道网络分离（GetWholeLaneNet / GetSurroundingLaneNet）：通过 KD 树
 *      半径搜索截取自车周围的车道，构建 surrounding_lane_net_
 *    - 周围车辆筛选（GetSurroundingVehicles）：以欧氏距离为判据，筛选距离小于
 *      surrounding_search_radius_ 的车辆
 *
 * 2. 伪感知管线（FakeMapper）：
 *    - RayCastingOnObstacleMap：以自车几何中心为原点，使用 Roguelike 光线投射
 *      算法对8个扇区进行射线扫描，发现障碍物栅格坐标
 *    - 维护 obs_grids_ 记忆缓存：保留历史扫描结果，清除远离车辆(>80%地图半边长)的过期栅格
 *    - 将射线扫描发现的障碍物叠加回栅格地图（标记为 SCANNED_OCCUPIED）
 *
 * 3. 观测噪声注入（InjectObservationNoise）：
 *    - 仅对 ego_id_ == 0 的智能体生效
 *    - 每10帧对 surround_vehicles 中随机选取最多3辆车注入高斯噪声：
 *      - 横向位置噪声：std=0.2m
 *      - 纵向位置噪声：std=0.7m
 *      - 角度噪声：std=0.22rad
 *    - 角度偏差超过 1.5*std 且非故障车的车辆被标记为 uncertain_vehicle_ids_
 *
 * 4. 在所有数据准备完成后，调用 p_semantic_map_manager_->UpdateSemanticMap(...)
 *    将渲染后的数据推入世界模型。
 *
 * @version 0.1
 * @date 2019-03-20
 */

#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_DATA_RENDERER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_DATA_RENDERER_H_

#include <random>
#include <assert.h>
#include <iostream>
#include <set>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "nanoflann/include/nanoflann.hpp"
#include "semantic_map_manager/semantic_map_manager.h"

namespace semantic_map_manager {

/// @class DataRenderer
/// @brief 将原始的ROS仿真数据（车道、车辆、障碍物）"渲染"为语义地图管理器可消费的内部格式
///
/// 该类是数据管道的第一个处理阶段，相当于感知模块的模拟替代：
/// 它接收来自物理仿真器的 ArenaInfo 消息，进行栅格化、搜索截取、噪声注入等操作，
/// 最终将所有处理好的数据送入 SemanticMapManager::UpdateSemanticMap。
class DataRenderer {
 public:
  using ObstacleMapType = uint8_t;  ///< 障碍物地图单元类型
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;  ///< 二维栅格地图

  /// @brief 构造函数，从 SemanticMapManager 中获取初始配置
  /// @param smm_ptr 指向 SemanticMapManager 的指针（用于获取ego_id、障碍物地图信息等）
  DataRenderer(SemanticMapManager *smm_ptr);
  ~DataRenderer() {}

  inline void set_ego_id(const int id) { ego_id_ = id; }
  inline void set_obstacle_map_info(const common::GridMapMetaInfo &info) {
    obstacle_map_info_ = info;
  }

  /// @brief 主渲染入口——将ROS仿真数据解包、处理、注入语义地图管理器
  ///
  /// 执行顺序：
  ///   1. GetEgoVehicle —— 提取自车（必须在障碍物地图之前）
  ///   2. GetObstacleMap —— 构建障碍物栅格地图
  ///   3. GetWholeLaneNet / GetSurroundingLaneNet —— 车道网络截取
  ///   4. GetSurroundingVehicles —— 周围车辆筛选
  ///   5. [可选] InjectObservationNoise —— 注入跟踪噪声
  ///   6. FakeMapper —— 伪感知管线（射线投射）
  ///   7. UpdateSemanticMap —— 推送至世界模型
  ///
  /// @param time_stamp 当前时间戳
  /// @param lane_net 完整车道网络
  /// @param vehicle_set 所有车辆集合
  /// @param obstacle_set 障碍物集合（含圆形和多边形障碍物）
  ErrorType Render(const double &time_stamp, const common::LaneNet &lane_net,
                   const common::VehicleSet &vehicle_set,
                   const common::ObstacleSet &obstacle_set);

 private:
  /// @brief nanoflann KD树类型定义——用于车道网络和障碍物的2D近邻搜索
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<decimal_t, common::PointVecForKdTree>,
      common::PointVecForKdTree, 2>
      KdTreeFor2dPointVec;

  // ======================== 数据提取函数 ========================

  /// @brief 从车辆集合中按 ego_id_ 提取自车
  ErrorType GetEgoVehicle(const common::VehicleSet &vehicle_set);
  /// @brief 将障碍物集合栅格化到 p_obstacle_grid_ 中
  ErrorType GetObstacleMap(const common::ObstacleSet &obstacle_set);
  /// @brief 存储完整车道网络（直接赋值）
  ErrorType GetWholeLaneNet(const common::LaneNet &lane_net);
  /// @brief 通过KD树半径搜索截取自车周围车道
  ErrorType GetSurroundingLaneNet(const common::LaneNet &lane_net);
  /// @brief 按欧氏距离筛选自车周围的车辆
  ErrorType GetSurroundingVehicles(const common::VehicleSet &vehicle_set);
  /// @brief [预留] 获取周围物体
  ErrorType GetSurroundingObjects();

  // ======================== 伪感知管线 ========================

  /// @brief 使用 Roguelike 光线投射算法扫描障碍物地图
  ///
  /// 以自车几何中心为起点，对8个FOV扇区进行射线扫描，
  /// 发现障碍物栅格并将其坐标存入 obs_grids_
  ErrorType RayCastingOnObstacleMap();
  /// @brief 组合射线投射结果与历史缓存，维护带记忆的障碍物地图
  ///
  /// 逻辑：
  ///   1. 调用 RayCastingOnObstacleMap 获取当前帧的障碍物栅格
  ///   2. 遍历历史 obs_grids_：清除远离车辆的过期栅格（距离>80%地图半边长）
  ///   3. 将历史缓存中的障碍物标记回栅格地图（SCANNED_OCCUPIED 状态）
  ErrorType FakeMapper();

  /// @brief 注入观测噪声——模拟真实感知系统的跟踪不确定性
  /// @note 仅 ego_id_ == 0 时生效，每10帧执行一次
  ErrorType InjectObservationNoise();

  // ======================== 车道网络KD树相关 ========================

  bool if_kdtree_lane_net_updated_ = false;  ///< 车道网络KD树是否需要更新
  common::PointVecForKdTree lane_net_pts_;    ///< 车道网络采样点（用于KD树构建）
  std::shared_ptr<KdTreeFor2dPointVec> kdtree_lane_net_;  ///< 车道网络KD树

  // ======================== 障碍物KD树相关 ========================

  bool if_kdtree_obstacle_set_updated_ = false;  ///< 障碍物KD树是否需要更新
  common::PointVecForKdTree obstacle_set_pts_;    ///< 障碍物采样点
  std::shared_ptr<KdTreeFor2dPointVec> kdtree_obstacle_set_;  ///< 障碍物KD树

  common::PointVecForKdTree vehicle_set_pts_;     ///< 车辆位置点集
  std::shared_ptr<KdTreeFor2dPointVec> kdtree_vehicle_;  ///< 车辆KD树

  int ego_id_ = 0;                    ///< 自车ID
  int ray_casting_num_ = 1440;        ///< 光线投射射线数量

  /// @brief 障碍物占据的栅格坐标集合（含历史缓存），key为 (x, y) 世界坐标
  std::set<std::array<decimal_t, 2>> obs_grids_;
  /// @brief 空闲栅格坐标集合
  std::set<std::array<decimal_t, 2>> free_grids_;

  double time_stamp_;                ///< 当前数据时间戳

  common::Vehicle ego_vehicle_;      ///< 自车完整信息
  common::VehicleParam ego_param_;   ///< 自车参数（长宽轴距等）
  common::State ego_state_;          ///< 自车状态（位置速度等）

  common::VehicleSet surrounding_vehicles_;  ///< 周围车辆集合

  common::GridMapMetaInfo obstacle_map_info_;  ///< 障碍物地图元信息（分辨率、尺寸）
  GridMap2D *p_obstacle_grid_;                 ///< 障碍物栅格地图指针

  decimal_t surrounding_search_radius_;  ///< 周围搜索半径（米）

  common::LaneNet surrounding_lane_net_;  ///< 局部车道网络
  common::LaneNet whole_lane_net_;        ///< 完整车道网络

  SemanticMapManager *p_semantic_map_manager_;  ///< 语义地图管理器指针（数据输出目标）

  // ======================== 噪声注入相关 ========================

  std::vector<int> uncertain_vehicle_ids_;  ///< 因噪声而状态不确定的车辆ID
  std::mt19937 random_engine_;              ///< Mersenne Twister 随机数引擎
  int cnt_random_ = 0;                      ///< 噪声注入帧计数器（每10帧触发一次）
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_DATA_RENDERER_H_
