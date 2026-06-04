/**
 * @file arena_loader.h
 * @author HKUST Aerial Robotics Group
 * @brief 竞技场场景加载器 - 负责从 JSON 配置文件解析场景数据
 *
 * @version 0.1
 * @date 2019-03-18
 *
 * @copyright Copyright (c) 2019
 *
 * 本模块负责将自动驾驶仿真场景的 JSON 配置文件解析为 C++ 数据结构。
 * 每个场景通常包含以下 4 个 JSON 文件：
 *
 *   1. vehicle_set.json  —— 车辆配置（ID、类型、初始状态、物理参数）
 *   2. obstacles_norm.json —— 障碍物信息（多边形区域，标准化坐标系）
 *   3. lane_net_norm.json  —— 车道网络拓扑（车道中心线、连接关系、变换规则）
 *   4. agent_config.json   —— 规划器/代理配置（agent 行为参数）
 *
 * ArenaLoader 支持三种构造/配置方式：
 *   - 默认构造（路径留空，由外部调用 set_* 方法设置）
 *   - 三参数构造（一次性传入路径）
 *   - 逐路径设置（通过 set_* 方法）
 *
 * 解析出的数据分别填充到对应的 common 命名空间数据结构中：
 *   - common::VehicleSet
 *   - common::ObstacleSet
 *   - common::LaneNet
 */
#ifndef _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ARENA_LOADER_H_
#define _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ARENA_LOADER_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <json/json.hpp>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"

#include "phy_simulator/basics.h"

namespace phy_simulator {

/// @brief 竞技场/场景加载器
///
/// 使用 nlohmann/json 库解析 JSON 格式的场景配置文件，
/// 将数据提取到 C++ 数据结构中供 PhySimulation 使用。
///
/// JSON 解析流程：
///   1. ParseVehicleSet() —— 解析车辆配置：创建 common::Vehicle 对象，设置 ID、
///      类型（type/subclass）、初始状态（位置、朝向、速度等）、物理参数
///      （尺寸、轴距、最大转向角、最大纵向/横向加速度等）
///   2. ParseMapInfo() —— 解析障碍物：提取 GeoJSON Feature 格式的多边形障碍物，
///      包括顶点坐标、ID、类型标记（is_spec/is_valid）
///   3. ParseLaneNetInfo() —— 解析车道网络：提取车道 ID、方向、连接关系
///      （child_id/father_id）、相邻车道 ID（left_id/right_id）、变道权限、
///      车道中心线采样点等
class ArenaLoader {
 public:
  /**
   * @brief Default constructor
   */
  /// @brief 默认构造函数，初始化空路径
  ArenaLoader();

  /**
   * @brief Construct a new ArenaLoader object
   *
   * @param vehicle_set_path
   * @param map_path
   * @param lane_net_path
   */
  /// @brief 带路径参数的构造函数
  /// @param vehicle_set_path 车辆集合 JSON 配置文件路径
  /// @param map_path 障碍物地图 JSON 配置文件路径
  /// @param lane_net_path 车道网络 JSON 配置文件路径
  ArenaLoader(const std::string &vehicle_set_path, const std::string &map_path,
              const std::string &lane_net_path);

  /// @brief 获取车辆配置文件路径
  inline std::string vehicle_set_path() const { return vehicle_set_path_; }
  /// @brief 获取障碍物地图配置文件路径
  inline std::string map_path() const { return map_path_; }
  /// @brief 获取车道网络配置文件路径
  inline std::string lane_net_path() const { return lane_net_path_; }

  /// @brief 设置车辆配置文件路径
  inline void set_vehicle_set_path(const std::string &path) {
    vehicle_set_path_ = path;
  }
  /// @brief 设置障碍物地图配置文件路径
  inline void set_map_path(const std::string &path) { map_path_ = path; }
  /// @brief 设置车道网络配置文件路径
  inline void set_lane_net_path(const std::string &path) {
    lane_net_path_ = path;
  }

  /**
   * @brief Parse vehicles info from json
   *
   * @param p_vehicle_set
   * @return true Parsing success
   * @return false Parsing failed
   */
  /// @brief 解析车辆配置 JSON 文件
  /// @param p_vehicle_set [输出] 解析结果填充的 VehicleSet 对象指针
  /// @return 解析成功返回 true
  ///
  /// 从 vehicle_set_path_ 指定的 JSON 文件中读取所有车辆的定义信息。
  /// JSON 结构为 root["vehicles"]["info"] 数组，每个元素包含：
  ///   - id: 车辆唯一标识
  ///   - subclass/type: 车辆类型和子类型（如 "car"/"ego_vehicle"）
  ///   - init_state: 车辆初始状态（位置、朝向、曲率、速度、加速度、转向角）
  ///   - params: 车辆物理参数（尺寸、轴距、悬架位置、最大转向角、加速度限制等）
  ///
  /// 注意：最大转向角在 JSON 中存储为角度制，解析时转换为弧度制（* kPi / 180.0）。
  /// 同时计算 d_cr（后轴到质心的纵向距离）= length/2 - rear_suspension。
  bool ParseVehicleSet(common::VehicleSet *p_vehicle_set);

  /**
   * @brief Parse obstacle info from json
   *
   * @param p_obstacle_set
   * @return true Parsing success
   * @return false Parsing failed
   */
  /// @brief 解析障碍物地图 JSON 文件
  /// @param p_obstacle_set [输出] 解析结果填充的 ObstacleSet 对象指针
  /// @return kSuccess 解析成功，其他值表示出错
  ///
  /// 从 map_path_ 指定的 JSON 文件中读取障碍物信息。
  /// JSON 结构为 GeoJSON FeatureCollection 格式，每个 Feature 对应一个多边形障碍物。
  /// 解析时跳过 is_valid 为 false 的无效障碍物。
  /// 障碍物类型（is_spec 字段）：0 表示普通障碍物，非 0 表示特殊障碍物（如边界）。
  ErrorType ParseMapInfo(common::ObstacleSet *p_obstacle_set);

  /**
   * @brief Parse map info from json
   *
   * @param p_lane_net
   * @return true Parsing success
   * @return false Parsing failed
   */
  /// @brief 解析车道网络 JSON 文件
  /// @param p_lane_net [输出] 解析结果填充的 LaneNet 对象指针
  /// @return kSuccess 解析成功，其他值表示出错
  ///
  /// 从 lane_net_path_ 指定的 JSON 文件中读取车道网络拓扑信息。
  /// JSON 结构为 GeoJSON FeatureCollection 格式，每个 Feature 对应一条车道。
  /// 车道属性包括：
  ///   - id/length/dir: 车道 ID、长度、方向
  ///   - child_id/father_id: 前驱和后继车道 ID（支持逗号分隔的多值）
  ///   - left_id/right_id: 左右相邻车道 ID
  ///   - lchg_vld/rchg_vld: 左右变道是否允许
  ///   - behavior: 车道行为类型（如 "直行"、"转弯"）
  ///   - geometry.coordinates: 车道中心线采样点坐标序列
  ErrorType ParseLaneNetInfo(common::LaneNet *p_lane_net);

 private:
  std::string vehicle_set_path_;     ///< 车辆配置文件路径
  std::string map_path_;             ///< 障碍物地图配置文件路径
  std::string lane_net_path_;        ///< 车道网络配置文件路径
  std::string pedestrian_set_path_;  ///< 行人集合配置文件路径（当前未使用）
};

}  // namespace phy_simulator

#endif  // _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ARENA_LOADER_H_
