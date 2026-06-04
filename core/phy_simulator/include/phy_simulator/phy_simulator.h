/**
 * @file phy_simulator.h
 * @author HKUST Aerial Robotics Group
 * @brief 物理仿真器核心模块 - 提供自动驾驶场景的完整物理仿真功能
 *
 * @version 0.1
 * @date 2019-03-20
 *
 * @copyright Copyright (c) 2019
 *
 * 本模块是 EPSILON 自动驾驶仿真系统的核心物理仿真引擎。
 * 其主要职责包括：
 *   - 加载并管理场景数据（车道网络、障碍物集合、车辆集合）
 *   - 为每辆车辆创建并维护独立的车辆运动学模型（VehicleModel）
 *   - 每个时间步根据外部控制信号更新所有车辆的运动状态
 *   - 支持运行时动态添加/删除临时障碍物
 *
 * 典型工作流程：
 *   1. 通过 ArenaLoader 加载 JSON 格式的场景配置文件
 *   2. 调用 SetupVehicleModelForVehicleSet() 为场景中的每辆车创建运动学模型
 *   3. 在每个仿真周期中，外部规划模块下发控制信号集（VehicleControlSignalSet）
 *   4. 调用 UpdateSimulatorUsingSignalSet() 根据控制信号和时间步长更新所有车辆状态
 *   5. 更新后的状态通过 RosAdapter 发布给 ROS 网络中的其他节点
 *
 * 仿真核心维护的数据结构：
 *   - vehicle_set_：车辆语义信息（ID、类型、参数、状态）
 *   - vehicle_model_set_：车辆运动学模型（实际执行仿真的对象）
 *   - lane_net_：车道网络拓扑结构
 *   - obstacle_set_：静态及临时障碍物集合
 */
#ifndef _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_PHY_SIMULATOR_H_
#define _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_PHY_SIMULATOR_H_

#include <assert.h>
#include <iostream>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"
#include "phy_simulator/arena_loader.h"
#include "phy_simulator/basics.h"
#include "vehicle_model/vehicle_model.h"

namespace phy_simulator {

/// @brief 物理仿真核心类
///
/// 维护仿真场景中所有实体的状态，并负责按时间步长推进整个仿真。
/// 是本模块的中枢调度类，协调场景加载、车辆建模和状态更新三个子任务。
///
/// 核心数据流：
///   场景 JSON 文件 --> ArenaLoader --> vehicle_set_/lane_net_/obstacle_set_
///   --> SetupVehicleModelForVehicleSet() --> vehicle_model_set_
///   --> 外部控制信号 --> UpdateSimulatorUsingSignalSet() --> 更新车辆状态
class PhySimulation {
 public:
  /// @brief 默认构造函数，使用 ArenaLoader 的默认路径加载场景
  PhySimulation();

  /// @brief 指定路径构造函数
  /// @param vehicle_set_path 车辆配置文件（vehicle_set.json）的路径
  /// @param map_path 障碍物地图配置文件（obstacles_norm.json）的路径
  /// @param lane_net_path 车道网络配置文件（lane_net_norm.json）的路径
  PhySimulation(const std::string &vehicle_set_path,
                const std::string &map_path, const std::string &lane_net_path);
  ~PhySimulation() {}

  /// @brief 获取车道网络
  common::LaneNet lane_net() const { return lane_net_; }

  /// @brief 获取障碍物集合
  common::ObstacleSet obstacle_set() const { return obstacle_set_; };

  /// @brief 获取车辆集合
  common::VehicleSet vehicle_set() const { return vehicle_set_; }

  /// @brief 获取所有车辆的 ID 列表
  const std::vector<int> vehicle_ids() const { return vehicle_ids_; }

  /// @brief 向场景中动态添加一个临时障碍物
  /// @param pt 障碍物的中心点坐标
  /// @param size 障碍物的边长（创建正方形障碍物）
  /// @return 成功返回 true
  ///
  /// 临时障碍物用于模拟运行时突然出现的障碍物（如施工区域、临时路障等），
  /// 障碍物会被同时加入到 obstacle_set_ 和 temp_obstacle_set_ 中，
  /// 以便后续单独追踪和移除。
  bool AddTemporaryObstacleToMap(const common::Point &pt, const double &size);

  /// @brief 根据位置移除附近的临时障碍物
  /// @param pt 目标点坐标
  /// @param s 移除半径阈值（中心距小于此值的临时障碍物将被移除）
  /// @return 成功返回 true
  ///
  /// 遍历 temp_obstacle_set_ 中所有临时障碍物，计算其中心点与 pt 的距离，
  /// 距离小于 s 的障碍物将被从 obstacle_set_ 和 temp_obstacle_set_ 中同时删除。
  bool RemoveTemporaryObstacle(const common::Point &pt, const double &s);

  /// @brief 使用控制信号集更新仿真状态（主仿真步进函数）
  /// @param signal_set 包含所有车辆控制信号的集合
  /// @param dt 仿真时间步长（秒）
  /// @return 成功返回 true
  ///
  /// 每个仿真周期由外部调用一次此函数，将控制信号集中的每条指令
  /// 分发给对应车辆的 VehicleModel，并使其前进一步。
  bool UpdateSimulatorUsingSignalSet(
      const common::VehicleControlSignalSet &signal_set, const decimal_t &dt);

 private:
  /// @brief 从 ArenaLoader 中获取所有场景数据
  /// @return 成功返回 true
  ///
  /// 依次调用 ArenaLoader 的 ParseVehicleSet()、ParseMapInfo()、ParseLaneNetInfo()
  /// 方法，将 JSON 数据填充到 vehicle_set_、obstacle_set_、lane_net_ 中。
  bool GetDataFromArenaLoader();

  /// @brief 为车辆集合中的每辆车创建对应的运动学模型
  /// @return 成功返回 true
  ///
  /// 遍历 vehicle_set_ 中的所有车辆，根据每辆车的参数（轴距、最大转向角）
  /// 创建一个 VehicleModel 实例，并用车辆的初始状态初始化模型状态。
  /// 同时将车辆 ID 收集到 vehicle_ids_ 向量中，供外部订阅控制信号使用。
  bool SetupVehicleModelForVehicleSet();

  /// @brief 根据控制信号和时间步长更新所有车辆的状态
  /// @param signal_set 控制信号集合
  /// @param dt 时间步长
  /// @return 成功返回 true
  ///
  /// 对每辆车：
  ///   - 若非开环模式（is_openloop == false）：设置控制量（转向速率、加速度），
  ///     并调用 VehicleModel::Step(dt) 进行一步运动学仿真
  ///   - 若为开环模式：直接使用控制信号中的目标状态覆盖车辆当前状态
  ///   - 将更新后的状态回写到 vehicle_set_ 中
  bool UpdateVehicleStates(const common::VehicleControlSignalSet &signal_set,
                           const decimal_t &dt);

  /// @brief 场景加载器指针，负责解析 JSON 配置文件
  ArenaLoader *p_arena_loader_;

  /// @brief 车辆语义信息集合（包含 ID、类型、参数、状态）
  common::VehicleSet vehicle_set_;

  /// @brief 车辆运动学模型映射表（key=车辆ID, value=VehicleModel实例）
  ///
  /// 此数据结构是仿真的核心执行单元，每个模型负责一辆车的运动学计算。
  /// 与 vehicle_set_ 的区别：这里的 VehicleModel 有实际的运动学计算能力。
  std::unordered_map<int, simulator::VehicleModel> vehicle_model_set_;

  /// @brief 车辆 ID 列表，按 vehicle_set_ 的插入顺序排列
  /// 用于方便遍历以及为每辆车创建信号订阅
  std::vector<int> vehicle_ids_;

  /// @brief 车道网络拓扑结构
  common::LaneNet lane_net_;

  /// @brief 障碍物集合（包含静态加载的和运行时动态添加的）
  common::ObstacleSet obstacle_set_;

  /// @brief 临时障碍物计数器，用于生成唯一的临时障碍物 ID
  int temp_obstacle_cnt_ = 0;

  /// @brief 临时障碍物 ID 起始偏移量，防止与静态障碍物 ID 冲突
  int temp_obs_idx_offset_ = 10000;

  /// @brief 仅临时障碍物的独立集合，用于追踪和批量删除
  /// 与 obstacle_set_ 中的临时障碍物部分是冗余副本，便于单独管理
  std::unordered_map<int, common::PolygonObstacle> temp_obstacle_set_;
};

}  // namespace phy_simulator

#endif  // _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_PHY_SIMULATOR_H_
