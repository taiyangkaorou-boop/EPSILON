/**
 * @file phy_simulator.cc
 * @author HKUST Aerial Robotics Group
 * @brief 物理仿真器核心模块实现 —— 车辆运动学仿真与场景管理
 *
 * @version 0.1
 * @date 2019-03-20
 *
 * @copyright Copyright (c) 2019
 *
 * 本文件实现了 PhySimulation 类的所有核心方法，是整个物理仿真系统的执行引擎。
 *
 * 核心功能实现：
 *   1. 场景加载流程：构造 ArenaLoader → 解析 JSON → 获取场景数据
 *   2. 车辆建模：遍历 VehicleSet 中每辆车 → 创建 VehicleModel 实例
 *   3. 仿真步进：接收控制信号 → 分发到各车辆模型 → 调用 Step() 推进运动学
 *   4. 临时障碍物管理：运行时动态追加和删除障碍物
 *
 * 仿真算法核心（UpdateVehicleStates）：
 *   每辆车的控制信号分为两种模式：
 *   - 闭环模式（is_openloop == false）：设置加速踏板和转向速率，
 *     由 VehicleModel 内部执行一步运动学仿真（自行车模型）
 *   - 开环模式（is_openloop == true）：直接使用控制信号中的目标状态
 *     覆盖车辆当前状态，用于精确回放或调试场景
 *
 * 临时障碍物采用 ID 偏移策略（temp_obs_idx_offset_ = 10000），
 * 确保不与静态障碍物 ID 冲突。同时维护独立的 temp_obstacle_set_
 * 用于追踪临时障碍物，支持基于空间距离的批量移除。
 */
#include "phy_simulator/phy_simulator.h"


namespace phy_simulator {

/// @brief 默认构造函数
///
/// 使用默认 ArenaLoader（路径为空），调用 GetDataFromArenaLoader()
/// 加载场景数据。如果加载失败，触发 assert(false) 终止程序。
PhySimulation::PhySimulation() {
  p_arena_loader_ = new ArenaLoader();
  if (!GetDataFromArenaLoader()) assert(false);
}

/// @brief 带路径参数的构造函数
///
/// 1. 创建 ArenaLoader 实例
/// 2. 设置三个文件的路径
/// 3. 从 ArenaLoader 解析场景数据（GetDataFromArenaLoader）
/// 4. 为所有车辆创建运动学模型（SetupVehicleModelForVehicleSet）
PhySimulation::PhySimulation(const std::string &vehicle_set_path,
                             const std::string &map_path,
                             const std::string &lane_net_path) {
  std::cout << "[PhySimulation] Constructing..." << std::endl;
  p_arena_loader_ = new ArenaLoader();

  // 设置场景配置文件的路径
  p_arena_loader_->set_vehicle_set_path(vehicle_set_path);
  p_arena_loader_->set_map_path(map_path);
  p_arena_loader_->set_lane_net_path(lane_net_path);

  // 加载并解析场景数据
  GetDataFromArenaLoader();
  // 为每辆车创建独立的运动学模型
  SetupVehicleModelForVehicleSet();
}

/// @brief 从 ArenaLoader 获取所有场景数据
///
/// 依次调用 ArenaLoader 的三个解析方法，将 JSON 数据提取到内部数据结构中：
///   1. ParseVehicleSet → vehicle_set_（车辆定义）
///   2. ParseMapInfo → obstacle_set_（障碍物集合）
///   3. ParseLaneNetInfo → lane_net_（车道网络拓扑）
bool PhySimulation::GetDataFromArenaLoader() {
  std::cout << "[PhySimulation] Parsing simulation info..." << std::endl;
  p_arena_loader_->ParseVehicleSet(&vehicle_set_);
  p_arena_loader_->ParseMapInfo(&obstacle_set_);
  p_arena_loader_->ParseLaneNetInfo(&lane_net_);
  return true;
}

/// @brief 动态添加临时障碍物
///
/// 以 pt 为中心、s 为边长创建一个正方形障碍物：
///   - 四个顶点分别为 (pt.x - s/2, pt.y - s/2) 等四个角点
///   - 障碍物类型设为 1（标记为非普通障碍物）
///   - ID 使用偏移量 temp_obs_idx_offset_ + 计数器，确保唯一性
///
/// 同时将障碍物加入：
///   - obstacle_set_：全局障碍物集合
///   - temp_obstacle_set_：仅临时障碍物的独立集合（用于追踪和删除）
bool PhySimulation::AddTemporaryObstacleToMap(const common::Point &pt,
                                              const double &s) {
  common::PolygonObstacle obs;
  // 构造正方形顶点：从左上角顺时针排列
  obs.polygon.points.push_back(common::Point(pt.x - s / 2, pt.y - s / 2));
  obs.polygon.points.push_back(common::Point(pt.x - s / 2, pt.y + s / 2));
  obs.polygon.points.push_back(common::Point(pt.x + s / 2, pt.y + s / 2));
  obs.polygon.points.push_back(common::Point(pt.x + s / 2, pt.y - s / 2));
  obs.type = 1;

  // 使用偏移量生成唯一 ID，避免与静态障碍物 ID 冲突
  obs.id = temp_obs_idx_offset_ + temp_obstacle_cnt_;
  obstacle_set_.obs_polygon.insert(
      std::pair<int, common::PolygonObstacle>(obs.id, obs));

  // 在独立的临时障碍物集合中也记录一份，便于后续追踪和批量移除
  temp_obstacle_set_.insert(
      std::pair<int, common::PolygonObstacle>(obs.id, obs));

  ++temp_obstacle_cnt_;
  return true;
}

/// @brief 为车辆集合中的所有车辆创建运动学模型
///
/// 遍历 vehicle_set_ 中的每辆车，执行以下步骤：
///   1. 创建 VehicleModel(轴距, 最大转向角) 实例
///   2. 用车辆的初始状态初始化模型状态
///   3. 将 <车辆ID, VehicleModel> 键值对插入 vehicle_model_set_
///   4. 将车辆 ID 追加到 vehicle_ids_ 列表中
///
/// VehicleModel 使用自行车运动学模型（bicycle kinematic model），
/// 输入为加速度和控制转向速率，输出为更新后的位置和朝向。
bool PhySimulation::SetupVehicleModelForVehicleSet() {
  for (const auto &p : vehicle_set_.vehicles) {
    // 使用车辆物理参数（轴距、最大转向角）初始化车辆运动学模型
    simulator::VehicleModel vehicle_model(
        p.second.param().wheel_base(), p.second.param().max_steering_angle());
    // 将车辆的 JSON 初始状态设置到模型中
    vehicle_model.set_state(p.second.state());
    vehicle_model_set_.insert(
        std::pair<int, simulator::VehicleModel>(p.first, vehicle_model));

    // 收集车辆 ID，供外部创建信号订阅 topic 使用
    vehicle_ids_.push_back(p.first);
  }
  return true;
}

/// @brief 核心仿真步进函数 —— 使用控制信号集更新所有车辆状态
///
/// 这是仿真循环中每个时间步必须调用的入口函数。
/// 内部委托给 UpdateVehicleStates() 完成实际的状态更新。
///
/// 使用 TicToc 计时器测量每次更新的耗时，用于性能监控。
bool PhySimulation::UpdateSimulatorUsingSignalSet(
    const common::VehicleControlSignalSet &signal_set, const decimal_t &dt) {
  TicToc updata_vehicle_time;
  UpdateVehicleStates(signal_set, dt);
  return true;
}

/// @brief 更新所有车辆状态的实现函数
///
/// 算法的核心逻辑：
///   1. 检查控制信号数量是否与车辆数量一致，不一致则报错并终止
///   2. 遍历 vehicle_set_ 中的每辆车：
///      a. 根据车辆 ID 查找对应的控制信号
///      b. 提取 steer_rate（转向速率）和 acc（加速度）
///      c. 在 vehicle_model_set_ 中查找对应的运动学模型
///      d. 如果控制信号不是开环模式：
///         - 设置模型的加速度和转向速率
///         - 调用 model.Step(dt) 执行一步运动学仿真
///      e. 如果控制信号是开环模式：
///         - 直接用控制信号中的目标状态覆盖模型状态
///         （用于精确回放或调试场景）
///      f. 将更新后的模型状态回写到 vehicle_set_ 中的对应车辆
bool PhySimulation::UpdateVehicleStates(
    const common::VehicleControlSignalSet &signal_set, const decimal_t &dt) {
  // 安全校验：控制信号数量必须与车辆数量匹配
  if (signal_set.signal_set.size() != vehicle_set_.vehicles.size()) {
    std::cerr << "[PhySimulation] ERROR - Signal number error." << std::endl;
    std::cerr << "[PhySimulation] signal_set num: "
              << signal_set.signal_set.size()
              << ", vehicle_set num: " << vehicle_set_.vehicles.size()
              << std::endl;
    assert(false);
  }

  // 逐车更新状态
  for (auto iter = vehicle_set_.vehicles.begin();
       iter != vehicle_set_.vehicles.end(); ++iter) {
    int id = iter->first;
    // 获取对应车辆的控制信号
    common::VehicleControlSignal signal = signal_set.signal_set.at(id);
    decimal_t steer_rate = signal.steer_rate;  // 转向角变化速率
    decimal_t acc = signal.acc;                // 纵向加速度

    auto model_iter = vehicle_model_set_.find(id);

    if (!signal.is_openloop) {
      // 闭环仿真模式：设置控制量，执行一步运动学更新
      model_iter->second.set_control(
          simulator::VehicleModel::Control(steer_rate, acc));
      model_iter->second.Step(dt);
    } else {
      // 开环模式：直接用目标状态覆盖（用于轨迹回放或调试）
      model_iter->second.set_state(signal.state);
    }
    // 将更新后的状态同步回 vehicle_set_，供其他模块读取
    iter->second.set_state(model_iter->second.state());
  }
  return true;
}

/// @brief 移除指定位置附近的临时障碍物
///
/// 遍历 temp_obstacle_set_ 中的所有临时障碍物：
///   1. 计算障碍物多边形所有顶点的平均坐标作为中心点
///   2. 计算中心点与目标点 pt 之间的欧氏距离
///   3. 如果距离小于阈值 s，则从 obstacle_set_ 和 temp_obstacle_set_
///      中同时删除该障碍物
///
/// 设计意图：支持运行时通过鼠标点击等方式清除已添加的临时障碍物，
/// 阈值为用户提供了一个"拾取半径"的容差。
bool PhySimulation::RemoveTemporaryObstacle(const common::Point &pt,
                                            const double &s) {
  for (auto it = temp_obstacle_set_.begin(); it != temp_obstacle_set_.end();) {
    // 计算障碍物多边形的几何中心（所有顶点的算术平均）
    decimal_t sum_x = 0, sum_y = 0;
    for (const auto &p : it->second.polygon.points) {
      sum_x += p.x;
      sum_y += p.y;
    }
    decimal_t center_x = sum_x / it->second.polygon.points.size();
    decimal_t center_y = sum_y / it->second.polygon.points.size();

    // 计算中心点到目标点的欧氏距离
    decimal_t dx = center_x - pt.x;
    decimal_t dy = center_y - pt.y;
    decimal_t d = std::hypot(dx, dy);

    if (d < s) {
      // 距离在阈值内：从全局障碍物集合中移除
      auto it_obs_set = obstacle_set_.obs_polygon.find(it->first);
      if (it_obs_set != obstacle_set_.obs_polygon.end()) {
        obstacle_set_.obs_polygon.erase(it_obs_set);
      }
      // 从临时障碍物追踪集合中移除（erase 返回下一个有效迭代器）
      it = temp_obstacle_set_.erase(it);
    } else {
      ++it;
    }
  }
  return true;
}

}  // namespace phy_simulator
