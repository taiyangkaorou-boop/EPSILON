/**
 * @file arena_loader.cc
 * @author HKUST Aerial Robotics Group
 * @brief 竞技场场景加载器实现 —— JSON 文件解析与数据结构填充
 *
 * @version 0.1
 * @date 2019-03-18
 *
 * @copyright Copyright (c) 2019
 *
 * 本文件实现了 ArenaLoader 类的三个核心解析方法，负责将
 * JSON 格式的场景描述文件解析为 C++ 数据结构。
 *
 * 解析目标文件：
 *   1. vehicle_set.json  → ParseVehicleSet()
 *      - 解析车辆 ID、类型、初始状态、物理参数
 *      - 参数转换：最大转向角（度 → 弧度）、d_cr（质心到后轴距离）
 *
 *   2. obstacles_norm.json → ParseMapInfo()
 *      - 解析 GeoJSON FeatureCollection 格式的多边形障碍物
 *      - 过滤无效障碍物（is_valid == false）
 *      - 记录障碍物类型（is_spec：普通/特殊）
 *
 *   3. lane_net_norm.json → ParseLaneNetInfo()
 *      - 解析车道网络拓扑（GeoJSON FeatureCollection 格式）
 *      - 解析 child_id/father_id（逗号分隔的字符串 → int 向量）
 *      - 提取车道中心线采样点序列
 *      - 记录变道权限和车道行为类型
 *
 * 所有方法均使用 nlohmann::json 库进行 JSON 解析。
 * JSON 文件路径在解析前必须通过构造函数或 set_* 方法设置。
 */
#include "phy_simulator/arena_loader.h"

namespace phy_simulator {

using Json = nlohmann::json;

/// @brief 默认构造函数（路径为空）
ArenaLoader::ArenaLoader() {}

/// @brief 带路径参数的构造函数，使用初始化列表设置路径
ArenaLoader::ArenaLoader(const std::string &vehicle_set_path,
                         const std::string &map_path,
                         const std::string &lane_net_path)
    : vehicle_set_path_(vehicle_set_path),
      map_path_(map_path),
      lane_net_path_(lane_net_path) {}

/// @brief 解析车辆配置文件（vehicle_set.json）
///
/// JSON 结构：
///   root["vehicles"]["info"]: 车辆信息数组
///     每辆车包含：
///       - id: 车辆唯一标识（int）
///       - subclass: 子类型字符串（如 "ego_vehicle"）
///       - type: 类型字符串（如 "car"）
///       - init_state: 初始状态对象 {x, y, angle, curvature, velocity, acceleration, steer}
///       - params: 物理参数对象 {width, length, wheel_base, front_suspension,
///                 rear_suspension, max_steering_angle, max_longitudinal_acc, max_lateral_acc}
///
/// 数据处理细节：
///   - max_steering_angle：JSON 中为角度值，乘以 kPi/180.0 转换为弧度
///   - d_cr（质心到后轴纵向距离）：由 length/2 - rear_suspension 计算得出
///
/// 解析完成后打印车辆集合的摘要信息。
bool ArenaLoader::ParseVehicleSet(common::VehicleSet *p_vehicle_set) {
  printf("\n[ArenaLoader] Loading vehicle set\n");

  // 打开并解析 JSON 文件
  std::fstream fs(vehicle_set_path_);
  Json root;
  fs >> root;

  Json vehicles_json = root["vehicles"];
  Json info_json = vehicles_json["info"];
  // ~ 遍历所有车辆条目（支持只加载部分车辆用于调试）
  // ~ allow loading part of the vehicles for debugging purpose
  for (int i = 0; i < static_cast<int>(info_json.size()); ++i) {
    common::Vehicle vehicle;
    // 解析车辆基础信息
    vehicle.set_id(info_json[i]["id"].get<int>());
    vehicle.set_subclass(info_json[i]["subclass"].get<std::string>());
    vehicle.set_type(info_json[i]["type"].get<std::string>());

    // ---- 解析初始状态 ----
    Json state_json = info_json[i]["init_state"];
    common::State state;
    state.vec_position(0) = state_json["x"].get<double>();
    state.vec_position(1) = state_json["y"].get<double>();
    state.angle = state_json["angle"].get<double>();              // 朝向角（弧度）
    state.curvature = state_json["curvature"].get<double>();      // 路径曲率
    state.velocity = state_json["velocity"].get<double>();        // 初始速度
    state.acceleration = state_json["acceleration"].get<double>(); // 初始加速度
    state.steer = state_json["steer"].get<double>();              // 初始转向角
    vehicle.set_state(state);

    // ---- 解析物理参数 ----
    Json params_json = info_json[i]["params"];
    common::VehicleParam param;
    param.set_width(params_json["width"].get<double>());             // 车宽
    param.set_length(params_json["length"].get<double>());            // 车长
    param.set_wheel_base(params_json["wheel_base"].get<double>());   // 轴距
    param.set_front_suspension(params_json["front_suspension"].get<double>()); // 前悬长度
    param.set_rear_suspension(params_json["rear_suspension"].get<double>());   // 后悬长度
    // 最大转向角：JSON 中以角度存储，转换为弧度
    auto max_steering_angle = params_json["max_steering_angle"].get<double>();
    param.set_max_steering_angle(max_steering_angle * kPi / 180.0);
    param.set_max_longitudinal_acc(
        params_json["max_longitudinal_acc"].get<double>());         // 最大纵向加速度
    param.set_max_lateral_acc(params_json["max_lateral_acc"].get<double>());  // 最大横向加速度

    // d_cr：质心到后轴中心的纵向距离（正值向前）
    // 计算公式：车长/2 - 后悬长度
    param.set_d_cr(param.length() / 2 - param.rear_suspension());
    vehicle.set_param(param);

    // 插入车辆集合（以 ID 为键）
    p_vehicle_set->vehicles.insert(
        std::pair<int, common::Vehicle>(vehicle.id(), vehicle));
  }
  p_vehicle_set->print();  // 打印车辆集合摘要

  fs.close();
  return true;
}

/// @brief 解析障碍物地图配置文件（obstacles_norm.json）
///
/// JSON 结构（GeoJSON FeatureCollection 格式）：
///   root["features"]: 障碍物特征数组
///     每个特征包含：
///       - properties.id: 障碍物 ID
///       - properties.is_valid: 是否有效（0=无效，跳过）
///       - properties.is_spec: 类型标记（0=普通障碍物，非0=特殊障碍物）
///       - geometry.coordinates[0][0]: 多边形顶点坐标数组 [[x1, y1], [x2, y2], ...]
///
/// 注意：GeoJSON 多边形类别的 geometry.coordinates 为 [外环, [内环1], ...]，
/// 此处取 coordinates[0] 为外环，coordinates[0][0] 为外环的顶点坐标数组。
ErrorType ArenaLoader::ParseMapInfo(common::ObstacleSet *p_obstacle_set) {
  printf("\n[ArenaLoader] Loading map info\n");

  std::fstream fs(map_path_);
  Json root;
  fs >> root;

  Json obstacles_json = root["features"];
  for (int i = 0; i < static_cast<int>(obstacles_json.size()); ++i) {
    Json obs = obstacles_json[i];
    printf("Obstacle id %d.\n", obs["properties"]["id"].get<int>());
    // 检查障碍物是否有效，跳过无效障碍物
    auto is_valid = obs["properties"]["is_valid"].get<int>();
    if (!static_cast<bool>(is_valid)) {
      continue;
    }

    common::PolygonObstacle poly;
    poly.id = obs["properties"]["id"].get<int>();
    poly.type = obs["properties"]["is_spec"].get<int>();  // 0=一般障碍物，非0=特殊类型（如边界）

    // 解 GeoJSON 多边形坐标（外环顶点数组）
    Json coord = obs["geometry"]["coordinates"][0][0];
    int num_pts = static_cast<int>(coord.size());
    for (int k = 0; k < num_pts; ++k) {
      common::Point point(coord[k][0].get<double>(), coord[k][1].get<double>());
      poly.polygon.points.push_back(point);
    }
    // 插入障碍物集合
    p_obstacle_set->obs_polygon.insert(
        std::pair<int, common::PolygonObstacle>(poly.id, poly));
  }
  p_obstacle_set->print();

  fs.close();
  return kSuccess;
}

/// @brief 解析车道网络配置文件（lane_net_norm.json）
///
/// JSON 结构（GeoJSON FeatureCollection 格式）：
///   root["features"]: 车道特征数组
///     每个特征包含：
///       - properties.id: 车道 ID
///       - properties.length: 车道长度
///       - properties.child_id/father_id: 前驱/后继车道 ID（逗号分隔字符串或单个数字）
///       - properties.left_id/right_id: 左右相邻车道 ID
///       - properties.lchg_vld/rchg_vld: 左右变道是否允许
///       - properties.behavior: 车道行为（如 "直行"、"转弯"）
///       - geometry.coordinates[0]: 车道中心线采样点数组 [[x1, y1], [x2, y2], ...]
///
/// child_id 和 father_id 的解析：
///   原始格式为逗号分隔的字符串（如 "2,5,7"），每个逗号分隔的值代表一个相邻车道 ID。
///   值为 0 时表示无效/空连接，被过滤掉。
///   这允许一条车道连接到多条前驱或后继车道（如道路合并、分叉场景）。
ErrorType ArenaLoader::ParseLaneNetInfo(common::LaneNet *p_lane_net) {
  printf("\n[ArenaLoader] Loading lane net info\n");

  std::fstream fs(lane_net_path_);
  Json root;
  fs >> root;

  Json lane_net_json = root["features"];

  for (int i = 0; i < static_cast<int>(lane_net_json.size()); ++i) {
    common::LaneRaw lane_raw;
    Json lane_json = lane_net_json[i];
    Json lane_meta = lane_json["properties"];

    // ---- 基础属性 ----
    lane_raw.id = lane_meta["id"].get<int>();
    lane_raw.length = lane_meta["length"].get<double>();
    lane_raw.dir = 1;  // 车道方向固定为 1（正方向）

    printf("-Lane id %d.\n", lane_raw.id);

    // ---- 解析子车道（前驱）ID 列表 ----
    // child_id 以逗号分隔的字符串形式存储，需拆分并转换为 int 向量
    std::string str_child_id = lane_meta["child_id"].get<std::string>();
    {
      std::vector<std::string> str_vec;
      common::SplitString(str_child_id, ",", &str_vec);
      for (const auto &str : str_vec) {
        auto lane_id = std::stoi(str);
        if (lane_id != 0) lane_raw.child_id.push_back(lane_id);  // 过滤掉 0（无效 ID）
      }
    }

    // ---- 解析父车道（后继）ID 列表 ----
    std::string str_father_id = lane_meta["father_id"].get<std::string>();
    {
      std::vector<std::string> str_vec;
      common::SplitString(str_father_id, ",", &str_vec);
      for (const auto &str : str_vec) {
        auto lane_id = std::stoi(str);
        if (lane_id != 0) lane_raw.father_id.push_back(lane_id);
      }
    }

    // ---- 相邻车道与变道权限 ----
    lane_raw.l_lane_id = lane_meta["left_id"].get<int>();
    lane_raw.r_lane_id = lane_meta["right_id"].get<int>();

    int lchg_vld = lane_meta["lchg_vld"].get<int>();
    int rchg_vld = lane_meta["rchg_vld"].get<int>();
    lane_raw.l_change_avbl = static_cast<bool>(lchg_vld);  // 是否允许向左变道
    lane_raw.r_change_avbl = static_cast<bool>(rchg_vld);  // 是否允许向右变道
    lane_raw.behavior = lane_meta["behavior"].get<std::string>();

    // ---- 解析车道中心线采样点 ----
    Json lane_coordinates = lane_json["geometry"]["coordinates"][0];
    int num_pts = static_cast<int>(lane_coordinates.size());
    for (int k = 0; k < num_pts; ++k) {
      Vec2f pt(lane_coordinates[k][0].get<double>(),
               lane_coordinates[k][1].get<double>());
      lane_raw.lane_points.emplace_back(pt);
    }
    // 记录车道起点和终点（用于快速查询）
    lane_raw.start_point = *(lane_raw.lane_points.begin());
    lane_raw.final_point = *(lane_raw.lane_points.rbegin());

    // 插入车道集合
    p_lane_net->lane_set.insert(
        std::pair<int, common::LaneRaw>(lane_raw.id, lane_raw));
  }
  p_lane_net->print();

  fs.close();
  return kSuccess;
}

}  // namespace phy_simulator
