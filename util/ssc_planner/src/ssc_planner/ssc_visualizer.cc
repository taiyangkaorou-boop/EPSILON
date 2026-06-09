/**
 * @file ssc_visualizer.cc
 * @brief SSC 规划器 ROS2 可视化器的实现
 *
 * [概述]
 * SscVisualizer 将 SSC 规划器的内部数据转换为 ROS MarkerArray 消息,
 * 在 RViz2 中以三维形式呈现。所有可视化以 "ssc_map" 为参考坐标系,
 * 其中:
 *   - X 轴 = s (沿车道的纵向位置, 米)
 *   - Y 轴 = d (横向偏移, 米)
 *   - Z 轴 = t - start_time_ (相对于规划起始时间的偏移, 秒)
 *
 * [可视化内容及话题]
 *   /vis/agent_{id}/ssc/map_vis            : SSC 三维占据栅格地图
 *   /vis/agent_{id}/ssc/risk_grid_vis      : 概率风险栅格地图
 *   /vis/agent_{id}/ssc/ego_fs_vis         : 自车在 Frenet 空间的位置和轮廓
 *   /vis/agent_{id}/ssc/forward_trajs_vis  : 各行为的前向仿真轨迹
 *   /vis/agent_{id}/ssc/qp_vis             : QP 优化的 Bezier 样条轨迹
 *   /vis/agent_{id}/ssc/sur_vehicle_trajs_vis : 周围车辆的 Frenet 预测轨迹
 *   /vis/agent_{id}/ssc/corridor_vis       : 时空走廊 (立方体 + 种子点)
 *
 * [Marker 管理]
 *   每个发布者维护一个 last_*_mk_cnt 计数器, 每次发布时通过
 *   FillHeaderIdInMarkerArray 从该计数开始递增 Marker ID,
 *   确保新帧不会残留旧帧的 Marker 片段。
 */
#include "ssc_planner/ssc_visualizer.h"

#include <algorithm>

namespace planning {

/// @brief 构造函数: 创建所有 ROS 可视化发布者
///
/// 话题名格式: /vis/agent_{node_id}/ssc/{content}_vis
SscVisualizer::SscVisualizer(rclcpp::Node::SharedPtr node, int node_id)
    : node_(node), node_id_(node_id) {
  std::cout << "node_id_ = " << node_id_ << std::endl;

  // 构造各可视化话题名
  std::string ssc_map_vis_topic = std::string("/vis/agent_") +
                                  std::to_string(node_id_) +
                                  std::string("/ssc/map_vis");
  std::string risk_grid_vis_topic = std::string("/vis/agent_") +
                                    std::to_string(node_id_) +
                                    std::string("/ssc/risk_grid_vis");
  std::string ego_vehicle_vis_topic = std::string("/vis/agent_") +
                                      std::to_string(node_id_) +
                                      std::string("/ssc/ego_fs_vis");
  std::string forward_trajs_vis_topic = std::string("/vis/agent_") +
                                        std::to_string(node_id_) +
                                        std::string("/ssc/forward_trajs_vis");
  std::string sur_vehicle_trajs_vis_topic = std::string("/vis/agent_") +
                                            std::to_string(node_id_) +
                                            std::string("/ssc/sur_vehicle_trajs_vis");
  std::string corridor_vis_topic = std::string("/vis/agent_") +
                                   std::to_string(node_id_) +
                                   std::string("/ssc/corridor_vis");
  std::string qp_vis_topic = std::string("/vis/agent_") +
                             std::to_string(node_id_) +
                             std::string("/ssc/qp_vis");

  // 创建发布者 (latch=false, QoS depth=1)
  ssc_map_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(ssc_map_vis_topic, 1);
  risk_grid_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(risk_grid_vis_topic, 1);
  qp_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(qp_vis_topic, 1);
  ego_vehicle_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(ego_vehicle_vis_topic, 1);
  forward_trajs_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(forward_trajs_vis_topic, 1);
  sur_vehicle_trajs_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(sur_vehicle_trajs_vis_topic, 1);
  corridor_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(corridor_vis_topic, 1);
}

/// @brief 主入口: 按顺序发布所有可视化数据
void SscVisualizer::VisualizeDataWithStamp(const rclcpp::Time &stamp,
                                           const SscPlanner &planner) {
  start_time_ = planner.time_origin();  // 记录时间原点用于计算相对时间

  VisualizeSscMap(stamp, planner.p_ssc_map());
  VisualizeRiskGridInSscSpace(stamp, planner.p_ssc_map());
  VisualizeEgoVehicleInSscSpace(stamp, planner.fs_ego_vehicle());
  VisualizeForwardTrajectoriesInSscSpace(stamp, planner.forward_trajs_fs(),
                                         planner.p_ssc_map());
  VisualizeSurroundingVehicleTrajInSscSpace(
      stamp, planner.surround_forward_trajs_fs(), planner.p_ssc_map());
  VisualizeCorridorsInSscSpace(
      stamp, planner.p_ssc_map()->driving_corridor_vec(), planner.p_ssc_map());
  VisualizeQpTrajs(stamp, planner.qp_trajs());
}

/// @brief 可视化 QP 优化的 Bezier 样条轨迹
///
/// 将每条 5阶2维 Bezier 样条在 (s, d, t) 空间中绘制为 LINE_STRIP:
///   - X = s(t) = bezier_spline.evaluate(t, 0)[0]
///   - Y = d(t) = bezier_spline.evaluate(t, 0)[1]
///   - Z = t - start_time_ (相对时间)
///
/// 采样步长: 0.02s
/// 颜色: 品红色 (magenta)
void SscVisualizer::VisualizeQpTrajs(
    const rclcpp::Time &stamp, const vec_E<common::BezierSpline<5, 2>> &trajs) {
  if (trajs.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[SscQP]No valid qp trajs.");
    return;
  }

  int id = 0;
  visualization_msgs::msg::MarkerArray traj_mk_arr;

  for (int i = 0; i < static_cast<int>(trajs.size()); i++) {
    visualization_msgs::msg::Marker traj_mk;
    traj_mk.type = visualization_msgs::msg::Marker::LINE_STRIP;  // 线带
    traj_mk.action = visualization_msgs::msg::Marker::MODIFY;
    traj_mk.id = id++;
    Vecf<2> pos;

    // 按 0.02s 步长采样 Bezier 样条
    for (decimal_t t = trajs[i].begin(); t < trajs[i].end() + kEPS; t += 0.02) {
      if (trajs[i].evaluate(t, 0, &pos) == kSuccess) {
        geometry_msgs::msg::Point pt;
        pt.x = pos[0];             // s 坐标
        pt.y = pos[1];             // d 坐标
        pt.z = t - start_time_;    // 相对时间 → Z 轴
        traj_mk.points.push_back(pt);
      }
    }

    common::VisualizationUtil::FillScaleColorInMarker(
        Vec3f(0.2, 0.2, 0.2), common::cmap.at("magenta"), &traj_mk);
    traj_mk_arr.markers.push_back(traj_mk);
  }

  int num_markers = static_cast<int>(traj_mk_arr.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("ssc_map"), last_qp_traj_mk_cnt, &traj_mk_arr);
  qp_pub_->publish(traj_mk_arr);
  last_qp_traj_mk_cnt = num_markers;
}

/// @brief 可视化 SSC 三维占据栅格地图
///
/// 发布两个 Marker:
///   1. 障碍物占据的体素栅格 (通过 GetRosMarkerCubeListUsingGripMap3D)
///   2. 参考车道区域边界框 (AxisAlignedBoundingBox):
///      s 方向范围 = [origin[0] + s_back_len, origin[0] + s_len - s_back_len]
///      d 方向范围 = [-1.75, 1.75] (标准车道半宽)
///      颜色: 半透明蓝色
void SscVisualizer::VisualizeSscMap(const rclcpp::Time &stamp,
                                    const SscMap *p_ssc_map) {
  visualization_msgs::msg::MarkerArray map_marker_arr;
  visualization_msgs::msg::Marker map_marker;

  // 体素栅格: 将 3D 占据栅格转换为立方体列表
  common::VisualizationUtil::GetRosMarkerCubeListUsingGripMap3D(
      p_ssc_map->p_3d_grid(), stamp, "ssc_map", Vec3f(0, 0, 0), &map_marker);

  // 参考车道区域边界框
  auto origin = p_ssc_map->p_3d_grid()->origin();
  decimal_t s_len =
      p_ssc_map->config().map_resolution[0] * p_ssc_map->config().map_size[0];
  decimal_t x = s_len / 2 - p_ssc_map->config().s_back_len + origin[0];
  decimal_t y = 0;  // d 方向居中
  std::array<decimal_t, 3> aabb_coord = {x, y, 0};
  std::array<decimal_t, 3> aabb_len = {s_len, 3.5, 0.01};  // 3.5m 标准车道全宽
  common::AxisAlignedBoundingBoxND<3> map_aabb(aabb_coord, aabb_len);
  visualization_msgs::msg::Marker map_aabb_marker;
  map_aabb_marker.header.frame_id = "ssc_map";
  map_aabb_marker.header.stamp = stamp;
  map_aabb_marker.id = 1;
  // 半透明蓝色
  common::VisualizationUtil::GetRosMarkerCubeUsingAxisAlignedBoundingBox3D(
      map_aabb, common::ColorARGB(0.2, 1, 0, 1), &map_aabb_marker);

  map_marker_arr.markers.push_back(map_marker);
  map_marker_arr.markers.push_back(map_aabb_marker);
  ssc_map_pub_->publish(map_marker_arr);
}

/// @brief 可视化概率风险栅格地图
///
/// risk grid 当前是 SSC map 的并行调试侧通道，本函数只读 risk_grid()
/// 并发布 RViz Marker，不回写二值占据图、不影响 corridor/QP/control。
/// 坐标语义保持与 SSC map 一致：
///   - X = s 栅格中心
///   - Y = d 栅格中心
///   - Z = t - start_time_，仅用于时空调试图中的相对时间高度
void SscVisualizer::VisualizeRiskGridInSscSpace(const rclcpp::Time &stamp,
                                                const SscMap *p_ssc_map) {
  visualization_msgs::msg::MarkerArray risk_marker_arr;
  visualization_msgs::msg::Marker risk_marker;
  risk_marker.ns = "risk_grid";
  risk_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  risk_marker.action = visualization_msgs::msg::Marker::MODIFY;
  risk_marker.pose.orientation.w = 1.0;

  if (p_ssc_map == nullptr || p_ssc_map->p_3d_grid() == nullptr) {
    // 空地图时仍发布空 marker，用相同 id 覆盖上一帧内容，避免 RViz 残留。
    risk_marker_arr.markers.push_back(risk_marker);
    common::VisualizationUtil::FillHeaderIdInMarkerArray(
        stamp, std::string("ssc_map"), last_risk_grid_mk_cnt, &risk_marker_arr);
    risk_grid_pub_->publish(risk_marker_arr);
    last_risk_grid_mk_cnt = 1;
    return;
  }

  const auto *p_grid = p_ssc_map->p_3d_grid();
  const auto &risk_grid = p_ssc_map->risk_grid();
  const auto &dims_size = p_grid->dims_size();
  const auto &dims_resolution = p_grid->dims_resolution();

  // MVP-1C 只做调试可视化：阈值取极小正数，确保 MVP-0/1A 中固定 1.0 的风险能显示。
  constexpr float kRiskGridVisualizationThreshold = 1.0e-6f;
  // 限制时间层和总体点数，避免默认 1000*100*81 体素全量发布拖慢 RViz/DDS。
  constexpr int kMaxRiskGridVisualizationLayers = 20;
  constexpr int kMaxRiskGridVisualizationCells = 5000;

  const int s_dim = dims_size[0];
  const int d_dim = dims_size[1];
  const int t_dim = dims_size[2];
  const int layer_cell_num = s_dim * d_dim;
  const int visualized_t_dim = std::min(t_dim, kMaxRiskGridVisualizationLayers);

  risk_marker.scale.x = dims_resolution[0];
  risk_marker.scale.y = dims_resolution[1];
  risk_marker.scale.z = dims_resolution[2];
  risk_marker.points.reserve(kMaxRiskGridVisualizationCells);
  risk_marker.colors.reserve(kMaxRiskGridVisualizationCells);

  int visualized_cells = 0;
  for (int t_idx = 0; t_idx < visualized_t_dim; ++t_idx) {
    for (int d_idx = 0; d_idx < d_dim; ++d_idx) {
      for (int s_idx = 0; s_idx < s_dim; ++s_idx) {
        const int risk_idx = t_idx * layer_cell_num + d_idx * s_dim + s_idx;
        if (risk_idx >= static_cast<int>(risk_grid.size())) break;

        const float risk = risk_grid[risk_idx];
        if (risk <= kRiskGridVisualizationThreshold) continue;

        // 风险栅格沿用 SscMap 的一维布局：idx = t*s_dim*d_dim + d*s_dim + s。
        std::array<int, 3> coord = {s_idx, d_idx, t_idx};
        std::array<decimal_t, 3> p_w;
        p_grid->GetGlobalPositionUsingCoordinate(coord, &p_w);

        geometry_msgs::msg::Point pt;
        pt.x = p_w[0];
        pt.y = p_w[1];
        pt.z = p_w[2] - start_time_;
        risk_marker.points.push_back(pt);

        const float clamped_risk = std::min(std::max(risk, 0.0f), 1.0f);
        const float time_ratio = visualized_t_dim > 1
                                     ? static_cast<float>(t_idx) /
                                           static_cast<float>(visualized_t_dim - 1)
                                     : 0.0f;
        std_msgs::msg::ColorRGBA color;
        color.r = 1.0f;
        color.g = 1.0f - clamped_risk;
        color.b = 0.05f * (1.0f - clamped_risk);
        // 风险越高越不透明，时间层越远越透明，便于观察近时域高风险。
        color.a = 0.15f + 0.70f * clamped_risk * (1.0f - 0.50f * time_ratio);
        risk_marker.colors.push_back(color);

        ++visualized_cells;
        if (visualized_cells >= kMaxRiskGridVisualizationCells) break;
      }
      if (visualized_cells >= kMaxRiskGridVisualizationCells) break;
    }
    if (visualized_cells >= kMaxRiskGridVisualizationCells) break;
  }

  risk_marker_arr.markers.push_back(risk_marker);
  const int num_markers = static_cast<int>(risk_marker_arr.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("ssc_map"), last_risk_grid_mk_cnt, &risk_marker_arr);
  risk_grid_pub_->publish(risk_marker_arr);
  last_risk_grid_mk_cnt = num_markers;
}

/// @brief 可视化自车在 SSC 空间中的位置
///
/// 发布两个 Marker:
///   1. 自车轮廓多边形 (LINE_STRIP, 红色半透明)
///   2. 自车 Frenet 状态位置球体 (SPHERE, 红色)
///
/// Z 轴偏移: 使用 dt = fs_ego_vehicle.frenet_state.time_stamp - start_time_
void SscVisualizer::VisualizeEgoVehicleInSscSpace(
    const rclcpp::Time &stamp, const common::FsVehicle &fs_ego_vehicle) {
  if (fs_ego_vehicle.vertices.empty()) return;
  visualization_msgs::msg::MarkerArray ego_vehicle_mks;

  // 自车轮廓多边形 (LINE_STRIP)
  visualization_msgs::msg::Marker ego_contour_marker;
  common::ColorARGB color(0.8, 1.0, 0.0, 0.0);  // 红色, 80%不透明
  decimal_t dt = fs_ego_vehicle.frenet_state.time_stamp - start_time_;
  vec_E<Vec2f> contour = fs_ego_vehicle.vertices;
  contour.push_back(contour.front());  // 闭合多边形
  common::VisualizationUtil::GetRosMarkerLineStripUsing2DofVecWithOffsetZ(
      contour, color, Vec3f(0.3, 0.3, 0.3), dt, 0, &ego_contour_marker);
  ego_contour_marker.header.frame_id = "ssc_map";
  ego_contour_marker.header.stamp = stamp;
  ego_vehicle_mks.markers.push_back(ego_contour_marker);

  // 自车状态位置球体
  visualization_msgs::msg::Marker ego_fs_mk;
  common::VisualizationUtil::GetRosMarkerSphereUsingPoint(
      Vec3f(fs_ego_vehicle.frenet_state.vec_s[0],   // s 位置
            fs_ego_vehicle.frenet_state.vec_dt[0],  // d 位置
            dt),                                     // 相对时间
      common::cmap.at("red"), Vec3f(0.3, 0.3, 0.3), 1, &ego_fs_mk);
  ego_fs_mk.header.frame_id = "ssc_map";
  ego_fs_mk.header.stamp = stamp;
  ego_vehicle_mks.markers.push_back(ego_fs_mk);

  ego_vehicle_pub_->publish(ego_vehicle_mks);
}

/// @brief 可视化各行为的前向仿真轨迹
///
/// 每条轨迹的每一帧都绘制:
///   - 车辆轮廓多边形 (LINE_STRIP, 按帧序号渐变着色)
///   - Frenet 状态位置球体 (SPHERE, 青色)
///
/// 颜色使用 Jet colormap: 从蓝色(起点)渐变到红色(终点)
/// 过滤: 跳过 s <= 0 的帧 (无效数据)
void SscVisualizer::VisualizeForwardTrajectoriesInSscSpace(
    const rclcpp::Time &stamp, const vec_E<vec_E<common::FsVehicle>> &trajs,
    const SscMap *p_ssc_map) {
  if (trajs.empty()) return;
  visualization_msgs::msg::MarkerArray trajs_markers;
  int id_cnt = 0;

  for (int i = 0; i < static_cast<int>(trajs.size()); ++i) {
    if (trajs[i].empty()) continue;

    for (int k = 0; k < static_cast<int>(trajs[i].size()); ++k) {
      visualization_msgs::msg::Marker vehicle_marker;
      vec_E<Vec2f> contour = trajs[i][k].vertices;

      // 有效性过滤: 所有顶点的 s 坐标必须 > 0
      bool is_valid = true;
      for (const auto &v : contour) {
        if (v(0) <= 0) {
          is_valid = false;
          break;
        }
      }
      if (!is_valid) continue;

      // Jet 渐变着色 (蓝→青→绿→黄→红)
      common::ColorARGB color = common::GetJetColorByValue(
          static_cast<decimal_t>(k),
          static_cast<decimal_t>(trajs[i].size() - 1), 0.0);
      contour.push_back(contour.front());  // 闭合
      decimal_t dt = trajs[i][k].frenet_state.time_stamp - start_time_;

      // 车辆轮廓
      common::VisualizationUtil::GetRosMarkerLineStripUsing2DofVecWithOffsetZ(
          contour, color, Vec3f(0.1, 0.1, 0.1), dt, id_cnt++, &vehicle_marker);
      trajs_markers.markers.push_back(vehicle_marker);

      // Frenet 状态位置球体 (青色)
      visualization_msgs::msg::Marker fs_mk;
      auto fs = trajs[i][k].frenet_state;
      decimal_t x = fs.vec_s[0];
      decimal_t y = fs.vec_dt[0];
      common::VisualizationUtil::GetRosMarkerSphereUsingPoint(
          Vec3f(x, y, dt), common::cmap.at("cyan"), Vec3f(0.2, 0.2, 0.2),
          id_cnt++, &fs_mk);
      trajs_markers.markers.push_back(fs_mk);
    }
  }

  int num_markers = static_cast<int>(trajs_markers.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("ssc_map"), last_forward_traj_mk_cnt, &trajs_markers);
  forward_trajs_pub_->publish(trajs_markers);
  last_forward_traj_mk_cnt = num_markers;
}

/// @brief 可视化周围车辆在 SSC 空间中的预测轨迹
///
/// 仅显示第一组行为下的周围车辆轨迹 (trajs_set.front()),
/// 每帧绘制车辆轮廓多边形 (LINE_STRIP, Jet 渐变着色)。
/// 过滤: 跳过无效帧 (s <= 0)
void SscVisualizer::VisualizeSurroundingVehicleTrajInSscSpace(
    const rclcpp::Time &stamp,
    const vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>> &trajs_set,
    const SscMap *p_ssc_map) {
  visualization_msgs::msg::MarkerArray trajs_markers;
  if (!trajs_set.empty()) {
    auto trajs = trajs_set.front();  // 仅显示第一组行为
    if (!trajs.empty()) {
      int id_cnt = 0;
      // 遍历每辆周围车辆
      for (auto it = trajs.begin(); it != trajs.end(); ++it) {
        if (it->second.empty()) continue;
        // 遍历该车的每一帧
        for (int k = 0; k < static_cast<int>(it->second.size()); ++k) {
          visualization_msgs::msg::Marker vehicle_marker;
          common::ColorARGB color = common::GetJetColorByValue(
              static_cast<decimal_t>(k),
              static_cast<decimal_t>(it->second.size() - 1), 0.0);
          vec_E<Vec2f> contour = it->second[k].vertices;

          // 有效帧过滤
          bool is_valid = true;
          for (const auto &v : contour) {
            if (v(0) <= 0) {
              is_valid = false;
              break;
            }
          }
          if (!is_valid) continue;

          contour.push_back(contour.front());
          decimal_t dt = it->second[k].frenet_state.time_stamp - start_time_;
          common::VisualizationUtil::
              GetRosMarkerLineStripUsing2DofVecWithOffsetZ(
                  contour, color, Vec3f(0.1, 0.1, 0.1), dt, id_cnt++,
                  &vehicle_marker);
          vehicle_marker.header.frame_id = "ssc_map";
          vehicle_marker.header.stamp = stamp;
          trajs_markers.markers.push_back(vehicle_marker);
        }
      }
    }
  }

  int num_markers = static_cast<int>(trajs_markers.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("ssc_map"), last_sur_vehicle_traj_mk_cnt,
      &trajs_markers);
  sur_vehicle_trajs_pub_->publish(trajs_markers);
  last_sur_vehicle_traj_mk_cnt = num_markers;
}

/// @brief 可视化时空走廊
///
/// 对每个走廊的每个立方体:
///   1. 种子点可视化: 将栅格坐标转为物理坐标, 绘制为球体 (SPHERE, Jet渐变)
///   2. 立方体可视化: 将栅格坐标转为物理坐标, 绘制为半透明立方体 (CUBE)
///      s 对应 X 轴, d 对应 Y 轴, t-start_time_ 对应 Z 轴
void SscVisualizer::VisualizeCorridorsInSscSpace(
    const rclcpp::Time &stamp, const vec_E<common::DrivingCorridor> corridor_vec,
    const SscMap *p_ssc_map) {
  if (corridor_vec.empty()) return;
  visualization_msgs::msg::MarkerArray corridor_vec_marker;
  int id_cnt = 0;

  for (const auto &corridor : corridor_vec) {
    int cube_cnt = 0;
    for (const auto &driving_cube : corridor.cubes) {
      // Jet 渐变着色 (基于立方体序号)
      common::ColorARGB color = common::GetJetColorByValue(
          cube_cnt, corridor.cubes.size() - 1, -kEPS);

      // 种子点球体
      for (const auto &seed : driving_cube.seeds) {
        visualization_msgs::msg::Marker seed_marker;
        decimal_t s_x, s_y, s_z;
        // 从栅格索引转为物理坐标
        p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(seed(0), 0, &s_x);
        p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(seed(1), 1, &s_y);
        p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(seed(2), 2, &s_z);
        common::VisualizationUtil::GetRosMarkerSphereUsingPoint(
            Vec3f(s_x, s_y, s_z - start_time_), color, Vec3f(0.3, 0.3, 0.3),
            id_cnt++, &seed_marker);
        corridor_vec_marker.markers.push_back(seed_marker);
      }

      // 立方体: 半透明绿色
      decimal_t x_max, x_min;
      decimal_t y_max, y_min;
      decimal_t z_max, z_min;
      p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(
          driving_cube.cube.upper_bound[0], 0, &x_max);
      p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(
          driving_cube.cube.lower_bound[0], 0, &x_min);
      p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(
          driving_cube.cube.upper_bound[1], 1, &y_max);
      p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(
          driving_cube.cube.lower_bound[1], 1, &y_min);
      p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(
          driving_cube.cube.upper_bound[2], 2, &z_max);
      p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(
          driving_cube.cube.lower_bound[2], 2, &z_min);

      // 计算立方体中心和尺寸
      decimal_t dx = x_max - x_min;
      decimal_t dy = y_max - y_min;
      decimal_t dz = z_max - z_min;
      decimal_t x = x_min + dx / 2.0;
      decimal_t y = y_min + dy / 2.0;
      decimal_t z = z_min - start_time_ + dz / 2.0;

      std::array<decimal_t, 3> aabb_coord = {x, y, z};
      std::array<decimal_t, 3> aabb_len = {dx, dy, dz};
      common::AxisAlignedBoundingBoxND<3> map_aabb(aabb_coord, aabb_len);
      visualization_msgs::msg::Marker map_aabb_marker;
      map_aabb_marker.id = id_cnt++;
      // 半透明绿色: ARGB(0.15, 0.3, 1.0, 0.3)
      common::VisualizationUtil::GetRosMarkerCubeUsingAxisAlignedBoundingBox3D(
          map_aabb, common::ColorARGB(0.15, 0.3, 1.0, 0.3), &map_aabb_marker);
      corridor_vec_marker.markers.push_back(map_aabb_marker);
      ++cube_cnt;
    }
  }

  int num_markers = static_cast<int>(corridor_vec_marker.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      node_->get_clock()->now(), std::string("ssc_map"), last_corridor_mk_cnt,
      &corridor_vec_marker);
  corridor_pub_->publish(corridor_vec_marker);
  last_corridor_mk_cnt = num_markers;
}

}  // namespace planning
