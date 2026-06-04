/**
 * @file visualizer.cc
 * @author EPSILON Autonomous Driving Group
 * @brief Visualizer 类实现——将语义地图数据发布为 ROS 可视化消息
 *
 * @details
 * 该文件实现了所有可视化功能的发布逻辑：
 *   - 每个智能体发布到独立的 /vis/agent_{id}/ 命名空间
 *   - 使用 visualization_msgs::MarkerArray 表示几何形状
 *   - 使用 nav_msgs::OccupancyGrid 表示障碍物占据栅格地图
 *   - 通过 tf2_ros::TransformBroadcaster 发送坐标变换
 *
 * 可视化元素配色方案（部分）：
 *   - 自车：灰色OBB + 品红色速度向量 + 青色转向指示
 *   - 周车：紫色OBB（普通）/ 红色OBB（故障车 "brokencar"）
 *   - 车道：天蓝色线 + 粉红色起止点
 *   - 关键车辆：提高不透明度(alpha=0.9)
 *   - 限速标志：白色六边形（限速）/ 红色六边形（红灯禁止）
 */

#include "semantic_map_manager/visualizer.h"

namespace semantic_map_manager {

/// @brief 构造函数——初始化ROS发布器和TF广播器
///
/// 为每个智能体创建独立的命名空间话题：
///   /vis/agent_{node_id}/ego_vehicle_vis          自车可视化
///   /vis/agent_{node_id}/obstacle_map              障碍物栅格地图
///   /vis/agent_{node_id}/surrounding_lane_net_vis  局部车道网络
///   /vis/agent_{node_id}/local_lanes_vis           局部拼接车道
///   /vis/agent_{node_id}/ego_behavior_vis          自车行为
///   /vis/agent_{node_id}/pred_initial_intention_vis 意图预测
///   /vis/agent_{node_id}/pred_traj_openloop_vis    开环轨迹
///   /vis/agent_{node_id}/surrounding_vehicle_vis   周车
///   /vis/agent_{node_id}/speed_limit               限速标志
///
/// @param node ROS2节点共享指针
/// @param node_id 智能体ID
Visualizer::Visualizer(rclcpp::Node::SharedPtr node, int node_id)
    : node_(node), node_id_(node_id), ego_to_map_tf_(node) {
  // 构造自车TF框架名称
  ego_tf_name_ = "ego_vehicle_vis_" + std::to_string(node_id_);

  std::cout << "node_id_ = " << node_id_ << std::endl;
  std::cout << "ego_tf_name_ = " << ego_tf_name_ << std::endl;

  // ====== 为所有可视化内容创建ROS话题发布器 ======
  // 每个话题队列深度为1（只保留最新消息）
  std::string ego_vehicle_vis_topic = std::string("/vis/agent_") +
                                      std::to_string(node_id_) +
                                      std::string("/ego_vehicle_vis");
  std::string obstacle_map_vis_topic = std::string("/vis/agent_") +
                                       std::to_string(node_id_) +
                                       std::string("/obstacle_map");
  std::string surrounding_lane_net_vis_topic =
      std::string("/vis/agent_") + std::to_string(node_id_) +
      std::string("/surrounding_lane_net_vis");
  std::string local_lanes_vis_topic = std::string("/vis/agent_") +
                                      std::to_string(node_id_) +
                                      std::string("/local_lanes_vis");
  std::string ego_vehicle_behavior_topic = std::string("/vis/agent_") +
                                           std::to_string(node_id_) +
                                           std::string("/ego_behavior_vis");
  std::string pred_intention_topic = std::string("/vis/agent_") +
                                     std::to_string(node_id_) +
                                     std::string("/pred_initial_intention_vis");
  std::string pred_traj_openloop_topic = std::string("/vis/agent_") +
                                         std::to_string(node_id_) +
                                         std::string("/pred_traj_openloop_vis");
  std::string surrounding_vehicle_topic =
      std::string("/vis/agent_") + std::to_string(node_id_) +
      std::string("/surrounding_vehicle_vis");
  std::string speed_limit_topic = std::string("/vis/agent_") +
                                  std::to_string(node_id_) +
                                  std::string("/speed_limit");

  // 创建各发布器
  ego_vehicle_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(ego_vehicle_vis_topic, 1);
  obstacle_map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(obstacle_map_vis_topic, 1);
  surrounding_lane_net_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(surrounding_lane_net_vis_topic, 1);
  local_lanes_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(local_lanes_vis_topic, 1);
  behavior_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(ego_vehicle_behavior_topic, 1);
  pred_traj_openloop_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(pred_traj_openloop_topic, 1);
  pred_intention_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(pred_intention_topic, 1);
  surrounding_vehicle_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(surrounding_vehicle_topic, 1);
  speed_limit_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(speed_limit_topic, 1);
}

/// @brief 可视化数据（自动从 SemanticMapManager 获取时间戳）
/// @note 若时间戳未设置(< kEPS)，则跳过可视化
void Visualizer::VisualizeData(const SemanticMapManager &smm) {
  if (smm.time_stamp() < kEPS) return;  // 时间戳未设置，跳过
  auto time_stamp = rclcpp::Time(smm.time_stamp());
  VisualizeDataWithStamp(time_stamp, smm);
  SendTfWithStamp(time_stamp, smm);
}

/// @brief 使用显式时间戳可视化所有数据
///
/// 依次调用各子模块的可视化函数：
///   1. 自车
///   2. 障碍物地图
///   3. 周围车道网络
///   4. 局部拼接车道
///   5. 自车行为
///   6. 意图预测
///   7. 开环轨迹预测
///   8. 周围车辆（含关键车辆高亮）
///   9. 限速标志
void Visualizer::VisualizeDataWithStamp(const rclcpp::Time &stamp,
                                        const SemanticMapManager &smm) {
  VisualizeEgoVehicle(stamp, smm.ego_vehicle());
  VisualizeObstacleMap(stamp, smm.obstacle_map());
  VisualizeSurroundingLaneNet(stamp, smm.surrounding_lane_net(),
                              std::vector<int>());
  VisualizeLocalLanes(stamp, smm.local_lanes(), smm, std::vector<int>());
  VisualizeBehavior(stamp, smm.ego_behavior());
  VisualizeIntentionPrediction(stamp, smm.semantic_surrounding_vehicles());
  VisualizeOpenloopTrajPrediction(stamp, smm.openloop_pred_trajs());
  VisualizeSurroundingVehicles(stamp, smm.surrounding_vehicles(),
                               smm.key_vehicle_ids());
  VisualizeSpeedLimit(stamp, smm.RetTrafficInfoSpeedLimit());
}

/// @brief 回放模式可视化——支持过滤已删除的车道ID
/// @param deleted_lane_ids 回放中需要隐藏的车道ID列表
void Visualizer::VisualizeDataWithStampForPlayback(
    const rclcpp::Time &stamp, const SemanticMapManager &smm,
    const std::vector<int> &deleted_lane_ids) {
  VisualizeEgoVehicle(stamp, smm.ego_vehicle());
  VisualizeObstacleMap(stamp, smm.obstacle_map());
  VisualizeSurroundingLaneNet(stamp, smm.surrounding_lane_net(),
                              deleted_lane_ids);
  VisualizeLocalLanes(stamp, smm.local_lanes(), smm, deleted_lane_ids);
  VisualizeBehavior(stamp, smm.ego_behavior());
  VisualizeIntentionPrediction(stamp, smm.semantic_surrounding_vehicles());
  VisualizeOpenloopTrajPrediction(stamp, smm.openloop_pred_trajs());
  VisualizeSurroundingVehicles(stamp, smm.surrounding_vehicles(),
                               smm.key_vehicle_ids());
  VisualizeSpeedLimit(stamp, smm.RetTrafficInfoSpeedLimit());
}

/// @brief 可视化周围车辆
///
/// 视觉区分：
///   - 普通车辆：紫色 OBB（alpha=0.7）
///   - 故障车辆（brokencar）：红色 OBB（alpha=0.2）
///   - 关键车辆（nearby_ids 中存在）：提高不透明度（alpha=0.9）
void Visualizer::VisualizeSurroundingVehicles(
    const rclcpp::Time &stamp, const common::VehicleSet &vehicle_set,
    const std::vector<int> &nearby_ids) {
  visualization_msgs::msg::MarkerArray vehicle_marker_list;
  for (const auto &v : vehicle_set.vehicles) {
    visualization_msgs::msg::MarkerArray vehicle_marker;

    // 默认配色：紫色OBB + 品红色速度向量 + 青色转向指示
    common::ColorARGB color_obb(0.5, 0.2, 0.7, 1.0);
    common::ColorARGB color_vel_vec(0.4, 0.0, 1.0, 1.0);
    common::ColorARGB color_steer(0.4, 1.0, 1.0, 1.0);

    // 故障车特殊配色：红色OBB
    if (v.second.type().compare("brokencar") == 0) {
      color_obb = common::ColorARGB(0.4, 1.0, 0.2, 0.2);
    }
    // 关键车辆高亮：提高alpha
    if (nearby_ids.end() !=
        std::find(nearby_ids.begin(), nearby_ids.end(), v.second.id())) {
      color_obb.a = 0.9;
      color_vel_vec.a = 0.9;
      color_steer.a = 0.9;
    }
    common::VisualizationUtil::GetRosMarkerArrayUsingVehicle(
        v.second, color_obb, color_vel_vec, color_steer, 1, &vehicle_marker);
    for (auto &marker : vehicle_marker.markers)
      vehicle_marker_list.markers.push_back(marker);
  }
  int num_markers = static_cast<int>(vehicle_marker_list.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("map"), last_surrounding_vehicle_marker_cnt_,
      &vehicle_marker_list);
  last_surrounding_vehicle_marker_cnt_ = num_markers;
  surrounding_vehicle_vis_pub_->publish(vehicle_marker_list);
}

/// @brief 可视化自车——灰色OBB + 品红色速度向量 + 青色转向
void Visualizer::VisualizeEgoVehicle(const rclcpp::Time &stamp,
                                     const common::Vehicle &vehicle) {
  visualization_msgs::msg::MarkerArray vehicle_marker;
  common::ColorARGB color_obb(1.0, 0.66, 0.66, 0.66);
  common::ColorARGB color_vel_vec(1.0, 0.0, 1.0, 1.0);
  common::ColorARGB color_steer(1.0, 1.0, 1.0, 1.0);
  common::VisualizationUtil::GetRosMarkerArrayUsingVehicle(
      vehicle, color_obb, color_vel_vec, color_steer, 1, &vehicle_marker);
  common::VisualizationUtil::FillStampInMarkerArray(stamp, &vehicle_marker);
  ego_vehicle_pub_->publish(vehicle_marker);
}

/// @brief 可视化障碍物占据栅格地图（nav_msgs::OccupancyGrid 格式）
void Visualizer::VisualizeObstacleMap(
    const rclcpp::Time &stamp,
    const common::GridMapND<ObstacleMapType, 2> &obstacle_map) {
  if (obstacle_map.data_size() < 1) return;  // 无数据时跳过
  nav_msgs::msg::OccupancyGrid occ_map;
  common::VisualizationUtil::GetRosOccupancyGridUsingGripMap2D(
      obstacle_map, node_->get_clock()->now(), &occ_map);
  occ_map.header.stamp = stamp;
  obstacle_map_pub_->publish(occ_map);
}

/// @brief 发送 map -> ego_vehicle 的 TF 坐标变换
///
/// 使 RViz 中的其他显示可以参照自车坐标框架。
/// 使用静态 TransformBroadcaster 实例发送。
void Visualizer::SendTfWithStamp(const rclcpp::Time &stamp,
                                 const SemanticMapManager &smm) {
  if (smm.time_stamp() < kEPS) {
    return;  // 时间戳未设置，跳过
  }
  // * 发布 TF: map -> ego_vehicle
  Vec3f state = smm.ego_vehicle().Ret3DofState();
  geometry_msgs::msg::Pose pose;
  common::VisualizationUtil::GetRosPoseFrom3DofState(state, &pose);
  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = stamp;
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = ego_tf_name_.c_str();
  transformStamped.transform.translation.x = pose.position.x;
  transformStamped.transform.translation.y = pose.position.y;
  transformStamped.transform.translation.z = pose.position.z;
  transformStamped.transform.rotation = pose.orientation;

  // 使用 TransformBroadcaster 发送变换
  static tf2_ros::TransformBroadcaster br(node_);
  br.sendTransform(transformStamped);
}

/// @brief 可视化局部车道网络
///
/// 为每条车道绘制：
///   - 中心线（天蓝色线带）
///   - 起点标记（粉红色球体）
///   - 终点标记（粉红色球体）
///   - 车道ID文本（红色）
///
/// 支持通过 deleted_lane_ids 过滤已删除的车道
void Visualizer::VisualizeSurroundingLaneNet(
    const rclcpp::Time &stamp, const common::LaneNet &lane_net,
    const std::vector<int> &deleted_lane_ids) {
  visualization_msgs::msg::MarkerArray lane_net_marker;
  int id_cnt = 0;
  for (auto iter = lane_net.lane_set.begin(); iter != lane_net.lane_set.end();
       ++iter) {
    // 跳过已删除的车道
    if (deleted_lane_ids.end() != std::find(deleted_lane_ids.begin(),
                                            deleted_lane_ids.end(),
                                            iter->second.id)) {
      continue;
    }
    visualization_msgs::msg::Marker lane_marker;
    // 天蓝色车道中心线
    common::VisualizationUtil::GetRosMarkerLineStripUsing2DofVec(
        iter->second.lane_points, common::cmap.at("sky blue"),
        Vec3f(0.1, 0.1, 0.1), iter->second.id, &lane_marker);
    lane_marker.header.stamp = stamp;
    lane_marker.header.frame_id = "map";
    lane_marker.id = id_cnt++;
    lane_net_marker.markers.push_back(lane_marker);

    // Visualize the start and end point 起点和终点可视化
    visualization_msgs::msg::Marker start_point_marker, end_point_marker,
        lane_id_text_marker;
    {
      // 车道起点——粉红色球体
      start_point_marker.header.stamp = stamp;
      start_point_marker.header.frame_id = "map";
      Vec2f pt = *(iter->second.lane_points.begin());
      common::VisualizationUtil::GetRosMarkerSphereUsingPoint(
          Vec3f(pt(0), pt(1), 0.0), common::ColorARGB(1.0, 0.2, 0.6, 1.0),
          Vec3f(0.5, 0.5, 0.5), id_cnt++, &start_point_marker);
      // 车道ID文本标签（红色）
      lane_id_text_marker.header.stamp = stamp;
      lane_id_text_marker.header.frame_id = "map";
      common::VisualizationUtil::GetRosMarkerTextUsingPositionAndString(
          Vec3f(pt(0), pt(1), 0.5), std::to_string(iter->second.id),
          common::ColorARGB(1.0, 0.0, 0.0, 1.0), Vec3f(0.6, 0.6, 0.6), id_cnt++,
          &lane_id_text_marker);
    }
    {
      // 车道终点——粉红色球体
      end_point_marker.header.stamp = stamp;
      end_point_marker.header.frame_id = "map";
      Vec2f pt = *(iter->second.lane_points.rbegin());
      common::VisualizationUtil::GetRosMarkerSphereUsingPoint(
          Vec3f(pt(0), pt(1), 0.0), common::ColorARGB(1.0, 0.2, 0.6, 1.0),
          Vec3f(0.5, 0.5, 0.5), id_cnt++, &end_point_marker);
    }

    lane_net_marker.markers.push_back(start_point_marker);
    lane_net_marker.markers.push_back(end_point_marker);
    lane_net_marker.markers.push_back(lane_id_text_marker);
  }
  int num_markers = static_cast<int>(lane_net_marker.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("map"), last_surrounding_lanes_cnt_,
      &lane_net_marker);
  last_surrounding_lanes_cnt_ = num_markers;
  surrounding_lane_net_pub_->publish(lane_net_marker);
}

/// @brief 可视化局部拼接后的长车道（品红色半透明面域）
/// @note 支持通过 deleted_lane_ids 过滤——若局部车道包含任何已删除段车道，则跳过
void Visualizer::VisualizeLocalLanes(
    const rclcpp::Time &stamp,
    const std::unordered_map<int, common::Lane> &local_lanes,
    const SemanticMapManager &smm, const std::vector<int> &deleted_lane_ids) {
  static int last_mks_num = 0;
  visualization_msgs::msg::MarkerArray mks;
  for (const auto &p_lane : local_lanes) {
    // 检查该局部车道是否包含已删除的段车道
    bool is_to_del = false;
    for (const auto &del_id : deleted_lane_ids) {
      if (smm.IsLocalLaneContainsLane(p_lane.first, del_id)) {
        is_to_del = true;
        break;
      }
    }
    if (is_to_del) continue;

    visualization_msgs::msg::Marker mk;
    // 品红色半透明（alpha=0.2）车道面域 + 红色边界线
    common::VisualizationUtil::GetMarkerByLane(
        p_lane.second, 1.0, Vec3f(1.0, 0.0, 0.0),
        common::cmap.at("magenta").set_a(0.2), -0.3, &mk);
    mks.markers.push_back(mk);
  }
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("map"), last_mks_num, &mks);
  last_mks_num = mks.markers.size();
  local_lanes_pub_->publish(mks);
}

/// @brief 可视化自车的语义行为决策
void Visualizer::VisualizeBehavior(const rclcpp::Time &stamp,
                                   const common::SemanticBehavior &behavior) {
  visualization_msgs::msg::MarkerArray behavior_marker_arr;
  common::VisualizationUtil::GetRosMarkerArrUsingSemanticBehavior(
      behavior, &behavior_marker_arr);
  common::VisualizationUtil::FillStampInMarkerArray(stamp,
                                                    &behavior_marker_arr);

  int num_markers = static_cast<int>(behavior_marker_arr.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("map"), last_behavior_marker_cnt_,
      &behavior_marker_arr);
  last_behavior_marker_cnt_ = num_markers;

  behavior_vis_pub_->publish(behavior_marker_arr);
}

/// @brief 可视化限速标志和红绿灯
///
/// 视觉区分：
///   - vel_range(1) < kEPS（速度上限为0）：红灯/禁止通行标志——红色六边形
///   - vel_range(1) > 0：限速标志——白色六边形 + 限速值文本
///
/// 每个标志渲染为：起点六边形标志 + 起点文本 + 终点六边形标志 + "Release"文本
void Visualizer::VisualizeSpeedLimit(
    const rclcpp::Time &stamp, const vec_E<common::SpeedLimit> &speed_limits) {
  visualization_msgs::msg::MarkerArray traffic_signal_arr;
  int id_cnt = 0;
  decimal_t offset_len = 0;  // 标志牌偏移距离

  for (int i = 0; i < static_cast<int>(speed_limits.size()); ++i) {
    std::string str_start = std::string("Speed limit: ");
    std::string str_end = std::string("Release\n");

    // 默认配色：白色标牌
    common::ColorARGB start_marker_color =
        common::ColorARGB(1.0, 1.0, 1.0, 0.0);
    common::ColorARGB end_marker_color = common::ColorARGB(1.0, 0.0, 1.0, 0.0);

    // 若限速上限为0（或接近0），视为红灯/禁止通行
    if (speed_limits[i].vel_range()(1) < kEPS) {
      str_start = std::string("Red light: ");
      str_end = std::string("Forbidden\n");
      // 红色标牌
      start_marker_color = common::ColorARGB(1.0, 1.0, 0.0, 0.0);
      end_marker_color = common::ColorARGB(1.0, 1.0, 0.0, 0.0);
    }

    // ====== 起点标志牌 ======
    visualization_msgs::msg::Marker start_marker;
    start_marker.header.stamp = stamp;
    start_marker.header.frame_id = "map";
    decimal_t start_point_x = speed_limits[i].start_point()(0);
    decimal_t start_point_y = speed_limits[i].start_point()(1);
    decimal_t start_point_angle = speed_limits[i].start_angle();

    decimal_t start_x = start_point_x - offset_len * cos(start_point_angle);
    decimal_t start_y = start_point_y - offset_len * sin(start_point_angle);

    // 六边形标志牌（Mesh）
    common::VisualizationUtil::GetRosMarkerMeshHexagonSignUsingPosition(
        Vec3f(start_x, start_y, start_point_angle), 1.0, start_marker_color,
        ++id_cnt, &start_marker);
    traffic_signal_arr.markers.push_back(start_marker);

    // 起点文本标注（限速值或红灯提示）
    visualization_msgs::msg::Marker start_text_marker;
    start_text_marker.header.stamp = stamp;
    start_text_marker.header.frame_id = "map";
    str_start += std::string(common::GetStringByValueWithPrecision<decimal_t>(
                                 speed_limits[i].vel_range()(1), 1) +
                             "m/s");
    common::VisualizationUtil::GetRosMarkerTextUsingPositionAndString(
        Vec3f(start_x, start_y, 3.5), str_start, common::cmap.at("black"),
        Vec3f(0.75, 0.75, 0.75), ++id_cnt, &start_text_marker);
    traffic_signal_arr.markers.push_back(start_text_marker);

    // ====== 终点标志牌（Release/Forbidden） ======
    visualization_msgs::msg::Marker end_marker;
    end_marker.header.stamp = stamp;
    end_marker.header.frame_id = "map";
    decimal_t end_point_x = speed_limits[i].end_point()(0);
    decimal_t end_point_y = speed_limits[i].end_point()(1);
    decimal_t end_point_angle = speed_limits[i].end_angle();

    decimal_t end_x = end_point_x - offset_len * cos(end_point_angle);
    decimal_t end_y = end_point_y - offset_len * sin(end_point_angle);

    common::VisualizationUtil::GetRosMarkerMeshHexagonSignUsingPosition(
        Vec3f(end_x, end_y, speed_limits[i].end_angle()), 1.0, end_marker_color,
        ++id_cnt, &end_marker);
    traffic_signal_arr.markers.push_back(end_marker);

    // 终点文本标注
    visualization_msgs::msg::Marker end_text_marker;
    end_text_marker.header.stamp = stamp;
    end_text_marker.header.frame_id = "map";
    common::VisualizationUtil::GetRosMarkerTextUsingPositionAndString(
        Vec3f(end_x, end_y, 3.5), str_end, common::cmap.at("black"),
        Vec3f(0.75, 0.75, 0.75), ++id_cnt, &end_text_marker);
    traffic_signal_arr.markers.push_back(end_text_marker);
  }
  int num_markers = static_cast<int>(traffic_signal_arr.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("map"), last_speed_limit_marker_cnt_,
      &traffic_signal_arr);
  last_speed_limit_marker_cnt_ = num_markers;
  speed_limit_vis_pub_->publish(traffic_signal_arr);
}

/// @brief 可视化周车意图预测——用箭头表示横向行为概率
///
/// 为每辆周车的每种有效横向行为绘制箭头：
///   - 车道保持：箭头指向正前方（0度偏移）
///   - 左换道：箭头指向左侧（+90度偏移）
///   - 右换道：箭头指向右侧（-90度偏移）
///   - 箭头长度 = 概率值 * 2.0m
///   - 箭头颜色：黄色
///
/// @note 概率 < kEPS 的行为不显示
void Visualizer::VisualizeIntentionPrediction(
    const rclcpp::Time &stamp,
    const common::SemanticVehicleSet &semantic_vehicles) {
  visualization_msgs::msg::MarkerArray mks;
  int cnt = 0;
  for (const auto &p_sv : semantic_vehicles.semantic_vehicles) {
    auto semantic_vehicle = p_sv.second;
    // * 横向行为预测
    common::State state = semantic_vehicle.vehicle.state();
    geometry_msgs::msg::Point pt0, pt1;
    decimal_t z = 2.0;  // 箭头绘制高度

    for (const auto &entry : semantic_vehicle.probs_lat_behaviors.probs) {
      common::LateralBehavior beh = entry.first;
      decimal_t prob = entry.second;

      if (prob < kEPS) continue;  // 概率过小的行为不显示

      decimal_t angle_offset = 0.0;
      decimal_t length = prob * 2.0;  // 箭头长度正比于概率

      // 根据行为类型设置箭头方向
      if (beh == common::LateralBehavior::kLaneChangeRight) {
        angle_offset = -kPi / 2.0;  // 右换道：-90度
      } else if (beh == common::LateralBehavior::kLaneChangeLeft) {
        angle_offset = +kPi / 2.0;  // 左换道：+90度
      } else if (beh == common::LateralBehavior::kLaneKeeping) {
        angle_offset = 0.0;         // 车道保持：正前方
      }

      pt0.x = state.vec_position(0);
      pt0.y = state.vec_position(1);
      pt0.z = z;

      pt1.x = state.vec_position(0) + cos(state.angle + angle_offset) * length;
      pt1.y = state.vec_position(1) + sin(state.angle + angle_offset) * length;
      pt1.z = z;

      // 创建 ARROW 类型的 Marker
      visualization_msgs::msg::Marker behavior_mk;
      behavior_mk.id = cnt++;
      behavior_mk.type = visualization_msgs::msg::Marker::ARROW;
      behavior_mk.action = visualization_msgs::msg::Marker::MODIFY;
      behavior_mk.points.push_back(pt0);
      behavior_mk.points.push_back(pt1);
      behavior_mk.scale.x = 0.2;  // 箭头轴直径
      behavior_mk.scale.y = 0.5;  // 箭头头部直径
      common::VisualizationUtil::FillColorInMarker(common::cmap.at("yellow"),
                                                   &behavior_mk);
      mks.markers.push_back(behavior_mk);
    }
  }
  int num_mks = static_cast<int>(mks.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("map"), last_intention_marker_cnt_, &mks);
  last_intention_marker_cnt_ = num_mks;
  pred_intention_vis_pub_->publish(mks);
}

/// @brief 可视化开环轨迹预测——点+线表示
///
/// 为每条预测轨迹绘制：
///   - 离散点：天蓝色圆柱体（半径0.5m，高度0.1m）
///   - 连线：天蓝色线带（宽度0.1m）
void Visualizer::VisualizeOpenloopTrajPrediction(
    const rclcpp::Time &stamp,
    const std::unordered_map<int, vec_E<common::State>> &openloop_pred_trajs) {
  visualization_msgs::msg::MarkerArray mks;
  // * 开环轨迹预测
  for (const auto &p_traj : openloop_pred_trajs) {
    std::vector<common::Point> points;
    for (const auto &ps : p_traj.second) {
      common::Point pt(ps.vec_position(0), ps.vec_position(1));
      pt.z = 0.25;
      points.push_back(pt);
      // 每个预测点绘制为一个天蓝色圆柱体
      visualization_msgs::msg::Marker point_marker;
      common::VisualizationUtil::GetRosMarkerCylinderUsingPoint(
          common::Point(pt), Vec3f(0.5, 0.5, 0.1), common::cmap.at("sky blue"),
          0, &point_marker);
      mks.markers.push_back(point_marker);
    }
    // 绘制预测轨迹的连线
    visualization_msgs::msg::Marker line_marker;
    common::VisualizationUtil::GetRosMarkerLineStripUsingPoints(
        points, Vec3f(0.1, 0.1, 0.1), common::cmap.at("sky blue"), 0,
        &line_marker);
    mks.markers.push_back(line_marker);
  }
  int num_mks = static_cast<int>(mks.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("map"), last_traj_list_marker_cnt_, &mks);
  last_traj_list_marker_cnt_ = num_mks;
  pred_traj_openloop_vis_pub_->publish(mks);
}

}  // namespace semantic_map_manager
