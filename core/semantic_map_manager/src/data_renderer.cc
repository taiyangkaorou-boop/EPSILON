/**
 * @file data_renderer.cc
 * @author EPSILON Autonomous Driving Group
 * @brief DataRenderer 类实现——将原始ROS仿真数据渲染为内部数据结构
 *
 * @details
 * 该文件实现了 DataRenderer 的所有核心功能，包括：
 *   - 从 vehicle_set 按 ID 提取自车
 *   - 使用 OpenCV 栅格化障碍物（圆形和多边形）
 *   - 使用 KD 树半径搜索截取周围车道
 *   - 按欧氏距离筛选周围车辆
 *   - Roguelike 光线投射伪感知管线
 *   - 观测噪声注入（模拟真实感知不确定性）
 *   - 最终调用 SemanticMapManager::UpdateSemanticMap 完成数据注入
 */

#include "semantic_map_manager/data_renderer.h"
#include "roguelike_ray_casting/roguelike_ray_casting.h"

#include <algorithm>
#include <iterator>
#include <random>

namespace semantic_map_manager {

/// @brief 构造函数——从 SemanticMapManager 获取初始配置并初始化障碍物栅格地图
DataRenderer::DataRenderer(SemanticMapManager *smm_ptr)
    : p_semantic_map_manager_(smm_ptr) {
  // 从语义地图管理器获取 ego_id
  ego_id_ = p_semantic_map_manager_->ego_id();
  // 获取障碍物地图元信息（分辨率、尺寸）
  obstacle_map_info_ =
      p_semantic_map_manager_->agent_config_info().obstacle_map_meta_info;
  // 获取周围搜索半径
  surrounding_search_radius_ =
      p_semantic_map_manager_->agent_config_info().surrounding_search_radius;

  // 构建二维栅格地图的数据尺寸和分辨率参数
  std::array<int, 2> map_size = {
      {obstacle_map_info_.height, obstacle_map_info_.width}};
  std::array<decimal_t, 2> map_resl = {
      {obstacle_map_info_.resolution, obstacle_map_info_.resolution}};
  std::array<std::string, 2> map_name = {{"height", "width"}};

  // 分配障碍物栅格地图内存
  p_obstacle_grid_ =
      new common::GridMapND<ObstacleMapType, 2>(map_size, map_resl, map_name);

  printf("[DataRenderer] Initialization finished\n");
}

/// @brief 主渲染入口——将ROS仿真数据解包、处理、注入语义地图管理器
///
/// 执行顺序（有严格依赖关系）：
///   1. GetEgoVehicle：提取自车（必须先执行，后续步骤依赖 ego_state_）
///   2. GetObstacleMap：栅格化障碍物
///   3. GetWholeLaneNet：存储完整车道网络
///   4. GetSurroundingLaneNet：截取周围车道（依赖 ego_state_ 位置）
///   5. GetSurroundingVehicles：筛选周围车辆（依赖 ego_state_ 位置）
///   6. [可选] InjectObservationNoise：注入跟踪噪声
///   7. FakeMapper：光线投射伪感知
///   8. UpdateSemanticMap：推送到世界模型
ErrorType DataRenderer::Render(const double &time_stamp,
                               const common::LaneNet &lane_net,
                               const common::VehicleSet &vehicle_set,
                               const common::ObstacleSet &obstacle_set) {
  time_stamp_ = time_stamp;
  GetEgoVehicle(vehicle_set);  // ~ 必须先更新自车，后续障碍物地图和周围搜索都依赖自车位置
  GetObstacleMap(obstacle_set);
  GetWholeLaneNet(lane_net);
  GetSurroundingLaneNet(lane_net);
  GetSurroundingVehicles(vehicle_set);

  // 可选：注入跟踪噪声（模拟真实感知系统的不确定性）
  if (p_semantic_map_manager_->agent_config_info().enable_tracking_noise) {
    InjectObservationNoise();
    // 将受噪声影响的车辆ID通知语义地图管理器
    p_semantic_map_manager_->set_uncertain_vehicle_ids(uncertain_vehicle_ids_);
  }

  TicToc timer;
  // 执行伪感知管线（光线投射+历史缓存维护）
  FakeMapper();
  // printf("[RayCasting]Time cost: %lf ms\n", timer.toc());

  // 将所有渲染好的数据推送到语义地图管理器
  p_semantic_map_manager_->UpdateSemanticMap(
      time_stamp_, ego_vehicle_, whole_lane_net_, surrounding_lane_net_,
      *p_obstacle_grid_, obs_grids_, surrounding_vehicles_);

  return kSuccess;
}

/// @brief 注入观测噪声——模拟真实感知系统对周围车辆的跟踪不确定性
///
/// 注入策略：
///   - 仅对 ego_id_ == 0 的智能体生效（一般为主车）
///   - 每10帧（cnt_random_ == 10）执行一次
///   - 从周围车辆中随机选取最多3辆
///   - 对选取的车辆注入高斯噪声：
///     - 横向位置噪声：std=0.2m（垂直于车辆航向）
///     - 纵向位置噪声：std=0.7m（沿车辆航向）
///     - 角度噪声：std=0.22rad
///   - 角度噪声超过 1.5*std 且非故障车的车辆 -> 标记为 uncertain_vehicle_ids_
///
/// @note 位置噪声分解为横向和纵向分量，分别施加在车辆的侧向和前进方向上
ErrorType DataRenderer::InjectObservationNoise() {
  // * 仅对主车生效
  if (ego_id_ != 0) return kSuccess;
  cnt_random_++;

  // 每10帧执行一次噪声注入
  if (cnt_random_ == 10) {
    uncertain_vehicle_ids_.clear();

    // 噪声分布参数定义
    const decimal_t angle_noise_std = 0.22;
    std::normal_distribution<double> lat_pos_dist(0.0, 0.2);   // 横向位置噪声 N(0, 0.04)
    std::normal_distribution<double> long_pos_dist(0.0, 0.7);   // 纵向位置噪声 N(0, 0.49)
    std::normal_distribution<double> angle_dist(0.0, angle_noise_std); // 角度噪声 N(0, 0.0484)

    // * 收集所有周围车辆的ID
    std::vector<int> surrounding_ids;
    for (const auto &v : surrounding_vehicles_.vehicles) {
      surrounding_ids.push_back(v.first);
    }
    // 随机打乱ID列表
    std::shuffle(surrounding_ids.begin(), surrounding_ids.end(),
                 random_engine_);

    // 随机选取最多3辆车注入噪声
    std::vector<int> sampled_ids;
    for (int i = 0; i < 3 && i < surrounding_ids.size(); i++) {
      sampled_ids.push_back(surrounding_ids[i]);
    }

    // * 对选取的车辆注入噪声
    for (auto &v : surrounding_vehicles_.vehicles) {
      // 跳过未被选中的车辆
      if (std::find(sampled_ids.begin(), sampled_ids.end(), v.first) ==
          sampled_ids.end())
        continue;

      // 生成噪声值
      decimal_t lateral_position_noise = lat_pos_dist(random_engine_);
      decimal_t long_position_noise = long_pos_dist(random_engine_);
      decimal_t angle_noise = angle_dist(random_engine_);

      common::State original_state = v.second.state();
      Vec2f original_position = original_state.vec_position;
      decimal_t angle = original_state.angle;

      // 将噪声变换到世界坐标系：
      //   lateral（横向）= sin(angle)方向，long（纵向）= cos(angle)方向
      Vec2f augmented_position =
          Vec2f(original_position.x() + lateral_position_noise * sin(angle) +
                    long_position_noise * cos(angle),
                original_position.y() - lateral_position_noise * cos(angle) +
                    long_position_noise * sin(angle));
      original_state.vec_position = augmented_position;
      original_state.angle =
          normalize_angle(original_state.angle + angle_noise);

      // 角度噪声过大且非故障车的车辆标记为"不确定"
      if (fabs(angle_noise) > 1.5 * angle_noise_std &&
          v.second.type().compare("brokencar") != 0)
        uncertain_vehicle_ids_.push_back(v.first);

      v.second.set_state(original_state);
    }
    cnt_random_ = 0;
  }
  return kSuccess;
}

/// @brief 从车辆集合中按 ego_id_ 提取自车信息
ErrorType DataRenderer::GetEgoVehicle(const common::VehicleSet &vehicle_set) {
  ego_vehicle_ = vehicle_set.vehicles.at(ego_id_);
  ego_param_ = ego_vehicle_.param();
  ego_state_ = ego_vehicle_.state();
  return kSuccess;
}

/// @brief 将障碍物集合栅格化到 OccupancyGrid 中
///
/// 使用 OpenCV 进行高效的栅格化操作：
///   1. 以自车位置为中心，将栅格地图对齐到全局坐标系的整数位置
///      （提高跨帧一致性——减少因浮点误差导致的栅格抖动）
///   2. 将所有圆形障碍物用 cv::circle 填充（-1 表示实心圆）
///   3. 将所有多边形障碍物用 cv::fillPoly 填充
///   4. 地图初始化为 UNKNOWN(255)，障碍物区域填充为 OCCUPIED(100)
///
/// @note OccupancyGrid 原点在左下角，x 向右，y 向上
ErrorType DataRenderer::GetObstacleMap(
    const common::ObstacleSet &obstacle_set) {
  // ~ 注意：OccupancyGrid 坐标原点在左下角，x右，y上

  // 计算栅格地图左下角世界坐标（以自车为中心）
  decimal_t x = ego_state_.vec_position(0) - obstacle_map_info_.h_metric / 2.0;
  decimal_t y = ego_state_.vec_position(1) - obstacle_map_info_.w_metric / 2.0;
  // 对齐到全局整数坐标以提高一致性（避免帧间栅格漂移）
  decimal_t x_r = std::round(x);
  decimal_t y_r = std::round(y);

  // 初始化地图所有单元为 UNKNOWN（255）
  p_obstacle_grid_->fill_data(GridMap2D::UNKNOWN);
  std::array<decimal_t, 2> origin = {{x_r, y_r}};
  p_obstacle_grid_->set_origin(origin);

  // 创建 OpenCV Mat 视图到栅格地图的内存（零拷贝，直接操作底层数据）
  cv::Mat grid_mat =
      cv::Mat(obstacle_map_info_.height, obstacle_map_info_.width,
              CV_MAKETYPE(cv::DataType<ObstacleMapType>::type, 1),
              p_obstacle_grid_->get_data_ptr());

  // 栅格化所有圆形障碍物
  for (const auto &obs : obstacle_set.obs_circle) {
    std::array<decimal_t, 2> center_w = {
        {obs.second.circle.center.x, obs.second.circle.center.y}};
    auto center_coord = p_obstacle_grid_->GetCoordUsingGlobalPosition(center_w);
    cv::Point2i center(center_coord[0], center_coord[1]);
    int radius = obs.second.circle.radius / obstacle_map_info_.resolution;
    cv::circle(grid_mat, center, radius, cv::Scalar(GridMap2D::OCCUPIED), -1);
  }
  // 栅格化所有多边形障碍物
  std::vector<std::vector<cv::Point>> polys;
  for (const auto &obs : obstacle_set.obs_polygon) {
    std::vector<cv::Point> poly;
    for (const auto &pt : obs.second.polygon.points) {
      std::array<decimal_t, 2> pt_w = {{pt.x, pt.y}};
      auto coord = p_obstacle_grid_->GetCoordUsingGlobalPosition(pt_w);
      cv::Point2i coord_cv(coord[0], coord[1]);
      poly.push_back(coord_cv);
    }
    polys.push_back(poly);
  }
  cv::fillPoly(grid_mat, polys, cv::Scalar(GridMap2D::OCCUPIED));

  return kSuccess;
}

/// @brief 伪感知管线——组合光线投射与历史缓存，维护带记忆的障碍物地图
///
/// 核心逻辑：
///   1. 调用 RayCastingOnObstacleMap 获取当前帧的射线扫描结果
///   2. 遍历历史障碍物栅格 obs_grids_：
///      - 若栅格距自车超过 dist_thres（地图半边长 * 0.8），则清除（过期）
///      - 否则将历史栅格标记为 SCANNED_OCCUPIED
///
/// 这种"带记忆"的设计避免了因瞬时遮挡导致障碍物"消失"的问题，
/// 使地图在时序上更稳定（类似于SLAM中的占据概率保持）
ErrorType DataRenderer::FakeMapper() {
  decimal_t dist_thres = obstacle_map_info_.h_metric / 2.0 * 0.8; // 80% 地图半边长
  // 执行当前帧的光线投射扫描
  RayCastingOnObstacleMap();

  decimal_t ego_pos_x = ego_state_.vec_position(0);
  decimal_t ego_pos_y = ego_state_.vec_position(1);

  // 维护障碍物栅格缓存：清除过期的，保留近处的
  for (auto it = obs_grids_.begin(); it != obs_grids_.end();) {
    decimal_t dx = abs((*it)[0] - ego_pos_x);
    decimal_t dy = abs((*it)[1] - ego_pos_y);
    if (dx >= dist_thres || dy >= dist_thres) {
      // 栅格已远离自车，清除
      it = obs_grids_.erase(it);
      continue;
    } else {
      // 将历史缓存的障碍物标记回地图（SCANNED_OCCUPIED 状态）
      p_obstacle_grid_->SetValueUsingGlobalPosition(
          *it, GridMap2D::SCANNED_OCCUPIED);
      ++it;
    }
  }
  return kSuccess;
}

/// @brief 使用 Roguelike 光线投射算法扫描障碍物地图
///
/// 以自车几何中心为射线起点，对8个FOV扇区进行射线扫描。
/// 发现的障碍物栅格坐标被收集到 obs_grids_ 集合中。
///
/// @note Roguelike 光线投射的 FOV 扇区：共8个扇区（传统 roguelike 游戏的
///        阴影投射算法），每个扇区负责 45 度的扫描范围
ErrorType DataRenderer::RayCastingOnObstacleMap() {
  Vec3f ray_casting_origin;
  // 获取自车的几何中心（而非后轴中心）作为射线原点
  ego_vehicle_.Ret3DofStateAtGeometryCenter(&ray_casting_origin);

  std::array<decimal_t, 2> origin = {
      {ray_casting_origin(0), ray_casting_origin(1)}};
  auto coord = p_obstacle_grid_->GetCoordUsingGlobalPosition(origin);
  // 射线最大半径：地图行数的 3/4
  int r_idx_max = p_obstacle_grid_->dims_size(0) * 3.0 / 4.0;

  std::vector<uint8_t> render_mat(p_obstacle_grid_->data_size(), 0);
  // 对8个FOV扇区分别进行光线投射
  for (int i = 0; i < 8; ++i) {
    std::set<std::array<int, 2>> obs_coord_set;
    roguelike_ray_casting::RasterizeFOVOctant(
        coord[0], coord[1], r_idx_max, p_obstacle_grid_->dims_size(0),
        p_obstacle_grid_->dims_size(1), i, p_obstacle_grid_->data_ptr(),
        render_mat.data(), &obs_coord_set);
    // 将扫描到的障碍物栅格坐标转为世界坐标并加入 obs_grids_
    for (const auto coord : obs_coord_set) {
      std::array<decimal_t, 2> p_w;
      p_obstacle_grid_->GetGlobalPositionUsingCoordinate(coord, &p_w);
      obs_grids_.insert(p_w);
    }
  }
  // 合并原始障碍物地图和光线投射结果（取两者的并集）
  for (int i = 0; i < p_obstacle_grid_->data_size(); ++i) {
    render_mat[i] = std::max(render_mat[i], p_obstacle_grid_->data(i));
  }

  p_obstacle_grid_->set_data(render_mat);

  return kSuccess;
}

/// @brief 存储完整车道网络（直接赋值）
ErrorType DataRenderer::GetWholeLaneNet(const common::LaneNet &lane_net) {
  whole_lane_net_ = lane_net;
  return kSuccess;
}

/// @brief 通过 KD 树半径搜索截取自车周围的车道网络
///
/// 算法流程：
///   1. 将整个 lane_net 的所有车道采样点提取为点云（每个点携带 lane_id 和索引）
///   2. 构建 nanoflann KD 树（叶子节点最大10个元素）
///   3. 以自车位置为中心，以 surrounding_search_radius_ * 2 为半径进行半径搜索
///   4. 收集搜索到的所有唯一车道ID，构建 surrounding_lane_net_
///
/// @note 搜索半径使用 surrounding_search_radius_ * 2（即直径），确保覆盖整个搜索圆
ErrorType DataRenderer::GetSurroundingLaneNet(const common::LaneNet &lane_net) {
  surrounding_lane_net_.clear();
  // TODO(lu.zhang): 建议改为沿车道距离而非欧氏距离

  // 提取所有车道采样点，构建点云
  lane_net_pts_.pts.clear();
  for (auto iter = lane_net.lane_set.begin(); iter != lane_net.lane_set.end();
       ++iter) {
    for (int i = 0; i < static_cast<int>(iter->second.lane_points.size());
         ++i) {
      common::PointWithValue<int> p;
      int id = iter->second.id;
      p.pt.x = iter->second.lane_points[i](0);
      p.pt.y = iter->second.lane_points[i](1);
      p.values.push_back(id);   // 存储车道ID
      p.values.push_back(i);    // 存储采样点序号
      lane_net_pts_.pts.push_back(p);
    }
  }

  // 构建 KD 树（叶子节点最多10个元素）
  kdtree_lane_net_ = std::make_shared<KdTreeFor2dPointVec>(
      2, lane_net_pts_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  kdtree_lane_net_->buildIndex();

  // 以自车位置为查询中心
  const decimal_t query_pt[2] = {ego_vehicle_.state().vec_position(0),
                                 ego_vehicle_.state().vec_position(1)};
  // 搜索半径扩大为直径（*2），然后平方（KD树使用平方距离）
  const decimal_t search_radius =
      surrounding_search_radius_ * surrounding_search_radius_ * 4;
  std::vector<std::pair<size_t, decimal_t>> ret_matches;
  nanoflann::SearchParams params;
  kdtree_lane_net_->radiusSearch(&query_pt[0], search_radius, ret_matches,
                                 params);

  // 收集搜索结果中的所有唯一车道ID
  std::set<size_t> matched_lane_id_set;
  for (const auto &e : ret_matches) {
    matched_lane_id_set.insert(lane_net_pts_.pts[e.first].values[0]);
  }

  // 将匹配到的车道添加到 surrounding_lane_net_
  for (const auto id : matched_lane_id_set) {
    surrounding_lane_net_.lane_set.insert(
        std::pair<int, common::LaneRaw>(id, lane_net.lane_set.at(id)));
  }
  return kSuccess;
}

/// @brief 按欧氏距离筛选自车周围的车辆
/// @note 排除自车本身（v.second.id() == ego_id_），仅保留距离小于 surrounding_search_radius_ 的车辆
ErrorType DataRenderer::GetSurroundingVehicles(
    const common::VehicleSet &vehicle_set) {
  surrounding_vehicles_.vehicles.clear();

  for (const auto &v : vehicle_set.vehicles) {
    if (v.second.id() == ego_id_) continue; // 排除自车
    double dx =
        v.second.state().vec_position(0) - ego_vehicle_.state().vec_position(0);
    double dy =
        v.second.state().vec_position(1) - ego_vehicle_.state().vec_position(1);
    double dist = std::hypot(dx, dy);  // 欧氏距离
    if (dist < surrounding_search_radius_) {
      surrounding_vehicles_.vehicles.insert(v);
    }
  }

  return kSuccess;
}

}  // namespace semantic_map_manager
