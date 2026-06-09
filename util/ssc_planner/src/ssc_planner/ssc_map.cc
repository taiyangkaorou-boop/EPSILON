/**
 * @file ssc_map.cc
 * @author HKUST Aerial Robotics Group
 * @brief SSC 三维时空占据栅格地图的实现
 *
 * [概述]
 * 本文件实现了 SscMap 的全部功能，主要包括：
 *
 * 1. 地图初始化与重置:
 *    - 构造函数分配 (s, d, t) 三维栅格内存
 *    - ResetSscMap 清空旧数据并更新原点
 *
 * 2. 障碍物填充:
 *    - FillStaticPart:  将静态障碍物栅格填充到所有时间层 (s-d 固定, 所有 t)
 *    - FillDynamicPart: 将周围车辆预测轨迹逐帧填充到对应时间层
 *    - FillMapWithFsVehicleTraj: 使用 OpenCV fillPoly 实现精确的车辆形状填充
 *
 * 3. 时空走廊构建 (核心算法):
 *    - ConstructCorridorUsingInitialTrajectory:
 *      Stage I  - 种子点采样: 沿前向轨迹的每一帧 Frenet 状态采集种子坐标
 *      Stage II - 立方体膨胀: 从相邻种子对生成初始轴对齐立方体,
 *                 六方向膨胀直到接触障碍物或运动学边界
 *
 * 4. 立方体膨胀与碰撞检测:
 *    - InflateCubeIn3dGrid: 膨胀主循环, 交替在 s+, s-, d+, d- 方向膨胀,
 *      运动学约束限制膨胀范围
 *    - CheckIfCubeIsFree:   遍历立方体内所有栅格, 确认全部为空闲
 *    - CheckIfPlaneIsFreeOn*: 检查固定坐标平面的占用状态
 *
 * 5. 走廊后处理:
 *    - CorridorRelaxation:   在相邻立方体连接处扩大 s/d 方向的连接空间
 *    - GetFinalGlobalMetricCubesList: 从栅格索引转换为物理单位 (米, 秒)
 *
 * [种子膨胀算法详解]
 *   从参考轨迹采样 N 个种子点, 对每对相邻种子 (seed_i, seed_{i+1}):
 *     a. 生成初始立方体: bounds = [min(s_i,s_{i+1}), max(s_i,s_{i+1})] x ... x ...
 *     b. 碰撞检测: 初始立方体必须完全无障碍
 *     c. 六方向迭代膨胀:
 *        - s+ (纵向正): 每次尝试 20 个栅格, 直到碰到障碍物或地图边界或 s 运动学上界
 *        - s- (纵向负): 每次尝试  5 个栅格, 直到碰到障碍物或地图边界或 s 运动学下界
 *        - d+ (横向正): 每次尝试 10 个栅格, 直到碰到障碍物或地图边界
 *        - d- (横向负): 每次尝试 10 个栅格, 直到碰到障碍物或地图边界
 *        - t+ (时间正): 每次尝试  1 个栅格, 最多膨胀 kMaxNumOfGridAlongTime 个
 *        - t- (时间负): 默认禁用 (不倒流)
 *     d. 若新种子仍在当前立方体内则归入当前立方体, 否则另起新立方体
 *       并裁剪当前立方体的时间上界
 *
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */

#include "ssc_planner/ssc_map.h"

#include <fstream>  // 用于 MVP-1B RiskGridStats CSV 文件追加写入

#include <glog/logging.h>

namespace planning {

/// @brief 构造函数：创建两张 3D 栅格地图 (原始 + 膨胀)
SscMap::SscMap(const SscMap::Config &config) : config_(config) {
  config_.Print();

  // 原始障碍物占据栅格 (s, d, t 三维)
  p_3d_grid_ = new common::GridMapND<SscMapDataType, 3>(
      config_.map_size, config_.map_resolution, config_.axis_name);
  // 膨胀后的障碍物占据栅格 (用于更保守的安全检查)
  p_3d_inflated_grid_ = new common::GridMapND<SscMapDataType, 3>(
      config_.map_size, config_.map_resolution, config_.axis_name);

  /// @brief 初始化风险占据栅格图 —— 大小与原始 binary map 一致，初始值为 0.0f（无风险）
  /// @note 风险图容量 = s_dim * d_dim * t_dim，与 GridMapND 的平铺布局一致
  int total_cells = config_.map_size[0] * config_.map_size[1] * config_.map_size[2];
  p_3d_risk_grid_.resize(total_cells, 0.0f);
}

/// @brief 重置 SSC 地图: 清空走廊数据, 清空栅格数据, 更新地图原点
/// @param ini_fs 当前帧的初始 Frenet 状态, 用于重新计算地图原点
ErrorType SscMap::ResetSscMap(const common::FrenetState &ini_fs) {
  ClearDrivingCorridor();
  ClearGridMap();

  /// @brief 同步清空风险占据图 —— 与 binary map 保持生命周期一致
  /// @note 每次规划循环开始时，风险图需要与 binary map 同时清零，
  ///       避免上一帧的风险数据污染当前帧
  ResetRiskMap();

  start_time_ = ini_fs.time_stamp;
  UpdateMapOrigin(ini_fs);

  return kSuccess;
}

/// @brief 更新 3D 栅格地图的原点
///
/// 坐标系映射:
///   原点[0] (s):  = ori_fs.vec_s[0] - s_back_len
///                即从当前 s 位置向后预留 s_back_len 的距离
///   原点[1] (d):  = -(map_size[1] - 1) * resolution[1] / 2
///                横向居中: 地图中心对齐 d=0 (车道中心线)
///   原点[2] (t):  = ori_fs.time_stamp
///                时间原点对齐当前时刻
void SscMap::UpdateMapOrigin(const common::FrenetState &ori_fs) {
  initial_fs_ = ori_fs;

  std::array<decimal_t, 3> map_origin;
  map_origin[0] = ori_fs.vec_s[0] - config_.s_back_len;
  map_origin[1] =
      -1 * (config_.map_size[1] - 1) * config_.map_resolution[1] / 2.0;  // d 居中
  map_origin[2] = ori_fs.time_stamp;                                     // t 对齐当前

  p_3d_grid_->set_origin(map_origin);
  p_3d_inflated_grid_->set_origin(map_origin);
}

/// @brief 根据两个种子点生成初始轴对齐立方体
/// 初始立方体在 (s, d, t) 三个维度的边界分别由种子的 min/max 确定
ErrorType SscMap::GetInitialCubeUsingSeed(
    const Vec3i &seed_0, const Vec3i &seed_1,
    common::AxisAlignedCubeNd<int, 3> *cube) const {
  std::array<int, 3> lb;
  std::array<int, 3> ub;
  // 取种子在各维度的最小值和最大值作为初始边界
  lb[0] = std::min(seed_0(0), seed_1(0));  // s_lower
  lb[1] = std::min(seed_0(1), seed_1(1));  // d_lower
  lb[2] = std::min(seed_0(2), seed_1(2));  // t_lower
  ub[0] = std::max(seed_0(0), seed_1(0));  // s_upper
  ub[1] = std::max(seed_0(1), seed_1(1));  // d_upper
  ub[2] = std::max(seed_0(2), seed_1(2));  // t_upper

  *cube = common::AxisAlignedCubeNd<int, 3>(ub, lb);
  return kSuccess;
}

/// @brief 构建 SSC 时空占据地图: 填充静态 + 动态障碍物
ErrorType SscMap::ConstructSscMap(
    const std::unordered_map<int, vec_E<common::FsVehicle>>
        &sur_vehicle_trajs_fs,
    const vec_E<Vec2f> &obstacle_grids,
    const std::unordered_map<int, decimal_t> &traj_probs) {
  // 清空两张栅格地图
  p_3d_grid_->clear_data();
  p_3d_inflated_grid_->clear_data();
  ResetRiskMap();

  // 第1步: 填充静态障碍物 (对所有时间层)
  FillStaticPart(obstacle_grids);
  // 第2步: 填充动态障碍物 (按时间层)
  FillDynamicPart(sur_vehicle_trajs_fs);

  /// @brief 第3步（新增）: 概率化填充动态障碍物到风险占据图
  /// @note 此步骤在原始 binary 填充之后执行，不影响二进制占据图。
  ///       MVP-2 中 existence_prob 来自周车 argmax 横向行为概率。
  FillDynamicPartProbabilistic(sur_vehicle_trajs_fs, traj_probs);

  /// @brief 第4步（新增 MVP-1A）: 计算并输出 risk grid 统计日志
  /// @note 仅读取 p_3d_risk_grid_，不修改任何地图数据，不影响规划决策
  const auto risk_stats = ComputeRiskGridStats();
  PrintRiskGridStatsIfNeeded(risk_stats);
  AppendRiskGridStatsToCsv(risk_stats);

  return kSuccess;
}

/// @brief 获取膨胀方向配置
///
/// 对各立方体:
///   - 所有空间方向 (s+/s-/d+/d-/t+) 默认允许膨胀
///   - 第一个立方体的 t- 方向也允许 (因为需要覆盖起始时刻)
///   - 非首个立方体的 t- 方向禁止 (prevents backward time expansion)
ErrorType SscMap::GetInflationDirections(const bool &if_first_cube,
                                         std::array<bool, 6> *dirs_disabled) {
  (*dirs_disabled)[0] = false;  // s+ 允许
  (*dirs_disabled)[1] = false;  // s- 允许
  (*dirs_disabled)[2] = false;  // d+ 允许
  (*dirs_disabled)[3] = false;  // d- 允许
  (*dirs_disabled)[4] = false;  // t+ 允许
  (*dirs_disabled)[5] = !if_first_cube;  // t- 仅第一个立方体允许

  return kSuccess;
}

ErrorType SscMap::ClearGridMap() {
  p_3d_grid_->clear_data();
  p_3d_inflated_grid_->clear_data();
  return kSuccess;
}

ErrorType SscMap::ClearDrivingCorridor() {
  driving_corridor_vec_.clear();
  return kSuccess;
}

/// @brief 重置风险占据图 —— 使用 std::fill 将所有栅格值置零
/// @return kSuccess
/// @note 时间复杂度 O(N)，N 为栅格总数 (s_dim * d_dim * t_dim)。
///       std::fill 对 float 数组高度优化，通常被编译器向量化
ErrorType SscMap::ResetRiskMap() {
  std::fill(p_3d_risk_grid_.begin(), p_3d_risk_grid_.end(), 0.0f);
  return kSuccess;
}

/// @brief 计算风险占据栅格的统计信息 —— 只读操作，不修改任何地图数据
/// @return RiskGridStats 结构体
/// @note MVP-1A: 纯调试函数。逐栅格遍历 p_3d_risk_grid_，
///       按 layer_idx = idx / (w * h) 分层统计非零栅格数和风险值。
///       sum_risk 使用 double 累加以避免 float 截断误差。
///       该函数不参与任何规划决策。
RiskGridStats SscMap::ComputeRiskGridStats() const {
  RiskGridStats stats;

  /// @brief 风险图总栅格数 —— 直接取自 p_3d_risk_grid_.size()
  stats.total_cells = p_3d_risk_grid_.size();

  /// @brief 获取三维尺寸: w = s方向栅格数, h = d方向栅格数, t = 时间层数
  const auto dims_size = p_3d_grid_->dims_size();
  int w = dims_size[0];  // s 方向栅格数
  int h = dims_size[1];  // d 方向栅格数
  int t = dims_size[2];  // t 方向栅格数（时间层数）

  /// @brief 每层非零栅格计数，初始化为 t 个 0
  if (t > 0) {
    stats.nonzero_cells_per_layer.resize(static_cast<size_t>(t), 0);
  }

  /// @brief 异常尺寸或空风险图直接返回空统计，避免调试函数触发除零或越界
  if (w <= 0 || h <= 0 || t <= 0 || stats.total_cells == 0) {
    return stats;
  }

  const size_t layer_cell_num =
      static_cast<size_t>(w) * static_cast<size_t>(h);

  /// @brief 单次遍历完成所有统计: 非零计数、最大值、总和、分层计数
  stats.max_risk = 0.0f;
  stats.sum_risk = 0.0;
  for (size_t idx = 0; idx < stats.total_cells; ++idx) {
    RiskMapDataType risk = p_3d_risk_grid_[idx];

    /// @brief 非零判断: risk > 0.0f 视为占据栅格
    if (risk > 0.0f) {
      ++stats.nonzero_cells;

      /// @brief 更新最大风险值
      if (risk > stats.max_risk) {
        stats.max_risk = risk;
      }

      /// @brief double 累加风险值，避免 float 截断误差
      stats.sum_risk += static_cast<double>(risk);

      /// @brief 按 idx / (w * h) 得到当前栅格所在的时间层索引
      size_t layer_idx = idx / layer_cell_num;
      if (layer_idx < static_cast<size_t>(t)) {
        ++stats.nonzero_cells_per_layer[layer_idx];
      }
    }
  }

  /// @brief 统计活跃时间层: 至少有一个非零栅格的时间层
  stats.active_time_layers = 0;
  for (size_t layer = 0; layer < static_cast<size_t>(t); ++layer) {
    if (stats.nonzero_cells_per_layer[layer] > 0) {
      ++stats.active_time_layers;
    }
  }

  return stats;
}

/// @brief 按需输出风险占据栅格统计日志
/// @param stats 由 ComputeRiskGridStats() 计算得到的统计结构
/// @note MVP-1A: 使用 LOG(WARNING) 输出，前缀固定为 [Ssc][RiskGridStats]。
///       输出内容: summary 行（total/nonzero/max/sum/active_layers）
///       + per-layer 行（每时间层的非零栅格数）。
///       该函数不触发任何规划逻辑。
void SscMap::PrintRiskGridStatsIfNeeded(const RiskGridStats &stats) const {
  LOG(WARNING) << "[Ssc][RiskGridStats] total_cells=" << stats.total_cells
               << " nonzero_cells=" << stats.nonzero_cells
               << " max_risk=" << stats.max_risk
               << " sum_risk=" << stats.sum_risk
               << " active_time_layers=" << stats.active_time_layers
               << "/" << stats.nonzero_cells_per_layer.size();

  /// @brief 逐层输出非零栅格数，用于验证 risk grid 是否被正确填充到各个时间层
  for (size_t layer = 0; layer < stats.nonzero_cells_per_layer.size(); ++layer) {
    LOG(WARNING) << "[Ssc][RiskGridStats] layer[" << layer
                 << "] nonzero_cells=" << stats.nonzero_cells_per_layer[layer];
  }
}

/// @brief 将风险占据栅格统计结果追加写入 CSV 文件
/// @param stats 由 ComputeRiskGridStats() 计算得到的统计结构
/// @note MVP-1B: 只做实验数据记录，不修改 risk grid / binary map / corridor。
///       CSV 字段固定为:
///       cycle,total_cells,nonzero_cells,max_risk,sum_risk,active_time_layers,total_time_layers
void SscMap::AppendRiskGridStatsToCsv(const RiskGridStats &stats) const {
  /// @brief 若 CSV 导出被关闭，立即返回，保持原始规划流程不受影响
  if (!risk_stats_csv_enabled_) {
    return;
  }

  /// @brief 检查目标文件是否为空；空文件需要先写 header，已有数据则直接追加
  bool csv_file_empty = true;
  {
    std::ifstream existing_file(risk_stats_csv_path_);
    csv_file_empty =
        !existing_file.good() ||
        existing_file.peek() == std::ifstream::traits_type::eof();
  }

  /// @brief 以 append 模式打开 CSV，避免覆盖同一次实验中前面 planning cycle 的数据
  std::ofstream csv_file(risk_stats_csv_path_, std::ofstream::out | std::ofstream::app);
  if (!csv_file.is_open()) {
    LOG(WARNING) << "[Ssc][RiskGridStatsCsv] failed to open "
                 << risk_stats_csv_path_;
    return;
  }

  /// @brief 文件为空时写入一次 header，便于后续 Python/pandas 直接读取
  if (csv_file_empty) {
    csv_file << "cycle,total_cells,nonzero_cells,max_risk,sum_risk,"
             << "active_time_layers,total_time_layers\n";
    risk_stats_csv_header_written_ = true;
  } else if (!risk_stats_csv_header_written_) {
    /// @brief 文件已有 header 或历史数据时，仅同步进程内状态，避免重复写 header
    risk_stats_csv_header_written_ = true;
  }

  /// @brief 记录当前 planning cycle 编号；该计数只服务 CSV 实验追踪，不参与规划
  const size_t current_cycle = risk_stats_cycle_count_;
  ++risk_stats_cycle_count_;

  /// @brief 追加一行统计数据，total_time_layers 直接来自 per-layer 统计数组长度
  csv_file << current_cycle << "," << stats.total_cells << ","
           << stats.nonzero_cells << "," << stats.max_risk << ","
           << stats.sum_risk << "," << stats.active_time_layers << ","
           << stats.nonzero_cells_per_layer.size() << "\n";

  /// @brief 写入失败只输出日志，不回滚地图或影响本次规划结果
  if (!csv_file.good()) {
    LOG(WARNING) << "[Ssc][RiskGridStatsCsv] failed to write "
                 << risk_stats_csv_path_;
  }
}

/// @brief 沿初始参考轨迹构建时空走廊 —— 核心算法
///
/// 两阶段算法:
///
/// Stage I — 种子点采样:
///   遍历前向轨迹的每一帧状态, 对第一帧在 Frenet (s,d,t) 空间中
///   从 initial_fs_ 和轨迹第一帧分别采样两个种子。
///   后续帧从前一帧的立方体中采样。跳过超出地图范围或时间跨度过长的帧。
///
/// Stage II — 立方体膨胀与连接:
///   对每对相邻种子:
///     - 生成初始轴对齐立方体
///     - 碰撞检测: 初始立方体必须无障碍
///     - 六方向膨胀 (InflateCubeIn3dGrid)
///     - 后续种子如果在已有立方体内 → 归入该立方体
///     - 后续种子如果超出已有立方体 → 裁剪当前立方体 t 上界, 另起新立方体
///
/// @param p_grid 3D 占据栅格地图
/// @param trajs  Frenet 坐标下的前向车辆轨迹
/// @return 种子点少于 2 时返回错误
ErrorType SscMap::ConstructCorridorUsingInitialTrajectory(
    GridMap3D *p_grid, const vec_E<common::FsVehicle> &trajs) {
  // ===================================================================
  // Stage I: 采样种子点
  // ===================================================================
  vec_E<Vec3i> traj_seeds;  // 收集所有有效的栅格坐标种子
  int num_states = static_cast<int>(trajs.size());

  if (num_states > 1) {
    bool first_seed_determined = false;  // 是否已确定初始种子对

    for (int k = 0; k < num_states; ++k) {
      std::array<decimal_t, 3> p_w = {};

      if (!first_seed_determined) {
        // 第一个种子对: seed_0 = initial_fs_ 的位置, seed_1 = 轨迹首帧位置
        decimal_t s_0 = initial_fs_.vec_s[0];
        decimal_t d_0 = initial_fs_.vec_dt[0];
        decimal_t t_0 = initial_fs_.time_stamp;
        std::array<decimal_t, 3> p_w_0 = {s_0, d_0, t_0};
        auto coord_0 = p_grid->GetCoordUsingGlobalPosition(p_w_0);

        decimal_t s_1 = trajs[k].frenet_state.vec_s[0];
        decimal_t d_1 = trajs[k].frenet_state.vec_dt[0];
        decimal_t t_1 = trajs[k].frenet_state.time_stamp;
        std::array<decimal_t, 3> p_w_1 = {s_1, d_1, t_1};
        auto coord_1 = p_grid->GetCoordUsingGlobalPosition(p_w_1);

        // 过滤: 超出地图范围或时间过小的帧 (可能为历史数据)
        if (!p_grid->CheckCoordInRange(coord_1)) {
          continue;
        }
        if (coord_1[2] <= 0) {  // 时间索引小于等于0 → 过去时刻
          continue;
        }

        first_seed_determined = true;
        traj_seeds.push_back(Vec3i(coord_0[0], coord_0[1], coord_0[2]));
        traj_seeds.push_back(Vec3i(coord_1[0], coord_1[1], coord_1[2]));
      } else {
        // 后续种子: 直接从轨迹帧的 Frenet 状态采集
        decimal_t s = trajs[k].frenet_state.vec_s[0];
        decimal_t d = trajs[k].frenet_state.vec_dt[0];
        decimal_t t = trajs[k].frenet_state.time_stamp;
        p_w = {s, d, t};
        auto coord = p_grid->GetCoordUsingGlobalPosition(p_w);

        // 过滤超出范围的帧
        if (!p_grid->CheckCoordInRange(coord)) {
          continue;
        }
        traj_seeds.push_back(Vec3i(coord[0], coord[1], coord[2]));
      }
    }
  }

  // ===================================================================
  // Stage II: 从种子对出发, 膨胀立方体构建走廊
  // ===================================================================
  common::DrivingCorridor driving_corridor;
  bool is_valid = true;
  auto seed_num = static_cast<int>(traj_seeds.size());

  // 至少需要 2 个种子才能构建立方体
  if (seed_num < 2) {
    driving_corridor.is_valid = false;
    driving_corridor_vec_.push_back(driving_corridor);
    is_valid = false;
    return kWrongStatus;
  }

  for (int i = 0; i < seed_num; ++i) {
    if (i == 0) {
      // ~~~ 处理第一个立方体 ~~~

      // 从第 0 和第 1 个种子生成初始立方体
      common::AxisAlignedCubeNd<int, 3> cube;
      GetInitialCubeUsingSeed(traj_seeds[i], traj_seeds[i + 1], &cube);

      // 碰撞检测: 初始立方体必须完全空闲
      if (!CheckIfCubeIsFree(p_grid, cube)) {
        LOG(ERROR) << "[Ssc] SccMap - Initial cube is not free, seed id: " << i;

        // 记录无效立方体信息 (用于调试)
        common::DrivingCube driving_cube;
        driving_cube.cube = cube;
        driving_cube.seeds.push_back(traj_seeds[i]);
        driving_cube.seeds.push_back(traj_seeds[i + 1]);
        driving_corridor.cubes.push_back(driving_cube);

        driving_corridor.is_valid = false;
        driving_corridor_vec_.push_back(driving_corridor);
        is_valid = false;
        break;
      }

      // 六方向膨胀立方体
      std::array<bool, 6> dirs_disabled = {false, false, false,
                                           false, false, false};
      InflateCubeIn3dGrid(p_grid, dirs_disabled, config_.inflate_steps, &cube);

      // 记录膨胀后的立方体
      common::DrivingCube driving_cube;
      driving_cube.cube = cube;
      driving_cube.seeds.push_back(traj_seeds[i]);
      driving_corridor.cubes.push_back(driving_cube);

    } else {
      // ~~~ 处理后续立方体 ~~~
      // 关键逻辑: 检查当前种子是否仍在最后一个立方体内

      if (CheckIfCubeContainsSeed(driving_corridor.cubes.back().cube,
                                  traj_seeds[i])) {
        // 种子仍在当前立方体范围内 → 归入该立方体
        driving_corridor.cubes.back().seeds.push_back(traj_seeds[i]);
        continue;
      } else {
        // 种子超出当前立方体 → 需要新建立方体
        // 步骤1: 取出最后一个立方体的最后一个种子
        Vec3i seed_r = driving_corridor.cubes.back().seeds.back();
        driving_corridor.cubes.back().seeds.pop_back();
        // 步骤2: 裁剪当前立方体的时间上界 (在最后一个有效时间处切断)
        driving_corridor.cubes.back().cube.upper_bound[2] = seed_r(2);
        // 步骤3: 指针回退, 用裁剪后的种子和新种子生成下一个立方体
        i = i - 1;

        common::AxisAlignedCubeNd<int, 3> cube;
        GetInitialCubeUsingSeed(traj_seeds[i], traj_seeds[i + 1], &cube);

        // 碰撞检测
        if (!CheckIfCubeIsFree(p_grid, cube)) {
          LOG(ERROR) << "[Ssc] SccMap - Initial cube is not free, seed id: "
                     << i;
          common::DrivingCube driving_cube;
          driving_cube.cube = cube;
          driving_cube.seeds.push_back(traj_seeds[i]);
          driving_cube.seeds.push_back(traj_seeds[i + 1]);
          driving_corridor.cubes.push_back(driving_cube);

          driving_corridor.is_valid = false;
          driving_corridor_vec_.push_back(driving_corridor);
          is_valid = false;
          break;
        }

        // 膨胀新立方体
        std::array<bool, 6> dirs_disabled = {false, false, false,
                                             false, false, false};
        InflateCubeIn3dGrid(p_grid, dirs_disabled, config_.inflate_steps,
                            &cube);

        common::DrivingCube driving_cube;
        driving_cube.cube = cube;
        driving_cube.seeds.push_back(traj_seeds[i]);
        driving_corridor.cubes.push_back(driving_cube);
      }
    }
  }

  // 走廊完成: 保存结果
  if (is_valid) {
    // 走廊松弛 (可选, 当前禁用): 在连接处扩大空间
    // CorridorRelaxation(p_grid, &driving_corridor);

    // 裁剪最后一个立方体的时间上界
    driving_corridor.cubes.back().cube.upper_bound[2] = traj_seeds.back()(2);
    driving_corridor.is_valid = true;
    driving_corridor_vec_.push_back(driving_corridor);
  }

  return kSuccess;
}

/// @brief 获取沿时间轴方向可达的立方体索引列表
/// @param p_corridor 走廊对象
/// @param start_idx  起始立方体索引
/// @param dir        方向: 1=向前 (沿时间增大), 0=向后 (沿时间减小)
/// @param t_trans    目标累计时间跨度 (以单位栅格计)
/// @param idx_list   输出: 按方向遍历到的立方体索引
ErrorType SscMap::GetTimeCoveredCubeIndices(
    const common::DrivingCorridor *p_corridor, const int &start_idx,
    const int &dir, const int &t_trans, std::vector<int> *idx_list) const {
  int dt = 0;                          // 累计时间跨度
  int num_cube = p_corridor->cubes.size();
  int idx = start_idx;

  while (idx < num_cube && idx >= 0) {
    // 累加当前立方体的时间跨度
    dt += p_corridor->cubes[idx].cube.upper_bound[2] -
          p_corridor->cubes[idx].cube.lower_bound[2];
    idx_list->push_back(idx);

    if (dir == 1) {
      ++idx;  // 向前遍历
    } else {
      --idx;  // 向后遍历
    }

    if (dt >= t_trans) {
      break;  // 达到目标时间跨度
    }
  }

  return kSuccess;
}

/// @brief 走廊松弛: 在相邻立方体连接处扩展 s 和 d 方向的可用空间
///
/// 目的: 在立方体之间的时间边界上, 扩大 s 和 d 方向的过渡空间,
///       使 QP 优化有更多可行解 (避免过窄的连接处导致不可行)。
///
/// 策略:
///   - s 方向: 若相邻立方体 s 边界间隙 < 50 栅格, 沿时间轴双向扩展
///   - d 方向: 若相邻立方体 d 边界间隙 < 10 栅格, 沿时间轴双向扩展
///
/// 当前在主流程中被禁用 (if(1) 块, CorridorRelaxation 调用被注释)
ErrorType SscMap::CorridorRelaxation(GridMap3D *p_grid,
                                     common::DrivingCorridor *p_corridor) {
  std::array<int, 2> margin = {{50, 10}};  // s/d 方向容忍间隙
  int t_trans = 7;                          // 时间轴上搜索的跨度

  int num_cube = p_corridor->cubes.size();

  for (int i = 0; i < num_cube - 1; ++i) {
    // --- s 方向松弛 ---
    if (1)  // ~ 启用 s 方向松弛
    {
      int cube_0_lb = p_corridor->cubes[i].cube.lower_bound[0];
      int cube_0_ub = p_corridor->cubes[i].cube.upper_bound[0];
      int cube_1_lb = p_corridor->cubes[i + 1].cube.lower_bound[0];
      int cube_1_ub = p_corridor->cubes[i + 1].cube.upper_bound[0];

      // 情况1: cube_0 的上界接近 cube_1 的下界 (即间隙在 margin 内)
      if (abs(cube_0_ub - cube_1_lb) < margin[0]) {
        int room = margin[0] - abs(cube_0_ub - cube_1_lb);
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        // 上游立方体向左 (s-) 扩展, 下游立方体向右 (s+) 扩展
        for (const auto &idx : up_idx_list) {
          InflateCubeOnXNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto &idx : down_idx_list) {
          InflateCubeOnXPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
      // 情况2: cube_0 的下界接近 cube_1 的上界
      if (abs(cube_0_lb - cube_1_ub) < margin[0]) {
        int room = margin[0] - abs(cube_0_lb - cube_1_ub);
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto &idx : up_idx_list) {
          InflateCubeOnXPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto &idx : down_idx_list) {
          InflateCubeOnXNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
    }

    // --- d 方向松弛 ---
    if (1)  // ~ 启用 d 方向松弛
    {
      int cube_0_lb = p_corridor->cubes[i].cube.lower_bound[1];
      int cube_0_ub = p_corridor->cubes[i].cube.upper_bound[1];
      int cube_1_lb = p_corridor->cubes[i + 1].cube.lower_bound[1];
      int cube_1_ub = p_corridor->cubes[i + 1].cube.upper_bound[1];

      if (abs(cube_0_ub - cube_1_lb) < margin[1]) {
        int room = margin[1] - abs(cube_0_ub - cube_1_lb);
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto &idx : up_idx_list) {
          InflateCubeOnYNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto &idx : down_idx_list) {
          InflateCubeOnYPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
      if (abs(cube_0_lb - cube_1_ub) < margin[1]) {
        int room = margin[1] - abs(cube_0_lb - cube_1_ub);
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto idx : up_idx_list) {
          InflateCubeOnYPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto idx : down_idx_list) {
          InflateCubeOnYNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
    }
  }

  return kSuccess;
}

/// @brief 对障碍物栅格进行车辆物理尺寸的膨胀
///
/// 膨胀量 = 车辆半长 - 后轴到车尾距离 (用于计算车辆前后超出后轴的尺寸)
///
/// 注意: 此函数在 EUDM 项目中为节省时间而不在主流程中调用,
///       障碍物膨胀部分由行为规划层的 MPC 前向仿真处理。
ErrorType SscMap::InflateObstacleGrid(const common::VehicleParam &param) {
  // s 方向膨胀量 (基于车辆长度和后轴位置)
  decimal_t s_p_inflate_len = param.length() / 2.0 - param.d_cr();
  decimal_t s_n_inflate_len = param.length() - s_p_inflate_len;
  int num_s_p_inflate_grids =
      std::floor(s_p_inflate_len / config_.map_resolution[0]);
  int num_s_n_inflate_grids =
      std::floor(s_n_inflate_len / config_.map_resolution[0]);
  // d 方向膨胀量 (基于车辆宽度, 减去默认的 0.5m 缓冲区)
  int num_d_inflate_grids =
      std::floor((param.width() - 0.5) / 2.0 / config_.map_resolution[1]);
  bool is_free = false;

  // 遍历整个 3D 地图, 对每个被占用的栅格进行膨胀
  for (int i = 0; i < config_.map_size[0]; ++i) {
    for (int j = 0; j < config_.map_size[1]; ++j) {
      for (int k = 0; k < config_.map_size[2]; ++k) {
        std::array<int, 3> coord = {i, j, k};
        p_3d_grid_->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
        if (!is_free) {
          // 如果该栅格被占用, 在 s 和 d 方向上膨胀
          for (int s = -num_s_n_inflate_grids; s < num_s_p_inflate_grids; s++) {
            for (int d = -num_d_inflate_grids; d < num_d_inflate_grids; d++) {
              coord = {i + s, j + d, k};  // 时间维度保持不变
              p_3d_inflated_grid_->SetValueUsingCoordinate(coord, 100);
            }
          }
        }
      }
    }
  }

  return kSuccess;
}

/// @brief 六方向立方体膨胀主循环
///
/// 膨胀策略:
///   1. 在 s+, s-, d+, d- 四个空间方向上交替尝试膨胀
///   2. 每次膨胀 n_step 个栅格 (由 inflate_steps 配置)
///   3. 运动学约束: 根据初始速度和最大加速度计算 s 的可达范围
///      s_upper = s_0 + v_0 * t + 0.5 * a_max * t^2 + d_comp
///      s_lower = s_0 + v_0 * t + 0.5 * a_min * t^2 - d_comp
///   4. t+ 方向单独膨胀 (最多 kMaxNumOfGridAlongTime 个栅格)
///   5. t- 方向不允许 (时间不能倒流)
///
/// @param p_grid       3D 占据栅格地图
/// @param dir_disabled  六方向禁用标记
/// @param dir_step     六方向每次膨胀步长
/// @param cube         待膨胀立方体 (in/out)
ErrorType SscMap::InflateCubeIn3dGrid(GridMap3D *p_grid,
                                      const std::array<bool, 6> &dir_disabled,
                                      const std::array<int, 6> &dir_step,
                                      common::AxisAlignedCubeNd<int, 3> *cube) {
  // 标记各方向是否完成
  bool x_p_finish = dir_disabled[0];  // s+
  bool x_n_finish = dir_disabled[1];  // s-
  bool y_p_finish = dir_disabled[2];  // d+
  bool y_n_finish = dir_disabled[3];  // d-
  bool z_p_finish = dir_disabled[4];  // t+

  // 各方向步长
  int x_p_step = dir_step[0];
  int x_n_step = dir_step[1];
  int y_p_step = dir_step[2];
  int y_n_step = dir_step[3];
  int z_p_step = dir_step[4];

  // ===================================================================
  // 计算 s 方向的运动学可达边界
  // ===================================================================
  // t_max: 基于当前立方体和最大预期时间膨胀的最远时间
  int t_max_grids = cube->lower_bound[2] + config_.kMaxNumOfGridAlongTime;
  decimal_t t = t_max_grids * p_grid->dims_resolution(2);
  decimal_t a_max = config_.kMaxLongitudinalAcc;
  decimal_t a_min = config_.kMaxLongitudinalDecel;
  decimal_t d_comp = initial_fs_.vec_s[1] * 1;  // 速度补偿项

  // s 可达上界: s_0 + v*t + 0.5*a_max*t^2 + d_comp
  decimal_t s_u = initial_fs_.vec_s[0] + initial_fs_.vec_s[1] * t +
                  0.5 * a_max * t * t + d_comp;
  // s 可达下界: s_0 + v*t + 0.5*a_min*t^2 - d_comp
  decimal_t s_l = initial_fs_.vec_s[0] + initial_fs_.vec_s[1] * t +
                  0.5 * a_min * t * t - d_comp;

  int s_idx_u, s_idx_l;
  p_grid->GetCoordUsingGlobalMetricOnSingleDim(s_u, 0, &s_idx_u);
  p_grid->GetCoordUsingGlobalMetricOnSingleDim(s_l, 0, &s_idx_l);
  // 保证 s 下界不低于安全阈值
  s_idx_l = std::max(s_idx_l, static_cast<int>((config_.s_back_len / 2.0) /
                                               config_.map_resolution[0]));

  // ===================================================================
  // 空间方向的交替膨胀
  // ===================================================================
  while (!(x_p_finish && x_n_finish && y_p_finish && y_n_finish)) {
    // s+ 方向膨胀
    if (!x_p_finish) x_p_finish = InflateCubeOnXPosAxis(p_grid, x_p_step, cube);
    // s- 方向膨胀
    if (!x_n_finish) x_n_finish = InflateCubeOnXNegAxis(p_grid, x_n_step, cube);
    // d+ 方向膨胀
    if (!y_p_finish) y_p_finish = InflateCubeOnYPosAxis(p_grid, y_p_step, cube);
    // d- 方向膨胀
    if (!y_n_finish) y_n_finish = InflateCubeOnYNegAxis(p_grid, y_n_step, cube);

    // 运动学约束: 若膨胀超出 s 的可达范围则停止
    if (cube->upper_bound[0] >= s_idx_u) x_p_finish = true;
    if (cube->lower_bound[0] <= s_idx_l) x_n_finish = true;
  }

  // ===================================================================
  // t+ 方向膨胀 (单独处理, 不与其他方向交替)
  // ===================================================================
  while (!z_p_finish) {
    if (!z_p_finish) z_p_finish = InflateCubeOnZPosAxis(p_grid, z_p_step, cube);

    // 时间膨胀上限控制
    if (cube->upper_bound[2] - cube->lower_bound[2] >=
        config_.kMaxNumOfGridAlongTime) {
      z_p_finish = true;
    }
  }

  return kSuccess;
}

// ===================================================================
// 六方向单步膨胀函数
//
// 每个函数:
//   1. 尝试在指定方向上扩展 n_step 个栅格
//   2. 每次尝试扩展 1 个栅格, 检查扩展后的平面是否无障碍
//   3. 如果碰到障碍物或地图边界, 设置完成标志并停止
//   4. 如果到达运动学约束边界 (仅在 s 方向), 也会停止
//
// 返回值: true=该方向已完成膨胀, false=还可以继续膨胀
// ===================================================================

bool SscMap::InflateCubeOnXPosAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int x = cube->upper_bound[0] + 1;  // 尝试的下一个 s 索引
    if (!p_grid->CheckCoordInRangeOnSingleDim(x, 0)) {
      return true;  // 超出地图范围, 停止膨胀
    } else {
      // 检查 x 处的 (d,t) 平面是否完全空闲
      if (CheckIfPlaneIsFreeOnXAxis(p_grid, *cube, x)) {
        cube->upper_bound[0] = x;  // 平面空闲, 扩展上界
      } else {
        return true;  // 碰到障碍物, 停止膨胀
      }
    }
  }
  return false;  // 完成本步膨胀, 但可能还能继续
}

bool SscMap::InflateCubeOnXNegAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int x = cube->lower_bound[0] - 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(x, 0)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnXAxis(p_grid, *cube, x)) {
        cube->lower_bound[0] = x;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnYPosAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int y = cube->upper_bound[1] + 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(y, 1)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnYAxis(p_grid, *cube, y)) {
        cube->upper_bound[1] = y;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnYNegAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int y = cube->lower_bound[1] - 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(y, 1)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnYAxis(p_grid, *cube, y)) {
        cube->lower_bound[1] = y;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnZPosAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int z = cube->upper_bound[2] + 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(z, 2)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnZAxis(p_grid, *cube, z)) {
        cube->upper_bound[2] = z;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnZNegAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int z = cube->lower_bound[2] - 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(z, 2)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnZAxis(p_grid, *cube, z)) {
        cube->lower_bound[2] = z;
      } else {
        return true;
      }
    }
  }
  return false;
}

// ===================================================================
// 碰撞检测函数
// ===================================================================

/// @brief 检查立方体内部是否完全空闲
///
/// 三重循环遍历立方体(s_min..s_max, d_min..d_max, t_min..t_max)内的
/// 所有栅格单元格, 检查每个单元格的值是否为 0 (空闲)。
/// 一旦发现任一单元格不为 0, 立即返回 false。
bool SscMap::CheckIfCubeIsFree(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube) const {
  int f0_min = cube.lower_bound[0];
  int f0_max = cube.upper_bound[0];
  int f1_min = cube.lower_bound[1];
  int f1_max = cube.upper_bound[1];
  int f2_min = cube.lower_bound[2];
  int f2_max = cube.upper_bound[2];

  std::array<int, 3> coord;
  bool is_free;

  for (int i = f0_min; i <= f0_max; ++i) {
    for (int j = f1_min; j <= f1_max; ++j) {
      for (int k = f2_min; k <= f2_max; ++k) {
        coord = {i, j, k};
        p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
        if (!is_free) {
          return false;  // 发现障碍物, 立即返回
        }
      }
    }
  }
  return true;
}

/// @brief 检查 X 轴某固定坐标处的 (d, t) 平面是否完全空闲
/// @param x 固定 s 坐标
bool SscMap::CheckIfPlaneIsFreeOnXAxis(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube,
    const int &x) const {
  int f0_min = cube.lower_bound[1];  // d 下界
  int f0_max = cube.upper_bound[1];  // d 上界
  int f1_min = cube.lower_bound[2];  // t 下界
  int f1_max = cube.upper_bound[2];  // t 上界
  std::array<int, 3> coord;
  bool is_free;
  for (int i = f0_min; i <= f0_max; ++i) {
    for (int j = f1_min; j <= f1_max; ++j) {
      coord = {x, i, j};  // (s=x, d=i, t=j)
      p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
      if (!is_free) {
        return false;
      }
    }
  }
  return true;
}

/// @brief 检查 Y 轴某固定坐标处的 (s, t) 平面是否完全空闲
/// @param y 固定 d 坐标
bool SscMap::CheckIfPlaneIsFreeOnYAxis(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube,
    const int &y) const {
  int f0_min = cube.lower_bound[0];  // s 下界
  int f0_max = cube.upper_bound[0];  // s 上界
  int f1_min = cube.lower_bound[2];  // t 下界
  int f1_max = cube.upper_bound[2];  // t 上界
  std::array<int, 3> coord;
  bool is_free;
  for (int i = f0_min; i <= f0_max; ++i) {
    for (int j = f1_min; j <= f1_max; ++j) {
      coord = {i, y, j};  // (s=i, d=y, t=j)
      p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
      if (!is_free) {
        return false;
      }
    }
  }
  return true;
}

/// @brief 检查 Z 轴某固定坐标处的 (s, d) 平面是否完全空闲
/// @param z 固定 t 坐标
bool SscMap::CheckIfPlaneIsFreeOnZAxis(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube,
    const int &z) const {
  int f0_min = cube.lower_bound[0];  // s 下界
  int f0_max = cube.upper_bound[0];  // s 上界
  int f1_min = cube.lower_bound[1];  // d 下界
  int f1_max = cube.upper_bound[1];  // d 上界
  std::array<int, 3> coord;
  bool is_free;
  for (int i = f0_min; i <= f0_max; ++i) {
    for (int j = f1_min; j <= f1_max; ++j) {
      coord = {i, j, z};  // (s=i, d=j, t=z)
      p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
      if (!is_free) {
        return false;
      }
    }
  }
  return true;
}

/// @brief 检查种子点是否在立方体内部
///
/// 遍历 (s, d, t) 三个维度, 确认:
///   lower_bound[i] <= seed(i) <= upper_bound[i] 对所有 i 成立
bool SscMap::CheckIfCubeContainsSeed(
    const common::AxisAlignedCubeNd<int, 3> &cube_a, const Vec3i &seed) const {
  for (int i = 0; i < 3; ++i) {
    if (cube_a.lower_bound[i] > seed(i) || cube_a.upper_bound[i] < seed(i)) {
      return false;
    }
  }
  return true;
}

/// @brief 将栅格坐标下的走廊转换为物理坐标 (SpatioTemporalSemanticCube)
///
/// 转换操作:
///   1. 将栅格索引通过 GetGlobalMetricUsingCoordOnSingleDim 转为物理值
///   2. 填充速度和加速度的上下界 (源自 config 中的运动学约束)
///   3. 检查第一个立方体的 d 范围是否包含初始 d 位置
///   4. 构建 final_corridor_vec_ 和 if_corridor_valid_ 输出
ErrorType SscMap::GetFinalGlobalMetricCubesList() {
  final_corridor_vec_.clear();
  if_corridor_valid_.clear();

  for (const auto &corridor : driving_corridor_vec_) {
    vec_E<common::SpatioTemporalSemanticCubeNd<2>> cubes;

    if (!corridor.is_valid) {
      if_corridor_valid_.push_back(0);  // 标记为无效
    } else {
      if_corridor_valid_.push_back(1);  // 标记为有效

      for (int k = 0; k < static_cast<int>(corridor.cubes.size()); ++k) {
        common::SpatioTemporalSemanticCubeNd<2> cube;
        decimal_t x_lb, x_ub;  // s 下/上界 (米)
        decimal_t y_lb, y_ub;  // d 下/上界 (米)
        decimal_t z_lb, z_ub;  // t 下/上界 (秒)

        // 从栅格坐标转换为物理坐标
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.lower_bound[0], 0, &x_lb);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.upper_bound[0], 0, &x_ub);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.lower_bound[1], 1, &y_lb);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.upper_bound[1], 1, &y_ub);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.lower_bound[2], 2, &z_lb);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.upper_bound[2], 2, &z_ub);

        // 设置时间边界
        cube.t_lb = z_lb;
        cube.t_ub = z_ub;

        // 设置 s 方向位置、速度、加速度边界
        cube.p_lb[0] = x_lb;
        cube.p_ub[0] = x_ub;
        cube.v_lb[0] = config_.kMinLongitudinalVel;
        cube.v_ub[0] = config_.kMaxLongitudinalVel;
        cube.a_lb[0] = config_.kMaxLongitudinalDecel;
        cube.a_ub[0] = config_.kMaxLongitudinalAcc;

        // 设置 d 方向位置、速度、加速度边界
        cube.p_lb[1] = y_lb;
        cube.p_ub[1] = y_ub;
        cube.v_lb[1] = -config_.kMaxLateralVel;       // 横向速度有正负
        cube.v_ub[1] = config_.kMaxLateralVel;
        cube.a_lb[1] = -config_.kMaxLateralAcc;       // 横向加速度有正负
        cube.a_ub[1] = config_.kMaxLateralAcc;

        // 一致性检查: 第一个立方体的 d 范围必须包含初始 d 位置
        if (k == 0) {
          if (y_lb > initial_fs_.vec_dt[0] || y_ub < initial_fs_.vec_dt[0]) {
            LOG(ERROR) << "[Ssc] SscMap - Initial state out of bound d: "
                       << initial_fs_.vec_dt[0] << ", lb: " << y_lb
                       << ", ub: " << y_ub;
            return kWrongStatus;
          }
        }

        cubes.push_back(cube);
      }
    }
    final_corridor_vec_.push_back(cubes);
  }

  return kSuccess;
}

/// @brief 填充静态障碍物到所有时间层
///
/// 对于每个障碍物栅格 (s, d):
///   将该栅格在 s-d 平面上的所有时间层 (0..map_size[2]) 全部标记为占用。
///   这意味着静态障碍物在时间维度上是"永恒的" —— 无论未来何时都不能进入。
///
/// 过滤: 跳过 s <= 0 的栅格 (可能为无效数据或自车后方)
ErrorType SscMap::FillStaticPart(const vec_E<Vec2f> &obs_grid_fs) {
  for (int i = 0; i < static_cast<int>(obs_grid_fs.size()); ++i) {
    if (obs_grid_fs[i](0) <= 0) {
      continue;  // 跳过无效的 s 坐标
    }
    // 在所有时间层上标记该 (s, d) 为占用
    for (int k = 0; k < config_.map_size[2]; ++k) {
      std::array<decimal_t, 3> pt = {{obs_grid_fs[i](0), obs_grid_fs[i](1),
                                      (double)k * config_.map_resolution[2]}};
      auto coord = p_3d_grid_->GetCoordUsingGlobalPosition(pt);
      if (p_3d_grid_->CheckCoordInRange(coord)) {
        p_3d_grid_->SetValueUsingCoordinate(coord, 100);  // 100 = 占用标记
      }
    }
  }
  return kSuccess;
}

/// @brief 填充动态障碍物: 遍历所有周围车辆, 逐条填充其预测轨迹
ErrorType SscMap::FillDynamicPart(
    const std::unordered_map<int, vec_E<common::FsVehicle>>
        &sur_vehicle_trajs_fs) {
  for (auto it = sur_vehicle_trajs_fs.begin(); it != sur_vehicle_trajs_fs.end();
       ++it) {
    FillMapWithFsVehicleTraj(it->second);
  }
  return kSuccess;
}

/// @brief 将单条 Frenet 车辆轨迹填充到 3D 栅格中
///
/// 方法: 多边形填充 (使用 OpenCV fillPoly)
///
/// 对轨迹中的每一帧:
///   1. 验证所有车辆顶点是否有效 (s > 0)
///   2. 计算各顶点的栅格坐标
///   3. 确定时间层索引 t_idx
///   4. 在对应的 s-d 平面层上, 使用 cv::fillPoly 填充车辆轮廓多边形
///
/// 为什么用多边形填充而非简单的矩形/圆形?
///   车辆轮廓是精确的多边形 (通过 GetVehicleVertices 计算),
///   多边形填充能准确表达车辆在 Frenet 坐标系下的实际形状。
ErrorType SscMap::FillMapWithFsVehicleTraj(
    const vec_E<common::FsVehicle> traj) {
  if (traj.size() == 0) {
    LOG(ERROR) << "[Ssc] SscMap - Trajectory is empty.";
    return kWrongStatus;
  }

  for (int i = 0; i < static_cast<int>(traj.size()); ++i) {
    bool is_valid = true;

    // 验证: 所有轮廓顶点的 s 坐标必须 > 0
    for (const auto &v : traj[i].vertices) {
      if (v(0) <= 0) {
        is_valid = false;
        break;
      }
    }
    if (!is_valid) {
      continue;
    }

    // 转换所有顶点到栅格坐标
    decimal_t z = traj[i].frenet_state.time_stamp;  // 当前帧时间戳
    int t_idx = 0;
    std::vector<common::Point2i> v_coord;  // 栅格坐标下的顶点列表
    std::array<decimal_t, 3> p_w;

    for (const auto &v : traj[i].vertices) {
      p_w = {v(0), v(1), z};
      auto coord = p_3d_grid_->GetCoordUsingGlobalPosition(p_w);
      t_idx = coord[2];  // 时间层索引 (所有顶点应在同一层)
      if (!p_3d_grid_->CheckCoordInRange(coord)) {
        is_valid = false;
        break;
      }
      v_coord.push_back(common::Point2i(coord[0], coord[1]));
    }
    if (!is_valid) {
      continue;
    }

    // 使用 OpenCV 多边形填充
    std::vector<std::vector<cv::Point2i>> vv_coord_cv;
    std::vector<cv::Point2i> v_coord_cv;
    common::ShapeUtils::GetCvPoint2iVecUsingCommonPoint2iVec(v_coord,
                                                             &v_coord_cv);
    vv_coord_cv.push_back(v_coord_cv);

    // 计算该时间层在平铺数据中的偏移量
    int w = p_3d_grid_->dims_size()[0];  // s 方向栅格数
    int h = p_3d_grid_->dims_size()[1];  // d 方向栅格数
    int layer_offset = t_idx * w * h;    // 跳转到第 t_idx 层的起始位置

    // 创建 cv::Mat 视图, 直接映射到 3D 栅格的对应层
    cv::Mat layer_mat =
        cv::Mat(h, w, CV_MAKETYPE(cv::DataType<SscMapDataType>::type, 1),
                p_3d_grid_->get_data_ptr() + layer_offset);
    // 多边形填充 (值100 = 占用)
    cv::fillPoly(layer_mat, vv_coord_cv, 100);
  }

  return kSuccess;
}

/// @brief 概率化填充动态障碍物 —— 遍历所有周围车辆，逐条写入风险占据图
/// @param sur_vehicle_trajs_fs 周围车辆在 Frenet 坐标下的预测轨迹集合
///                              (key=车辆ID, value=该车辆的Frenet轨迹序列)
/// @return kSuccess
/// @note 此函数的结构与 FillDynamicPart 完全对称，区别在于调用
///       FillMapWithFsVehicleTrajProbabilistic 而非 FillMapWithFsVehicleTraj
ErrorType SscMap::FillDynamicPartProbabilistic(
    const std::unordered_map<int, vec_E<common::FsVehicle>>& sur_vehicle_trajs_fs,
    const std::unordered_map<int, decimal_t> &traj_probs) {
  /// @brief 逐车填充风险占据图 —— 遍历 sur_vehicle_trajs_fs 中的每辆车
  /// @note 对每辆周围车辆的完整预测轨迹调用概率化填充，
  ///       填充值优先使用 traj_probs[vehicle_id]，缺失时保守回退 1.0f。
  for (auto it = sur_vehicle_trajs_fs.begin(); it != sur_vehicle_trajs_fs.end(); ++it) {
    float existence_prob = 1.0f;
    auto prob_it = traj_probs.find(it->first);
    if (prob_it != traj_probs.end()) {
      existence_prob =
          static_cast<float>(std::max<decimal_t>(
              0.0, std::min<decimal_t>(1.0, prob_it->second)));
    }
    FillMapWithFsVehicleTrajProbabilistic(it->second, existence_prob);
  }
  return kSuccess;
}

/// @brief 概率化填充单条车辆 Frenet 轨迹到风险占据图
/// @param traj 单辆周围车辆在 Frenet 坐标系下的完整预测轨迹 [frame_0, ... , frame_N]
/// @return kSuccess 填充成功 / kWrongStatus 轨迹为空
///
/// @note 算法流程（与 FillMapWithFsVehicleTraj 几何逻辑相同，写入目标不同）:
///       1. 空轨迹检查：traj.size() == 0 -> 返回错误
///       2. 逐帧遍历：对每个时间帧执行:
///          a. 顶点有效性验证: 所有轮廓顶点 s > 0
///          b. Frenet坐标 -> 栅格坐标: 通过 p_3d_grid_->GetCoordUsingGlobalPosition
///          c. 范围检查: p_3d_grid_->CheckCoordInRange
///          d. 时间层计算: t_idx = coord[2]
///          e. 偏移量计算: layer_offset = t_idx * w * h
///          f. OpenCV填充: cv::fillPoly 写入 CV_32FC1 浮点图层
///       3. 越界/无效帧静默跳过，不影响其他帧
///
/// @note MVP-0: existence_prob = 1.0f，将确定性轨迹镜像写入风险图。
///       重叠区域会被后续 fillPoly 覆盖（最后写入者胜出），
///       不做概率累加，不做 max 逻辑，不做时间衰减。
///       后续 MVP-1 将传入真实存在概率替代 1.0f
ErrorType SscMap::FillMapWithFsVehicleTrajProbabilistic(
    const vec_E<common::FsVehicle> traj, const float existence_prob) {
  /// @brief Step 1: 空轨迹检查 —— 轨迹为空时为无效输入
  if (traj.size() == 0) {
    LOG(ERROR) << "[Ssc] SscMap - Trajectory is empty (risk).";
    return kWrongStatus;
  }

  /// @brief Step 2: 逐帧遍历轨迹，将每帧车辆轮廓写入风险栅格
  for (int i = 0; i < static_cast<int>(traj.size()); ++i) {
    bool is_valid = true;

    /// @brief Step 2a: 验证所有轮廓顶点的 s 坐标 > 0
    /// @note s <= 0 表示该点在自车后方或为无效数据，需跳过整帧
    for (const auto& v : traj[i].vertices) {
      if (v(0) <= 0) {
        is_valid = false;
        break;
      }
    }
    if (!is_valid) continue;

    /// @brief Step 2b: 将 Frenet 坐标 (s, d, t) 转换为 3D 栅格坐标 (ix, iy, it)
    decimal_t z = traj[i].frenet_state.time_stamp;  // 当前帧的时间戳
    int t_idx = 0;
    std::vector<common::Point2i> v_coord;  // 该帧轮廓顶点的栅格坐标 (ix, iy)
    std::array<decimal_t, 3> p_w;          // 临时 Frenet 坐标容器 (s, d, t)

    for (const auto& v : traj[i].vertices) {
      p_w = {v(0), v(1), z};
      auto coord = p_3d_grid_->GetCoordUsingGlobalPosition(p_w);
      t_idx = coord[2];  // 所有顶点应在同一时间层
      if (!p_3d_grid_->CheckCoordInRange(coord)) {
        is_valid = false;
        break;
      }
      v_coord.push_back(common::Point2i(coord[0], coord[1]));
    }
    if (!is_valid) continue;

    /// @brief Step 2c: 将栅格顶点转为 OpenCV Point2i 多边形格式
    std::vector<std::vector<cv::Point2i>> vv_coord_cv;
    std::vector<cv::Point2i> v_coord_cv;
    common::ShapeUtils::GetCvPoint2iVecUsingCommonPoint2iVec(v_coord, &v_coord_cv);
    vv_coord_cv.push_back(v_coord_cv);

    /// @brief Step 2d: 计算该时间层在平铺一维数组中的偏移量
    /// @note 布局: 按 t 分层，每层为 (d 行 × s 列) 的 row-major 矩阵
    int w = p_3d_grid_->dims_size()[0];  // s 方向栅格数（图像宽度）
    int h = p_3d_grid_->dims_size()[1];  // d 方向栅格数（图像高度）
    int layer_offset = t_idx * w * h;    // 跳转到第 t_idx 时间层的起始地址

    /// @brief Step 2e: 在风险占据图上执行多边形填充
    /// @note CV_32FC1 = 单通道 32-bit 浮点数，值域 [0.0, 1.0]
    ///       cv::Scalar(existence_prob) 将所有多边形内部像素设为存在概率
    cv::Mat layer_mat(h, w, CV_32FC1, p_3d_risk_grid_.data() + layer_offset);
    cv::fillPoly(layer_mat, vv_coord_cv, cv::Scalar(existence_prob));
  }

  return kSuccess;
}

}  // namespace planning
