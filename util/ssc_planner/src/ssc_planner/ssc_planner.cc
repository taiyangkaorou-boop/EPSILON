/**
 * @file ssc_planner.cc
 * @author HKUST Aerial Robotics Group
 * @brief SSC（时空语义走廊）规划器的核心实现
 *
 * [文件重要程度: 最高]
 * 本文件实现了 EPSILON 系统中低层运动规划的全部核心算法。
 *
 * [完整算法流程]
 *
 *  Step 1. 数据准备 (RunOnce 前半部分)
 *     a. 从地图接口读取当前环境快照：自车状态、参考车道、行为决策、障碍物、
 *        多行为前向仿真轨迹、周围车辆预测轨迹
 *     b. 通过 StateTransformer 将初始状态从全局坐标系转换至 Frenet 坐标系，
 *        确定起始位置的 (s, d) 坐标及速度/加速度分量
 *     c. 将自车速度与 low_speed_threshold 比较，确定运行模式：
 *        - 高速: is_lateral_independent_ = true  (横向/纵向解耦)
 *        - 低速: is_lateral_independent_ = false (使用 primitive 轨迹)
 *
 *  Step 2. 批量坐标变换 (StateTransformForInputData)
 *     a. Stage I  - 数据打包:
 *        将自车状态、所有前向轨迹状态、周围车辆预测状态，以及它们的车辆轮廓
 *        顶点，统一打包成 global_state_vec 和 global_point_vec 两个扁平的
 *        一维向量，以便批量处理。
 *     b. Stage II - 执行变换 (单线程或 OpenMP 4 线程):
 *        对 global_state_vec 中每个状态，调用 stf_.GetFrenetStateFromState()
 *        转换为 FrenetState（包含 s, s_dot, s_ddot, d, d_dot, d_ddot）。
 *        对 global_point_vec 中每个点，调用 stf_.GetFrenetPointFromPoint()
 *        转换为 Frenet 坐标下的二维点。
 *     c. Stage III - 结果恢复:
 *        按打包时的偏移量从 frenet_state_vec 和 fs_point_vec 中提取各分量，
 *        重建 fs_ego_vehicle_, forward_trajs_fs_, surround_forward_trajs_fs_,
 *        obstacle_grids_fs_ 等 Frenet 坐标系下的数据结构。
 *
 *  Step 3. SSC 地图构建与走廊生成 (RunOnce 中的 SscMap 部分)
 *     a. 设置时间原点 time_origin_ 为初始状态的时间戳
 *     b. 调用 p_ssc_map_->ResetSscMap() 清空地图并更新原点
 *     c. 对每一种行为 (i = 0..num_behaviors):
 *        - 调用 ConstructSscMap() 构建 (s, d, t) 三维占据栅格:
 *          * FillStaticPart:  将静态障碍物栅格填充到所有时间层
 *          * FillDynamicPart: 将周围车辆预测轨迹逐帧填充（多边形填充）
 *        - 调用 ConstructCorridorUsingInitialTrajectory() 构建时空走廊:
 *          * Stage I  - 种子采样: 沿前向轨迹均匀采样点作为种子
 *          * Stage II - 立方体膨胀: 从种子对生成初始立方体, 六方向膨胀
 *        - (可选) InflateObstacleGrid() 对障碍物进行车辆尺寸膨胀
 *     d. 调用 GetFinalGlobalMetricCubesList() 将栅格坐标走廊转为物理单位
 *
 *  Step 4. QP 轨迹优化 (RunQpOptimization)
 *     a. 从 SscMap 获取各行为的时空走廊 cube_list 和有效性标志
 *     b. 对每条有效走廊:
 *        i.   提取起始约束 (3 组 Vecf<2>):
 *             - [0]: 起始位置    (s, d)
 *             - [1]: 起始速度    (s_dot, d_dot)，纵向速度有奇异保护(≥eps)
 *             - [2]: 起始加速度  (s_ddot, d_ddot)
 *        ii.  提取终止约束 (2 组 Vecf<2>):
 *             - [0]: 终止位置    (s, d)  = 前向轨迹末端状态
 *             - [1]: 终止速度    (s_dot, d_dot)
 *             (终止加速度约束被注释掉以增加优化可行性)
 *        iii. 修正走廊的终止时间: cube.back().t_ub = 前向轨迹终端时间
 *        iv.  检查走廊可行性 (CorridorFeasibilityCheck): 验证相邻立方体
 *             时间轴对齐 (t_ub[i-1] == t_lb[i])
 *        v.   构建参考点序列 (ref_stamps, ref_points, ref_states):
 *             以初始轨迹的各帧位置作为优化参考
 *        vi.  调用 spline_generator.GetBezierSplineUsingCorridor():
 *             - 输出: 5阶 2维 Bezier 样条 (s(t), d(t))
 *             - 优化目标: 最小化 jerk 和加速度
 *             - 约束: 所有采样时刻的轨迹点必须在时空走廊内
 *             - 软约束: 尽量靠近参考点, 权重为 weight_proximity
 *        vii. 低速模式下额外生成 primitive 轨迹 (FrenetPrimitive::Connect)
 *     c. 将成功生成的轨迹、对应的走廊和参考点存入结果列表
 *
 *  Step 5. 轨迹选择 (UpdateTrajectoryWithCurrentBehavior)
 *     a. 在有效轨迹中按优先级查找匹配行为:
 *        优先级1: 精确匹配 ego_behavior_ (如 LeftLaneChange)
 *        优先级2: 回退到 LaneKeeping 行为
 *     a+. (MVP-6 可选) 对 QP 成功候选采样查询 risk grid, 计算 risk exposure
 *        软代价；默认关闭，开启重选时也只在已有可行候选之间选择，不改 QP 约束。
 *     b. 将匹配的 Bezier 样条和 primitive 轨迹分别封装为
 *        FrenetBezierTrajectory 和 FrenetPrimitiveTrajectory
 *     c. 通过 trajectory() 方法：高速返回 Bezier 样条轨迹，
 *        低速返回 primitive 轨迹
 *
 * [性能优化]
 *   - 使用 OpenMP (宏 USE_OPENMP) 可启用 4 线程并行坐标变换
 *   - 默认关闭 OpenMP 以避免多核调度的不确定性
 *   - 每个阶段有独立的 TicToc 计时器输出耗时
 *
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#include "ssc_planner/ssc_planner.h"

#include <algorithm>
#include <unistd.h>
#include <fstream>
#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <iomanip>
#include <omp.h>

// ! 多核任务调度会显著影响性能，默认关闭OpenMP
// 可通过将此宏改为 1 启用4线程并行坐标变换
#define USE_OPENMP 0

namespace planning {

/// @brief 返回规划器名称标识
std::string SscPlanner::Name() { return std::string("ssc_planner"); }

/// @brief 获取最终选中候选对应的 risk grid 快照
const RiskGridMap3D* SscPlanner::selected_risk_grid_snapshot() const {
  if (selected_candidate_index_ < 0 ||
      selected_candidate_index_ >=
          static_cast<int>(candidate_risk_grid_snapshots_.size()) ||
      selected_candidate_index_ >= static_cast<int>(qp_trajs_.size()) ||
      selected_candidate_index_ >= static_cast<int>(valid_behaviors_.size())) {
    return nullptr;
  }
  return &candidate_risk_grid_snapshots_[static_cast<size_t>(
      selected_candidate_index_)];
}

/// @brief 获取原始 baseline 候选对应的 risk grid 快照
const RiskGridMap3D* SscPlanner::baseline_risk_grid_snapshot() const {
  if (baseline_candidate_index_ < 0 ||
      baseline_candidate_index_ >=
          static_cast<int>(candidate_risk_grid_snapshots_.size()) ||
      baseline_candidate_index_ >= static_cast<int>(qp_trajs_.size()) ||
      baseline_candidate_index_ >= static_cast<int>(valid_behaviors_.size())) {
    return nullptr;
  }
  return &candidate_risk_grid_snapshots_[static_cast<size_t>(
      baseline_candidate_index_)];
}

/// @brief 初始化规划器
///
/// 分两步：
///   1. ReadConfig: 从 protobuf 文本文件加载配置
///   2. 创建 SscMap: 将 protobuf 配置映射为 SscMap::Config 并实例化
///
/// @param config_path protobuf 文本格式配置文件路径
ErrorType SscPlanner::Init(const std::string config_path) {
  // 第一步：读取 protobuf 配置
  if (ReadConfig(config_path) != kSuccess) {
    return kWrongStatus;
  }

  // 打印规划器配置参数
  printf("\nSscPlanner Config:\n");
  printf(" -- weight_proximity: %lf\n", cfg_.planner_cfg().weight_proximity());
  printf(" -- enable_risk_exposure_eval: %s\n",
         cfg_.planner_cfg().enable_risk_exposure_eval() ? "true" : "false");
  printf(" -- enable_risk_exposure_reselect: %s\n",
         cfg_.planner_cfg().enable_risk_exposure_reselect() ? "true" : "false");
  printf(" -- risk_exposure_weight: %lf\n",
         cfg_.planner_cfg().risk_exposure_weight());
  printf(" -- enable_adaptive_risk_weight: %s\n",
         cfg_.planner_cfg().enable_adaptive_risk_weight() ? "true" : "false");

  LOG(INFO) << "[Ssc]SscPlanner Config:";
  LOG(INFO) << "[Ssc] -- low spd threshold: "
            << cfg_.planner_cfg().low_speed_threshold();
  LOG(INFO) << "[Ssc] -- weight_proximity: "
            << cfg_.planner_cfg().weight_proximity();
  LOG(INFO) << "[Ssc] -- enable_risk_exposure_eval: "
            << cfg_.planner_cfg().enable_risk_exposure_eval();
  LOG(INFO) << "[Ssc] -- enable_risk_exposure_reselect: "
            << cfg_.planner_cfg().enable_risk_exposure_reselect();
  LOG(INFO) << "[Ssc] -- risk_exposure_weight: "
            << cfg_.planner_cfg().risk_exposure_weight();
  LOG(INFO) << "[Ssc] -- enable_adaptive_risk_weight: "
            << cfg_.planner_cfg().enable_adaptive_risk_weight();
  LOG(INFO) << "[Ssc] -- multimodal_prediction_time: "
            << cfg_.planner_cfg().multimodal_prediction_time();
  LOG(INFO) << "[Ssc] -- multimodal_prediction_step: "
            << cfg_.planner_cfg().multimodal_prediction_step();

  // 第二步：构建 SscMap 配置并从 protobuf 映射参数
  SscMap::Config map_cfg;

  // --- 地图尺寸与分辨率 ---
  map_cfg.map_size[0] = cfg_.map_cfg().map_size_x();          // s轴栅格数
  map_cfg.map_size[1] = cfg_.map_cfg().map_size_y();          // d轴栅格数
  map_cfg.map_size[2] = cfg_.map_cfg().map_size_z();          // t轴栅格数
  map_cfg.map_resolution[0] = cfg_.map_cfg().map_resl_x();    // s分辨率 (m)
  map_cfg.map_resolution[1] = cfg_.map_cfg().map_resl_y();    // d分辨率 (m)
  map_cfg.map_resolution[2] = cfg_.map_cfg().map_resl_z();    // t分辨率 (s)
  map_cfg.s_back_len = cfg_.map_cfg().s_back_len();           // s向后预留

  // --- 运动学约束 (从 protobuf 的 dyn_bounds 子消息读取) ---
  map_cfg.kMaxLongitudinalVel = cfg_.map_cfg().dyn_bounds().max_lon_vel();
  // 最小纵向速度取配置值和速度奇异阈值中的较大值,
  // 避免极小速度导致的数值不稳定
  map_cfg.kMinLongitudinalVel =
      std::max(cfg_.map_cfg().dyn_bounds().min_lon_vel(),
               cfg_.planner_cfg().velocity_singularity_eps());
  map_cfg.kMaxLongitudinalAcc = cfg_.map_cfg().dyn_bounds().max_lon_acc();
  map_cfg.kMaxLongitudinalDecel = cfg_.map_cfg().dyn_bounds().max_lon_dec();
  map_cfg.kMaxLateralVel = cfg_.map_cfg().dyn_bounds().max_lat_vel();
  map_cfg.kMaxLateralAcc = cfg_.map_cfg().dyn_bounds().max_lat_acc();
  map_cfg.kMaxNumOfGridAlongTime = cfg_.map_cfg().max_grids_along_time();

  // --- 六方向膨胀步长 (依次为: s+, s-, d+, d-, t+, t-) ---
  map_cfg.inflate_steps[0] = cfg_.map_cfg().infl_steps().x_p();
  map_cfg.inflate_steps[1] = cfg_.map_cfg().infl_steps().x_n();
  map_cfg.inflate_steps[2] = cfg_.map_cfg().infl_steps().y_p();
  map_cfg.inflate_steps[3] = cfg_.map_cfg().infl_steps().y_n();
  map_cfg.inflate_steps[4] = cfg_.map_cfg().infl_steps().z_p();
  map_cfg.inflate_steps[5] = cfg_.map_cfg().infl_steps().z_n();
  // --- MVP-5 风险感知 corridor 参数 ---
  // 默认配置关闭该功能；打开后只通过 high-risk occupied 收缩 corridor，
  // 不改 QP 目标函数或 control 输出接口。
  map_cfg.enable_risk_aware_corridor =
      cfg_.map_cfg().enable_risk_aware_corridor();
  map_cfg.risk_occupied_threshold =
      static_cast<RiskMapDataType>(std::max(
          0.0, std::min(1.0, cfg_.map_cfg().risk_occupied_threshold())));

  // 创建 SSC 地图实例
  p_ssc_map_ = new SscMap(map_cfg);
  ConfigureMapInterfacePredictionHorizon();

  return kSuccess;
}

/// @brief 从 protobuf 文本文件读取配置
/// @param config_path 配置文件的绝对路径或相对路径
ErrorType SscPlanner::ReadConfig(const std::string config_path) {
  printf("\n[EudmPlanner] Loading ssc planner config\n");
  using namespace google::protobuf;
  int fd = open(config_path.c_str(), O_RDONLY);
  if (fd < 0) {
    LOG(ERROR) << "[Ssc] failed to open config: " << config_path;
    return kWrongStatus;
  }
  io::FileInputStream fstream(fd);
  const bool parse_ok = TextFormat::Parse(&fstream, &cfg_);
  close(fd);
  if (!parse_ok) {
    LOG(ERROR) << "[Ssc] failed to parse config text from " << config_path;
    return kWrongStatus;
  }
  if (!cfg_.IsInitialized()) {
    LOG(ERROR) << "[Ssc] config is not fully initialized: " << config_path;
    return kWrongStatus;
  }
  return kSuccess;
}

/// @brief 将多模态预测时间参数同步到地图接口
/// @note 配置值 <=0 时从 SSC map 时间域推导，避免默认多模态预测 horizon 短于 risk grid。
void SscPlanner::ConfigureMapInterfacePredictionHorizon() {
  if (map_itf_ == nullptr || !cfg_.has_planner_cfg() || !cfg_.has_map_cfg()) {
    return;
  }

  const decimal_t map_horizon =
      std::max<decimal_t>(cfg_.map_cfg().map_resl_z(),
                          cfg_.map_cfg().map_size_z() * cfg_.map_cfg().map_resl_z());
  const decimal_t prediction_time =
      cfg_.planner_cfg().multimodal_prediction_time() > 0.0
          ? cfg_.planner_cfg().multimodal_prediction_time()
          : map_horizon;
  const decimal_t prediction_step =
      cfg_.planner_cfg().multimodal_prediction_step() > 0.0
          ? cfg_.planner_cfg().multimodal_prediction_step()
          : cfg_.map_cfg().map_resl_z();

  map_itf_->ConfigureMultiModalPrediction(prediction_time, prediction_step);
  LOG(INFO) << "[Ssc] multimodal prediction configured time="
            << prediction_time << " step=" << prediction_step;
}

/// @brief 设置规划起始状态（外部接口，用于闭环仿真）
/// 调用后 has_initial_state_ 置为 true，下一帧 RunOnce 将使用此状态而非自车当前状态
ErrorType SscPlanner::set_initial_state(const State& state) {
  initial_state_ = state;
  has_initial_state_ = true;
  return kSuccess;
}

/// @brief 执行一次完整的规划循环 —— SSC 算法的顶层入口
///
/// 完整流程的每一步都有独立的 TicToc 计时器，输出各阶段耗时用于性能分析。
/// 总耗时通过 time_cost_ 记录，各子阶段之和与总耗时的差异 (diff) 反映
/// 步骤间的调度开销。
ErrorType SscPlanner::RunOnce() {
  // 本帧开始前先清空候选 risk grid 状态，避免失败帧继续暴露上一帧快照。
  behavior_risk_grid_snapshots_.clear();
  candidate_risk_grid_snapshots_.clear();
  baseline_candidate_index_ = -1;
  selected_candidate_index_ = -1;

  if (map_itf_ == nullptr || !map_valid_ || !map_itf_->IsValid() ||
      p_ssc_map_ == nullptr) {
    LOG(ERROR) << "[Ssc] RunOnce called before valid map interface or SSC map.";
    return kWrongStatus;
  }

  // 获取地图时间戳作为本帧标识
  stamp_ = map_itf_->GetTimeStamp();
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Ssc]******************** RUNONCE START: " << stamp_
               << " ********************\n";
  static TicToc ssc_timer;
  ssc_timer.tic();

  // ===================================================================
  // 阶段1: 数据准备 - 从地图接口读取环境快照 (prepare)
  // ===================================================================
  static TicToc timer_prepare;
  timer_prepare.tic();

  // 1a. 获取自车车辆信息（状态 + 物理参数）
  if (map_itf_->GetEgoVehicle(&ego_vehicle_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get ego vehicle info.";
    return kWrongStatus;
  }

  // 1b. 确定初始规划状态
  // 若有外部设置的状态 (from Replan) 则使用之, 否则使用自车当前状态
  if (!has_initial_state_) {
    initial_state_ = ego_vehicle_.state();
  }
  has_initial_state_ = false;  // 单次消费后复位

  // 1c. 判断横向独立性 (高速 vs 低速模式)
  // 高速时横向运动可解耦, 利于 QP 优化的可行性
  is_lateral_independent_ =
      initial_state_.velocity > cfg_.planner_cfg().low_speed_threshold()
          ? true
          : false;

  // 1d. 获取局部参考车道 (用于 StateTransformer 初始化)
  if (map_itf_->GetLocalReferenceLane(&nav_lane_local_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to find ego lane.";
    return kWrongStatus;
  }
  // 用参考车道初始化状态变换器 (笛卡尔 <-> Frenet)
  stf_ = common::StateTransformer(nav_lane_local_);

  // 1e. 将初始状态由笛卡尔坐标转换为 Frenet 坐标
  if (stf_.GetFrenetStateFromState(initial_state_, &initial_frenet_state_) !=
      kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get init state frenet state.";
    return kWrongStatus;
  }

  // 1f. 获取自车当前行为决策 (LaneKeeping / LeftLaneChange / RightLaneChange)
  if (map_itf_->GetEgoDiscretBehavior(&ego_behavior_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get ego behavior.";
    return kWrongStatus;
  }

  // 1g. 获取 2D 障碍物占据栅格地图
  if (map_itf_->GetObstacleMap(&grid_map_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get obstacle map.";
    return kWrongStatus;
  }

  // 1h. 获取障碍物占据栅格坐标集合 (离散化的障碍物位置)
  if (map_itf_->GetObstacleGrids(&obstacle_grids_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get obstacle grids.";
    return kWrongStatus;
  }

  // 1i. 获取多行为前向仿真轨迹 + 周围车辆预测轨迹
  if (map_itf_->GetForwardTrajectories(&forward_behaviors_, &forward_trajs_,
                                       &surround_forward_trajs_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get forward trajectories.";
    return kWrongStatus;
  }

  // 1j. MVP-2: 获取周车当前确定性预测轨迹的存在概率。
  //     该概率只写入并行 risk grid，不改变 binary SSC map/corridor/QP。
  if (map_itf_->GetSurroundingTrajectoryExistenceProbabilities(
          &surround_traj_existence_probs_) != kSuccess) {
    LOG(WARNING) << "[Ssc]fail to get surrounding trajectory probabilities, "
                 << "fallback risk existence probability to 1.0.";
    surround_traj_existence_probs_.clear();
  }

  // 1k. MVP-3: 获取周车多模态预测轨迹，仅用于 risk grid。
  //     失败时清空该旁路，后续 risk grid 回退到 MVP-2 单轨迹概率填图。
  behavior_conditioned_multimodal_surround_trajs_.clear();
  if (map_itf_->GetBehaviorConditionedMultiModalSurroundingTrajectories(
          &behavior_conditioned_multimodal_surround_trajs_) == kSuccess) {
    LOG(INFO) << "[Ssc]behavior-conditioned multimodal trajectories enabled, "
              << "ego_behavior_count="
              << behavior_conditioned_multimodal_surround_trajs_.size();
  } else {
    LOG(WARNING) << "[Ssc]fail to get behavior-conditioned multimodal "
                 << "trajectories, fallback to global multimodal risk input.";
    behavior_conditioned_multimodal_surround_trajs_.clear();
  }
  if (map_itf_->GetMultiModalSurroundingTrajectories(
          &multimodal_surround_trajs_) != kSuccess) {
    LOG(WARNING) << "[Ssc]fail to get multimodal surrounding trajectories, "
                 << "fallback risk grid to deterministic trajectories.";
    multimodal_surround_trajs_.clear();
  }

  auto t_prepare = timer_prepare.toc();
  LOG(WARNING) << "[Ssc]prepare time cost: " << t_prepare << " ms";

  // ===================================================================
  // 阶段2: 批量坐标变换 - 笛卡尔 -> Frenet (state transform)
  // ===================================================================
  static TicToc timer_stf;
  timer_stf.tic();
  if (StateTransformForInputData() != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to transform state into ff.";
    return kWrongStatus;
  }
  auto t_stf = timer_stf.toc();
  LOG(WARNING) << "[Ssc]state transform time cost: " << t_stf << " ms";

  // ===================================================================
  // 阶段3: SSC 地图构建 + 时空走廊生成 (SscMap part)
  // 注意: 此阶段有时可能非常慢, 可能与CPU调度有关
  // ===================================================================
  static TicToc timer_sscmap;
  timer_sscmap.tic();

  // 3a. 设置时间原点 (整个规划的时间基线)
  time_origin_ = initial_state_.time_stamp;
  // 重置 SscMap: 清空栅格和走廊, 更新地图原点
  p_ssc_map_->ResetSscMap(initial_frenet_state_);

  // 3b. 对每种行为: 构建地图并生成走廊
  int num_behaviors = forward_behaviors_.size();
  if (forward_trajs_fs_.size() != forward_behaviors_.size() ||
      surround_forward_trajs_fs_.size() != forward_behaviors_.size()) {
    LOG(ERROR) << "[Ssc]transformed trajectory size mismatch, behaviors="
               << static_cast<int>(forward_behaviors_.size())
               << ", ego_forward_fs="
               << static_cast<int>(forward_trajs_fs_.size())
               << ", surround_forward_fs="
               << static_cast<int>(surround_forward_trajs_fs_.size());
    return kWrongStatus;
  }
  // RViz 需要显示 selected/baseline 候选对应的 risk grid，而 SscMap 在每个
  // behavior 构图时会复用并重置同一份地图，因此无论是否开启 risk exposure，
  // 都保存每个 ego behavior 的风险图快照。
  behavior_risk_grid_snapshots_.reserve(static_cast<size_t>(num_behaviors));
  for (int i = 0; i < num_behaviors; ++i) {
    const MultiModalSurroundingFsTrajectories* multimodal_trajs_for_behavior =
        &multimodal_surround_trajs_fs_;
    if (i < static_cast<int>(
                behavior_conditioned_multimodal_surround_trajs_fs_.size())) {
      // MVP-10: 每个 ego candidate 优先使用自己对应的周车多模态风险场；
      // 若该行为没有条件化结果，则回退到全局多模态旁路。
      multimodal_trajs_for_behavior =
          &behavior_conditioned_multimodal_surround_trajs_fs_[i];
    }

    // 3b-i. 若非仅拟合模式 (is_fitting_only=false), 构建时空占据地图
    if (!cfg_.planner_cfg().is_fitting_only()) {
      // 将当前行为下的周围车辆轨迹和静态障碍物栅格写入 3D 栅格
      if (p_ssc_map_->ConstructSscMap(surround_forward_trajs_fs_[i],
                                      obstacle_grids_fs_,
                                      surround_traj_existence_probs_,
                                      *multimodal_trajs_for_behavior,
                                      stamp_, i,
                                      common::SemanticsUtils::RetLatBehaviorName(
                                          forward_behaviors_[i]))) {
        LOG(ERROR) << "[Ssc]fail to construct ssc map.";
        return kWrongStatus;
      }
    }

    // MVP-6/10: 每个行为的风险图在下一轮 ConstructSscMap 时会被清空重建，
    // 因此必须在当前行为构图后立即保存快照，供候选 exposure 与 RViz 使用。
    if (cfg_.planner_cfg().is_fitting_only()) {
      behavior_risk_grid_snapshots_.push_back(RiskGridMap3D());
    } else {
      behavior_risk_grid_snapshots_.push_back(p_ssc_map_->risk_grid());
    }

    // 3b-ii. (可选) 障碍物膨胀 — EUDM 项目中为节省时间而省略
    // InflateObstacleGrid 会根据车辆尺寸对障碍物进行膨胀
    // p_ssc_map_->InflateObstacleGrid(ego_vehicle_.param());

    // 3b-iii. 沿该行为的前向轨迹构建时空走廊
    // 核心：从初始轨迹采样种子，膨胀为无碰撞立方体序列
    if (p_ssc_map_->ConstructCorridorUsingInitialTrajectory(
            p_ssc_map_->p_3d_grid(), forward_trajs_fs_[i]) != kSuccess) {
      LOG(ERROR) << "[Ssc]fail to construct corridor for behavior " << i;
      return kWrongStatus;
    }
  }

  // 3c. 将每种行为的走廊从栅格坐标转换为物理单位 (米, 秒)
  if (kSuccess != p_ssc_map_->GetFinalGlobalMetricCubesList()) {
    LOG(ERROR) << "[Ssc]fail to get final corridor";
    return kWrongStatus;
  }

  auto t_sscmap = timer_sscmap.toc();
  LOG(WARNING) << "[Ssc]construct ssc map and corridor time cost: " << t_sscmap
               << " ms";

  // ===================================================================
  // 阶段4: QP 轨迹优化 (RunQpOptimization)
  // ===================================================================
  static TicToc timer_opt;
  timer_opt.tic();

  // 4a. 运行 QP 优化: 在时空走廊约束内拟合光滑 Bezier 样条轨迹
  if (RunQpOptimization() != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to optimize qp trajectories.\n";
    return kWrongStatus;
  }

  // 4b. 从优化结果中选择与当前行为匹配的轨迹
  if (UpdateTrajectoryWithCurrentBehavior() != kSuccess) {
    LOG(ERROR) << "[Ssc]fail: current behavior "
               << static_cast<int>(ego_behavior_) << " not valid.";
    LOG(ERROR) << "[Ssc]fail: has " << qp_trajs_.size() << " traj, "
               << valid_behaviors_.size() << " behaviors.";
    return kWrongStatus;
  }

  // 4c. (已禁用) 轨迹验证: 检查起点、终点一致性和曲率约束
  // #if 0 块内的 ValidateTrajectory 检查因可能导致过度严格的拒绝而默认关闭
#if 0
  auto traj = trajectory();
  if (ValidateTrajectory(*traj) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail: infeasible traj.";
    return kWrongStatus;
  }
#endif

  auto t_opt = timer_opt.toc();
  LOG(WARNING) << "[Ssc]optimization time cost: " << t_opt << " ms";

  // ===================================================================
  // 汇总计时: 输出各阶段耗时和调度开销
  // ===================================================================
  auto t_sum = t_prepare + t_stf + t_sscmap + t_opt;
  time_cost_ = ssc_timer.toc();
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Ssc]Sum of time: " << t_sum
               << " ms, diff: " << time_cost_ - t_sum << " ms";
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Ssc]******************** RUNONCE FINISH: " << stamp_ << " +"
               << time_cost_ << " ms ********************\n";

  return kSuccess;
}  // namespace planning

/// @brief QP 轨迹优化 —— 在时空走廊约束内拟合光滑轨迹的核心函数
///
/// 算法详解:
///   对每种行为对应的时空走廊，调用 SplineGenerator 生成 5 阶 2 维 Bezier 样条。
///   Bezier 样条定义在归一化参数区间 [0,1] 上，实际时间线性映射到该区间。
///   优化问题形式化为:
///     min  w_j * ∫(jerk^2) + w_a * ∫(acc^2) + w_p * Σ||p(t_k) - ref_k||^2
///     s.t. 对于所有 t_k ∈ [t_start, t_end]:
///             p_lb(t_k) ≤ p(t_k) ≤ p_ub(t_k)  (位置在走廊内)
///             v_lb(t_k) ≤ v(t_k) ≤ v_ub(t_k)  (速度边界)
///             a_lb(t_k) ≤ a(t_k) ≤ a_ub(t_k)  (加速度边界)
///           起始状态硬约束
///             p(t_start) = p_start, v(t_start) = v_start, a(t_start) = a_start
///
/// @return kSuccess 表示至少有一条行为生成了有效的优化轨迹
ErrorType SscPlanner::RunQpOptimization() {
  // 获取各行为的物理坐标时空走廊和有效性标志
  vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> cube_list =
      p_ssc_map_->final_corridor_vec();
  std::vector<int> if_corridor_valid = p_ssc_map_->if_corridor_valid();

  if (cube_list.empty()) return kWrongStatus;

  // 一致性检查: 走廊、行为、有效标志和前向轨迹必须同维，避免异常输入越界。
  if (cube_list.size() != forward_behaviors_.size() ||
      if_corridor_valid.size() != cube_list.size() ||
      forward_trajs_fs_.size() != cube_list.size() ||
      forward_trajs_.size() != cube_list.size()) {
    LOG(ERROR) << "[Ssc]cube list " << static_cast<int>(cube_list.size())
               << " not consist with behavior size: "
               << static_cast<int>(forward_behaviors_.size())
               << ", forward traj " << static_cast<int>(forward_trajs_.size())
               << ", forward traj fs "
               << static_cast<int>(forward_trajs_fs_.size())
               << ", flag size " << static_cast<int>(if_corridor_valid.size());
    return kWrongStatus;
  }

  // 清空上一帧优化结果
  qp_trajs_.clear();
  primitive_trajs_.clear();
  valid_behaviors_.clear();
  corridors_.clear();
  ref_states_list_.clear();
  candidate_risk_grid_snapshots_.clear();
  baseline_candidate_index_ = -1;
  selected_candidate_index_ = -1;

  // 遍历每种行为的走廊，逐一优化
  for (int i = 0; i < static_cast<int>(cube_list.size()); i++) {
    // 跳过无效走廊
    if (if_corridor_valid[i] == 0) {
      LOG(ERROR) << "[Ssc]fail: for behavior "
                 << static_cast<int>(forward_behaviors_[i])
                 << " has no valid corridor.";
      continue;
    }

    auto fs_vehicle_traj = forward_trajs_fs_[i];
    int num_states = static_cast<int>(fs_vehicle_traj.size());
    if (num_states < 1) {
      LOG(ERROR) << "[Ssc]empty forward trajectory for behavior "
                 << static_cast<int>(forward_behaviors_[i]);
      continue;
    }

    // ===================================================================
    // 步骤1: 构建起始约束 (3组 Vecf<2>)
    //   约束0: 起始位置    (s, d)
    //   约束1: 起始速度    (s_dot, d_dot)，纵向速度做奇异保护
    //   约束2: 起始加速度  (s_ddot, d_ddot)
    // ===================================================================
    vec_E<Vecf<2>> start_constraints;
    // 起始位置约束
    start_constraints.push_back(
        Vecf<2>(ego_frenet_state_.vec_s[0], ego_frenet_state_.vec_dt[0]));
    // 起始速度约束: 纵向速度使用 max(v, eps) 防止零速奇异性
    start_constraints.push_back(
        Vecf<2>(std::max(ego_frenet_state_.vec_s[1],
                         cfg_.planner_cfg().velocity_singularity_eps()),
                ego_frenet_state_.vec_dt[1]));
    // 起始加速度约束
    start_constraints.push_back(
        Vecf<2>(ego_frenet_state_.vec_s[2], ego_frenet_state_.vec_dt[2]));

    // ===================================================================
    // 步骤2: 构建终止约束 (2组 Vecf<2>)
    //   约束0: 终止位置    (s, d) = 前向轨迹最后一帧的位置
    //   约束1: 终止速度    (s_dot, d_dot) = 前向轨迹最后一帧的速度
    //   注意: 终止加速度约束被注释掉，仅以位置和速度作为软目标
    // ===================================================================
    vec_E<Vecf<2>> end_constraints;
    // 终止位置约束
    end_constraints.push_back(
        Vecf<2>(fs_vehicle_traj[num_states - 1].frenet_state.vec_s[0],
                fs_vehicle_traj[num_states - 1].frenet_state.vec_dt[0]));
    // 终止速度约束 (同样做奇异保护)
    end_constraints.push_back(
        Vecf<2>(std::max(fs_vehicle_traj[num_states - 1].frenet_state.vec_s[1],
                         cfg_.planner_cfg().velocity_singularity_eps()),
                fs_vehicle_traj[num_states - 1].frenet_state.vec_dt[1]));

    // 实例化 5阶2维样条生成器
    common::SplineGenerator<5, 2> spline_generator;
    BezierSpline bezier_spline;

    // 修正走廊终止时间: 使用前向轨迹的实际终端时间
    cube_list[i].back().t_ub = fs_vehicle_traj.back().frenet_state.time_stamp;

    // 检查走廊可行性: 验证相邻立方体时间轴是否连续可通
    if (CorridorFeasibilityCheck(cube_list[i]) != kSuccess) {
      LOG(ERROR) << "[Ssc]fail: corridor not valid for optimization.";
      continue;
    }

    // ===================================================================
    // 步骤3: 构建优化参考点序列
    //   以初始前向轨迹的各帧 Frenet 状态作为优化的软参考目标。
    //   优化器会尽量使结果靠近这些参考点 (受 weight_proximity 权重控制)。
    // ===================================================================
    std::vector<decimal_t> ref_stamps;       // 参考时间戳序列
    vec_E<Vecf<2>> ref_points;               // 参考 (s, d) 位置序列
    vec_E<common::FrenetState> ref_states;   // 参考 Frenet 状态序列
    for (int n = 0; n < num_states; n++) {
      ref_stamps.push_back(fs_vehicle_traj[n].frenet_state.time_stamp);
      ref_points.push_back(Vecf<2>(fs_vehicle_traj[n].frenet_state.vec_s[0],
                                   fs_vehicle_traj[n].frenet_state.vec_dt[0]));
      ref_states.push_back(fs_vehicle_traj[n].frenet_state);
    }

    // ===================================================================
    // 步骤4: 调用 Bezier 样条生成器 — 核心的 QP 优化过程
    //
    // GetBezierSplineUsingCorridor 内部执行:
    //   a. 根据时间区间构建归一化参数映射
    //   b. 通过 QP 求解最优 Bezier 控制点
    //   c. 约束采样: 在每段时间区间内采样多个配置点
    //   d. 目标函数: w_j*Σ||jerk||^2 + w_a*Σ||acc||^2 + w_p*Σ||p-ref||^2
    //   e. 返回优化结果 bezier_spline
    // ===================================================================
    bool bezier_spline_gen_success = true;
    if (spline_generator.GetBezierSplineUsingCorridor(
            cube_list[i], start_constraints, end_constraints, ref_stamps,
            ref_points, cfg_.planner_cfg().weight_proximity(),
            &bezier_spline) != kSuccess) {
      // QP 优化失败时的详细诊断日志输出
      if (is_lateral_independent_) {
        LOG(ERROR) << "[Ssc]fail: solver error for behavior "
                   << static_cast<int>(forward_behaviors_[i]);
        // 打印完整走廊信息供调试
        decimal_t t0 = cube_list[i].front().t_lb;
        for (auto& cube : cube_list[i]) {
          LOG(ERROR) << std::fixed << std::setprecision(3) << "[Ssc] t: ["
                     << cube.t_lb - t0 << ", " << cube.t_ub - t0 << "], x: ["
                     << cube.p_lb[0] << ", " << cube.p_ub[0] << "], y: ["
                     << cube.p_lb[1] << ", " << cube.p_ub[1] << "]";
        }
        // 打印参考点序列
        LOG(ERROR) << "[Ssc]ref points: ";
        for (size_t k = 0; k < ref_stamps.size(); ++k) {
          LOG(ERROR) << std::fixed << std::setprecision(4) << "[Ssc]" << k
                     << " t: " << ref_stamps[k] << ", x: " << ref_points[k].x()
                     << ", y: " << ref_points[k].y();
        }
        // 打印全局坐标下的前向轨迹
        LOG(ERROR) << "[Ssc]forward traj: ";
        for (size_t k = 0; k < forward_trajs_[i].size(); ++k) {
          auto v = forward_trajs_[i][k];
          LOG(ERROR) << std::fixed << std::setprecision(4) << "[Ssc]" << k
                     << " t: " << v.state().time_stamp
                     << ", x: " << v.state().vec_position.x()
                     << ", y: " << v.state().vec_position.y()
                     << ", v: " << v.state().velocity;
        }
        LOG(ERROR) << "[Ssc]ref lane range: [" << nav_lane_local_.begin()
                   << ", " << nav_lane_local_.end() << "]";

        // 打印起始终止约束的具体数值
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]Start sd velocity (" << start_constraints[1](0)
                   << ", " << start_constraints[1](1) << ")";
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]Start sd acceleration (" << start_constraints[2](0)
                   << ", " << start_constraints[2](1) << ")";
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]End sd position (" << end_constraints[0](0) << ", "
                   << end_constraints[0](1) << ")";
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]End sd velocity (" << end_constraints[1](0) << ", "
                   << end_constraints[1](1) << ")";
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]End state stamp: "
                   << fs_vehicle_traj[num_states - 1].frenet_state.time_stamp;
      }
      bezier_spline_gen_success = false;
    }

    // ===================================================================
    // 步骤5: 低速模式下生成备选 primitive 轨迹
    //   primitive 轨迹通过 Connect 方法连接当前 Frenet 状态到目标状态
    // ===================================================================
    FrenetPrimitive primitive;
    if (!is_lateral_independent_) {
      // 低速模式：使用横向依赖的 primitive 连接
      primitive.Connect(initial_frenet_state_,
                        fs_vehicle_traj.back().frenet_state,
                        initial_frenet_state_.time_stamp,
                        fs_vehicle_traj.back().frenet_state.time_stamp -
                            initial_frenet_state_.time_stamp,
                        is_lateral_independent_);
    }

    // 高速模式且 Bezier 生成失败: 跳过此行为
    if (is_lateral_independent_ && !bezier_spline_gen_success) continue;

    // 保存成功结果
    qp_trajs_.push_back(bezier_spline);
    primitive_trajs_.push_back(primitive);
    corridors_.push_back(cube_list[i]);
    ref_states_list_.push_back(ref_states);
    valid_behaviors_.push_back(forward_behaviors_[i]);
    if (i < static_cast<int>(behavior_risk_grid_snapshots_.size())) {
      candidate_risk_grid_snapshots_.push_back(
          behavior_risk_grid_snapshots_[static_cast<size_t>(i)]);
    } else {
      candidate_risk_grid_snapshots_.push_back(RiskGridMap3D());
    }
  }

  return kSuccess;
}

/// @brief 根据当前自车行为从有效轨迹中选择匹配的轨迹
///
/// 选择策略 (两级回退):
///   Level 1: 精确匹配 ego_behavior_ (如自车正在左变道, 优先选左变道轨迹)
///   Level 2: 若无精确匹配, 回退到 kLaneKeeping 行为 (安全性兜底)
///
/// 结果:
///   - trajectory_:               选中的 Bezier 样条轨迹
///   - low_spd_alternative_traj_: 选中的备选 primitive 轨迹
///   - final_corridor_:           选中轨迹对应的时空走廊
///   - final_ref_states_:         选中轨迹的参考状态
///
/// @return 若有效轨迹为空且无回退行为, 返回 kWrongStatus
ErrorType SscPlanner::UpdateTrajectoryWithCurrentBehavior() {
  int num_valid_behaviors = static_cast<int>(valid_behaviors_.size());
  if (num_valid_behaviors < 1) {
    return kWrongStatus;
  }

  // 先按原始 SSC 规则得到 baseline 候选。MVP-6 默认关闭时，最终仍使用该索引。
  const int baseline_index = FindBaselineTrajectoryIndex();
  if (baseline_index < 0) {
    return kWrongStatus;
  }
  baseline_candidate_index_ = baseline_index;

  int index = baseline_index;
  std::vector<RiskExposureMetrics> risk_metrics;
  AdaptiveRiskWeightContext risk_context = BuildBaseAdaptiveRiskWeightContext();
  SafetyFallbackResult fallback_result;
  if (ShouldComputeRiskExposureMetrics()) {
    EnsureRiskExposureMetrics(num_valid_behaviors, risk_context, &risk_metrics);
    UpdateAdaptiveRiskWeightContextByMetrics(risk_metrics, &risk_context);
    for (auto& metric : risk_metrics) {
      metric.risk_score = risk_context.risk_weight * metric.exposure_sum;
    }
    if (IsRiskExposureEvaluationEnabled()) {
      index = SelectRiskAwareTrajectoryIndex(
          baseline_index, risk_metrics, risk_context);
    }
    fallback_result = ApplySafetyFallbackIfNeeded(&index, risk_metrics);
    if (fallback_result.triggered) {
      LOG(WARNING) << "[Ssc][MVP8SafetyFallback] triggered=true"
                   << " switched=" << fallback_result.switched
                   << " original_index=" << fallback_result.original_index
                   << " fallback_index=" << fallback_result.fallback_index
                   << " original_max_risk="
                   << fallback_result.original_max_risk
                   << " fallback_max_risk="
                   << fallback_result.fallback_max_risk;
    }
    AppendRiskExposureMetricsToCsv(
        risk_metrics, baseline_index, index, risk_context, fallback_result);
  }

  // 封装最终轨迹: 高速用 Bezier, 低速用 primitive
  trajectory_ = FrenetBezierTrajectory(qp_trajs_[index], stf_);
  low_spd_alternative_traj_ =
      FrenetPrimitiveTrajectory(primitive_trajs_[index], stf_);
  final_corridor_ = corridors_[index];
  final_ref_states_ = ref_states_list_[index];
  selected_candidate_index_ = index;

  return kSuccess;
}

/// @brief 按原始 SSC 两级策略查找候选轨迹索引
/// @return 精确匹配 ego_behavior_ 或 LaneKeeping 回退的索引，找不到时返回 -1
/// @note 该函数抽出原有逻辑，确保 MVP-6 风险重选关闭时不改变 baseline 行为。
int SscPlanner::FindBaselineTrajectoryIndex() const {
  const int num_valid_behaviors = static_cast<int>(valid_behaviors_.size());
  int index = -1;

  // Level 1: 精确匹配当前行为。若存在多个同类候选，沿用原实现的“后出现覆盖”语义。
  for (int i = 0; i < num_valid_behaviors; i++) {
    if (valid_behaviors_[i] == ego_behavior_) {
      index = i;
    }
  }
  if (index >= 0) {
    return index;
  }

  // Level 2: 精确行为不可用时，回退到 LaneKeeping 作为安全候选。
  const LateralBehavior candidate_behavior = common::LateralBehavior::kLaneKeeping;
  for (int i = 0; i < num_valid_behaviors; i++) {
    if (valid_behaviors_[i] == candidate_behavior) {
      index = i;
    }
  }

  return index;
}

/// @brief 计算单条候选执行轨迹的风险暴露
/// @param candidate_index qp_trajs_ / valid_behaviors_ 中的候选索引
/// @return 风险暴露统计
/// @note MVP-6: 沿候选最终执行轨迹采样 (s,d,t)，从 risk grid 查询风险。
///       高速模式采样 Bezier 轨迹，低速模式采样 primitive 轨迹，避免风险重选
///       和 safety fallback 评价的轨迹与实际输出轨迹不一致。这里不改变轨迹、
///       不把风险写回 QP 目标或 corridor 约束。
SscPlanner::RiskExposureMetrics SscPlanner::ComputeRiskExposureForCandidate(
    const int candidate_index, const decimal_t high_risk_threshold) const {
  RiskExposureMetrics metrics;
  metrics.candidate_index = candidate_index;
  if (candidate_index < 0 || candidate_index >= static_cast<int>(qp_trajs_.size()) ||
      candidate_index >= static_cast<int>(valid_behaviors_.size()) || !p_ssc_map_) {
    return metrics;
  }

  metrics.behavior = valid_behaviors_[candidate_index];
  const FrenetBezierTrajectory bezier_traj(qp_trajs_[candidate_index], stf_);
  const FrenetPrimitiveTrajectory primitive_traj(primitive_trajs_[candidate_index],
                                                stf_);
  const common::FrenetTrajectory* candidate_traj =
      is_lateral_independent_
          ? static_cast<const common::FrenetTrajectory*>(&bezier_traj)
          : static_cast<const common::FrenetTrajectory*>(&primitive_traj);
  if (candidate_traj == nullptr || !candidate_traj->IsValid()) {
    return metrics;
  }

  const decimal_t begin_t = candidate_traj->begin();
  const decimal_t end_t = candidate_traj->end();

  // 采样步长做下限保护，避免配置为 0 或负数时进入死循环。
  const decimal_t sample_dt =
      std::max<decimal_t>(cfg_.planner_cfg().risk_exposure_sample_dt(), 1.0e-3);
  const decimal_t high_threshold = std::max<decimal_t>(0.0, high_risk_threshold);

  for (decimal_t t = begin_t; t <= end_t + kEPS; t += sample_dt) {
    const decimal_t query_t = std::min(t, end_t);
    common::FrenetState fs;
    if (candidate_traj->GetFrenetState(query_t, &fs) != kSuccess) {
      continue;
    }

    RiskMapDataType risk = 0.0f;
    if (candidate_index <
        static_cast<int>(candidate_risk_grid_snapshots_.size())) {
      risk = p_ssc_map_->QueryRiskByMetricPositionInGrid(
          candidate_risk_grid_snapshots_[static_cast<size_t>(candidate_index)],
          fs.vec_s[0], fs.vec_dt[0], query_t);
    } else {
      risk = p_ssc_map_->QueryRiskByMetricPosition(fs.vec_s[0], fs.vec_dt[0],
                                                   query_t);
    }
    metrics.exposure_sum += static_cast<decimal_t>(risk);
    metrics.exposure_max =
        std::max(metrics.exposure_max, static_cast<decimal_t>(risk));
    if (static_cast<decimal_t>(risk) > high_threshold) {
      ++metrics.high_risk_hits;
    }
    ++metrics.sample_count;
  }

  if (metrics.sample_count > 0) {
    metrics.exposure_mean =
        metrics.exposure_sum / static_cast<decimal_t>(metrics.sample_count);
  }

  metrics.risk_score =
      std::max<decimal_t>(0.0, cfg_.planner_cfg().risk_exposure_weight()) *
      metrics.exposure_sum;
  return metrics;
}

/// @brief 根据风险软代价选择候选轨迹
/// @param baseline_index 原始 SSC 选择的候选索引
/// @param metrics 所有候选轨迹的风险暴露统计
/// @return 最终候选轨迹索引
/// @note 重选是 soft ranking：只在 QP 已成功的候选集合中比较，不制造新轨迹。
int SscPlanner::SelectRiskAwareTrajectoryIndex(
    const int baseline_index,
    const std::vector<RiskExposureMetrics>& metrics,
    const AdaptiveRiskWeightContext& risk_context) const {
  if (!cfg_.planner_cfg().enable_risk_exposure_reselect() ||
      risk_context.risk_weight <= 0.0 || baseline_index < 0 ||
      baseline_index >= static_cast<int>(metrics.size())) {
    return baseline_index;
  }

  int risk_selected_index = baseline_index;
  decimal_t best_score = metrics[baseline_index].risk_score;
  for (const auto& metric : metrics) {
    if (metric.candidate_index < 0 || metric.sample_count == 0) {
      continue;
    }
    if (metric.risk_score < best_score) {
      best_score = metric.risk_score;
      risk_selected_index = metric.candidate_index;
    }
  }

  const decimal_t baseline_score = metrics[baseline_index].risk_score;
  const decimal_t switch_margin = std::max<decimal_t>(0.0, risk_context.switch_margin);
  if (risk_selected_index != baseline_index &&
      baseline_score - best_score > switch_margin) {
    LOG(WARNING) << "[Ssc][MVP6RiskReselect] baseline_behavior="
                 << common::SemanticsUtils::RetLatBehaviorName(
                        valid_behaviors_[baseline_index])
                 << " selected_behavior="
                 << common::SemanticsUtils::RetLatBehaviorName(
                        valid_behaviors_[risk_selected_index])
                 << " baseline_score=" << baseline_score
                 << " selected_score=" << best_score
                 << " switch_margin=" << switch_margin;
    return risk_selected_index;
  }

  LOG(WARNING) << "[Ssc][MVP6RiskReselect] keep_baseline_behavior="
               << common::SemanticsUtils::RetLatBehaviorName(
                      valid_behaviors_[baseline_index])
               << " baseline_score=" << baseline_score
               << " best_score=" << best_score
               << " switch_margin=" << switch_margin;
  return baseline_index;
}

/// @brief 将候选轨迹风险暴露统计追加写入 CSV
/// @param metrics 所有 QP 成功候选的风险暴露统计
/// @param baseline_index 原始 SSC 选择的候选索引
/// @param selected_index MVP-6 最终选择的候选索引
/// @note CSV 用于论文实验复现；写入失败只记录 warning，不中断规划。
void SscPlanner::AppendRiskExposureMetricsToCsv(
    const std::vector<RiskExposureMetrics>& metrics,
    const int baseline_index, const int selected_index,
    const AdaptiveRiskWeightContext& risk_context,
    const SafetyFallbackResult& fallback_result) const {
  const std::string csv_path = cfg_.planner_cfg().risk_exposure_csv_path();
  if (csv_path.empty()) {
    return;
  }

  bool csv_file_empty = true;
  {
    std::ifstream existing_file(csv_path);
    csv_file_empty =
        (!existing_file.good()) || (existing_file.peek() == std::ifstream::traits_type::eof());
  }

  std::ofstream csv_file(csv_path, std::ofstream::out | std::ofstream::app);
  if (!csv_file.is_open()) {
    LOG(WARNING) << "[Ssc][MVP6RiskExposureCsv] failed to open " << csv_path;
    return;
  }

  if (csv_file_empty) {
    csv_file << "cycle,stamp,candidate_index,behavior,is_baseline,is_selected,"
                "sample_count,exposure_sum,exposure_mean,exposure_max,"
                "high_risk_hits,risk_score,adaptive_enabled,high_speed,"
                "lane_change,high_interaction_risk,adaptive_risk_weight,"
                "adaptive_switch_margin,adaptive_high_risk_threshold,"
                "adaptive_max_candidate_risk,adaptive_applied_scale,"
                "safety_fallback_triggered,safety_fallback_switched,"
                "safety_original_index,safety_fallback_index,"
                "safety_original_max_risk,safety_fallback_max_risk\n";
    risk_exposure_csv_header_written_ = true;
  } else if (!risk_exposure_csv_header_written_) {
    // 文件已有内容时认为 header 已存在，避免重复写入表头。
    risk_exposure_csv_header_written_ = true;
  }

  const size_t current_cycle = risk_exposure_cycle_count_;
  ++risk_exposure_cycle_count_;
  for (const auto& metric : metrics) {
    const bool is_baseline = metric.candidate_index == baseline_index;
    const bool is_selected = metric.candidate_index == selected_index;
    LOG(WARNING) << "[Ssc][MVP6RiskExposure] behavior="
                 << common::SemanticsUtils::RetLatBehaviorName(metric.behavior)
                 << " candidate_index=" << metric.candidate_index
                 << " sample_count=" << metric.sample_count
                 << " exposure_sum=" << metric.exposure_sum
                 << " exposure_mean=" << metric.exposure_mean
                 << " exposure_max=" << metric.exposure_max
                 << " high_risk_hits=" << metric.high_risk_hits
                 << " risk_score=" << metric.risk_score
                 << " adaptive_enabled=" << risk_context.enabled
                 << " high_speed=" << risk_context.high_speed
                 << " lane_change=" << risk_context.lane_change
                 << " high_interaction_risk="
                 << risk_context.high_interaction_risk
                 << " adaptive_risk_weight=" << risk_context.risk_weight
                 << " adaptive_scale=" << risk_context.applied_scale
                 << " safety_fallback_triggered="
                 << fallback_result.triggered
                 << " safety_fallback_switched="
                 << fallback_result.switched
                 << " is_baseline=" << is_baseline
                 << " is_selected=" << is_selected;

    csv_file << current_cycle << "," << std::fixed << std::setprecision(4)
             << stamp_ << "," << metric.candidate_index << ","
             << common::SemanticsUtils::RetLatBehaviorName(metric.behavior) << ","
             << (is_baseline ? 1 : 0) << "," << (is_selected ? 1 : 0) << ","
             << metric.sample_count << "," << metric.exposure_sum << ","
             << metric.exposure_mean << "," << metric.exposure_max << ","
             << metric.high_risk_hits << "," << metric.risk_score << ","
             << (risk_context.enabled ? 1 : 0) << ","
             << (risk_context.high_speed ? 1 : 0) << ","
             << (risk_context.lane_change ? 1 : 0) << ","
             << (risk_context.high_interaction_risk ? 1 : 0) << ","
             << risk_context.risk_weight << ","
             << risk_context.switch_margin << ","
             << risk_context.high_risk_threshold << ","
             << risk_context.max_candidate_risk << ","
             << risk_context.applied_scale << ","
             << (fallback_result.triggered ? 1 : 0) << ","
             << (fallback_result.switched ? 1 : 0) << ","
             << fallback_result.original_index << ","
             << fallback_result.fallback_index << ","
             << fallback_result.original_max_risk << ","
             << fallback_result.fallback_max_risk << "\n";
  }

  if (!csv_file.good()) {
    LOG(WARNING) << "[Ssc][MVP6RiskExposureCsv] failed to write " << csv_path;
  }
}

/// @brief 判断是否启用 MVP-6 候选轨迹风险暴露评价
/// @return true 表示需要计算 exposure 并写入评价链路
/// @note risk grid 快照现在还服务 RViz selected-candidate 可视化，因此即使
///       eval/reselect 关闭也会保存快照；默认关闭时仍不采样、不写 exposure CSV，
///       轨迹选择直接退回原始 SSC baseline。
bool SscPlanner::IsRiskExposureEvaluationEnabled() const {
  return cfg_.planner_cfg().enable_risk_exposure_eval() ||
         cfg_.planner_cfg().enable_risk_exposure_reselect();
}

/// @brief 判断当前周期是否需要计算候选风险暴露
/// @return true 表示 MVP-6/7 风险评价链路或 MVP-8 兜底过滤器需要 risk metrics
/// @note safety fallback 默认关闭，因此不会改变 baseline/MVP-7 默认开销。
bool SscPlanner::ShouldComputeRiskExposureMetrics() const {
  return IsRiskExposureEvaluationEnabled() ||
         cfg_.planner_cfg().enable_safety_fallback();
}

/// @brief 按需计算候选轨迹风险暴露
/// @param num_valid_behaviors 当前 QP 成功候选数量
/// @param risk_context 风险参数上下文
/// @param risk_metrics 输出风险统计，索引与 qp_trajs_ / valid_behaviors_ 对齐
void SscPlanner::EnsureRiskExposureMetrics(
    const int num_valid_behaviors,
    const AdaptiveRiskWeightContext& risk_context,
    std::vector<RiskExposureMetrics>* risk_metrics) const {
  if (!risk_metrics || !risk_metrics->empty()) {
    return;
  }
  risk_metrics->reserve(static_cast<size_t>(std::max(0, num_valid_behaviors)));
  for (int i = 0; i < num_valid_behaviors; ++i) {
    risk_metrics->push_back(
        ComputeRiskExposureForCandidate(i, risk_context.high_risk_threshold));
  }
}

/// @brief 按风险阈值执行 MVP-8 安全兜底过滤
/// @param selected_index 输入/输出候选索引
/// @param metrics 候选风险暴露统计
/// @return 兜底过滤结果
/// @note 该函数只在已有 QP 成功候选内切换，不返回规划失败，也不直接控制制动。
SscPlanner::SafetyFallbackResult SscPlanner::ApplySafetyFallbackIfNeeded(
    int* selected_index, const std::vector<RiskExposureMetrics>& metrics) const {
  SafetyFallbackResult result;
  if (!selected_index || !cfg_.planner_cfg().enable_safety_fallback()) {
    return result;
  }

  const int current_index = *selected_index;
  if (current_index < 0 || current_index >= static_cast<int>(metrics.size())) {
    return result;
  }

  const RiskExposureMetrics& selected_metric =
      metrics[static_cast<size_t>(current_index)];
  result.original_index = current_index;
  result.fallback_index = current_index;
  result.original_max_risk = selected_metric.exposure_max;
  result.fallback_max_risk = selected_metric.exposure_max;

  const decimal_t max_risk_threshold = std::max<decimal_t>(
      0.0, cfg_.planner_cfg().safety_fallback_max_risk_threshold());
  const decimal_t exposure_sum_threshold = std::max<decimal_t>(
      0.0, cfg_.planner_cfg().safety_fallback_exposure_sum_threshold());
  const int high_risk_hits_threshold =
      std::max(0, cfg_.planner_cfg().safety_fallback_high_risk_hits_threshold());

  const bool trigger_by_max_risk =
      selected_metric.exposure_max > max_risk_threshold;
  const bool trigger_by_sum =
      selected_metric.exposure_sum > exposure_sum_threshold;
  // high_risk_hits_threshold == 0 表示关闭该触发条件，避免配置为 0 时
  // 因 high_risk_hits >= 0 恒成立而让 safety fallback 每帧都触发。
  const bool trigger_by_hits =
      high_risk_hits_threshold > 0 &&
      static_cast<int>(selected_metric.high_risk_hits) >=
          high_risk_hits_threshold;
  result.triggered = trigger_by_max_risk || trigger_by_sum || trigger_by_hits;
  if (!result.triggered) {
    return result;
  }

  const decimal_t min_reduction = std::max<decimal_t>(
      0.0, cfg_.planner_cfg().safety_fallback_min_risk_reduction());
  const bool prefer_lane_keeping =
      cfg_.planner_cfg().safety_fallback_prefer_lane_keeping();

  int best_index = current_index;
  decimal_t best_max_risk = selected_metric.exposure_max;
  size_t best_hits = selected_metric.high_risk_hits;
  decimal_t best_sum = selected_metric.exposure_sum;

  auto consider_candidate = [&](const RiskExposureMetrics& metric) {
    if (metric.candidate_index < 0 || metric.sample_count == 0) {
      return;
    }
    if (metric.exposure_max < best_max_risk ||
        (metric.exposure_max == best_max_risk &&
         metric.high_risk_hits < best_hits) ||
        (metric.exposure_max == best_max_risk &&
         metric.high_risk_hits == best_hits &&
         metric.exposure_sum < best_sum)) {
      best_index = metric.candidate_index;
      best_max_risk = metric.exposure_max;
      best_hits = metric.high_risk_hits;
      best_sum = metric.exposure_sum;
    }
  };

  if (prefer_lane_keeping) {
    for (const auto& metric : metrics) {
      if (metric.behavior == common::LateralBehavior::kLaneKeeping) {
        consider_candidate(metric);
      }
    }
  }
  if (best_index == current_index) {
    for (const auto& metric : metrics) {
      consider_candidate(metric);
    }
  }

  result.fallback_index = best_index;
  result.fallback_max_risk = best_max_risk;
  if (best_index != current_index &&
      selected_metric.exposure_max - best_max_risk >= min_reduction) {
    *selected_index = best_index;
    result.switched = true;
  }

  return result;
}

/// @brief 根据当前自车状态和行为构建基础自适应风险上下文
/// @return 当前周期生效的风险参数
/// @note 默认关闭时仅返回 MVP-6 静态参数；开启后按高速/变道场景放大风险权重。
SscPlanner::AdaptiveRiskWeightContext
SscPlanner::BuildBaseAdaptiveRiskWeightContext() const {
  AdaptiveRiskWeightContext context;
  context.enabled = cfg_.planner_cfg().enable_adaptive_risk_weight();
  context.risk_weight =
      std::max<decimal_t>(0.0, cfg_.planner_cfg().risk_exposure_weight());
  context.switch_margin =
      std::max<decimal_t>(0.0, cfg_.planner_cfg().risk_exposure_switch_margin());
  context.high_risk_threshold =
      std::max<decimal_t>(0.0, cfg_.planner_cfg().risk_exposure_high_threshold());

  if (!context.enabled) {
    return context;
  }

  const decimal_t high_speed_threshold =
      std::max<decimal_t>(0.0, cfg_.planner_cfg().adaptive_high_speed_threshold());
  context.high_speed = initial_state_.velocity > high_speed_threshold;
  context.lane_change =
      ego_behavior_ == common::LateralBehavior::kLaneChangeLeft ||
      ego_behavior_ == common::LateralBehavior::kLaneChangeRight;

  decimal_t weight_scale = 1.0;
  if (context.high_speed) {
    weight_scale *= std::max<decimal_t>(
        1.0, cfg_.planner_cfg().adaptive_high_speed_weight_scale());
  }
  if (context.lane_change) {
    weight_scale *= std::max<decimal_t>(
        1.0, cfg_.planner_cfg().adaptive_lane_change_weight_scale());
  }

  const decimal_t max_weight =
      std::max<decimal_t>(0.0, cfg_.planner_cfg().adaptive_max_risk_weight());
  context.risk_weight = std::min(context.risk_weight * weight_scale, max_weight);
  context.applied_scale = weight_scale;

  LOG(WARNING) << "[Ssc][MVP7AdaptiveRiskWeight] base_context"
               << " high_speed=" << context.high_speed
               << " lane_change=" << context.lane_change
               << " ego_speed=" << initial_state_.velocity
               << " risk_weight=" << context.risk_weight
               << " switch_margin=" << context.switch_margin
               << " high_risk_threshold=" << context.high_risk_threshold
               << " applied_scale=" << context.applied_scale;

  return context;
}

/// @brief 根据候选轨迹风险统计更新自适应风险上下文
/// @param metrics 当前周期所有 QP 成功候选的风险暴露统计
/// @param risk_context 输入/输出风险上下文
/// @note 高交互风险由候选最大单点风险触发；触发后进一步提高权重并降低切换裕度。
void SscPlanner::UpdateAdaptiveRiskWeightContextByMetrics(
    const std::vector<RiskExposureMetrics>& metrics,
    AdaptiveRiskWeightContext* risk_context) const {
  if (!risk_context) {
    return;
  }

  risk_context->max_candidate_risk = 0.0;
  for (const auto& metric : metrics) {
    risk_context->max_candidate_risk =
        std::max(risk_context->max_candidate_risk, metric.exposure_max);
  }

  if (!risk_context->enabled) {
    return;
  }

  const decimal_t high_interaction_threshold =
      std::max<decimal_t>(0.0, cfg_.planner_cfg().adaptive_high_risk_threshold());
  risk_context->high_interaction_risk =
      risk_context->max_candidate_risk > high_interaction_threshold;
  if (!risk_context->high_interaction_risk) {
    return;
  }

  const decimal_t risk_scale = std::max<decimal_t>(
      1.0, cfg_.planner_cfg().adaptive_high_risk_weight_scale());
  const decimal_t max_weight =
      std::max<decimal_t>(0.0, cfg_.planner_cfg().adaptive_max_risk_weight());
  risk_context->risk_weight =
      std::min(risk_context->risk_weight * risk_scale, max_weight);
  risk_context->applied_scale *= risk_scale;

  const decimal_t switch_margin_scale = std::max<decimal_t>(
      0.0, cfg_.planner_cfg().adaptive_switch_margin_scale());
  risk_context->switch_margin *= switch_margin_scale;

  LOG(WARNING) << "[Ssc][MVP7AdaptiveRiskWeight] enabled=true"
               << " high_speed=" << risk_context->high_speed
               << " lane_change=" << risk_context->lane_change
               << " high_interaction_risk="
               << risk_context->high_interaction_risk
               << " max_candidate_risk="
               << risk_context->max_candidate_risk
               << " risk_weight=" << risk_context->risk_weight
               << " switch_margin=" << risk_context->switch_margin
               << " high_risk_threshold="
               << risk_context->high_risk_threshold
               << " applied_scale=" << risk_context->applied_scale;
}

/// @brief 时空走廊可行性检查
///
/// 检查项:
///   1. 走廊至少有一个立方体
///   2. 相邻立方体的时间区间必须连续: t_ub[i-1] == t_lb[i]
///
/// 这是 QP 优化的重要前提条件。若时间轴不连续, 样条在过渡处
/// 可能没有有效的约束区间, 导致优化问题不可行或解不合法。
///
/// @param cubes 待检查的时空语义立方体序列
/// @return kSuccess 表示走廊连续有效, 可以安全交给 QP 优化
ErrorType SscPlanner::CorridorFeasibilityCheck(
    const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& cubes) {
  int num_cubes = static_cast<int>(cubes.size());
  if (num_cubes < 1) {
    LOG(ERROR) << "[Ssc]number of cubes not enough.";
    return kWrongStatus;
  }
  // 逐一检查相邻立方体的时间连续性
  for (int i = 1; i < num_cubes; i++) {
    if (cubes[i - 1].t_ub != cubes[i].t_lb) {
      LOG(ERROR) << "[Ssc]Err- Corridor not consist.";
      LOG(ERROR) << "[Ssc]Err - t: [" << cubes[i - 1].t_lb << ", "
                 << cubes[i - 1].t_ub << "], x: [" << cubes[i - 1].p_lb[0]
                 << ", " << cubes[i - 1].p_ub[0] << "], y: ["
                 << cubes[i - 1].p_lb[1] << ", " << cubes[i - 1].p_ub[1] << "]";
      LOG(ERROR) << "[Ssc]Err - t: [" << cubes[i].t_lb << ", " << cubes[i].t_ub
                 << "], x: [" << cubes[i].p_lb[0] << ", " << cubes[i].p_ub[0]
                 << "], y: [" << cubes[i].p_lb[1] << ", " << cubes[i].p_ub[1]
                 << "]";
      return kWrongStatus;
    }
  }
  return kSuccess;
}

/// @brief 批量坐标变换: 将所有需要的数据从笛卡尔坐标系转换至 Frenet 坐标系
///
/// 这是 SSC 算法中最重要的预处理步骤。所有后续的时空地图构建、
/// 走廊膨胀和 QP 优化都建立在 Frenet 坐标系之上。
///
/// 三阶段处理流程:
///   Stage I:   数据打包 — 将多种来源的状态和点平铺为连续向量
///   Stage II:  执行变换 — 使用 StateTransformer 批量转换
///   Stage III: 结果恢复 — 按打包偏移量重建各数据结构
///
/// 数据打包顺序 (决定了恢复时的偏移量计算):
///   1. 自车状态 + 自车轮廓顶点 (num_v 个顶点)
///   2. 各行为下自车前向轨迹的所有状态 + 轮廓顶点
///   3. 各行为下周围车辆预测轨迹的所有状态 + 轮廓顶点
///   4. 静态障碍物栅格坐标
///
/// 支持多线程 (OpenMP) 或单线程模式, 由编译宏 USE_OPENMP 控制。
///
/// @return 错误码
ErrorType SscPlanner::StateTransformForInputData() {
  vec_E<State> global_state_vec;    // 全局状态平铺向量
  vec_E<Vec2f> global_point_vec;    // 全局坐标点平铺向量
  int num_v;                        // 车辆轮廓顶点数

  // ===================================================================
  // Stage I: 将状态和点打包为平坦的连续向量
  // ===================================================================

  // * 1. 自车状态 + 顶点
  {
    global_state_vec.push_back(initial_state_);
    vec_E<Vec2f> v_vec;
    // 使用车辆参数计算在初始状态下自车矩形的四个顶点
    common::SemanticsUtils::GetVehicleVertices(ego_vehicle_.param(),
                                               initial_state_, &v_vec);
    num_v = v_vec.size();
    // global_point_vec 的前 num_v 个元素对应自车轮廓顶点
    global_point_vec.insert(global_point_vec.end(), v_vec.begin(), v_vec.end());
  }

  // * 2. 自车前向仿真轨迹的所有状态 + 顶点
  //    每条轨迹的每帧都包含一个状态 + num_v 个轮廓顶点
  {
    common::VehicleParam ego_param = ego_vehicle_.param();
    for (int i = 0; i < (int)forward_trajs_.size(); ++i) {
      if (forward_trajs_[i].size() < 1) continue;
      for (int k = 0; k < (int)forward_trajs_[i].size(); ++k) {
        State traj_state = forward_trajs_[i][k].state();
        global_state_vec.push_back(traj_state);

        vec_E<Vec2f> v_vec;
        common::SemanticsUtils::GetVehicleVertices(ego_param, traj_state,
                                                   &v_vec);
        global_point_vec.insert(global_point_vec.end(), v_vec.begin(),
                                v_vec.end());
      }
    }
  }

  // * 3. 周围车辆预测轨迹的所有状态 + 顶点
  //    结构: surround_forward_trajs_[behavior_i][vehicle_id][frame_k]
  {
    for (size_t i = 0; i < surround_forward_trajs_.size(); ++i) {
      for (auto it = surround_forward_trajs_[i].begin();
           it != surround_forward_trajs_[i].end(); ++it) {
        for (size_t k = 0; k < it->second.size(); ++k) {
          State traj_state = it->second[k].state();
          global_state_vec.push_back(traj_state);

          vec_E<Vec2f> v_vec;
          // 注意: 每辆车可能有不同的物理参数
          common::SemanticsUtils::GetVehicleVertices(it->second[k].param(),
                                                     traj_state, &v_vec);
          global_point_vec.insert(global_point_vec.end(), v_vec.begin(),
                                  v_vec.end());
        }
      }
    }
  }

  // * 4. 周围车辆多模态预测轨迹的所有状态 + 顶点
  //    该旁路只用于 risk grid，不替换 deterministic surround_forward_trajs_。
  {
    for (const auto& vehicle_modes : multimodal_surround_trajs_) {
      for (const auto& mode : vehicle_modes.second) {
        for (int k = 0; k < static_cast<int>(mode.traj.size()); ++k) {
          State traj_state = mode.traj[k].state();
          global_state_vec.push_back(traj_state);

          vec_E<Vec2f> v_vec;
          common::SemanticsUtils::GetVehicleVertices(mode.traj[k].param(),
                                                     traj_state, &v_vec);
          global_point_vec.insert(global_point_vec.end(), v_vec.begin(),
                                  v_vec.end());
        }
      }
    }
  }

  // * 5. 按自车候选行为条件化的周车多模态预测轨迹
  //    该结构为 [ego_behavior_index][vehicle_id][mode]，用于 MVP-10
  //    为每个 ego candidate 构建独立的 risk grid。
  {
    for (const auto& multimodal_trajs_for_behavior :
         behavior_conditioned_multimodal_surround_trajs_) {
      for (const auto& vehicle_modes : multimodal_trajs_for_behavior) {
        for (const auto& mode : vehicle_modes.second) {
          for (int k = 0; k < static_cast<int>(mode.traj.size()); ++k) {
            State traj_state = mode.traj[k].state();
            global_state_vec.push_back(traj_state);

            vec_E<Vec2f> v_vec;
            common::SemanticsUtils::GetVehicleVertices(mode.traj[k].param(),
                                                       traj_state, &v_vec);
            global_point_vec.insert(global_point_vec.end(), v_vec.begin(),
                                    v_vec.end());
          }
        }
      }
    }
  }

  // * 6. 静态障碍物栅格坐标
  {
    for (auto it = obstacle_grids_.begin(); it != obstacle_grids_.end(); ++it) {
      Vec2f pt((*it)[0], (*it)[1]);
      global_point_vec.push_back(pt);
    }
  }

  // 预分配结果向量（避免动态扩容）
  vec_E<FrenetState> frenet_state_vec(global_state_vec.size());
  vec_E<Vec2f> fs_point_vec(global_point_vec.size());

  // ===================================================================
  // Stage II: 执行坐标变换 (单线程或多线程)
  // ===================================================================
#if USE_OPENMP
  TicToc timer_stf;
  StateTransformUsingOpenMp(global_state_vec, global_point_vec,
                            &frenet_state_vec, &fs_point_vec);
  LOG(WARNING) << "[Ssc]OpenMp transform time cost: " << timer_stf.toc()
               << " ms.";
#else
  TicToc timer_stf;
  StateTransformSingleThread(global_state_vec, global_point_vec,
                             &frenet_state_vec, &fs_point_vec);
  LOG(WARNING) << "[Ssc]Single thread transform time cost: " << timer_stf.toc()
               << " ms.";
#endif

  // ===================================================================
  // Stage III: 按打包偏移量从结果向量中恢复各数据结构
  //            关键: offset 维护当前处理到的状态索引
  // ===================================================================
  int offset = 0;

  // * 恢复 1: 自车 Frenet 状态 + 轮廓顶点
  {
    fs_ego_vehicle_.frenet_state = frenet_state_vec[offset];
    fs_ego_vehicle_.vertices.clear();
    for (int i = 0; i < num_v; ++i) {
      fs_ego_vehicle_.vertices.push_back(fs_point_vec[offset * num_v + i]);
    }
    offset++;  // 自车状态消耗一个状态索引
  }

  // * 恢复 2: 各行为下自车前向仿真轨迹 (Frenet 坐标)
  {
    forward_trajs_fs_.clear();
    if (forward_trajs_.size() < 1) return kWrongStatus;
    for (int j = 0; j < (int)forward_trajs_.size(); ++j) {
      if (forward_trajs_[j].size() < 1) assert(false);
      vec_E<common::FsVehicle> traj_fs;
      for (int k = 0; k < (int)forward_trajs_[j].size(); ++k) {
        common::FsVehicle fs_v;
        fs_v.frenet_state = frenet_state_vec[offset];
        // 从 point 向量中提取该帧对应的 num_v 个轮廓顶点
        for (int i = 0; i < num_v; ++i) {
          fs_v.vertices.push_back(fs_point_vec[offset * num_v + i]);
        }
        traj_fs.emplace_back(fs_v);
        offset++;  // 每帧消耗一个状态索引
      }
      forward_trajs_fs_.emplace_back(traj_fs);
    }
  }

  // * 恢复 3: 各行为下周围车辆预测轨迹 (Frenet 坐标)
  {
    surround_forward_trajs_fs_.clear();
    for (size_t j = 0; j < surround_forward_trajs_.size(); ++j) {
      std::unordered_map<int, vec_E<common::FsVehicle>> sur_trajs;
      for (auto it = surround_forward_trajs_[j].begin();
           it != surround_forward_trajs_[j].end(); ++it) {
        int v_id = it->first;  // 车辆ID
        vec_E<common::FsVehicle> traj_fs;
        for (size_t k = 0; k < it->second.size(); ++k) {
          common::FsVehicle fs_v;
          fs_v.frenet_state = frenet_state_vec[offset];
          for (int i = 0; i < num_v; ++i) {
            fs_v.vertices.push_back(fs_point_vec[offset * num_v + i]);
          }
          traj_fs.emplace_back(fs_v);
          offset++;
        }
        sur_trajs.insert(
            std::pair<int, vec_E<common::FsVehicle>>(v_id, traj_fs));
      }
      surround_forward_trajs_fs_.emplace_back(sur_trajs);
    }
  }

  // * 恢复 4: 周围车辆多模态预测轨迹 (Frenet 坐标)
  {
    multimodal_surround_trajs_fs_.clear();
    for (const auto& vehicle_modes : multimodal_surround_trajs_) {
      const int vehicle_id = vehicle_modes.first;
      vec_E<SurroundingVehicleFsTrajectoryMode> fs_modes;
      for (const auto& mode : vehicle_modes.second) {
        SurroundingVehicleFsTrajectoryMode fs_mode;
        fs_mode.vehicle_id = mode.vehicle_id;
        fs_mode.lat_behavior = mode.lat_behavior;
        fs_mode.probability = mode.probability;
        for (int k = 0; k < static_cast<int>(mode.traj.size()); ++k) {
          common::FsVehicle fs_v;
          fs_v.frenet_state = frenet_state_vec[offset];
          for (int i = 0; i < num_v; ++i) {
            fs_v.vertices.push_back(fs_point_vec[offset * num_v + i]);
          }
          fs_mode.traj.emplace_back(fs_v);
          offset++;
        }
        fs_modes.emplace_back(fs_mode);
      }
      if (!fs_modes.empty()) {
        multimodal_surround_trajs_fs_.insert({vehicle_id, fs_modes});
      }
    }
  }

  // * 恢复 5: 按自车候选行为条件化的周车多模态预测轨迹 (Frenet 坐标)
  {
    behavior_conditioned_multimodal_surround_trajs_fs_.clear();
    behavior_conditioned_multimodal_surround_trajs_fs_.reserve(
        behavior_conditioned_multimodal_surround_trajs_.size());
    for (const auto& multimodal_trajs_for_behavior :
         behavior_conditioned_multimodal_surround_trajs_) {
      MultiModalSurroundingFsTrajectories fs_trajs_for_behavior;
      for (const auto& vehicle_modes : multimodal_trajs_for_behavior) {
        const int vehicle_id = vehicle_modes.first;
        vec_E<SurroundingVehicleFsTrajectoryMode> fs_modes;
        for (const auto& mode : vehicle_modes.second) {
          SurroundingVehicleFsTrajectoryMode fs_mode;
          fs_mode.vehicle_id = mode.vehicle_id;
          fs_mode.lat_behavior = mode.lat_behavior;
          fs_mode.probability = mode.probability;
          for (int k = 0; k < static_cast<int>(mode.traj.size()); ++k) {
            common::FsVehicle fs_v;
            fs_v.frenet_state = frenet_state_vec[offset];
            for (int i = 0; i < num_v; ++i) {
              fs_v.vertices.push_back(fs_point_vec[offset * num_v + i]);
            }
            fs_mode.traj.emplace_back(fs_v);
            offset++;
          }
          fs_modes.emplace_back(fs_mode);
        }
        if (!fs_modes.empty()) {
          fs_trajs_for_behavior.insert({vehicle_id, fs_modes});
        }
      }
      behavior_conditioned_multimodal_surround_trajs_fs_.emplace_back(
          fs_trajs_for_behavior);
    }
  }

  // * 恢复 6: 障碍物栅格的 Frenet 坐标
  {
    obstacle_grids_fs_.clear();
    for (int i = 0; i < static_cast<int>(obstacle_grids_.size()); ++i) {
      obstacle_grids_fs_.push_back(fs_point_vec[offset * num_v + i]);
    }
  }

  // 从自车 Frenet 车辆对象中提取状态作为便捷引用
  ego_frenet_state_ = fs_ego_vehicle_.frenet_state;
  return kSuccess;
}

/// @brief OpenMP 多线程坐标变换
///
/// 使用 4 线程并行执行:
///   - 线程组1: 遍历所有状态, 调用 GetFrenetStateFromState 转换
///   - 线程组2: 遍历所有点,   调用 GetFrenetPointFromPoint 转换
///
/// 这两个循环互相独立, 但由于 #pragma omp parallel for 的位置,
/// 实际上它们是顺序执行的, 各自内部使用 4 线程并行。
///
/// 注意: GetFrenetStateFromState 在网络查找等步骤中可能有线程安全性问题,
///       失败时保留原始时间戳以便后续追踪。
ErrorType SscPlanner::StateTransformUsingOpenMp(
    const vec_E<State>& global_state_vec, const vec_E<Vec2f>& global_point_vec,
    vec_E<FrenetState>* frenet_state_vec, vec_E<Vec2f>* fs_point_vec) const {
  int state_num = global_state_vec.size();
  int point_num = global_point_vec.size();

  // 获取原始数据指针用于 OpenMP 的指针算术访问
  auto ptr_state_vec = frenet_state_vec->data();
  auto ptr_point_vec = fs_point_vec->data();

  LOG(WARNING) << "[Ssc]OpenMp - Total number of queries: "
               << state_num + point_num;

  omp_set_num_threads(4);  // 固定使用 4 线程
  {
    // 状态变换并行区
#pragma omp parallel for
    for (int i = 0; i < state_num; ++i) {
      FrenetState fs;
      if (kSuccess != stf_.GetFrenetStateFromState(global_state_vec[i], &fs)) {
        // 转换失败时保留原始时间戳, 避免时间信息丢失
        fs.time_stamp = global_state_vec[i].time_stamp;
      }
      *(ptr_state_vec + i) = fs;
    }
  }
  {
    // 点变换并行区
#pragma omp parallel for
    for (int i = 0; i < point_num; ++i) {
      Vec2f fs_pt;
      stf_.GetFrenetPointFromPoint(global_point_vec[i], &fs_pt);
      *(ptr_point_vec + i) = fs_pt;
    }
  }

  return kSuccess;
}

/// @brief 单线程坐标变换
///
/// 顺序遍历所有状态和点执行变换，逻辑与 OpenMP 版本一致。
/// 适用于对确定性有严格要求的场景。
ErrorType SscPlanner::StateTransformSingleThread(
    const vec_E<State>& global_state_vec, const vec_E<Vec2f>& global_point_vec,
    vec_E<FrenetState>* frenet_state_vec, vec_E<Vec2f>* fs_point_vec) const {
  int state_num = global_state_vec.size();
  int point_num = global_point_vec.size();
  auto ptr_state_vec = frenet_state_vec->data();
  auto ptr_point_vec = fs_point_vec->data();

  // 顺序转换所有状态
  {
    for (int i = 0; i < state_num; ++i) {
      FrenetState fs;
      stf_.GetFrenetStateFromState(global_state_vec[i], &fs);
      *(ptr_state_vec + i) = fs;
    }
  }

  // 顺序转换所有点
  {
    for (int i = 0; i < point_num; ++i) {
      Vec2f fs_pt;
      stf_.GetFrenetPointFromPoint(global_point_vec[i], &fs_pt);
      *(ptr_point_vec + i) = fs_pt;
    }
  }

  return kSuccess;
}

/// @brief 设置地图接口
/// @param map_itf 非空的地图接口指针
ErrorType SscPlanner::set_map_interface(SscPlannerMapItf* map_itf) {
  if (map_itf == nullptr) return kIllegalInput;
  map_itf_ = map_itf;
  map_valid_ = true;
  ConfigureMapInterfacePredictionHorizon();
  return kSuccess;
}

/// @brief 轨迹验证 (当前默认禁用)
///
/// 检查项:
///   1. 起点位置与初始状态一致 (误差 < 0.1m)
///   2. 起点速度与初始状态一致 (误差 < 0.1m/s)
///   3. 全程曲率不超过 0.33 (≈ 最小转弯半径 ~3m)
///
/// 此验证较为严格，可能拒绝实际上可执行的轨迹，故默认关闭。
ErrorType SscPlanner::ValidateTrajectory(const FrenetTrajectory& traj) {
  // 生成采样时间点序列 (间隔 0.1s)
  std::vector<decimal_t> t_vec_xy;
  common::GetRangeVector<decimal_t>(traj.begin(), traj.end(), 0.1, true,
                                    &t_vec_xy);
  common::State state;

  // 检查起点状态
  if (traj.GetState(traj.begin(), &state) != kSuccess) {
    LOG(ERROR) << "[Ssc][Validate]State evaluation error";
    return kWrongStatus;
  }

  if ((state.vec_position - initial_state_.vec_position).norm() > 0.1) {
    LOG(ERROR) << "[Ssc][Validate]Init position miss match";
    return kWrongStatus;
  }

  if (fabs(state.velocity - initial_state_.velocity) > 0.1) {
    LOG(ERROR) << "[Ssc][Validate]Init vel miss match";
    return kWrongStatus;
  }

  // 检查终点状态
  if (traj.GetState(traj.end(), &state) != kSuccess) {
    LOG(ERROR) << "[Ssc][Validate]End state eval error";
    return kWrongStatus;
  }

  // 逐采样点检查曲率
  for (const auto t : t_vec_xy) {
    if (traj.GetState(t, &state) != kSuccess) {
      LOG(ERROR) << "[Ssc][Validate]State eval error";
      return kWrongStatus;
    }
    // 曲率阈值 0.33 ≈ 1/3, 对应最小转弯半径约 3m
    if (fabs(state.curvature) > 0.33) {
      LOG(ERROR) << "[Ssc][Validate]initial_state velocity "
                 << initial_state_.velocity << " Curvature " << state.curvature
                 << " invalid.";
      return kWrongStatus;
    }
  }

  return kSuccess;
}

}  // namespace planning
