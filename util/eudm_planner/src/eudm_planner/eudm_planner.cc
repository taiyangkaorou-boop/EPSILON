/**
 * @file eudm_planner.cc
 * @author EPSILON Autonomous Driving Team
 * @brief EUDM（高效不确定性感知决策）行为规划器核心实现
 * @version 0.1
 * @date 2019-07-07
 * @copyright Copyright (c) 2019
 *
 * ## 文件概述
 *
 * 本文件实现了EUDM行为规划系统的核心引擎——EudmPlanner类。
 * EUDM是EPSILON自动驾驶系统中用于高层行为决策的智能规划器，
 * 其核心能力是并行评估大量候选动作序列，结合周围车辆行为的
 * 不确定性模型，在效率、安全和导航三个维度上选出最优行为。
 *
 * ## EUDM算法流程
 *
 * ### 阶段1：初始化（Init/ReadConfig）
 * - 读取Protobuf配置文件
 * - 创建DCP-Tree（动作序列生成器）
 * - 从配置中提取仿真参数（IDM参数、横向控制、运动学约束）
 * - 初始化RSS安全配置（标准/严格前车/严格后车）
 *
 * ### 阶段2：规划循环入口（RunOnce）
 * 1. 从地图接口获取自车状态和当前车道ID
 * 2. 预筛除逻辑不合理（自相矛盾）的动作序列
 *    - 例如：连续LKR（左变道后立即右变道被筛除）
 * 3. 调用RunEudm执行核心仿真流程
 *
 * ### 阶段3：核心仿真（RunEudm）
 * 1. 获取周围语义车辆（含行为概率分布）
 * 2. 转换为前向仿真智能体（ForwardSimAgent）
 * 3. 准备多线程结果容器（PrepareMultiThreadContainers）
 * 4. 为每个动作序列创建独立线程，并行执行SimulateActionSequence
 * 5. 等待所有线程完成
 * 6. 汇总结果，评估每个序列的代价
 * 7. 调用EvaluateMultiThreadSimResults选择代价最低的序列
 *
 * ### 阶段4：单序列仿真（SimulateActionSequence -> SimulateScenario）
 * 对动作序列的每一层（每个DcpAction）循环执行：
 * 1. UpdateSimSetupForLayer: 更新Frenet坐标系、参考车道、间隙车辆、RSS预检查
 * 2. SimulateSingleAction: 多步前向仿真（IDM+PurePursuit）
 * 3. StrictSafetyCheck: 严格几何碰撞检查
 * 4. 检查横向动作完成状态，若完成则更新后续动作序列
 * 5. CostFunction: 计算该层的三维代价
 * 6. 轨迹和代价记录
 *
 * ### 阶段5：代价评估（CostFunction）
 * 三维代价函数：
 * - **效率Efficiency**：速度偏差（与期望速度差距）+ 前车速阻
 * - **安全Safety**：RSS安全检查+占位碰撞风险
 * - **导航Navigation**：变道偏好/惩罚+推荐变道奖励
 *
 * ## 关键设计决策
 *
 * 1. **多线程并行**：使用std::thread为每个动作序列创建独立线程，
 *    利用多核CPU加速大量前向仿真的计算
 * 2. **周围车辆行为确定性**：当前版本在前向仿真中周围车辆
 *    保持车道不变（lateral behavior固定），但代价函数中通过
 *    RSS安全检查建模变道风险
 * 3. **变道完成的在线更新**：若横向动作在执行过程中提前完成
 *    （CheckIfLateralActionFinished），则动态更新后续动作序列
 *    （UpdateLateralActionSequence）
 * 4. **严格分层检查**：layersetup时的RSS预检查（rss_for_layers_enable）
 *    可在仿真的开始阶段快速筛除明显不安全的变道动作，避免浪费计算
 */

#include "eudm_planner/eudm_planner.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include "common/rss/rss_checker.h"

namespace planning {

/// @brief 返回规划器名称标识
std::string EudmPlanner::Name() { return std::string("Eudm behavior planner"); }

// ============================================================================
//  初始化与配置
// ============================================================================

/// @brief 读取EUDM规划器的Protobuf文本配置文件
///
/// 使用Protobuf的TextFormat解析器从文本文件中读取配置。
/// 解析完成后检查cfg_是否已初始化（包含所有必要字段）。
/// 若解析失败则触发assert终止程序——配置文件错误是致命问题。
///
/// @param config_path 配置文件的绝对或相对路径
ErrorType EudmPlanner::ReadConfig(const std::string config_path) {
  printf("\n[EudmPlanner] Loading eudm planner config\n");
  using namespace google::protobuf;
  int fd = open(config_path.c_str(), O_RDONLY);
  io::FileInputStream fstream(fd);
  TextFormat::Parse(&fstream, &cfg_);
  if (!cfg_.IsInitialized()) {
    LOG(ERROR) << "failed to parse config from " << config_path;
    assert(false);
  }
  return kSuccess;
}

/// @brief 从Protobuf配置中提取前向仿真参数
///
/// 将配置中的分层参数（lon/lat/limit）映射到仿真器的参数结构体。
///
/// 提取的参数包括：
/// - IDM纵向模型参数：最小间距、期望时距、加速度、舒适/紧急制动减速度、指数
/// - 运动学约束：纵向加加速度限制（jerk）、横向加速度/加加速度限制
/// - 纯追踪横向控制器参数：控制增益、前视距离范围
/// - 转向限制：最大转角、最大转角速率、最大曲率
/// - 横向失败自动减速选项
///
/// @param cfg Protobuf前向仿真配置
/// @param sim_param [out] 输出仿真参数结构体
ErrorType EudmPlanner::GetSimParam(const planning::eudm::ForwardSimDetail& cfg,
                                   OnLaneForwardSimulation::Param* sim_param) {
  // ===== IDM纵向模型参数 =====
  sim_param->idm_param.kMinimumSpacing = cfg.lon().idm().min_spacing();          // 最小安全间距
  sim_param->idm_param.kDesiredHeadwayTime = cfg.lon().idm().head_time();         // 期望车间时距
  sim_param->idm_param.kAcceleration = cfg.lon().limit().acc();                   // 最大加速度
  sim_param->idm_param.kComfortableBrakingDeceleration =
      cfg.lon().limit().soft_brake();                                              // 舒适制动减速度
  sim_param->idm_param.kHardBrakingDeceleration =
      cfg.lon().limit().hard_brake();                                              // 紧急制动减速度
  sim_param->idm_param.kExponent = cfg.lon().idm().exponent();                    // IDM加速度公式中的指数

  // ===== 纵向运动学约束 =====
  sim_param->max_lon_acc_jerk = cfg.lon().limit().acc_jerk();                      // 最大纵向加速度变化率
  sim_param->max_lon_brake_jerk = cfg.lon().limit().brake_jerk();                  // 最大纵向制动减速度变化率

  // ===== 纯追踪横向控制参数 =====
  sim_param->steer_control_gain = cfg.lat().pure_pursuit().gain();                 // 纯追踪控制增益
  sim_param->steer_control_max_lookahead_dist =
      cfg.lat().pure_pursuit().max_lookahead_dist();                               // 最大前视距离
  sim_param->steer_control_min_lookahead_dist =
      cfg.lat().pure_pursuit().min_lookahead_dist();                               // 最小前视距离

  // ===== 横向运动学约束 =====
  sim_param->max_lat_acceleration_abs = cfg.lat().limit().acc();                   // 最大横向加速度绝对值
  sim_param->max_lat_jerk_abs = cfg.lat().limit().jerk();                          // 最大横向加加速度
  sim_param->max_curvature_abs = cfg.lat().limit().curvature();                    // 最大曲率
  sim_param->max_steer_angle_abs = cfg.lat().limit().steer_angle();                // 最大转向角
  sim_param->max_steer_rate = cfg.lat().limit().steer_rate();                      // 最大转向速率

  // ===== 辅助选项 =====
  sim_param->auto_decelerate_if_lat_failed = cfg.auto_dec_if_lat_failed();         // 横向失败时自动减速
  return kSuccess;
}

/// @brief EUDM规划器的初始化
///
/// 初始化步骤：
/// 1. 读取配置文件
/// 2. 创建DCP-Tree（根据配置中的tree_height和layer参数）
/// 3. 提取自车和周围车辆的仿真参数
/// 4. 初始化三种RSS安全配置：
///    - rss_config_: 标准RSS参数（用于仿真过程中的安全检查）
///    - rss_config_strict_as_front_: 作为前车时的严格RSS（自车太快时的阈值更保守）
///    - rss_config_strict_as_rear_: 作为后车时的严格RSS（自车太慢时的阈值更保守）
///
/// @param config 配置文件路径
ErrorType EudmPlanner::Init(const std::string config) {
  // 步骤1: 读取配置文件
  ReadConfig(config);

  // 步骤2: 创建DCP-Tree
  // 参数: 树高度（动作序列长度）、每层基础时间、最后一层时间
  dcp_tree_ptr_ = new DcpTree(cfg_.sim().duration().tree_height(),
                              cfg_.sim().duration().layer(),
                              cfg_.sim().duration().last_layer());
  LOG(INFO) << "[Eudm]Init.";
  LOG(INFO) << "[Eudm]ActionScript size: "
            << dcp_tree_ptr_->action_script().size() << std::endl;

  // 步骤3: 提取仿真参数（区分自车和周围车辆）
  GetSimParam(cfg_.sim().ego(), &ego_sim_param_);
  GetSimParam(cfg_.sim().agent(), &agent_sim_param_);

  // 步骤4: 构建三种RSS安全配置
  // 4a. 标准RSS配置
  rss_config_ = common::RssChecker::RssConfig(
      cfg_.safety().rss().response_time(),              // 响应时间
      cfg_.safety().rss().longitudinal_acc_max(),       // 纵向最大加速度
      cfg_.safety().rss().longitudinal_brake_min(),     // 纵向最小制动减速度
      cfg_.safety().rss().longitudinal_brake_max(),     // 纵向最大制动减速度
      cfg_.safety().rss().lateral_acc_max(),            // 横向最大加速度
      cfg_.safety().rss().lateral_brake_min(),          // 横向最小制动
      cfg_.safety().rss().lateral_brake_max(),          // 横向最大制动
      cfg_.safety().rss().lateral_miu());              // 横向摩擦系数

  // 4b. 作为前车时的严格RSS配置（用于检查自车是否太快靠近前车）
  rss_config_strict_as_front_ = common::RssChecker::RssConfig(
      cfg_.safety().rss_strict_as_front().response_time(),
      cfg_.safety().rss_strict_as_front().longitudinal_acc_max(),
      cfg_.safety().rss_strict_as_front().longitudinal_brake_min(),
      cfg_.safety().rss_strict_as_front().longitudinal_brake_max(),
      cfg_.safety().rss_strict_as_front().lateral_acc_max(),
      cfg_.safety().rss_strict_as_front().lateral_brake_min(),
      cfg_.safety().rss_strict_as_front().lateral_brake_max(),
      cfg_.safety().rss_strict_as_front().lateral_miu());

  // 4c. 作为后车时的严格RSS配置（用于检查自车是否太慢导致后车追尾）
  rss_config_strict_as_rear_ = common::RssChecker::RssConfig(
      cfg_.safety().rss_strict_as_rear().response_time(),
      cfg_.safety().rss_strict_as_rear().longitudinal_acc_max(),
      cfg_.safety().rss_strict_as_rear().longitudinal_brake_min(),
      cfg_.safety().rss_strict_as_rear().longitudinal_brake_max(),
      cfg_.safety().rss_strict_as_rear().lateral_acc_max(),
      cfg_.safety().rss_strict_as_rear().lateral_brake_min(),
      cfg_.safety().rss_strict_as_rear().lateral_brake_max(),
      cfg_.safety().rss_strict_as_rear().lateral_miu());

  return kSuccess;
}

// ============================================================================
//  动作翻译与分类
// ============================================================================

/// @brief 将DcpAction离散动作翻译为横向和纵向行为枚举
///
/// 一对一映射关系：
/// - DcpLatAction::kLaneKeeping -> LateralBehavior::kLaneKeeping
/// - DcpLatAction::kLaneChangeLeft -> LateralBehavior::kLaneChangeLeft
/// - DcpLatAction::kLaneChangeRight -> LateralBehavior::kLaneChangeRight
/// - DcpLonAction::kMaintain -> LongitudinalBehavior::kMaintain
/// - DcpLonAction::kAccelerate -> LongitudinalBehavior::kAccelerate
/// - DcpLonAction::kDecelerate -> LongitudinalBehavior::kDecelerate
ErrorType EudmPlanner::TranslateDcpActionToLonLatBehavior(
    const DcpAction& action, LateralBehavior* lat,
    LongitudinalBehavior* lon) const {
  // 横向动作翻译
  switch (action.lat) {
    case DcpLatAction::kLaneKeeping: {
      *lat = LateralBehavior::kLaneKeeping;
      break;
    }
    case DcpLatAction::kLaneChangeLeft: {
      *lat = LateralBehavior::kLaneChangeLeft;
      break;
    }
    case DcpLatAction::kLaneChangeRight: {
      *lat = LateralBehavior::kLaneChangeRight;
      break;
    }
    default: {
      LOG(ERROR) << "[Eudm]Lateral action translation error!";
      return kWrongStatus;
    }
  }

  // 纵向动作翻译
  switch (action.lon) {
    case DcpLonAction::kMaintain: {
      *lon = LongitudinalBehavior::kMaintain;
      break;
    }
    case DcpLonAction::kAccelerate: {
      *lon = LongitudinalBehavior::kAccelerate;
      break;
    }
    case DcpLonAction::kDecelerate: {
      *lon = LongitudinalBehavior::kDecelerate;
      break;
    }
    default: {
      LOG(ERROR) << "[Eudm]Longitudinal action translation error!";
      return kWrongStatus;
    }
  }

  return kSuccess;
}

/// @brief 分析动作序列的类型
///
/// 遍历动作序列，识别以下信息：
/// - **operation_at_seconds**: 第一个变道操作出现的时刻（从序列开始算）
/// - **lat_behavior**: 序列的横向行为类型（保持/左变道/右变道）
/// - **is_cancel_operation**: 是否存在变道后取消的模式
///
/// 分析逻辑：
/// 1. 累计遍历所有动作，找到第一个非保持的横向动作：
///    - 如果是左变道 -> lat_behavior = kLaneChangeLeft
///    - 如果是右变道 -> lat_behavior = kLaneChangeRight
/// 2. 如果找到了变道动作后，再找到保持动作 -> is_cancel = true
///    （说明变道后又恢复保持 = 变道取消模式）
/// 3. 如果全程没有变道动作 -> lat_behavior = kLaneKeeping
///    operation_at_seconds = 序列总时长 + 一层的时长（假设下个周期变道）
///
/// @param action_seq 输入的动作序列
/// @param operation_at_seconds [out] 从序列开始到执行横向操作的时间
/// @param lat_behavior [out] 该序列的横向行为类型
/// @param is_cancel_operation [out] 是否为变道后取消的操作
ErrorType EudmPlanner::ClassifyActionSeq(
    const std::vector<DcpAction>& action_seq, decimal_t* operation_at_seconds,
    common::LateralBehavior* lat_behavior, bool* is_cancel_operation) const {
  decimal_t duration = 0.0;
  decimal_t operation_at = 0.0;
  bool find_lat_active_behavior = false;
  *is_cancel_operation = false;

  // 遍历动作序列
  for (const auto& action : action_seq) {
    if (!find_lat_active_behavior) {
      // 尚未找到第一个变道动作
      if (action.lat == DcpLatAction::kLaneChangeLeft) {
        *operation_at_seconds = duration;
        *lat_behavior = common::LateralBehavior::kLaneChangeLeft;
        find_lat_active_behavior = true;
      }
      if (action.lat == DcpLatAction::kLaneChangeRight) {
        *operation_at_seconds = duration;
        *lat_behavior = common::LateralBehavior::kLaneChangeRight;
        find_lat_active_behavior = true;
      }
    } else {
      // 已经找到变道动作后，检查是否有后续的保持动作（取消模式）
      if (action.lat == DcpLatAction::kLaneKeeping) {
        *is_cancel_operation = true;
      }
    }
    duration += action.t;
  }

  // 全程保持车道（没找到变道动作）
  if (!find_lat_active_behavior) {
    *operation_at_seconds = duration + cfg_.sim().duration().layer();
    *lat_behavior = common::LateralBehavior::kLaneKeeping;
    *is_cancel_operation = false;
  }
  return kSuccess;
}

// ============================================================================
//  多线程容器准备
// ============================================================================

/// @brief 为多线程仿真准备结果容器
///
/// 将所有结果数组resize到n_sequence的大小，为每个动作序列
/// 预留独立的存储空间。多线程并行写入时，每个线程使用其序列ID
/// 作为索引写入对应的位置，因此不需要加锁。
///
/// 初始化的容器：
/// - sim_res_: 全部初始化为0（失败）
/// - risky_res_: 全部初始化为0（无风险）
/// - sim_info_: 全部初始化为空字符串
/// - final_cost_: 全部初始化为0.0
/// - 其他容器仅resize，保持默认构造值
///
/// @param n_sequence 动作序列总数（等于线程数）
ErrorType EudmPlanner::PrepareMultiThreadContainers(const int n_sequence) {
  LOG(INFO) << "[Eudm][Process]Prepare multi-threading - " << n_sequence
            << " threads.";

  sim_res_.clear();
  sim_res_.resize(n_sequence, 0);        // 全部初始化为0（仿真失败）

  risky_res_.clear();
  risky_res_.resize(n_sequence, 0);      // 全部初始化为0（无风险）

  sim_info_.clear();
  sim_info_.resize(n_sequence, std::string(""));

  final_cost_.clear();
  final_cost_.resize(n_sequence, 0.0);

  progress_cost_.clear();
  progress_cost_.resize(n_sequence);

  tail_cost_.clear();
  tail_cost_.resize(n_sequence);

  forward_trajs_.clear();
  forward_trajs_.resize(n_sequence);

  forward_lat_behaviors_.clear();
  forward_lat_behaviors_.resize(n_sequence);

  forward_lon_behaviors_.clear();
  forward_lon_behaviors_.resize(n_sequence);

  surround_trajs_.clear();
  surround_trajs_.resize(n_sequence);
  return kSuccess;
}

// ============================================================================
//  周围车辆仿真智能体构建
// ============================================================================

/// @brief 将语义车辆集合转换为前向仿真智能体集合
///
/// 这是EUDM建模周围车辆行为不确定性的关键入口。
/// 对于每个语义车辆，构建一个ForwardSimAgent：
///
/// **纵向行为预测**：
/// - 如果当前加速度>=0：假设车辆以当前速度匀速行驶（期望速度=当前速度）
/// - 如果当前加速度<0：根据当前减速度估计模拟结束时的速度作为期望速度
///   est_vel = max(0, current_vel + acceleration * sim_time_total)
///
/// **横向行为建模**（EUDM核心特征）：
/// - 保留语义车辆的横向行为概率分布（lat_probs）
/// - 保留当前观测到的横向行为（lat_behavior）
/// - 注意：当前版本在前向仿真中固定使用当前观测行为，
///   但概率分布数据已准备好，为未来的随机采样版本做预备
ErrorType EudmPlanner::GetSurroundingForwardSimAgents(
    const common::SemanticVehicleSet& surrounding_semantic_vehicles,
    ForwardSimAgentSet* fsagents) const {
  for (const auto& psv : surrounding_semantic_vehicles.semantic_vehicles) {
    ForwardSimAgent fsagent;

    // 设置车辆ID和当前状态
    int id = psv.second.vehicle.id();
    fsagent.id = id;
    fsagent.vehicle = psv.second.vehicle;

    // * 纵向仿真参数
    fsagent.sim_param = agent_sim_param_;

    common::State state = psv.second.vehicle.state();
    // 速度预测逻辑：
    // - 加速或匀速（acc >= 0）：假设匀速保持当前速度
    // - 减速（acc < 0）：估计减速到模拟结束时的速度
    if (state.acceleration >= 0) {
      fsagent.sim_param.idm_param.kDesiredVelocity = state.velocity;
    } else {
      decimal_t est_vel =
          std::max(0.0, state.velocity + state.acceleration * sim_time_total_);
      fsagent.sim_param.idm_param.kDesiredVelocity = est_vel;
    }

    // * 横向行为不确定性建模
    // 保留语义车辆的行为概率分布（EUDM区别于MPDM的核心特征）
    fsagent.lat_probs = psv.second.probs_lat_behaviors;
    fsagent.lat_behavior = psv.second.lat_behavior;

    // 设置车道和Frenet坐标变换器
    fsagent.lane = psv.second.lane;
    fsagent.stf = common::StateTransformer(fsagent.lane);

    // * 横向搜索范围
    fsagent.lat_range = cfg_.sim().agent().cooperative_lat_range();

    // 加入集合
    fsagents->forward_sim_agents.insert(std::make_pair(id, fsagent));
  }

  return kSuccess;
}

// ============================================================================
//  EUDM核心仿真循环
// ============================================================================

/// @brief [核心函数] 执行EUDM规划的核心仿真与评估
///
/// 这是EUDM算法的最顶层调度函数，管理从数据准备到结果评估的完整流程。
///
/// ## 详细流程
///
/// **步骤1: 获取周围车辆数据**
/// - 从地图接口获取关键语义车辆（包含行为概率分布）
/// - 将语义车辆转换为前向仿真智能体（ForwardSimAgent）
///   这一步保留了行为概率分布作为EUDM不确定性建模的基础
///
/// **步骤2: 获取候选动作序列**
/// - 从DcpTree获取action_script（所有可能的动作序列）
/// - n_sequence = 动作序列的总数（每个序列将由一个线程处理）
///
/// **步骤3: 多线程并行仿真**
/// - PrepareMultiThreadContainers: 分配结果存储空间
/// - 为每个动作序列创建std::thread，入口函数为SimulateActionSequence
/// - 每个线程独立运行前向仿真和代价计算
/// - join所有线程等待完成
///
/// **步骤4: 结果汇总与日志**
/// - 统计成功仿真的序列数量
/// - 打印每个序列的仿真结果概要
/// - 若无成功序列则返回错误（所有动作都导致碰撞或不安全）
///
/// **步骤5: 代价评估与优胜选择**
/// - EvaluateMultiThreadSimResults: 选择全局代价最低的序列
/// - 记录优胜序列的ID和代价分数
///
/// @return kSuccess if at least one valid behavior found
ErrorType EudmPlanner::RunEudm() {
  // * 步骤1: 获取周围车辆的语义信息
  common::SemanticVehicleSet surrounding_semantic_vehicles;
  if (map_itf_->GetKeySemanticVehicles(&surrounding_semantic_vehicles) !=
      kSuccess) {
    LOG(ERROR) << "[Eudm][Fatal]fail to get key semantic vehicles. Exit";
    return kWrongStatus;
  }

  // 转换为前向仿真智能体（保留行为概率分布）
  ForwardSimAgentSet surrounding_fsagents;
  GetSurroundingForwardSimAgents(surrounding_semantic_vehicles,
                                 &surrounding_fsagents);

  // * 步骤2: 获取DCP-Tree生成的所有候选动作序列
  auto action_script = dcp_tree_ptr_->action_script();
  int n_sequence = action_script.size();

  // * 步骤3: 准备多线程环境和容器
  std::vector<std::thread> thread_set(n_sequence);
  PrepareMultiThreadContainers(n_sequence);

  // * 步骤4: 启动多线程并行仿真
  // 每个动作序列在一个独立线程中运行，写���到以序列ID为索引的结果容器
  TicToc timer;
  for (int i = 0; i < n_sequence; ++i) {
    thread_set[i] =
        std::thread(&EudmPlanner::SimulateActionSequence, this, ego_vehicle_,
                    surrounding_fsagents, action_script[i], i);
  }
  // 等待所有线程完成
  for (int i = 0; i < n_sequence; ++i) {
    thread_set[i].join();
  }

  LOG(INFO) << "[Eudm][Process]Multi-thread forward simulation finished!";

  // * 步骤5: 汇总仿真结果
  bool sim_success = false;
  int num_valid_behaviors = 0;
  for (int i = 0; i < static_cast<int>(sim_res_.size()); ++i) {
    if (sim_res_[i] == 1) {
      sim_success = true;
      num_valid_behaviors++;
    }
  }

  // 打印每个序列的仿真结果概要
  for (int i = 0; i < n_sequence; ++i) {
    std::ostringstream line_info;
    line_info << "[Eudm][Result]" << i << " [";
    // 打印纵向动作序列（如 MADMA）
    for (const auto& a : action_script[i]) {
      line_info << DcpTree::RetLonActionName(a.lon);
    }
    line_info << "|";
    // 打印横向动作序列（如 KKLLK）
    for (const auto& a : action_script[i]) {
      line_info << DcpTree::RetLatActionName(a.lat);
    }
    line_info << "]";
    line_info << "[s:" << sim_res_[i] << "|r:" << risky_res_[i]
              << "|c:" << std::fixed << std::setprecision(3) << final_cost_[i]
              << "]";
    line_info << " " << sim_info_[i] << "\n";
    // 打印每层的详细代价（效率/安全/导航/权重）
    if (sim_res_[i]) {
      line_info << "[Eudm][Result][e;s;n;w:";
      for (const auto& c : progress_cost_[i]) {
        line_info << std::fixed << std::setprecision(2)
                  << c.efficiency.ego_to_desired_vel << "_"
                  << c.efficiency.leading_to_desired_vel << ";" << c.safety.rss
                  << "_" << c.safety.occu_lane << ";"
                  << c.navigation.lane_change_preference << ";" << c.weight;
        line_info << "|";
      }
      line_info << "]";
    }
    LOG(WARNING) << line_info.str();
  }
  LOG(WARNING) << "[Eudm][Result]Sim status: " << sim_success << " with "
               << num_valid_behaviors << " behaviors.";

  // 如果没有成功的行为，返回错误
  if (!sim_success) {
    LOG(ERROR) << "[Eudm][Fatal]Fail to find any valid behavior. Exit";
    return kWrongStatus;
  }

  // * 步骤6: 评估所有序列的代价，选择代价最低的优胜者
  if (EvaluateMultiThreadSimResults(&winner_id_, &winner_score_) != kSuccess) {
    LOG(ERROR)
        << "[Eudm][Fatal]fail to evaluate multi-thread sim results. Exit";
    return kWrongStatus;
  }
  return kSuccess;
}

// ============================================================================
//  仿真设置函数
// ============================================================================

/// @brief 根据动作更新自车的横向和纵向行为
///
/// 这是一个简单的翻译函数，将DCP动作转换为行为枚举。
ErrorType EudmPlanner::UpdateEgoBehaviorsUsingAction(
    const DcpAction& action, ForwardSimEgoAgent* ego_fsagent) const {
  LateralBehavior lat_behavior;
  LongitudinalBehavior lon_behavior;
  if (TranslateDcpActionToLonLatBehavior(action, &lat_behavior,
                                         &lon_behavior) != kSuccess) {
    printf("[Eudm]Translate action error\n");
    return kWrongStatus;
  }
  ego_fsagent->lat_behavior = lat_behavior;
  ego_fsagent->lon_behavior = lon_behavior;
  return kSuccess;
}

/// @brief 为整个场景（动作序列）设置仿真配置
///
/// 该函数在仿真开始前被调用一次，设置场景级别的参数。
///
/// ## 核心操作
///
/// ### 1. 序列类型识别
/// 调用ClassifyActionSeq确定序列的横向模式：
/// - AlwaysLaneKeep（全程保持）
/// - KeepThenChange（先保持后变道）
/// - AlwaysLaneChange（立即变道）
/// - ChangeThenCancel（变道后取消）
///
/// ### 2. 纵向参数配置（基于第二个动作）
/// **关键设计**：使用action_seq[1]（第二个动作）而非action_seq[0]（第一个动作）
/// 来确定纵向模式。这是因为第一个动作通常是当前正在执行的动作
/// （ongoing_action），其参数已经被设置。真正的新决策从第二个动作开始。
///
/// - **Accelerate**: desired_vel = min(current_vel + acc_gap, desired_velocity_)
///                 降低IDM最小间距和期望时距（乘以(1-aggressive_ratio)），
///                 让车辆更具侵略性地跟随前车
/// - **Decelerate**: desired_vel = max(current_vel - dec_gap, 0), clamped to desired_velocity_
/// - **Maintain**: desired_vel = min(current_vel, desired_velocity_)
///
/// ### 3. 设置横向范围
/// - 使用ego的cooperative_lat_range作为横向搜索范围
///
/// @param action_seq 动作序列
/// @param ego_fsagent [in/out] 自车仿真智能体
ErrorType EudmPlanner::UpdateSimSetupForScenario(
    const std::vector<DcpAction>& action_seq,
    ForwardSimEgoAgent* ego_fsagent) const {
  // * 步骤1: 分析动作序列类型
  common::LateralBehavior seq_lat_behavior;
  decimal_t operation_at_seconds;
  bool is_cancel_behavior;
  ClassifyActionSeq(action_seq, &operation_at_seconds, &seq_lat_behavior,
                    &is_cancel_behavior);
  ego_fsagent->operation_at_seconds = operation_at_seconds;
  ego_fsagent->is_cancel_behavior = is_cancel_behavior;
  ego_fsagent->seq_lat_behavior = seq_lat_behavior;

  // * 步骤2: 确定横向序列模式
  if (is_cancel_behavior) {
    // 变道后取消：长期行为=保持车道，模式=ChangeThenCancel
    ego_fsagent->lat_behavior_longterm = LateralBehavior::kLaneKeeping;
    ego_fsagent->seq_lat_mode = LatSimMode::kChangeThenCancel;
  } else {
    if (seq_lat_behavior == LateralBehavior::kLaneKeeping) {
      // 全程保持车道
      ego_fsagent->seq_lat_mode = LatSimMode::kAlwaysLaneKeep;
    } else if (action_seq.front().lat == DcpLatAction::kLaneKeeping) {
      // 第一个动作是保持，后面有变道 -> 先保持后变道
      ego_fsagent->seq_lat_mode = LatSimMode::kKeepThenChange;
    } else {
      // 第一个动作就是变道 -> 立即变道
      ego_fsagent->seq_lat_mode = LatSimMode::kAlwaysLaneChange;
    }
    ego_fsagent->lat_behavior_longterm = seq_lat_behavior;
  }

  // * 步骤3: 纵向参数配置
  // 获取当前车速（向下取整）
  decimal_t desired_vel = std::floor(ego_fsagent->vehicle.state().velocity);
  simulator::IntelligentDriverModel::Param idm_param_tmp;
  idm_param_tmp = ego_sim_param_.idm_param;

  // 使用action_seq[1]（第二个动作）来确定纵向模式
  // 原因：action_seq[0]是当前正在执行的动作，其参数已设置。
  //       真正的新决策从第二个动作开始。
  switch (action_seq[1].lon) {
    case DcpLonAction::kAccelerate: {
      // 加速模式：期望速度 = min(当前速度+加速gap, 全局期望速度)
      idm_param_tmp.kDesiredVelocity = std::min(
          desired_vel + cfg_.sim().acc_cmd_vel_gap(), desired_velocity_);
      // 减少最小间距和期望时距（更加侵略性地跟随前车）
      idm_param_tmp.kMinimumSpacing *=
          (1.0 - cfg_.sim().ego().lon_aggressive_ratio());
      idm_param_tmp.kDesiredHeadwayTime *=
          (1.0 - cfg_.sim().ego().lon_aggressive_ratio());
      break;
    }
    case DcpLonAction::kDecelerate: {
      // 减速模式：期望速度 = max(当前速度-减速gap, 0), 但不超过全局期望速度
      idm_param_tmp.kDesiredVelocity =
          std::min(std::max(desired_vel - cfg_.sim().dec_cmd_vel_gap(), 0.0),
                   desired_velocity_);
      break;
    }
    case DcpLonAction::kMaintain: {
      // 维持模式：期望速度 = min(当前速度, 全局期望速度)
      idm_param_tmp.kDesiredVelocity = std::min(desired_vel, desired_velocity_);
      break;
    }
    default: {
      printf("[Eudm]Error - Lon action not valid\n");
      assert(false);
    }
  }
  ego_fsagent->sim_param = ego_sim_param_;
  ego_fsagent->sim_param.idm_param = idm_param_tmp;
  // 设置横向搜索范围
  ego_fsagent->lat_range = cfg_.sim().ego().cooperative_lat_range();

  return kSuccess;
}

/// @brief 为当前动作层设置仿真配置
///
/// 该函数在每一层仿真开始前被调用，设置层级别的参数。
///
/// ## 操作流程
///
/// 1. **行为翻译**：将DcpAction转换为Lateral/Longitudinal行为
/// 2. **参考车道获取**：根据状态和横向行为，分别获取：
///    - current_lane: 当前保持车道的参考车道
///    - target_lane: 目标行为方向的参考车道（变道时为目标车道）
///    - longterm_lane: 长期行为方向的参考车道
///    每个车道附带Frenet坐标系变换器（StateTransformer）
/// 3. **变道间隙查找**（仅在变道行为时执行）：
///    - 获取目标车道上的前车和后车（Frenet状态）
///    - 记录前后车的ID为target_gap_ids
/// 4. **RSS预检查**（可选，启用时）：
///    - 在变道开始前，使用严格RSS参数检查目标间隙是否安全
///    - 前车间隙检查：自车作为后车，使用rss_config_strict_as_rear_
///    - 后车间隙检查：自车作为前车，使用rss_config_strict_as_front_
///    - 若任一检查失败，则返回错误（该变道动作不可行）
///
/// @param action 当前层的DCP动作
/// @param other_fsagent 周围车辆的仿真智能体集合
/// @param ego_fsagent [in/out] 自车仿真智能体
ErrorType EudmPlanner::UpdateSimSetupForLayer(
    const DcpAction& action, const ForwardSimAgentSet& other_fsagent,
    ForwardSimEgoAgent* ego_fsagent) const {
  // * 步骤1: 将DCP动作翻译为行为枚举
  LateralBehavior lat_behavior;
  LongitudinalBehavior lon_behavior;
  if (TranslateDcpActionToLonLatBehavior(action, &lat_behavior,
                                         &lon_behavior) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->lat_behavior = lat_behavior;
  ego_fsagent->lon_behavior = lon_behavior;

  // * 步骤2: 获取相关参考车道
  auto state = ego_fsagent->vehicle.state();
  // 计算参考车道前方的合理长度（基于当前速度）
  decimal_t forward_lane_len =
      std::min(std::max(state.velocity * cfg_.sim().ref_line().len_vel_coeff(),
                        cfg_.sim().ref_line().forward_len_min()),
               cfg_.sim().ref_line().forward_len_max());

  // 2a. 获取当前车道的参考车道（保持方向）
  common::Lane lane_current;
  if (map_itf_->GetRefLaneForStateByBehavior(
          state, std::vector<int>(), LateralBehavior::kLaneKeeping,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          &lane_current) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->current_lane = lane_current;
  ego_fsagent->current_stf = common::StateTransformer(lane_current);

  // 2b. 获取目标车道的参考车道（动作指定的方向）
  common::Lane lane_target;
  if (map_itf_->GetRefLaneForStateByBehavior(
          state, std::vector<int>(), ego_fsagent->lat_behavior,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          &lane_target) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->target_lane = lane_target;
  ego_fsagent->target_stf = common::StateTransformer(lane_target);

  // 2c. 获取长期目标车道的参考车道
  common::Lane lane_longterm;
  if (map_itf_->GetRefLaneForStateByBehavior(
          state, std::vector<int>(), ego_fsagent->lat_behavior_longterm,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          &lane_longterm) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->longterm_lane = lane_longterm;
  ego_fsagent->longterm_stf = common::StateTransformer(lane_longterm);

  // * 步骤3: 变道间隙查找
  if (ego_fsagent->lat_behavior != LateralBehavior::kLaneKeeping) {
    // 构建周围车辆集合用于间隙查找
    common::VehicleSet other_vehicles;
    for (const auto& pv : other_fsagent.forward_sim_agents) {
      other_vehicles.vehicles.insert(
          std::make_pair(pv.first, pv.second.vehicle));
    }

    // 查找目标车道上的前车和后车
    bool has_front_vehicle = false, has_rear_vehicle = false;
    common::Vehicle front_vehicle, rear_vehicle;
    common::FrenetState front_fs, rear_fs;
    map_itf_->GetLeadingAndFollowingVehiclesFrenetStateOnLane(
        ego_fsagent->target_lane, state, other_vehicles, &has_front_vehicle,
        &front_vehicle, &front_fs, &has_rear_vehicle, &rear_vehicle, &rear_fs);

    // 记录目标间隙的前后车辆ID（-1表示无该位置车辆）
    ego_fsagent->target_gap_ids(0) =
        has_front_vehicle ? front_vehicle.id() : -1;
    ego_fsagent->target_gap_ids(1) = has_rear_vehicle ? rear_vehicle.id() : -1;

    // * 步骤4: 严格RSS预检查（可选，用于快速筛除不安全的变道动作）
    if (cfg_.safety().rss_for_layers_enable()) {
      // 获取自车在目标车道Frenet坐标系下的状态
      common::FrenetState ego_fs;
      if (kSuccess != ego_fsagent->target_stf.GetFrenetStateFromState(
                          ego_fsagent->vehicle.state(), &ego_fs)) {
        return kWrongStatus;
      }
      // 计算自车的前后保险杠在Frenet S坐标下的位置
      decimal_t s_ego_fbumper = ego_fs.vec_s[0] +
                                ego_fsagent->vehicle.param().length() / 2.0 +
                                ego_fsagent->vehicle.param().d_cr();
      decimal_t s_ego_rbumper = ego_fs.vec_s[0] -
                                ego_fsagent->vehicle.param().length() / 2.0 +
                                ego_fsagent->vehicle.param().d_cr();

      // 检查前车间隙（自车作为后车，与前车的RSS距离）
      if (has_front_vehicle) {
        decimal_t s_front_rbumper = front_fs.vec_s[0] -
                                    front_vehicle.param().length() / 2.0 +
                                    front_vehicle.param().d_cr();
        decimal_t rss_dist;
        common::RssChecker::CalculateSafeLongitudinalDistance(
            ego_fsagent->vehicle.state().velocity,
            front_vehicle.state().velocity,
            common::RssChecker::LongitudinalDirection::Front,
            rss_config_strict_as_rear_, &rss_dist);

        if (s_front_rbumper - s_ego_fbumper < rss_dist) {
          // 违反严格RSS：与前车距离不足
          return kWrongStatus;
        }
      }

      // 检查后车间隙（自车作为前车，与后车的RSS距离）
      if (has_rear_vehicle) {
        decimal_t s_rear_fbumper = rear_fs.vec_s[0] +
                                   rear_vehicle.param().length() / 2.0 +
                                   rear_vehicle.param().d_cr();
        decimal_t rss_dist;
        common::RssChecker::CalculateSafeLongitudinalDistance(
            ego_fsagent->vehicle.state().velocity,
            rear_vehicle.state().velocity,
            common::RssChecker::LongitudinalDirection::Rear,
            rss_config_strict_as_front_, &rss_dist);

        if (s_ego_rbumper - s_rear_fbumper < rss_dist) {
          // 违反严格RSS：与后车距离不足
          return kWrongStatus;
        }
      }
    }
  }

  return kSuccess;
}

// ============================================================================
//  场景仿真（逐层前向仿真循环）
// ============================================================================

/// @brief [核心函数] 逐层仿真一个动作序列
///
/// 这是前向仿真的主循环函数，对动作序列中的每个动作（层）依次执行
/// 完整的仿真-检查-评估流程。
///
/// ## 仿真流程（对每层循环执行）
///
/// ```
/// for each action in action_seq_sim:
///   [1] UpdateSimSetupForLayer
///       - 设置Frenet坐标系、参考车道
///       - 查找目标间隙车辆
///       - 执行RSS预检查（可选，筛除不安全的变道）
///
///   [2] SimulateSingleAction
///       - 多步前向仿真（IDM + PurePursuit）
///       - 同时仿真自车和所有周围车辆
///
///   [3] StrictSafetyCheck
///       - 膨胀车辆模型后进行几何碰撞检查
///       - 若碰撞则标记失败
///
///   [4] CheckIfLateralActionFinished
///       - 检查横向变道动作是否已完成
///       - 若完成则调用UpdateLateralActionSequence更新后续序列
///
///   [5] 轨迹合并
///       - 将本层轨迹追加到多层轨迹容器
///
///   [6] CostFunction
///       - 计算本层的三维代价（效率/安全/导航）
///       - 应用时间折扣因子
///
/// end for
/// ```
///
/// ## 变道完成的在线更新
///
/// 如果在某层仿真中横向动作提前完成（例如变道在2秒内完成但该层有3秒），
/// 系统会动态调整后续动作序列：
/// - Left完成后：后续的Left变为Keep，Keep变为Right（避免二次同向变道）
/// - Right完成后：后续的Right变为Keep，Keep变为Left
///
/// @return kSuccess on successful simulation of all layers
ErrorType EudmPlanner::SimulateScenario(
    const common::Vehicle& ego_vehicle,
    const ForwardSimAgentSet& surrounding_fsagents,
    const std::vector<DcpAction>& action_seq, const int& seq_id,
    const int& sub_seq_id, std::vector<int>* sub_sim_res,
    std::vector<int>* sub_risky_res, std::vector<std::string>* sub_sim_info,
    std::vector<std::vector<CostStructure>>* sub_progress_cost,
    std::vector<CostStructure>* sub_tail_cost,
    vec_E<vec_E<common::Vehicle>>* sub_forward_trajs,
    std::vector<std::vector<LateralBehavior>>* sub_forward_lat_behaviors,
    std::vector<std::vector<LongitudinalBehavior>>* sub_forward_lon_behaviors,
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>*
        sub_surround_trajs) {
  // * 初始化多层轨迹追踪容器
  vec_E<common::Vehicle> ego_traj_multilayers{ego_vehicle};

  std::unordered_map<int, vec_E<common::Vehicle>> surround_trajs_multilayers;
  for (const auto& p_fsa : surrounding_fsagents.forward_sim_agents) {
    surround_trajs_multilayers.insert(std::pair<int, vec_E<common::Vehicle>>(
        p_fsa.first, vec_E<common::Vehicle>({p_fsa.second.vehicle})));
  }

  std::vector<LateralBehavior> ego_lat_behavior_multilayers;
  std::vector<LongitudinalBehavior> ego_lon_behavior_multilayers;
  std::vector<CostStructure> cost_multilayers;
  std::set<int> risky_ids_multilayers;

  // * 设置场景级别的仿真配置（纵向参数、序列模式等）
  ForwardSimEgoAgent ego_fsagent_this_layer;
  ego_fsagent_this_layer.vehicle = ego_vehicle;
  UpdateSimSetupForScenario(action_seq, &ego_fsagent_this_layer);

  ForwardSimAgentSet surrounding_fsagents_this_layer = surrounding_fsagents;

  int action_ref_lane_id = ego_lane_id_;  // 动作参考车道ID（变道完成前后会更新）
  bool is_sub_seq_risky = false;
  std::vector<DcpAction> action_seq_sim = action_seq;  // 可变的动作序列（变道完成后可能被修改）

  // * 逐层仿真循环
  for (int i = 0; i < static_cast<int>(action_seq_sim.size()); ++i) {
    auto action_this_layer = action_seq_sim[i];

    // [1] 设置本层的仿真环境
    if (kSuccess != UpdateSimSetupForLayer(action_this_layer,
                                           surrounding_fsagents_this_layer,
                                           &ego_fsagent_this_layer)) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Update setup F)");
      return kWrongStatus;
    }

    // [2] 执行本层的多步前向仿真
    vec_E<common::Vehicle> ego_traj_multisteps;
    std::unordered_map<int, vec_E<common::Vehicle>> surround_trajs_multisteps;
    if (SimulateSingleAction(action_this_layer, ego_fsagent_this_layer,
                             surrounding_fsagents_this_layer,
                             &ego_traj_multisteps,
                             &surround_trajs_multisteps) != kSuccess) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] +=
          std::string("(Sim ") + std::to_string(i) + std::string(" F)");
      return kWrongStatus;
    }

    // 更新仿真智能体的状态为本层最后一个状态
    ego_fsagent_this_layer.vehicle.set_state(
        ego_traj_multisteps.back().state());
    for (auto it = surrounding_fsagents_this_layer.forward_sim_agents.begin();
         it != surrounding_fsagents_this_layer.forward_sim_agents.end(); ++it) {
      it->second.vehicle.set_state(
          surround_trajs_multisteps.at(it->first).back().state());
    }

    // [3] 严格几何碰撞检查
    bool is_strictly_safe = false;
    int collided_id = 0;
    TicToc timer;
    if (StrictSafetyCheck(ego_traj_multisteps, surround_trajs_multisteps,
                          &is_strictly_safe, &collided_id) != kSuccess) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Check F)");
      return kWrongStatus;
    }

    if (!is_strictly_safe) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Strict F:") +
                                     std::to_string(collided_id) +
                                     std::string(")");
      return kWrongStatus;
    }

    // [4] 检查横向动作是否完成，若完成则动态更新后续动作序列
    int current_lane_id;
    if (CheckIfLateralActionFinished(
            ego_fsagent_this_layer.vehicle.state(), action_ref_lane_id,
            ego_fsagent_this_layer.lat_behavior, &current_lane_id)) {
      action_ref_lane_id = current_lane_id;
      if (kSuccess != UpdateLateralActionSequence(i, &action_seq_sim)) {
        (*sub_sim_res)[sub_seq_id] = 0;
        (*sub_sim_info)[sub_seq_id] += std::string("(Update Lat F)");
        return kWrongStatus;
      }
      // 变道完成后，长期行为恢复为保持车道
      ego_fsagent_this_layer.lat_behavior_longterm =
          LateralBehavior::kLaneKeeping;
    }

    // [5] 将本层轨迹合并到多层轨迹记录中
    ego_traj_multilayers.insert(ego_traj_multilayers.end(),
                                ego_traj_multisteps.begin(),
                                ego_traj_multisteps.end());
    ego_lat_behavior_multilayers.push_back(ego_fsagent_this_layer.lat_behavior);
    ego_lon_behavior_multilayers.push_back(ego_fsagent_this_layer.lon_behavior);

    for (const auto& v : surrounding_fsagents_this_layer.forward_sim_agents) {
      int id = v.first;
      surround_trajs_multilayers.at(id).insert(
          surround_trajs_multilayers.at(id).end(),
          surround_trajs_multisteps.at(id).begin(),
          surround_trajs_multisteps.at(id).end());
    }

    // [6] 计算本层的三维代价
    CostStructure cost;
    bool verbose = false;
    std::set<int> risky_ids;
    bool is_risky_action = false;
    CostFunction(action_this_layer, ego_fsagent_this_layer,
                 surrounding_fsagents_this_layer, ego_traj_multisteps,
                 surround_trajs_multisteps, verbose, &cost, &is_risky_action,
                 &risky_ids);

    // 收集风险ID（RSS不安全的车辆ID）
    if (is_risky_action) {
      is_sub_seq_risky = true;
      for (const auto& id : risky_ids) {
        risky_ids_multilayers.insert(id);
      }
    }
    // 应用时间折扣因子：越远期的代价，权重越小
    cost.weight = cost.weight * pow(cfg_.cost().discount_factor(), i);
    cost.valid_sample_index_ub = ego_traj_multilayers.size();
    cost_multilayers.push_back(cost);
  }

  // * 输出仿真结果
  (*sub_sim_res)[sub_seq_id] = 1;
  (*sub_risky_res)[sub_seq_id] = is_sub_seq_risky ? 1 : 0;
  if (is_sub_seq_risky) {
    std::string risky_id_list;
    for (auto it = risky_ids_multilayers.begin();
         it != risky_ids_multilayers.end(); it++) {
      risky_id_list += " " + std::to_string(*it);
    }
    (*sub_sim_info)[sub_seq_id] += std::string("(Risky)") + risky_id_list;
  }
  (*sub_progress_cost)[sub_seq_id] = cost_multilayers;
  (*sub_forward_trajs)[sub_seq_id] = ego_traj_multilayers;
  (*sub_forward_lat_behaviors)[sub_seq_id] = ego_lat_behavior_multilayers;
  (*sub_forward_lon_behaviors)[sub_seq_id] = ego_lon_behavior_multilayers;
  (*sub_surround_trajs)[sub_seq_id] = surround_trajs_multilayers;

  return kSuccess;
}

// ============================================================================
//  动作序列仿真（多线程入口）
// ============================================================================

/// @brief 仿真单个动作序列（多线程入口函数）
///
/// 首先检查该序列是否在预筛除列表中，若是则跳过。
/// 然后调用SimulateScenario完成实际的逐层仿真。
/// 结果直接写入多线程容器（使用seq_id作为索引）。
///
/// @param ego_vehicle 自车初始状态
/// @param surrounding_fsagents 周围车辆智能体
/// @param action_seq 待仿真的动作序列
/// @param seq_id 序列索引
ErrorType EudmPlanner::SimulateActionSequence(
    const common::Vehicle& ego_vehicle,
    const ForwardSimAgentSet& surrounding_fsagents,
    const std::vector<DcpAction>& action_seq, const int& seq_id) {
  // 检查是否在预筛除列表中
  if (pre_deleted_seq_ids_.find(seq_id) != pre_deleted_seq_ids_.end()) {
    sim_res_[seq_id] = 0;
    sim_info_[seq_id] = std::string("(Pre-deleted)");
    return kWrongStatus;
  }

  // 为每个ego序列，可以进一步分支出多个子线程。
  // 当前版本使用n_sub_threads = 1，即不做进一步分支。
  // 未来可以考虑基于行为概率分布采样创建多个子场景。
  int n_sub_threads = 1;

  // 分配子线程结果容器
  std::vector<int> sub_sim_res(n_sub_threads);
  std::vector<int> sub_risky_res(n_sub_threads);
  std::vector<std::string> sub_sim_info(n_sub_threads);
  std::vector<std::vector<CostStructure>> sub_progress_cost(n_sub_threads);
  std::vector<CostStructure> sub_tail_cost(n_sub_threads);
  vec_E<vec_E<common::Vehicle>> sub_forward_trajs(n_sub_threads);
  std::vector<std::vector<LateralBehavior>> sub_forward_lat_behaviors(
      n_sub_threads);
  std::vector<std::vector<LongitudinalBehavior>> sub_forward_lon_behaviors(
      n_sub_threads);
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> sub_surround_trajs(
      n_sub_threads);

  // 执行场景仿真（当前仅使用默认子场景0）
  SimulateScenario(ego_vehicle, surrounding_fsagents, action_seq, seq_id, 0,
                   &sub_sim_res, &sub_risky_res, &sub_sim_info,
                   &sub_progress_cost, &sub_tail_cost, &sub_forward_trajs,
                   &sub_forward_lat_behaviors, &sub_forward_lon_behaviors,
                   &sub_surround_trajs);

  // 若仿真失败，记录原因
  if (sub_sim_res.front() == 0) {
    sim_res_[seq_id] = 0;
    sim_info_[seq_id] = sub_sim_info.front();
    return kWrongStatus;
  }

  // 将默认子场景的结果写入主结果容器
  sim_res_[seq_id] = 1;
  risky_res_[seq_id] = sub_risky_res.front();
  sim_info_[seq_id] = sub_sim_info.front();
  progress_cost_[seq_id] = sub_progress_cost.front();
  tail_cost_[seq_id] = sub_tail_cost.front();
  forward_trajs_[seq_id] = sub_forward_trajs.front();
  forward_lat_behaviors_[seq_id] = sub_forward_lat_behaviors.front();
  forward_lon_behaviors_[seq_id] = sub_forward_lon_behaviors.front();
  surround_trajs_[seq_id] = sub_surround_trajs.front();

  return kSuccess;
}

// ============================================================================
//  横向动作序列动态更新
// ============================================================================

/// @brief 在横向动作完成后动态更新后续动作序列
///
/// 当变道操作提前完成时（例如左变道只用了2秒，而该层有3秒），
/// 需要修改后续动作序列中的横向方向，避免执行已经完成的动作。
///
/// 转换规则（以左变道为例）：
/// - LLLLL -> LLKKK: 将后续所有Left改为Keep（因为已经变到目标车道）
/// - LLKKK -> LLRRR: 将后续的Keep改为Right（变道完成后通常会回正或准备反向变道）
/// - LLRRR -> x (错误): 不允许连续的左右横向跳变
///
/// 右变道类似：
/// - RRRRR -> RRKKK
/// - RRKKK -> RRLLL
///
/// @param cur_idx 当前已完成动作在序列中的索引
/// @param action_seq [in/out] 待更新的动作序列
ErrorType EudmPlanner::UpdateLateralActionSequence(
    const int cur_idx, std::vector<DcpAction>* action_seq) const {
  // 如果是最后一层，无需更新
  if (cur_idx == static_cast<int>(action_seq->size()) - 1) {
    return kSuccess;
  }

  switch ((*action_seq)[cur_idx].lat) {
    case DcpLatAction::kLaneKeeping: {
      // 保持车道不需要更新
      break;
    }
    case DcpLatAction::kLaneChangeLeft: {
      // 左变道已完成，更新后续动作
      for (int i = cur_idx + 1; i < static_cast<int>(action_seq->size()); ++i) {
        if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeLeft) {
          // LLLLL -> LLKKK: 后续Left改为Keep
          (*action_seq)[i].lat = DcpLatAction::kLaneKeeping;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneKeeping) {
          // LLKKK -> LLRRR: 后续Keep改为Right（模拟变道后回正趋势）
          (*action_seq)[i].lat = DcpLatAction::kLaneChangeRight;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeRight) {
          // LLRRR -> x: 不允许连续的左右跳变
          return kWrongStatus;
        }
      }
      break;
    }
    case DcpLatAction::kLaneChangeRight: {
      // 右变道已完成，更新后续动作
      for (int i = cur_idx + 1; i < static_cast<int>(action_seq->size()); ++i) {
        if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeRight) {
          // RRRRR -> RRKKK
          (*action_seq)[i].lat = DcpLatAction::kLaneKeeping;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneKeeping) {
          // RRKKK -> RRLLL
          (*action_seq)[i].lat = DcpLatAction::kLaneChangeLeft;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeLeft) {
          // RRLLL -> x
          return kWrongStatus;
        }
      }
      break;
    }

    default: {
      std::cout << "[Eudm]Error - Invalid lateral behavior" << std::endl;
      assert(false);
    }
  }

  return kSuccess;
}

/// @brief 检查横向变道动作是否已经完成
///
/// 通过比较车辆当前位置对应的车道ID与动作开始时的参考车道ID来判断。
///
/// 判断逻辑：
/// 1. 如果当前横向行为是LaneKeeping，则无需判断（返回false）
/// 2. 使用地图接口查询车辆当前状态对应的最近车道ID
/// 3. 获取动作参考车道ID的"可能到达车道"列表（考虑横向行为方向）
/// 4. 如果当前车道ID在"可能到达车道"列表中，说明变道已完成
///
/// 例如：从车道A左变道，potential_lane_ids = [A的左侧相邻车道及其子车道]。
/// 如果当前车道ID在这个列表中，则左变道完成。
///
/// @return true表示横向动作已完成
bool EudmPlanner::CheckIfLateralActionFinished(
    const common::State& cur_state, const int& action_ref_lane_id,
    const LateralBehavior& lat_behavior, int* current_lane_id) const {
  if (lat_behavior == LateralBehavior::kLaneKeeping) {
    return false;
  }

  // 查询车辆当前位置对应的最近车道ID
  decimal_t current_lane_dist;
  decimal_t arc_len;
  map_itf_->GetNearestLaneIdUsingState(cur_state.ToXYTheta(),
                                       std::vector<int>(), current_lane_id,
                                       &current_lane_dist, &arc_len);

  // 获取动作参考车道按照横向行为方向可能到达的车道ID列表
  std::vector<int> potential_lane_ids;
  GetPotentialLaneIds(action_ref_lane_id, lat_behavior, &potential_lane_ids);

  // 如果当前车道ID在预期列表中，说明变道已完成
  auto it = std::find(potential_lane_ids.begin(), potential_lane_ids.end(),
                      *current_lane_id);
  if (it == potential_lane_ids.end()) {
    return false;
  } else {
    return true;
  }
}

// ============================================================================
//  规划循环入口
// ============================================================================

/// @brief [核心函数] EUDM规划循环的顶层入口
///
/// 每次规划周期调用一次，管理从数据获取到结果输出的完整流水线。
///
/// ## 执行流程
///
/// 1. **获取自车信息**：
///    - 从地图接口获取自车状态、ID、时间戳
///    - 获取自车当前位置对应的车道ID
///
/// 2. **获取RSS参考车道**：
///    - 获取保持车道方向的参考车道（130m前向 + 130m后向）
///    - 用于RSS安全检查的Frenet坐标变换
///
/// 3. **预筛除矛盾动作序列**：
///    - 遍历所有候选序列，标记逻辑不一致的序列
///    - 例如：序列中连续出现左变道后右变道（LKR模式）被筛除
///
/// 4. **执行EUDM核心仿真**：
///    - 调用RunEudm()执行多线程并行仿真和优胜选择
///
/// 5. **输出优胜结果**：
///    - 打印优胜序列的详细信息和代价
///    - 记录总时间开销
///
/// @return kSuccess if the planning cycle completes successfully
ErrorType EudmPlanner::RunOnce() {
  TicToc timer_runonce;

  // * 步骤1: 获取自车信息
  if (!map_itf_) {
    LOG(ERROR) << "[Eudm]map interface not initialized. Exit";
    return kWrongStatus;
  }

  if (map_itf_->GetEgoVehicle(&ego_vehicle_) != kSuccess) {
    LOG(ERROR) << "[Eudm]no ego vehicle found.";
    return kWrongStatus;
  }
  ego_id_ = ego_vehicle_.id();
  time_stamp_ = ego_vehicle_.state().time_stamp;

  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]------ Eudm Cycle Begins (stamp): " << time_stamp_
               << " ------- ";

  // 获取自车当前位置对应的车道ID
  int ego_lane_id_by_pos = kInvalidLaneId;
  if (map_itf_->GetEgoLaneIdByPosition(std::vector<int>(),
                                       &ego_lane_id_by_pos) != kSuccess) {
    LOG(ERROR) << "[Eudm]Fatal (Exit) ego not on lane.";
    return kWrongStatus;
  }

  // 打印自车状态和配置信息
  LOG(WARNING) << std::fixed << std::setprecision(3)
               << "[Eudm][Input]Ego plan state (x,y,theta,v,a,k):("
               << ego_vehicle_.state().vec_position[0] << ","
               << ego_vehicle_.state().vec_position[1] << ","
               << ego_vehicle_.state().angle << ","
               << ego_vehicle_.state().velocity << ","
               << ego_vehicle_.state().acceleration << ","
               << ego_vehicle_.state().curvature << ")"
               << " lane id:" << ego_lane_id_by_pos;

  LOG(WARNING) << "[Eudm][Setup]Desired vel:" << desired_velocity_
               << " sim_time total:" << sim_time_total_
               << " lc info[f_l,f_r,us_ol,us_or,solid_l,solid_r]:"
               << lc_info_.forbid_lane_change_left << ","
               << lc_info_.forbid_lane_change_right << ","
               << lc_info_.lane_change_left_unsafe_by_occu << ","
               << lc_info_.lane_change_right_unsafe_by_occu << ","
               << lc_info_.left_solid_lane << "," << lc_info_.right_solid_lane;

  ego_lane_id_ = ego_lane_id_by_pos;

  // * 步骤2: 获取RSS参考车道（用于RSS安全检查的Frenet坐标变换）
  const decimal_t forward_rss_check_range = 130.0;
  const decimal_t backward_rss_check_range = 130.0;
  const decimal_t forward_lane_len = forward_rss_check_range;
  const decimal_t backward_lane_len = backward_rss_check_range;
  if (map_itf_->GetRefLaneForStateByBehavior(
          ego_vehicle_.state(), std::vector<int>(),
          LateralBehavior::kLaneKeeping, forward_lane_len, backward_lane_len,
          false, &rss_lane_) != kSuccess) {
    LOG(ERROR) << "[Eudm]No Rss lane available. Rss disabled";
  }

  if (rss_lane_.IsValid()) {
    rss_stf_ = common::StateTransformer(rss_lane_);
  }

  // * 步骤3: 预筛除逻辑矛盾的动作序列
  // 筛除规则：序列中不允许出现连续相反方向的横向动作
  // 例如：左变道后立即右变道（LLRRR或LLRKK），或者右变道后立即左变道
  pre_deleted_seq_ids_.clear();
  int n_sequence = dcp_tree_ptr_->action_script().size();
  for (int i = 0; i < n_sequence; i++) {
    auto action_seq = dcp_tree_ptr_->action_script()[i];
    int num_actions = action_seq.size();
    for (int j = 1; j < num_actions; j++) {
      // 检查相邻两个动作是否存在方向冲突
      if ((action_seq[j - 1].lat == DcpLatAction::kLaneChangeLeft &&
           action_seq[j].lat == DcpLatAction::kLaneChangeRight) ||
          (action_seq[j - 1].lat == DcpLatAction::kLaneChangeRight &&
           action_seq[j].lat == DcpLatAction::kLaneChangeLeft)) {
        // 标记为预筛除
        pre_deleted_seq_ids_.insert(i);
      }
    }
  }

  // * 步骤4: 执行EUDM核心仿真与评估
  TicToc timer;
  if (RunEudm() != kSuccess) {
    LOG(ERROR) << std::fixed << std::setprecision(4)
               << "[Eudm]****** Eudm Cycle FAILED (stamp): " << time_stamp_
               << " time cost " << timer.toc() << " ms.";
    return kWrongStatus;
  }

  // * 步骤5: 输出优胜结果
  auto action_script = dcp_tree_ptr_->action_script();
  std::ostringstream line_info;
  line_info << "[Eudm]SUCCESS id:" << winner_id_ << " [";
  for (const auto& a : action_script[winner_id_]) {
    line_info << DcpTree::RetLonActionName(a.lon);
  }
  line_info << "|";
  for (const auto& a : action_script[winner_id_]) {
    line_info << DcpTree::RetLatActionName(a.lat);
  }
  line_info << "] cost: " << std::fixed << std::setprecision(3) << winner_score_
            << " time cost: " << timer.toc() << " ms.";
  LOG(WARNING) << line_info.str();

  time_cost_ = timer_runonce.toc();
  return kSuccess;
}

// ============================================================================
//  代价评估函数
// ============================================================================

/// @brief 评估单个动作序列的总代价（简单累加）
///
/// 将过程代价（每层的progress_cost）和尾部代价（tail_cost）累加。
/// tail_cost当前未使用（全为0），预留给未来扩展。
ErrorType EudmPlanner::EvaluateSinglePolicyTrajs(
    const std::vector<CostStructure>& progress_cost,
    const CostStructure& tail_cost, const std::vector<DcpAction>& action_seq,
    decimal_t* score) {
  decimal_t score_tmp = 0.0;
  for (const auto& c : progress_cost) {
    score_tmp += c.ave();  // 累加每层的加权平均代价
  }
  *score = score_tmp + tail_cost.ave();
  return kSuccess;
}

/// @brief 评估所有线程的仿真结果，选择最优动作序列
///
/// 遍历所有成功仿真的序列，计算每个的总代价，选出代价最小的作为优胜者。
/// 仿-真失败的序列直接跳过。
///
/// @param winner_id [out] 优胜序列的索引
/// @param winner_cost [out] 优胜序列的代价
ErrorType EudmPlanner::EvaluateMultiThreadSimResults(int* winner_id,
                                                     decimal_t* winner_cost) {
  decimal_t min_cost = kInf;
  int best_id = 0;
  int num_sequences = sim_res_.size();

  for (int i = 0; i < num_sequences; ++i) {
    // 跳过仿真失败的序列
    if (sim_res_[i] == 0) {
      continue;
    }

    decimal_t cost = 0.0;
    auto action_seq = dcp_tree_ptr_->action_script()[i];
    EvaluateSinglePolicyTrajs(progress_cost_[i], tail_cost_[i], action_seq,
                              &cost);
    final_cost_[i] = cost;

    // 选择代价最低的
    if (cost < min_cost) {
      min_cost = cost;
      best_id = i;
    }
  }

  *winner_cost = min_cost;
  *winner_id = best_id;
  return kSuccess;
}

// ============================================================================
//  RSS安全评估
// ============================================================================

/// @brief 评估两辆车轨迹之间的RSS安全状态
///
/// 对轨迹A和轨迹B的每对对应位置的状态进行RSS安全检查。
/// 使用rss_lane_作为参考车道的Frenet坐标变换基础。
///
/// RSS违反时的代价计算：
/// - **TooFast**（作为后车太快逼近前车）:
///   cost = linear_coeff * vel * 10^(power_coeff * |vel - rss_vel_up|)
///   超速越多，代价呈指数增长
/// - **TooSlow**（作为前车太慢导致后车追尾风险）:
///   cost = linear_coeff * vel * 10^(power_coeff * |vel - rss_vel_low|)
///   速度不足越多，代价呈指数增长
///
/// @param cost [out] RSS安全代价总和
/// @param is_rss_safe [out] 所有检查点是否都通过了RSS安全
/// @param risky_id [out] 若有违反，记录对方的ID
ErrorType EudmPlanner::EvaluateSafetyStatus(
    const vec_E<common::Vehicle>& traj_a, const vec_E<common::Vehicle>& traj_b,
    decimal_t* cost, bool* is_rss_safe, int* risky_id) {
  if (traj_a.size() != traj_b.size()) {
    return kWrongStatus;
  }
  // 如果RSS检查未启用或参考车道无效，跳过检查
  if (!cfg_.safety().rss_check_enable() || !rss_lane_.IsValid()) {
    return kSuccess;
  }

  int num_states = static_cast<int>(traj_a.size());
  decimal_t cost_tmp = 0.0;
  bool ret_is_rss_safe = true;
  const int check_per_state = 1;  // 每个状态都检查（=2时每隔一个状态检查）

  for (int i = 0; i < num_states; i += check_per_state) {
    bool is_rss_safe = true;
    common::RssChecker::LongitudinalViolateType type;
    decimal_t rss_vel_low, rss_vel_up;
    // 执行RSS安全检查
    common::RssChecker::RssCheck(traj_a[i], traj_b[i], rss_stf_, rss_config_,
                                 &is_rss_safe, &type, &rss_vel_low,
                                 &rss_vel_up);

    if (!is_rss_safe) {
      ret_is_rss_safe = false;
      *risky_id = traj_b.size() ? traj_b[0].id() : 0;

      // 如果启用了RSS代价计算
      if (cfg_.cost().safety().rss_cost_enable()) {
        if (type == common::RssChecker::LongitudinalViolateType::TooFast) {
          // 自车作为后车太快地逼近前车
          cost_tmp +=
              cfg_.cost().safety().rss_over_speed_linear_coeff() *
              traj_a[i].state().velocity *
              pow(10, cfg_.cost().safety().rss_over_speed_power_coeff() *
                          fabs(traj_a[i].state().velocity - rss_vel_up));
        } else if (type ==
                   common::RssChecker::LongitudinalViolateType::TooSlow) {
          // 自车作为前车太慢导致后车追尾风险
          cost_tmp +=
              cfg_.cost().safety().rss_lack_speed_linear_coeff() *
              traj_a[i].state().velocity *
              pow(10, cfg_.cost().safety().rss_lack_speed_power_coeff() *
                          fabs(traj_a[i].state().velocity - rss_vel_low));
        }
      }
    }
  }

  *is_rss_safe = ret_is_rss_safe;
  *cost = cost_tmp;
  return kSuccess;
}

/// @brief 严格安全检查：几何碰撞检测
///
/// 对自车轨迹和每个周围车辆轨迹的每对同时刻状态进行碰撞检测。
/// 碰撞检测前先将车辆模型按配置的膨胀参数进行膨胀（inflation_w/h），
/// 用于增加安全裕度。
///
/// @param ego_traj 自车轨迹
/// @param surround_trajs 周围车辆轨迹（key=车辆ID, value=轨迹）
/// @param is_safe [out] 是否安全（无碰撞）
/// @param collided_id [out] 若碰撞，对方的车辆ID
ErrorType EudmPlanner::StrictSafetyCheck(
    const vec_E<common::Vehicle>& ego_traj,
    const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
    bool* is_safe, int* collided_id) {
  // 如果严格检查未启用，直接返回安全
  if (!cfg_.safety().strict_check_enable()) {
    *is_safe = true;
    return kSuccess;
  }

  int num_points_ego = ego_traj.size();
  if (num_points_ego == 0) {
    *is_safe = true;
    return kSuccess;
  }

  // 遍历每个周围车辆
  for (auto it = surround_trajs.begin(); it != surround_trajs.end(); it++) {
    int num_points_other = it->second.size();
    // 轨迹长度不一致说明仿真记录不完整
    if (num_points_other != num_points_ego) {
      *is_safe = false;
      LOG(ERROR) << "[Eudm]unsafe due to incomplete sim record for vehicle";
      return kSuccess;
    }

    // 逐时刻检查碰撞
    for (int i = 0; i < num_points_ego; i++) {
      common::Vehicle inflated_a, inflated_b;
      // 膨胀自车模型
      common::SemanticsUtils::InflateVehicleBySize(
          ego_traj[i], cfg_.safety().strict().inflation_w(),
          cfg_.safety().strict().inflation_h(), &inflated_a);
      // 膨胀对方模型
      common::SemanticsUtils::InflateVehicleBySize(
          it->second[i], cfg_.safety().strict().inflation_w(),
          cfg_.safety().strict().inflation_h(), &inflated_b);

      // 几何碰撞检测
      bool is_collision = false;
      map_itf_->CheckCollisionUsingState(inflated_a.param(), inflated_a.state(),
                                         inflated_b.param(), inflated_b.state(),
                                         &is_collision);

      if (is_collision) {
        *is_safe = false;
        *collided_id = it->second[i].id();
        return kSuccess;
      }
    }
  }

  *is_safe = true;
  return kSuccess;
}

// ============================================================================
//  代价函数（Core Cost Function）
// ============================================================================

/// @brief [核心函数] 三维代价函数：EUDM决策的核心评估机制
///
/// 计算一个动作层（时间层）的完整代价，包含三个维度：
/// 效率、安全、导航。这是EUDM区别于MPDM的关键之一——
/// 更精细、更多维度的代价建模。
///
/// ## 1. 效率代价（Efficiency Cost）
///
/// ### 分量1.1：自车速度与期望速度的偏差
///
/// ```
/// 如果 v_ego < v_desired:
///   eco_ego = unit_cost * |v_ego - v_desired|
/// 如果 v_ego > v_desired + threshold:
///   eco_ego = unit_cost_over * |v_ego - v_desired - threshold|
/// ```
///
/// 设计意图：低速时不达期望速度有代价，高速时只有在超过容忍阈值后才有代价。
///
/// ### 分量1.2：前车速度对自车期望速度的阻碍
///
/// ```
/// 如果 v_ego < v_desired AND v_leading < v_desired AND dist < dist_th:
///   blockage = v_ego - v_leading (自车被前车阻碍的速度差)
///   lead_deficit = v_desired - v_leading (前车与期望速度的差距)
///   eco_lead = max(min_ratio, distance_residual_ratio) *
///              (blockage_unit_cost * blockage + lead_deficit_unit_cost * lead_deficit)
/// ```
///
/// distance_residual_ratio: 距离残留比，表示当前距离在前车有效影响范围中的比例。
/// 距离越近，比值越大，前车影响越强。
///
/// ## 2. 安全代价（Safety Cost）
///
/// ### 分量2.1：RSS安全检查
/// 对自车和每个周围车辆的轨迹对执行RSS安全检查（EvaluateSafetyStatus）。
/// 若RSS不安全，根据违规类型计算指数增长的安全代价。
/// 代价项累计到safety.rss。
///
/// ### 分量2.2：占位碰撞风险
/// 如果自车准备变道但目标车道被标记为禁止（forbid），
/// 施加速度比例的占位代价：
///   occu_cost = ego_velocity * occu_lane_unit_cost
///
/// ## 3. 导航代价（Navigation Cost）
///
/// ### 分量3.1：变道操作代价
/// 变道操作有固定代价（优先保持当前车道）：
///   lc_cost = max(min_vel, ego_velocity) * lc_unit_cost
/// 其中lc_unit_cost对左/右变道可以不同（考虑道路规则偏好）。
///
/// ### 分量3.2：变道取消惩罚
/// 如果序列包含变道后取消的模式：
///   cancel_cost = max(min_vel, ego_velocity) * cancel_unit_cost
///
/// ### 分量3.3：推荐变道的奖励（负代价）
/// 如果系统推荐变道（recommend_lc_left/right）且序列方向匹配：
///   reward = -max(min_vel, ego_velocity) * recommendation_reward
///
/// ### 分量3.4：延迟执行惩罚
/// 如果推荐变道但当前动作不是变道（延迟执行）：
///   late_penalty = max(min_vel, ego_velocity) * late_operate_unit_cost
///
/// ## 代价权重
/// 最后，cost.weight = duration（该层的时间长度），
/// 使得代价与时间长度成正比。
/// 上层的SimulateScenario会额外应用pow(discount_factor, i)进行时间折扣。
///
/// @param action 当前层动作
/// @param ego_fsagent 自车仿真智能体（包含行为和状态信息）
/// @param other_fsagent 周围车辆仿真智能体
/// @param ego_traj 自车在该层的时间步轨迹
/// @param surround_trajs 周围车辆在该层的时间步轨迹
/// @param verbose 是否详细输出（当前固定false）
/// @param cost [out] 输出的三维代价结构
/// @param is_risky [out] 是否存在RSS安全隐患
/// @param risky_ids [out] 产生安全隐患的车辆ID集合
ErrorType EudmPlanner::CostFunction(
    const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent,
    const ForwardSimAgentSet& other_fsagent,
    const vec_E<common::Vehicle>& ego_traj,
    const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
    bool verbose, CostStructure* cost, bool* is_risky,
    std::set<int>* risky_ids) {
  decimal_t duration = action.t;

  auto ego_lon_behavior_this_layer = ego_fsagent.lon_behavior;
  auto ego_lat_behavior_this_layer = ego_fsagent.lat_behavior;

  auto seq_lat_behavior = ego_fsagent.seq_lat_behavior;
  auto is_cancel_behavior = ego_fsagent.is_cancel_behavior;

  // 构建车辆集合（用于前车查询）
  common::VehicleSet vehicle_set;
  for (const auto& v : other_fsagent.forward_sim_agents) {
    vehicle_set.vehicles.insert(std::make_pair(v.first, v.second.vehicle));
  }

  decimal_t ego_velocity = ego_fsagent.vehicle.state().velocity;

  // ========================================================================
  // 1. 效率代价
  // ========================================================================
  CostStructure cost_tmp;

  // 1.1 自车速度与期望速度的偏差
  // 公式：若 v_ego < v_desired，罚 = c1 * |v_ego - v_desired|
  //      若 v_ego > v_desired + vth，罚 = c2 * |v_ego - v_desired - vth|
  // 代价单位为速度（最后乘以duration）
  if (ego_fsagent.vehicle.state().velocity < desired_velocity_) {
    cost_tmp.efficiency.ego_to_desired_vel =
        cfg_.cost().effciency().ego_lack_speed_to_desired_unit_cost() *
        fabs(ego_fsagent.vehicle.state().velocity - desired_velocity_);
  } else {
    if (ego_fsagent.vehicle.state().velocity >
        desired_velocity_ +
            cfg_.cost().effciency().ego_desired_speed_tolerate_gap()) {
      cost_tmp.efficiency.ego_to_desired_vel =
          cfg_.cost().effciency().ego_over_speed_to_desired_unit_cost() *
          fabs(ego_fsagent.vehicle.state().velocity - desired_velocity_ -
               cfg_.cost().effciency().ego_desired_speed_tolerate_gap());
    }
  }

  // 1.2 前车速度对期望速度的阻碍
  // 公式：若 v_ego < v_desired 且 v_leading < v_desired 且 dist < dist_th，
  //      则罚 = ratio * (c3 * blockage + c4 * lead_deficit)
  //      其中 ratio = max(min_ratio, distance_residual_ratio)
  common::Vehicle leading_vehicle;
  decimal_t distance_residual_ratio = 0.0;
  if (map_itf_->GetLeadingVehicleOnLane(
          ego_fsagent.target_lane, ego_fsagent.vehicle.state(), vehicle_set,
          ego_fsagent.lat_range, &leading_vehicle,
          &distance_residual_ratio) == kSuccess) {
    decimal_t distance_to_leading_vehicle =
        (leading_vehicle.state().vec_position -
         ego_fsagent.vehicle.state().vec_position)
            .norm();
    // 只有当前车确实在影响范围内时才计算代价
    if (ego_fsagent.vehicle.state().velocity < desired_velocity_ &&
        leading_vehicle.state().velocity < desired_velocity_ &&
        distance_to_leading_vehicle <
            cfg_.cost().effciency().leading_distance_th()) {
      // 自车被前车阻碍的速度差（自车速度快于前车时才有阻碍）
      decimal_t ego_blocked_by_leading_velocity =
          ego_fsagent.vehicle.state().velocity >
                  leading_vehicle.state().velocity
              ? ego_fsagent.vehicle.state().velocity -
                    leading_vehicle.state().velocity
              : 0.0;
      // 前车与期望速度的差距
      decimal_t leading_to_desired_velocity =
          leading_vehicle.state().velocity < desired_velocity_
              ? desired_velocity_ - leading_vehicle.state().velocity
              : 0.0;
      cost_tmp.efficiency.leading_to_desired_vel =
          std::max(cfg_.cost().effciency().min_distance_ratio(),
                   distance_residual_ratio) *
          (cfg_.cost().effciency().ego_speed_blocked_by_leading_unit_cost() *
               ego_blocked_by_leading_velocity +
           cfg_.cost()
                   .effciency()
                   .leading_speed_blocked_desired_vel_unit_cost() *
               leading_to_desired_velocity);
    }
  }

  // ========================================================================
  // 2. 安全代价
  // ========================================================================

  // 2.1 RSS安全检查：对每对（自车，周围车）执行RSS检查
  for (const auto& surround_traj : surround_trajs) {
    decimal_t safety_cost = 0.0;
    bool is_safe = true;
    int risky_id = 0;
    EvaluateSafetyStatus(ego_traj, surround_traj.second, &safety_cost, &is_safe,
                         &risky_id);
    if (!is_safe) {
      risky_ids->insert(risky_id);
      *is_risky = true;
    }
    cost_tmp.safety.rss += safety_cost;
  }

  // 2.2 占位碰撞风险：如果自车准备向被禁止的方向变道，施加代价
  if (cfg_.cost().safety().occu_lane_enable()) {
    if (lc_info_.forbid_lane_change_left &&
        seq_lat_behavior == LateralBehavior::kLaneChangeLeft) {
      cost_tmp.safety.occu_lane =
          ego_velocity * cfg_.cost().safety().occu_lane_unit_cost();
    } else if (lc_info_.forbid_lane_change_right &&
               seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
      cost_tmp.safety.occu_lane =
          ego_velocity * cfg_.cost().safety().occu_lane_unit_cost();
    }
  }

  // ========================================================================
  // 3. 导航代价
  // ========================================================================

  if (seq_lat_behavior == LateralBehavior::kLaneChangeLeft ||
      seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
    if (is_cancel_behavior) {
      // 3.1 变道取消惩罚
      cost_tmp.navigation.lane_change_preference =
          std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                   ego_velocity) *
          cfg_.cost().user().cancel_operation_unit_cost();
    } else {
      // 3.2 变道操作基础代价（优先保持当前车道）
      cost_tmp.navigation.lane_change_preference =
          std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                   ego_velocity) *
          (seq_lat_behavior == LateralBehavior::kLaneChangeLeft
               ? cfg_.cost().navigation().lane_change_left_unit_cost()
               : cfg_.cost().navigation().lane_change_right_unit_cost());

      // 3.3 推荐变道奖励（负代价 = 奖励）
      if (lc_info_.recommend_lc_left &&
          seq_lat_behavior == LateralBehavior::kLaneChangeLeft) {
        // 推荐左变道且序列匹配 -> 奖励（负代价）
        cost_tmp.navigation.lane_change_preference =
            -std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                      ego_velocity) *
            cfg_.cost().navigation().lane_change_left_recommendation_reward();
        // 3.4 如果当前动作不是变道（延迟执行推荐变道），额外惩罚
        if (action.lat != DcpLatAction::kLaneChangeLeft) {
          cost_tmp.navigation.lane_change_preference +=
              std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                       ego_velocity) *
              cfg_.cost().user().late_operate_unit_cost();
        }
      } else if (lc_info_.recommend_lc_right &&
                 seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
        cost_tmp.navigation.lane_change_preference =
            -std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                      ego_velocity) *
            cfg_.cost().navigation().lane_change_right_recommendation_reward();
        if (action.lat != DcpLatAction::kLaneChangeRight) {
          cost_tmp.navigation.lane_change_preference +=
              std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                       ego_velocity) *
              cfg_.cost().user().late_operate_unit_cost();
        }
      }
    }
  }

  // 代价权重 = 该层的时间长度
  cost_tmp.weight = duration;
  *cost = cost_tmp;
  return kSuccess;
}

// ============================================================================
//  前向仿真辅助函数
// ============================================================================

/// @brief 将动作持续时间分解为多个仿真时间步长
///
/// 例如 action.t = 2.3s, step_resolution = 0.2s:
/// - n_1 = floor(2.3 / 0.2) = 11
/// - dt_remain = 2.3 - 11*0.2 = 0.1
/// - steps = [0.1, 0.2, 0.2, 0.2, ..., 0.2] (12个元素)
/// 首个步长吸收余数0.1，其余步长为标准分辨率0.2
///
/// @param action DCP动作
/// @param dt_steps [out] 时间步长列表
ErrorType EudmPlanner::GetSimTimeSteps(const DcpAction& action,
                                       std::vector<decimal_t>* dt_steps) const {
  decimal_t sim_time_resolution = cfg_.sim().duration().step();
  decimal_t sim_time_total = action.t;
  int n_1 = std::floor(sim_time_total / sim_time_resolution);
  decimal_t dt_remain = sim_time_total - n_1 * sim_time_resolution;
  std::vector<decimal_t> steps(n_1, sim_time_resolution);
  if (fabs(dt_remain) > kEPS) {
    // 将余数作为首个步长插入
    steps.insert(steps.begin(), dt_remain);
  }
  *dt_steps = steps;

  return kSuccess;
}

/// @brief 仿真单个动作（一个时间层），执行多步前向仿真
///
/// 对一个动作层的持续时间，按仿真分辨率拆分为多个小步长，
/// 每步依次仿真自车和所有周围车辆。
///
/// ## 关键设计：仿真顺序
/// 在每一步中，先仿真自车，再仿真周围车辆。自车的仿真结果
/// 会立即更新到all_sim_vehicles中，周围车辆的仿真会基于更新后的状态。
/// 这模拟了自车"先行动"的效应。
///
/// ## 临时ID置零
/// 在单步仿真中，被仿真的车辆ID被临时设置为kInvalidAgentId，
/// 为了避免GetLeadingVehicleOnLane等查询将仿真车辆自身识别为前车。
/// 仿真完成后恢复原始ID。
///
/// @param action 当前DCP动作
/// @param ego_fsagent_this_layer 本层的自车仿真智能体
/// @param surrounding_fsagents_this_layer 本层的周围车辆仿真智能体
/// @param ego_traj [out] 自车在该层的多步轨迹
/// @param surround_trajs [out] 周围车辆在该层的多步轨迹
ErrorType EudmPlanner::SimulateSingleAction(
    const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent_this_layer,
    const ForwardSimAgentSet& surrounding_fsagents_this_layer,
    vec_E<common::Vehicle>* ego_traj,
    std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs) {
  // 准备轨迹容器
  ego_traj->clear();
  surround_trajs->clear();
  for (const auto& v : surrounding_fsagents_this_layer.forward_sim_agents) {
    surround_trajs->insert(std::pair<int, vec_E<common::Vehicle>>(
        v.first, vec_E<common::Vehicle>()));
  }

  // 获取仿真时间步长序列
  std::vector<decimal_t> dt_steps;
  GetSimTimeSteps(action, &dt_steps);

  // 复制当前层的智能体状态作为每步的初始状态
  ForwardSimEgoAgent ego_fsagent_this_step = ego_fsagent_this_layer;
  ForwardSimAgentSet surrounding_fsagents_this_step =
      surrounding_fsagents_this_layer;

  // 逐步仿真
  for (int i = 0; i < static_cast<int>(dt_steps.size()); i++) {
    decimal_t sim_time_step = dt_steps[i];

    // 缓存本步输出的状态
    common::State ego_state_cache_this_step;
    std::unordered_map<int, State> others_state_cache_this_step;

    // 构建所有仿真车辆的集合（用于前车查询）
    common::VehicleSet all_sim_vehicles;
    all_sim_vehicles.vehicles.insert(std::make_pair(
        ego_fsagent_this_step.vehicle.id(), ego_fsagent_this_step.vehicle));
    for (const auto& v : surrounding_fsagents_this_step.forward_sim_agents) {
      all_sim_vehicles.vehicles.insert(
          std::make_pair(v.first, v.second.vehicle));
    }

    // * 自车仿真
    {
      // 临时将自车ID置为无效，避免GetLeadingVehicleOnLane将自己识别为前车
      all_sim_vehicles.vehicles.at(ego_id_).set_id(kInvalidAgentId);

      common::State state_output;
      if (kSuccess != EgoAgentForwardSim(ego_fsagent_this_step,
                                         all_sim_vehicles, sim_time_step,
                                         &state_output)) {
        return kWrongStatus;
      }

      common::Vehicle v_tmp = ego_fsagent_this_step.vehicle;
      v_tmp.set_state(state_output);
      ego_traj->push_back(v_tmp);
      ego_state_cache_this_step = state_output;

      // 恢复自车ID
      all_sim_vehicles.vehicles.at(ego_id_).set_id(ego_id_);
    }

    // * 周围车辆仿真
    {
      for (const auto& p_fsa :
           surrounding_fsagents_this_step.forward_sim_agents) {
        // 临时将当前车辆ID置为无效
        all_sim_vehicles.vehicles.at(p_fsa.first).set_id(kInvalidAgentId);

        common::State state_output;
        if (kSuccess !=
            SurroundingAgentForwardSim(p_fsa.second, all_sim_vehicles,
                                       sim_time_step, &state_output)) {
          return kWrongStatus;
        }

        common::Vehicle v_tmp = p_fsa.second.vehicle;
        v_tmp.set_state(state_output);
        surround_trajs->at(p_fsa.first).push_back(v_tmp);
        others_state_cache_this_step.insert(
            std::make_pair(p_fsa.first, state_output));

        // 恢复车辆ID
        all_sim_vehicles.vehicles.at(p_fsa.first).set_id(p_fsa.first);
      }
    }

    // * 更新本步状态为下一步的初始状态
    ego_fsagent_this_step.vehicle.set_state(ego_state_cache_this_step);
    for (const auto& ps : others_state_cache_this_step) {
      surrounding_fsagents_this_step.forward_sim_agents.at(ps.first)
          .vehicle.set_state(ps.second);
    }
  }

  return kSuccess;
}

/// @brief 周围车辆前向仿真单步
///
/// 使用IDM模型沿车辆当前所在车道进行纵向仿真。
/// 查找前车作为IDM模型的输入。
///
/// 注意：当前版本仅考虑车道保持横向行为，不模拟变道。
/// 前向仿真的传播通过OnLaneForwardSimulation::PropagateOnce实现。
ErrorType EudmPlanner::SurroundingAgentForwardSim(
    const ForwardSimAgent& fsagent, const common::VehicleSet& all_sim_vehicles,
    const decimal_t& sim_time_step, common::State* state_out) const {
  common::Vehicle leading_vehicle;
  common::State state_output;
  decimal_t distance_residual_ratio = 0.0;

  // 查找前车（作为IDM模型的输入）
  map_itf_->GetLeadingVehicleOnLane(fsagent.lane, fsagent.vehicle.state(),
                                    all_sim_vehicles, fsagent.lat_range,
                                    &leading_vehicle, &distance_residual_ratio);

  // 执行单步前向仿真（IDM纵向 + 车道保持横向）
  if (planning::OnLaneForwardSimulation::PropagateOnce(
          fsagent.stf, fsagent.vehicle, leading_vehicle, sim_time_step,
          fsagent.sim_param, &state_output) != kSuccess) {
    return kWrongStatus;
  }
  *state_out = state_output;
  return kSuccess;
}

/// @brief 自车前向仿真单步
///
/// 根据自车的横向行为选择不同的仿真模式：
///
/// ## 车道保持模式（LaneKeeping）
/// 只考虑自车所在车道的前车，使用PropagateOnceAdvancedLK进行仿真。
/// 仿真实体为IDM纵向控制器 + 纯追踪横向控制器 + 车道横向偏移。
///
/// ## 变道模式（LaneChangeLeft/Right）
/// 更复杂的仿真，同时考虑多个关键车辆：
/// - current_leading_vehicle: 当前车道的前车
/// - gap_front_vehicle: 目标车道间隙的前车
/// - gap_rear_vehicle: 目标车道间隙的后车
///
/// 还包括**后车避让策略（evasive behavior）**：
/// - 如果后车在目标车道上与自车RSS不安全（TooSlow），
///   提高自车的期望速度以减少追尾风险
/// - 虚拟屏障（virtual_barrier）：如果与前后车间距不足，
///   调整横向偏移避免碰撞
///
/// 使用PropagateOnceAdvancedLC进行仿真，该函数同时处理纵向
/// 控制和横向变道轨迹生成。
///
/// @param ego_fsagent 自车仿真智能体
/// @param all_sim_vehicles 所有仿真车辆集合
/// @param sim_time_step 仿真时间步长
/// @param state_out [out] 仿真后的状态
ErrorType EudmPlanner::EgoAgentForwardSim(
    const ForwardSimEgoAgent& ego_fsagent,
    const common::VehicleSet& all_sim_vehicles, const decimal_t& sim_time_step,
    common::State* state_out) const {
  common::State state_output;

  if (ego_fsagent.lat_behavior == LateralBehavior::kLaneKeeping) {
    // ============================
    // 车道保持模式
    // ============================
    common::Vehicle leading_vehicle;
    decimal_t distance_residual_ratio = 0.0;
    // 查找当前车道的前车
    if (map_itf_->GetLeadingVehicleOnLane(
            ego_fsagent.target_lane, ego_fsagent.vehicle.state(),
            all_sim_vehicles, ego_fsagent.lat_range, &leading_vehicle,
            &distance_residual_ratio) == kSuccess) {
      // 碰撞预检查：如果已经碰撞，拒绝本次仿真
      bool is_collision = false;
      map_itf_->CheckCollisionUsingState(
          ego_fsagent.vehicle.param(), ego_fsagent.vehicle.state(),
          leading_vehicle.param(), leading_vehicle.state(), &is_collision);
      if (is_collision) {
        return kWrongStatus;
      }
    }

    // 执行高级车道保持仿真（IDM + PurePursuit + 横向偏移）
    decimal_t lat_track_offset = 0.0;
    if (planning::OnLaneForwardSimulation::PropagateOnceAdvancedLK(
            ego_fsagent.target_stf, ego_fsagent.vehicle, leading_vehicle,
            lat_track_offset, sim_time_step, ego_fsagent.sim_param,
            &state_output) != kSuccess) {
      return kWrongStatus;
    }
  } else {
    // ============================
    // 变道模式
    // ============================

    // 查找当前车道的前车
    common::Vehicle current_leading_vehicle;
    decimal_t distance_residual_ratio = 0.0;
    if (map_itf_->GetLeadingVehicleOnLane(
            ego_fsagent.current_lane, ego_fsagent.vehicle.state(),
            all_sim_vehicles, ego_fsagent.lat_range, &current_leading_vehicle,
            &distance_residual_ratio) == kSuccess) {
      // 碰撞预检查
      bool is_collision = false;
      map_itf_->CheckCollisionUsingState(
          ego_fsagent.vehicle.param(), ego_fsagent.vehicle.state(),
          current_leading_vehicle.param(), current_leading_vehicle.state(),
          &is_collision);
      if (is_collision) {
        return kWrongStatus;
      }
    }

    // 获取目标车道间隙的前车和后车
    common::Vehicle gap_front_vehicle;
    if (ego_fsagent.target_gap_ids(0) != -1) {
      gap_front_vehicle =
          all_sim_vehicles.vehicles.at(ego_fsagent.target_gap_ids(0));
    }
    common::Vehicle gap_rear_vehicle;
    if (ego_fsagent.target_gap_ids(1) != -1) {
      gap_rear_vehicle =
          all_sim_vehicles.vehicles.at(ego_fsagent.target_gap_ids(1));
    }

    decimal_t lat_track_offset = 0.0;
    auto sim_param = ego_fsagent.sim_param;

    // 后车避让策略（Evasive Behavior）
    // 当目标车道的后车与自车的RSS距离不安全时，
    // 调整自车的期望速度以满足RSS安全条件
    if (gap_rear_vehicle.id() != kInvalidAgentId &&
        cfg_.sim().ego().evasive().evasive_enable()) {
      common::FrenetState ego_on_tarlane_fs;
      common::FrenetState rear_on_tarlane_fs;
      if (ego_fsagent.target_stf.GetFrenetStateFromState(
              ego_fsagent.vehicle.state(), &ego_on_tarlane_fs) == kSuccess &&
          ego_fsagent.target_stf.GetFrenetStateFromState(
              gap_rear_vehicle.state(), &rear_on_tarlane_fs) == kSuccess) {
        // RSS检查：自车作为前车，后车是否safe
        bool is_rss_safe = true;
        common::RssChecker::LongitudinalViolateType type;
        decimal_t rss_vel_low, rss_vel_up;
        common::RssChecker::RssCheck(ego_fsagent.vehicle, gap_rear_vehicle,
                                     ego_fsagent.target_stf,
                                     rss_config_strict_as_rear_, &is_rss_safe,
                                     &type, &rss_vel_low, &rss_vel_up);
        if (!is_rss_safe) {
          if (type == common::RssChecker::LongitudinalViolateType::TooSlow) {
            // 自车太慢（后车太快逼近）：提高自车期望速度
            sim_param.idm_param.kDesiredVelocity = std::max(
                sim_param.idm_param.kDesiredVelocity,
                rss_vel_low + cfg_.sim().ego().evasive().lon_extraspeed());
            // 调整IDM参数以更快响应
            sim_param.idm_param.kDesiredHeadwayTime =
                cfg_.sim().ego().evasive().head_time();
            sim_param.idm_param.kAcceleration =
                cfg_.sim().ego().evasive().lon_acc();
            sim_param.max_lon_acc_jerk = cfg_.sim().ego().evasive().lon_jerk();
          }
        }

        // 虚拟屏障（Virtual Barrier）：如果与前后车的物理间距不足，
        // 调整横向偏移以避免碰撞
        if (cfg_.sim().ego().evasive().virtual_barrier_enable()) {
          // 与后车的虚拟屏障
          if (ego_on_tarlane_fs.vec_s[0] - rear_on_tarlane_fs.vec_s[0] <
              gap_rear_vehicle.param().length() +
                  cfg_.sim().ego().evasive().virtual_barrier_tic() *
                      gap_rear_vehicle.state().velocity) {
            lat_track_offset = ego_on_tarlane_fs.vec_dt[0];
          }
          // 与前车的虚拟屏障
          common::FrenetState front_on_tarlane_fs;
          if (gap_front_vehicle.id() != kInvalidAgentId &&
              ego_fsagent.target_stf.GetFrenetStateFromState(
                  gap_front_vehicle.state(), &front_on_tarlane_fs) ==
                  kSuccess) {
            if (front_on_tarlane_fs.vec_s[0] - ego_on_tarlane_fs.vec_s[0] <
                ego_fsagent.vehicle.param().length() +
                    cfg_.sim().ego().evasive().virtual_barrier_tic() *
                        ego_fsagent.vehicle.state().velocity) {
              lat_track_offset = ego_on_tarlane_fs.vec_dt[0];
            }
          }
        }
      }
    }

    // 执行高级变道仿真（多车约束下的IDM + 变道轨迹生成）
    if (planning::OnLaneForwardSimulation::PropagateOnceAdvancedLC(
            ego_fsagent.current_stf, ego_fsagent.target_stf,
            ego_fsagent.vehicle, current_leading_vehicle, gap_front_vehicle,
            gap_rear_vehicle, lat_track_offset, sim_time_step, sim_param,
            &state_output) != kSuccess) {
      return kWrongStatus;
    }
  }

  *state_out = state_output;
  return kSuccess;
}

// ============================================================================
//  辅助功能函数
// ============================================================================

/// @brief 根据车道ID判断横向行为类型
///
/// 通过检查当前车道ID属于哪个候选车道列表来判断行为：
/// - 若在potential_lk_lane_ids_中 -> LaneKeeping
/// - 若在potential_lcl_lane_ids_中 -> LaneChangeLeft
/// - 若在potential_lcr_lane_ids_中 -> LaneChangeRight
/// - 若都不在 -> Undefined
ErrorType EudmPlanner::JudgeBehaviorByLaneId(
    const int ego_lane_id_by_pos, LateralBehavior* behavior_by_lane_id) {
  if (ego_lane_id_by_pos == ego_lane_id_) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneKeeping;
    return kSuccess;
  }

  auto it = std::find(potential_lk_lane_ids_.begin(),
                      potential_lk_lane_ids_.end(), ego_lane_id_by_pos);
  auto it_lcl = std::find(potential_lcl_lane_ids_.begin(),
                          potential_lcl_lane_ids_.end(), ego_lane_id_by_pos);
  auto it_lcr = std::find(potential_lcr_lane_ids_.begin(),
                          potential_lcr_lane_ids_.end(), ego_lane_id_by_pos);

  if (it != potential_lk_lane_ids_.end()) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneKeeping;
    return kSuccess;
  }

  if (it_lcl != potential_lcl_lane_ids_.end()) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneChangeLeft;
    return kSuccess;
  }

  if (it_lcr != potential_lcr_lane_ids_.end()) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneChangeRight;
    return kSuccess;
  }

  *behavior_by_lane_id = common::LateralBehavior::kUndefined;
  return kSuccess;
}

/// @brief 更新自车车道ID
ErrorType EudmPlanner::UpdateEgoLaneId(const int new_ego_lane_id) {
  ego_lane_id_ = new_ego_lane_id;
  return kSuccess;
}

/// @brief 获取给定车道执行指定横向行为后可能到达的车道ID列表
///
/// - LaneKeeping/Undefined: 返回源车道的子车道ID列表
/// - LaneChangeLeft: 返回源车道左侧相邻车道及其所有子车道的ID列表
/// - LaneChangeRight: 返回源车道右侧相邻车道及其所有子车道的ID列表
///
/// @param source_lane_id 源车道ID
/// @param beh 横向行为
/// @param candidate_lane_ids [out] 候选车道ID列表
ErrorType EudmPlanner::GetPotentialLaneIds(
    const int source_lane_id, const LateralBehavior& beh,
    std::vector<int>* candidate_lane_ids) const {
  candidate_lane_ids->clear();
  if (beh == common::LateralBehavior::kUndefined ||
      beh == common::LateralBehavior::kLaneKeeping) {
    // 保持车道：返回子车道列表（前后继车道）
    map_itf_->GetChildLaneIds(source_lane_id, candidate_lane_ids);
  } else if (beh == common::LateralBehavior::kLaneChangeLeft) {
    // 左变道：获取左侧相邻车道及其子车道
    int l_lane_id;
    if (map_itf_->GetLeftLaneId(source_lane_id, &l_lane_id) == kSuccess) {
      map_itf_->GetChildLaneIds(l_lane_id, candidate_lane_ids);
      candidate_lane_ids->push_back(l_lane_id);
    }
  } else if (beh == common::LateralBehavior::kLaneChangeRight) {
    // 右变道：获取右侧相邻车道及其子车道
    int r_lane_id;
    if (map_itf_->GetRightLaneId(source_lane_id, &r_lane_id) == kSuccess) {
      map_itf_->GetChildLaneIds(r_lane_id, candidate_lane_ids);
      candidate_lane_ids->push_back(r_lane_id);
    }
  } else {
    assert(false);
  }
  return kSuccess;
}

// ============================================================================
//  公共接口函数
// ============================================================================

/// @brief 设置地图接口（依赖注入）
void EudmPlanner::set_map_interface(EudmPlannerMapItf* itf) { map_itf_ = itf; }

/// @brief 设置自车期望速度（m/s），不低于0
void EudmPlanner::set_desired_velocity(const decimal_t desired_vel) {
  desired_velocity_ = std::max(0.0, desired_vel);
}

/// @brief 设置变道信息（禁止/推荐/实线等信息）
void EudmPlanner::set_lane_change_info(const LaneChangeInfo& lc_info) {
  lc_info_ = lc_info;
}

/// @brief 获取当前期望速度
decimal_t EudmPlanner::desired_velocity() const { return desired_velocity_; }

/// @brief 获取优胜序列ID
int EudmPlanner::winner_id() const { return winner_id_; }

/// @brief 获取本次规划的时间开销
decimal_t EudmPlanner::time_cost() const { return time_cost_; }

/// @brief 更新DCP-Tree的进行中动作并重新生成动作序列
///
/// 在每次规划前由EudmManager调用，设定"当前正在执行"的动作。
/// 这会触发DCP-Tree重新生成所有候选动作序列，
/// 并更新sim_time_total_（总仿真时域）。
///
/// @param ongoing_action 当前正在执行的动作（从重规划上下文获取的期望动作）
void EudmPlanner::UpdateDcpTree(const DcpAction& ongoing_action) {
  dcp_tree_ptr_->set_ongoing_action(ongoing_action);
  dcp_tree_ptr_->UpdateScript();
  sim_time_total_ = dcp_tree_ptr_->planning_horizon();
}

/// @brief 获取地图接口指针
EudmPlannerMapItf* EudmPlanner::map_itf() const { return map_itf_; }

}  // namespace planning
