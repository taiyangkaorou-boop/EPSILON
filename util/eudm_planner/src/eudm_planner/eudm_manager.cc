/**
 * @file eudm_manager.cc
 * @author EPSILON Autonomous Driving Team
 * @brief EUDM管理器——完整的EUDM规划生命周期管理实现
 * @version 0.1
 * @date 2019-07-07
 * @copyright Copyright (c) 2019
 *
 * ## 文件概述
 *
 * 本文件实现了EudmManager类——EUDM行为规划系统的状态管理器和
 * 变道生命周期控制器。由于EUDM规划器本身是完全无状态的
 * （每个周期独立运行），Manager的作用是"赋予EUDM系统状态记忆"。
 *
 * ## 核心职责
 *
 * ### 1. 规划上下文管理（ReplanningContext）
 * 记录上一个规划周期的优胜动作序列和起始时间。
 * 在下一个周期中，根据已过时间确定"当前正在执行哪个动作"（ongoing action），
 * 从而实现跨周期的连续规划（而非每次从头开始）。
 *
 * ### 2. 变道生命周期管理
 * 支持两种变道触发机制：
 *
 * **用户拨杆触发（kStick）**：
 * - 接收HMI信号（user_perferred_behavior），转换为变道请求
 * - 支持立即触发和"等待合适时机触发"（trigger_when_appropriate）
 * - 支持用户取消变道（拨回或反向拨杆）
 *
 * **系统主动触发（kActive）**：
 * - EUDM自动判断场景优势并提出变道建议
 * - 需要连续N帧一致请求才能触发（通过preliminary_active_requests_累积）
 * - 冷启动间隔（cold_duration）防止频繁变道
 * - 速度范围约束（activate_speed_lower/upper_bound）
 *
 * ### 3. 上下文感知重选（ReselectByContext）
 * EUDM原始选出的优胜序列可能不符合当前的变道上下文。
 * 例如：如果系统正在进行左变道操作，原始选出的保持车道序列
 * 会被筛除，选择左变道或保持的序列（取决于当前时间与期望执行时间的关系）。
 *
 * ### 4. 参考速度评估
 * 基于道路曲率约束的安全速度上限：
 *   v_max_by_curvature = sqrt(a_lat_max / |curvature|)
 * 取前方道路上所有采样点的最小值。
 *
 * ## 工作流程（Run方法）
 *
 * ```
 * [I]    Prepare: 设置地图、获取期望动作、处理变道上下文、评估参考速度
 * [II]   RunOnce: 调用EudmPlanner::RunOnce()执行核心多线程仿真
 * [III]  Summarize: 保存仿真结果到Snapshot (SaveSnapshot)
 * [IV]   Reselect: 根据变道上下文重选最优序列 (ReselectByContext)
 * [V]    Update: 获取参考车道、更新规划上下文、生成变道提案
 * ```
 *
 * @see eudm_manager.h
 * @see eudm_planner.h
 */

#include "eudm_planner/eudm_manager.h"

#include <glog/logging.h>

namespace planning {

// ============================================================================
//  初始化
// ============================================================================

/// @brief 初始化EUDM管理器
///
/// 初始化步骤：
/// 1. 配置glog日志系统（将日志输出到~/.eudm_log/目录）
/// 2. 初始化EudmPlanner（读取配置、创建DCP-Tree）
/// 3. 设置地图适配器作为EudmPlanner的地图接口
/// 4. 存储工作频率
/// 5. 输出主动变道功能状态（启用/禁用）
///
/// @param config_path EUDM配置文件路径
/// @param work_rate 规划工作频率（Hz）
void EudmManager::Init(const std::string& config_path,
                       const decimal_t work_rate) {
  // 初始化glog日志系统
  google::InitGoogleLogging("eudm");
  google::SetLogDestination(google::GLOG_INFO, "~/.eudm_log/");
  google::SetLogDestination(google::GLOG_WARNING, "~/.eudm_log/");
  google::SetLogDestination(google::GLOG_ERROR, "~/.eudm_log/");
  google::SetLogDestination(google::GLOG_FATAL, "~/.eudm_log/");

  // 初始化EUDM规划器并注入地图适配器
  bp_.Init(config_path);
  bp_.set_map_interface(&map_adapter_);
  work_rate_ = work_rate;

  // 输出主动变道功能状态
  if (bp_.cfg().function().active_lc_enable()) {
    LOG(ERROR) << "[HMI]HMI enabled with active lane change ON.";
  } else {
    LOG(ERROR) << "[HMI]HMI enabled with active lane change OFF.";
  }
}

// ============================================================================
//  决策时间点计算
// ============================================================================

/// @brief 获取最近的未来决策时间点
///
/// 决策时间点被对齐到layer_time的整数倍，确保变道操作在
/// 规划周期的边界上触发，保持时间一致性。
///
/// 计算公式：
///   past_decision = floor((stamp + delta) / layer_time) * layer_time
///   next_decision = past_decision + layer_time
///
/// 示例：
/// - stamp=10.3, delta=0, layer_time=2.0 -> next_decision=12.0
/// - stamp=10.3, delta=1.0, layer_time=2.0 -> next_decision=14.0
///
/// @param stamp 当前时间戳
/// @param delta 从当前时间向前的最小偏移
/// @return 最近的未来决策点时间戳
decimal_t EudmManager::GetNearestFutureDecisionPoint(const decimal_t& stamp,
                                                     const decimal_t& delta) {
  // 考虑最近的对齐到layer_time的决策点
  decimal_t past_decision_point =
      std::floor((stamp + delta) / bp_.cfg().sim().duration().layer()) *
      bp_.cfg().sim().duration().layer();
  return past_decision_point + bp_.cfg().sim().duration().layer();
}

// ============================================================================
//  变道时机判断
// ============================================================================

/// @brief 检查当前是否适合执行指定的横向行为
///
/// 检查条件：在上次规划的所有安全且无风险的序列中，
/// 是否有足够多的序列在其第1至第3层包含了指定的变道方向。
///
/// 检查逻辑（以左变道为例）：
/// 1. 遍历上次规划的所有动作序列
/// 2. 只考虑仿真成功(sim_res=true)且无风险(risky_res=false)的序列
/// 3. 检查这些序列的第1-3层（索引1,2,3）是否包含左变道动作
///    且该动作的纵向为加速或维持（避免减速变道）
/// 4. 统计匹配的序列数量
/// 5. 如果匹配数量 >= kMinMatchSeqs(3)，认为时机合适
///
/// 设计意图：如果足够多的"安全竞选者"都认为某个变道方向是好的选择，
/// 那么当前时机适合执行该变道。
///
/// @param lat 待检查的横向行为
/// @return true表示时机合适
bool EudmManager::IsTriggerAppropriate(const LateralBehavior& lat) {
#if 1
  // 简化版本：始终返回true（总是允许触发）
  // 这可能是因为在实际道路测试中发现kMinMatchSeqs=3太保守
  return true;
#endif
  // 原版检查逻辑：
  // 检查是否有足够的安全序列在中间位置包含指定的变道方向
  if (!last_snapshot_.valid) return false;

  // 检查范围：序列的第1到第3个动作（索引1,2,3）
  // 设计原因：不检查第一个动作（可能是ongoing），也不检查太后面的动作
  // 示例序列 KKKLL (先保持3层再变道)
  //           KKLLL (先保持2层再变道)
  //           KLLLL (先保持1层再变道)
  const int kMinActionCheckIdx = 1;
  const int kMaxActionCheckIdx = 3;
  const int kMinMatchSeqs = 3;  // 最少需要3个一致的安全序列
  int num_action_seqs = last_snapshot_.action_script.size();
  int num_match_seqs = 0;

  for (int i = 0; i < num_action_seqs; i++) {
    // 只考虑安全且无风险的序列
    if (!last_snapshot_.sim_res[i] || last_snapshot_.risky_res[i]) continue;
    auto action_seq = last_snapshot_.action_script[i];
    int num_actions = action_seq.size();
    for (int j = kMinActionCheckIdx; j <= kMaxActionCheckIdx; j++) {
      if (lat == LateralBehavior::kLaneChangeLeft &&
          action_seq[j].lat == DcpLatAction::kLaneChangeLeft &&
          (action_seq[j].lon == DcpLonAction::kAccelerate ||
           action_seq[j].lon == DcpLonAction::kMaintain)) {
        // 找到左变道+加速/维持的序列
        num_match_seqs++;
        break;
      } else if (lat == LateralBehavior::kLaneChangeRight &&
                 action_seq[j].lat == DcpLatAction::kLaneChangeRight &&
                 (action_seq[j].lon == DcpLonAction::kAccelerate ||
                  action_seq[j].lon == DcpLonAction::kMaintain)) {
        // 找到右变道+加速/维持的序列
        num_match_seqs++;
        break;
      }
    }
  }

  // 匹配数量不足，认为时机不合适
  if (num_match_seqs < kMinMatchSeqs) return false;
  return true;
}

// ============================================================================
//  Prepare阶段
// ============================================================================

/// @brief [EUDM流水线阶段1] 准备阶段：设置地图和规划上下文
///
/// 在每个规划周期的第一阶段执行，负责：
///
/// 1. **地图设置**：注入最新的语义地图到地图适配器
/// 2. **期望动作获取**：从重规划上下文获取当前应该执行的动作
///    - 如果上下文无效（首个周期），默认使用保持车道+维持速度
///    - 期望动作的时间 = 下一个决策点时间 - 当前时间
/// 3. **车道ID获取**：获取自车当前位置对应的车道ID
/// 4. **变道上下文处理**：
///    - 根据新任务更新变道上下文（用户拨杆信号、自动取消条件等）
///    - 如果正在进行变道，将期望动作的横向方向锁定为保持
///      （等待变道上下文触发操作）
/// 5. **参考速度评估**：基于道路曲率计算安全速度上限
/// 6. **变道信息设置**：
///    - 如果到达变道操作时间，在lc_info中标记推荐变道方向
///    - 将增强后的lc_info传递给EudmPlanner
///
/// 日志输出：
/// - 重规划上下文详情（有效性、起始时间、当前序列动作）
/// - 变道上下文详情
/// - 期望速度信息
/// - 当前期望动作详情
///
/// @param stamp 当前时间戳
/// @param map_ptr 语义地图管理器共享指针
/// @param task EUDM规划任务
ErrorType EudmManager::Prepare(
    const decimal_t stamp,
    const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,
    const planning::eudm::Task& task) {
  // 步骤1: 注入最新的语义地图
  map_adapter_.set_map(map_ptr);

  // 步骤2: 从重规划上下文获取期望动作
  DcpAction desired_action;
  if (!GetReplanDesiredAction(stamp, &desired_action)) {
    // 上下文无效（首次规划周期）：使用默认动作
    desired_action.lat = DcpLatAction::kLaneKeeping;
    desired_action.lon = DcpLonAction::kMaintain;
    decimal_t fdp_stamp = GetNearestFutureDecisionPoint(stamp, 0.0);
    desired_action.t = fdp_stamp - stamp;  // 剩余时间到下一个决策点
  }

  // 步骤3: 获取自车当前车道ID
  if (map_adapter_.map()->GetEgoNearestLaneId(&ego_lane_id_) != kSuccess) {
    return kWrongStatus;
  }

  // 步骤4: 处理变道上下文
  UpdateLaneChangeContextByTask(stamp, task);
  if (lc_context_.completed) {
    // 没有进行中的变道：期望动作的横向方向保持为保持
    desired_action.lat = DcpLatAction::kLaneKeeping;
  }

  // 日志：重规划上下文详情
  {
    std::ostringstream line_info;
    line_info << "[Eudm][Manager]Replan context <valid, stamp, seq>:<"
              << context_.is_valid << "," << std::fixed << std::setprecision(3)
              << context_.seq_start_time << ",";
    for (auto& a : context_.action_seq) {
      line_info << DcpTree::RetLonActionName(a.lon);
    }
    line_info << "|";
    for (auto& a : context_.action_seq) {
      line_info << DcpTree::RetLatActionName(a.lat);
    }
    line_info << ">";
    LOG(WARNING) << line_info.str();
  }
  // 日志：变道上下文详情
  {
    std::ostringstream line_info;
    line_info << "[Eudm][Manager]LC context <completed, twa, tt, dt, l_id, "
                 "lat, type>:<"
              << lc_context_.completed << ","
              << lc_context_.trigger_when_appropriate << "," << std::fixed
              << std::setprecision(3) << lc_context_.trigger_time << ","
              << lc_context_.desired_operation_time << ","
              << lc_context_.ego_lane_id << ","
              << common::SemanticsUtils::RetLatBehaviorName(lc_context_.lat)
              << "," << static_cast<int>(lc_context_.type) << ">";
    LOG(WARNING) << line_info.str();
  }

  // 步骤5: 更新DCP-Tree并评估参考速度
  bp_.UpdateDcpTree(desired_action);
  decimal_t ref_vel;
  EvaluateReferenceVelocity(task, &ref_vel);
  LOG(WARNING) << "[Eudm][Manager]<task vel, ref_vel>:<"
               << task.user_desired_vel << "," << ref_vel << ">";
  bp_.set_desired_velocity(ref_vel);

  // 步骤6: 增强变道信息并传递给EudmPlanner
  auto lc_info = task.lc_info;
  // 如果正在进行变道且当前时间已到达期望操作时间
  if (!lc_context_.completed) {
    if (stamp >= lc_context_.desired_operation_time) {
      if (lc_context_.lat == LateralBehavior::kLaneChangeLeft) {
        lc_info.recommend_lc_left = true;   // 推荐左变道（奖励信号）
      } else if (lc_context_.lat == LateralBehavior::kLaneChangeRight) {
        lc_info.recommend_lc_right = true;  // 推荐右变道（奖励信号）
      }
    }
  }
  bp_.set_lane_change_info(lc_info);

  // 日志：期望动作详情
  LOG(WARNING) << "[Eudm][Manager]desired <lon,lat,t>:<"
               << DcpTree::RetLonActionName(desired_action.lon).c_str() << ","
               << DcpTree::RetLatActionName(desired_action.lat) << ","
               << desired_action.t << "> ego lane id:" << ego_lane_id_;

  return kSuccess;
}

// ============================================================================
//  系统主动变道提案生成
// ============================================================================

/// @brief 生成系统主动变道提案（Active Lane Change Proposal）
///
/// 这是EUDM系统自主决策变道的机制。系统周期性地检查EUDM规划的
/// 优胜序列是否包含变道行为，如果是则积累一致性请求。
///
/// ## 触发条件（所有条件必须全部满足）
///
/// 1. **主动变道功能启用**（active_lc_enable）
/// 2. **自动驾驶处于控制状态**（is_under_ctrl）
/// 3. **没有正在进行的变道操作**（lc_context_.completed == true）
/// 4. **满足冷启动间隔**（距上次触发 >= cold_duration）
/// 5. **速度在允许范围内**（activate_speed_lower_bound <= v <= activate_speed_upper_bound）
///
/// ## 一致性检查
///
/// 当前周期的变道请求与上次请求相比较：
/// - 车道ID必须一致（ego_lane_id检查）
/// - 变道方向必须一致（lat检查）
/// - 期望执行时间的偏差必须在容忍范围内（consistent_operate_time_min_gap）
///
/// 只有满足一致性的请求才能加入累积队列。
///
/// ## 提案生成
///
/// 当一致性请求队列达到consistent_min_num_frame（最小连续帧数）：
/// - operation_at_seconds <= activate_max_duration_in_seconds: 生成有效提案
/// - operation_at_seconds取max(原始值, active_min_operation_in_seconds)
/// - 清空累积队列
///
/// ## 清空条件
///
/// 以下情况会清空累积队列：
/// - 主动变道功能禁用
/// - 退出自动驾驶模式
/// - 变道上下文不完整
/// - 用户拨杆信号非零（未复位）
/// - 冷启动期内
/// - 速度超出范围
/// - 禁止信号出现（如果启用enable_clear_accumulation_by_forbid_signal）
/// - 优胜序列不是理想行为（保持或取消模式）
/// - 不一致的请求
///
/// @param stamp 当前时间戳
/// @param task 当前任务
ErrorType EudmManager::GenerateLaneChangeProposal(
    const decimal_t& stamp, const planning::eudm::Task& task) {
  // 条件1: 主动变道功能是否启用
  if (!bp_.cfg().function().active_lc_enable()) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to disabled lc:"
                 << stamp;
    return kSuccess;
  }

  // 条件2: 自动驾驶是否处于控制状态
  if (!task.is_under_ctrl) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to not under ctrl:"
                 << stamp;
    return kSuccess;
  }

  // 条件3: 是否有正在进行的变道操作
  if (!lc_context_.completed) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to not completed lc:"
                 << stamp;
    return kSuccess;
  }

  // 条件4: 拨杆是否已复位（非零值会阻止主动变道）
  if (lc_context_.completed && task.user_perferred_behavior != 0) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to stick not rest:"
                 << stamp;
    return kSuccess;
  }

  // 检查时间戳合法性
  if (stamp - last_lc_proposal_.trigger_time < 0.0) {
    last_lc_proposal_.valid = false;
    last_lc_proposal_.trigger_time = stamp;
    last_lc_proposal_.ego_lane_id = ego_lane_id_;
    last_lc_proposal_.lat = LateralBehavior::kLaneKeeping;
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to illegal stamp:"
                 << stamp;
    return kSuccess;
  }

  // 条件5: 冷启动间隔检查
  if (stamp - last_lc_proposal_.trigger_time <
      bp_.cfg().function().active_lc().cold_duration()) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to cold down:" << stamp
                 << " < " << last_lc_proposal_.trigger_time << " + "
                 << bp_.cfg().function().active_lc().cold_duration();
    return kSuccess;
  }

  // 条件6: 速度范围约束
  if (last_snapshot_.plan_state.velocity <
          bp_.cfg().function().active_lc().activate_speed_lower_bound() ||
      last_snapshot_.plan_state.velocity >
          bp_.cfg().function().active_lc().activate_speed_upper_bound()) {
    preliminary_active_requests_.clear();
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to illegal spd:"
                 << last_snapshot_.plan_state.velocity << " at time " << stamp;
    return kSuccess;
  }

  // 条件7: 禁止信号清除（如果启用）
  if (bp_.cfg()
          .function()
          .active_lc()
          .enable_clear_accumulation_by_forbid_signal() &&
      not preliminary_active_requests_.empty()) {
    auto last_request = preliminary_active_requests_.back();
    if (last_request.lat == LateralBehavior::kLaneChangeLeft &&
        task.lc_info.forbid_lane_change_left) {
      LOG(WARNING) << std::fixed << std::setprecision(5)
                   << "[Eudm][ActiveLc]Clear request due to forbid signal:"
                   << stamp;
      preliminary_active_requests_.clear();
      return kSuccess;
    }
    if (last_request.lat == LateralBehavior::kLaneChangeRight &&
        task.lc_info.forbid_lane_change_right) {
      LOG(WARNING) << std::fixed << std::setprecision(5)
                   << "[Eudm][ActiveLc]Clear request due to forbid signal:"
                   << stamp;
      preliminary_active_requests_.clear();
      return kSuccess;
    }
  }

  // 分析原始优胜序列的行为类型
  common::LateralBehavior lat_behavior;
  decimal_t operation_at_seconds;
  bool is_cancel_behavior;
  bp_.ClassifyActionSeq(
      last_snapshot_.action_script[last_snapshot_.original_winner_id],
      &operation_at_seconds, &lat_behavior, &is_cancel_behavior);

  // 条件8: 优胜序列的非理想行为（保持或取消）不触发主动变道
  if (lat_behavior == LateralBehavior::kLaneKeeping || is_cancel_behavior) {
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]Clear request due to not ideal behavior:"
                 << stamp;
    preliminary_active_requests_.clear();
    return kSuccess;
  }

  // 创建本次周期的变道请求
  ActivateLaneChangeRequest this_request;
  this_request.trigger_time = stamp;
  this_request.desired_operation_time = stamp + operation_at_seconds;
  this_request.ego_lane_id = ego_lane_id_;
  this_request.lat = lat_behavior;

  if (preliminary_active_requests_.empty()) {
    // 第一个请求：开始新的累积
    preliminary_active_requests_.push_back(this_request);
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]trigger time:" << this_request.trigger_time
                 << " Init requesting "
                 << common::SemanticsUtils::RetLatBehaviorName(this_request.lat)
                 << " at " << this_request.desired_operation_time
                 << " with lane id " << this_request.ego_lane_id;
  } else {
    // 已有累积请求：检查一致性
    LOG(WARNING) << std::fixed << std::setprecision(5)
                 << "[Eudm][ActiveLc]trigger time:" << this_request.trigger_time
                 << " Consequent requesting "
                 << common::SemanticsUtils::RetLatBehaviorName(this_request.lat)
                 << " at " << this_request.desired_operation_time
                 << " with lane id " << this_request.ego_lane_id;
    auto last_request = preliminary_active_requests_.back();

    // 一致性检查1: 车道ID必须一致
    if (last_request.ego_lane_id != this_request.ego_lane_id) {
      LOG(WARNING)
          << "[Eudm][ActiveLc]Invalid this request due to lane id inconsitent.";
      preliminary_active_requests_.clear();
      return kSuccess;
    }
    // 一致性检查2: 变道方向必须一致
    if (last_request.lat != this_request.lat) {
      LOG(WARNING) << "[Eudm][ActiveLc]Invalid this request due to behavior "
                      "inconsitent.";
      preliminary_active_requests_.clear();
      return kSuccess;
    }
    // 一致性检查3: 期望执行时间的偏差在容忍范围内
    if (fabs(last_request.desired_operation_time -
             this_request.desired_operation_time) >
        bp_.cfg().function().active_lc().consistent_operate_time_min_gap()) {
      LOG(WARNING)
          << "[Eudm][ActiveLc]Invalid this request due to time inconsitent.";
      preliminary_active_requests_.clear();
      return kSuccess;
    }
    // 通过一致性检查，加入累积队列
    preliminary_active_requests_.push_back(this_request);
    LOG(WARNING) << "[Eudm][ActiveLc]valid this request. Queue size "
                 << preliminary_active_requests_.size() << " and operate at "
                 << operation_at_seconds;
  }

  // 检查是否满足触发阈值（最小连续帧数）
  if (preliminary_active_requests_.size() >=
      bp_.cfg().function().active_lc().consistent_min_num_frame()) {
    if (operation_at_seconds <
        bp_.cfg().function().active_lc().activate_max_duration_in_seconds() +
            kEPS) {
      // 生成有效的变道提案
      last_lc_proposal_.valid = true;
      last_lc_proposal_.trigger_time = stamp;
      // 操作时间取max(原始值, 最小执行时间)或下一个决策点
      last_lc_proposal_.operation_at_seconds =
          operation_at_seconds > bp_.cfg()
                                     .function()
                                     .active_lc()
                                     .active_min_operation_in_seconds()
              ? operation_at_seconds
              : GetNearestFutureDecisionPoint(
                    stamp, bp_.cfg()
                               .function()
                               .active_lc()
                               .active_min_operation_in_seconds()) -
                    stamp;
      last_lc_proposal_.ego_lane_id = ego_lane_id_;
      last_lc_proposal_.lat = lat_behavior;
      preliminary_active_requests_.clear();
      LOG(WARNING) << std::fixed << std::setprecision(5)
                   << "[HMI]Gen proposal with trigger time "
                   << last_lc_proposal_.trigger_time << " lane id "
                   << last_lc_proposal_.ego_lane_id << " behavior "
                   << static_cast<int>(last_lc_proposal_.lat) << " operate at "
                   << last_lc_proposal_.operation_at_seconds;
    } else {
      // 期望执行时间超出最大允许范围，放弃
      preliminary_active_requests_.clear();
    }
  }

  return kSuccess;
}

// ============================================================================
//  变道上下文更新（核心状态机）
// ============================================================================

/// @brief [核心状态机] 根据新任务更新变道上下文状态
///
/// 这是EUDM系统中最复杂的函数之一，管理变道操作的完整生命周期状态机。
///
/// ## 状态变量
///
/// - lc_context_.completed: 是否有进行中的变道
/// - lc_context_.trigger_when_appropriate: 是否缓存了等待合适时机的变道请求
/// - lc_context_.lat: 当前/请求的变道方向
/// - lc_context_.type: 触发类型（kStick用户拨杆/kActive系统主动）
///
/// ## 状态变迁表
///
/// ### 1. 自动驾驶模式切换
/// - 激活时: 重置所有变道状态
/// - 取消时: 重置所有变道状态
///
/// ### 2. 变道进行中 (lc_context_.completed == false)
///
/// #### 2a. 变道完成检测
/// - 车道ID变化（IsLaneConsistent返回false）-> 标记完成
/// - 从旧车道进入了不一致的新车道，说明变道已成功跨越
///
/// #### 2b. 拨杆取消
/// - user_perferred_behavior从1/ -1变为非对应值 -> 取消变道
///
/// #### 2c. 主动变道自动取消条件
/// - 超时取消（auto_cancel_by_outdate_time）：
///   当前时间 > desired_operation_time + auto_cancel_if_late_for_seconds
/// - 禁止信号取消（auto_cancel_by_forbid_signal）：
///   变道方向被标记为禁止
/// - 用户反向拨杆取消（auto_canbel_by_stick_signal）：
///   用户拨杆方向与变道方向相反
///
/// ### 3. 空闲状态 (lc_context_.completed == true)
///
/// #### 3a. 清除缓存的触发状态
/// - 用户取消已缓存的等待触发
///
/// #### 3b. 用户拨杆触发
///
/// 右变道（behavior == 1）:
/// - 如果被禁止 -> 缓存触发（trigger_when_appropriate = true）
/// - 如果时机合适 -> 立即触发（设置completed=false等）
/// - 如果时机不合适 -> 缓存触发
///
/// 左变道（behavior == -1）:
/// - 同理
///
/// #### 3c. 缓存触发到期执行
/// - 当trigger_when_appropriate == true 且条件满足时触发
///
/// #### 3d. 主动变道提案执行
/// - 当last_lc_proposal_有效时，转换为主动变道触发
///
/// ## 重要设计约束
/// - 任何变道提案（proposal）只持续一个周期（函数末尾设置valid=false）
/// - last_task_在函数末尾更新，用于下一周期检测变化
///
/// @param stamp 当前时间戳
/// @param task 新收到的任务
void EudmManager::UpdateLaneChangeContextByTask(
    const decimal_t stamp, const planning::eudm::Task& task) {
  // ========================================================================
  // 1. 自动驾驶模式切换处理
  // ========================================================================
  if (!last_task_.is_under_ctrl && task.is_under_ctrl) {
    LOG(WARNING) << "[HMI]Autonomous mode activated!";
    lc_context_.completed = true;
    lc_context_.trigger_when_appropriate = false;
    last_lc_proposal_.trigger_time = stamp;
  }

  if (last_task_.is_under_ctrl && !task.is_under_ctrl) {
    LOG(WARNING) << "[HMI]Autonomous mode deactivated!";
    lc_context_.completed = true;
    lc_context_.trigger_when_appropriate = false;
    last_lc_proposal_.trigger_time = stamp;
  }

  // ========================================================================
  // 2. 日志：拨杆和禁止信号变化
  // ========================================================================
  if (task.user_perferred_behavior != last_task_.user_perferred_behavior) {
    LOG(WARNING) << "[HMI]stick state change from "
                 << last_task_.user_perferred_behavior << " to "
                 << task.user_perferred_behavior;
  }

  if ((task.lc_info.forbid_lane_change_left !=
       last_task_.lc_info.forbid_lane_change_left) ||
      (task.lc_info.forbid_lane_change_right !=
       last_task_.lc_info.forbid_lane_change_right)) {
    LOG(WARNING) << "[HMI]lane change forbid signal [left] "
                 << task.lc_info.forbid_lane_change_left << " [right] "
                 << task.lc_info.forbid_lane_change_right;
  }

  // ========================================================================
  // 3. 自动驾驶控制状态下的变道上下文处理
  // ========================================================================
  if (task.is_under_ctrl) {
    // ------------------------------------------------------------------
    // 3a. 变道进行中
    // ------------------------------------------------------------------
    if (!lc_context_.completed) {
      if (!map_adapter_.IsLaneConsistent(lc_context_.ego_lane_id,
                                         ego_lane_id_)) {
        // 车道ID变化不一致（变道跨越完成）
        LOG(WARNING) << "[HMI]lane change completed due to different lane id "
                     << lc_context_.ego_lane_id << " to " << ego_lane_id_
                     << ". Cd alc.";
        lc_context_.completed = true;
        lc_context_.trigger_when_appropriate = false;
        last_lc_proposal_.trigger_time = stamp;
      } else {
        // 车道ID一致，说明还在变道过程中（或等待执行时机）
        // 3a-i. 拨杆取消处理
        if (task.user_perferred_behavior != 1 &&
            last_task_.user_perferred_behavior == 1) {
          LOG(WARNING) << "[HMI]lane change cancel by stick "
                       << last_task_.user_perferred_behavior << " to "
                       << task.user_perferred_behavior << ". Cd alc.";
          lc_context_.completed = true;
          lc_context_.trigger_when_appropriate = false;
          last_lc_proposal_.trigger_time = stamp;
        } else if (task.user_perferred_behavior != -1 &&
                   last_task_.user_perferred_behavior == -1) {
          LOG(WARNING) << "[HMI]lane change cancel by stick "
                       << last_task_.user_perferred_behavior << " to "
                       << task.user_perferred_behavior << ". Cd alc.";
          lc_context_.completed = true;
          lc_context_.trigger_when_appropriate = false;
          last_lc_proposal_.trigger_time = stamp;
        }
        // 3a-ii. 主动变道的自动取消条件
        else if (lc_context_.type == LaneChangeTriggerType::kActive) {
          // 超时取消
          if (bp_.cfg()
                  .function()
                  .active_lc()
                  .enable_auto_cancel_by_outdate_time() &&
              stamp > lc_context_.desired_operation_time +
                          bp_.cfg()
                              .function()
                              .active_lc()
                              .auto_cancel_if_late_for_seconds()) {
            if (lc_context_.lat == LateralBehavior::kLaneChangeLeft) {
              LOG(WARNING)
                  << "[HMI]ACTIVE [Left] auto cancel due to outdated for "
                  << stamp - lc_context_.desired_operation_time
                  << " s. Cd alc.";
            } else {
              LOG(WARNING)
                  << "[HMI]ACTIVE [Right] auto cancel due to outdated for "
                  << stamp - lc_context_.desired_operation_time
                  << " s. Cd alc.";
            }
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          }
          // 禁止信号取消
          else if (bp_.cfg()
                         .function()
                         .active_lc()
                         .enable_auto_cancel_by_forbid_signal() &&
                     task.lc_info.forbid_lane_change_left &&
                     lc_context_.lat == LateralBehavior::kLaneChangeLeft) {
            LOG(WARNING) << "[HMI]ACTIVE [Left] canceled due to forbidden "
                            "signal. Cd alc.";
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          } else if (bp_.cfg()
                         .function()
                         .active_lc()
                         .enable_auto_cancel_by_forbid_signal() &&
                     task.lc_info.forbid_lane_change_right &&
                     lc_context_.lat == LateralBehavior::kLaneChangeRight) {
            LOG(WARNING)
                << "[HMI]ACTIVE [Right] canceled due to forbidden signal. "
                   "Cd alc.";
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          }
          // 用户反向拨杆取消
          else if (bp_.cfg()
                         .function()
                         .active_lc()
                         .enable_auto_canbel_by_stick_signal() &&
                     lc_context_.lat == LateralBehavior::kLaneChangeLeft &&
                     (task.user_perferred_behavior == 1 ||
                      task.user_perferred_behavior == 11)) {
            LOG(WARNING)
                << "[HMI]ACTIVE [left] canceled due to human opposite signal. "
                   "Cd alc.";
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          } else if (bp_.cfg()
                         .function()
                         .active_lc()
                         .enable_auto_canbel_by_stick_signal() &&
                     lc_context_.lat == LateralBehavior::kLaneChangeRight &&
                     (task.user_perferred_behavior == -1 ||
                      task.user_perferred_behavior == 12)) {
            LOG(WARNING) << "[HMI]ACTIVE canceled due to human active signal. "
                            "Cd alc.";
            lc_context_.completed = true;
            lc_context_.trigger_when_appropriate = false;
            last_lc_proposal_.trigger_time = stamp;
          }
        }
      }
    }
    // ------------------------------------------------------------------
    // 3b. 空闲状态（变道已完成）
    // ------------------------------------------------------------------
    else {
      // 3b-i. 清除缓存的触发状态（用户取消等待）
      if (task.user_perferred_behavior != 1 &&
          last_task_.user_perferred_behavior == 1 &&
          lc_context_.trigger_when_appropriate) {
        LOG(WARNING) << "[HMI]clear cached stick trigger state. Cd alc.";
        lc_context_.trigger_when_appropriate = false;
        last_lc_proposal_.trigger_time = stamp;
      } else if (task.user_perferred_behavior != -1 &&
                 last_task_.user_perferred_behavior == -1 &&
                 lc_context_.trigger_when_appropriate) {
        LOG(WARNING) << "[HMI]clear cached stick trigger state. Cd alc.";
        lc_context_.trigger_when_appropriate = false;
        last_lc_proposal_.trigger_time = stamp;
      }

      // 3b-ii. 用户拨杆触发：右变道（behavior == 1）
      if (task.user_perferred_behavior == 1 &&
          last_task_.user_perferred_behavior != 1) {
        if (task.lc_info.forbid_lane_change_right) {
          // 被禁止：缓存，等待合适时机
          LOG(WARNING)
              << "[HMI]cannot stick [Right]. Will trigger when appropriate.";
          lc_context_.trigger_when_appropriate = true;
          lc_context_.lat = LateralBehavior::kLaneChangeRight;
        } else {
          // 允许：检查时机是否合适
          if (IsTriggerAppropriate(LateralBehavior::kLaneChangeRight)) {
            // 时机合适，立即触发
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time = GetNearestFutureDecisionPoint(
                stamp, bp_.cfg().function().stick_lane_change_in_seconds());
            lc_context_.ego_lane_id = ego_lane_id_;
            lc_context_.lat = LateralBehavior::kLaneChangeRight;
            lc_context_.type = LaneChangeTriggerType::kStick;
            last_lc_proposal_.trigger_time = stamp;
            LOG(WARNING) << std::fixed << std::setprecision(5)
                         << "[HMI]stick [Right] triggered "
                         << last_task_.user_perferred_behavior << "->"
                         << task.user_perferred_behavior << " in "
                         << bp_.cfg().function().stick_lane_change_in_seconds()
                         << " s. Trigger time " << lc_context_.trigger_time
                         << " and absolute action time: "
                         << lc_context_.desired_operation_time << ". Cd alc.";
          } else {
            // 时机不合适：缓存等待
            lc_context_.trigger_when_appropriate = true;
            lc_context_.lat = LateralBehavior::kLaneChangeRight;
            lc_context_.type = LaneChangeTriggerType::kStick;
            LOG(WARNING)
                << std::fixed << std::setprecision(5)
                << "[HMI]stick [Right] triggered "
                << last_task_.user_perferred_behavior << "->"
                << task.user_perferred_behavior
                << " but not a good time. Will trigger when appropriate.";
          }
        }
      }
      // 3b-iii. 用户拨杆触发：左变道（behavior == -1）
      else if (task.user_perferred_behavior == -1 &&
               last_task_.user_perferred_behavior != -1) {
        if (task.lc_info.forbid_lane_change_left) {
          LOG(WARNING)
              << "[HMI]cannot stick [Left]. Will trigger when appropriate.";
          lc_context_.trigger_when_appropriate = true;
          lc_context_.lat = LateralBehavior::kLaneChangeLeft;
        } else {
          if (IsTriggerAppropriate(LateralBehavior::kLaneChangeLeft)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time = GetNearestFutureDecisionPoint(
                stamp, bp_.cfg().function().stick_lane_change_in_seconds());
            lc_context_.ego_lane_id = ego_lane_id_;
            lc_context_.lat = LateralBehavior::kLaneChangeLeft;
            lc_context_.type = LaneChangeTriggerType::kStick;
            last_lc_proposal_.trigger_time = stamp;
            LOG(WARNING) << std::fixed << std::setprecision(5)
                         << "[HMI]stick [Left] triggered "
                         << last_task_.user_perferred_behavior << "->"
                         << task.user_perferred_behavior << " in "
                         << bp_.cfg().function().stick_lane_change_in_seconds()
                         << " s. Trigger time " << lc_context_.trigger_time
                         << " and absolute action time: "
                         << lc_context_.desired_operation_time << ". Cd alc.";
          } else {
            lc_context_.trigger_when_appropriate = true;
            lc_context_.lat = LateralBehavior::kLaneChangeLeft;
            lc_context_.type = LaneChangeTriggerType::kStick;
            LOG(WARNING)
                << std::fixed << std::setprecision(5)
                << "[HMI]stick [Left] triggered "
                << last_task_.user_perferred_behavior << "->"
                << task.user_perferred_behavior
                << " but not a good time. Will trigger when appropriate.";
          }
        }
      }
      // 3b-iv. 缓存的触发条件满足，执行变道
      else if (lc_context_.trigger_when_appropriate) {
        if (lc_context_.lat == LateralBehavior::kLaneChangeLeft &&
            !task.lc_info.forbid_lane_change_left) {
          if (IsTriggerAppropriate(LateralBehavior::kLaneChangeLeft)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time = GetNearestFutureDecisionPoint(
                stamp, bp_.cfg().function().stick_lane_change_in_seconds());
            lc_context_.ego_lane_id = ego_lane_id_;
            lc_context_.lat = LateralBehavior::kLaneChangeLeft;
            lc_context_.type = LaneChangeTriggerType::kStick;
            last_lc_proposal_.trigger_time = stamp;
            LOG(WARNING) << std::fixed << std::setprecision(5)
                         << "[HMI][[cached]] stick [Left] appropriate in "
                         << bp_.cfg().function().stick_lane_change_in_seconds()
                         << " s. Trigger time " << lc_context_.trigger_time
                         << " and absolute action time: "
                         << lc_context_.desired_operation_time << ". Cd alc.";
          }
        } else if (lc_context_.lat == LateralBehavior::kLaneChangeRight &&
                   !task.lc_info.forbid_lane_change_right) {
          if (IsTriggerAppropriate(LateralBehavior::kLaneChangeRight)) {
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time = GetNearestFutureDecisionPoint(
                stamp, bp_.cfg().function().stick_lane_change_in_seconds());
            lc_context_.ego_lane_id = ego_lane_id_;
            lc_context_.lat = LateralBehavior::kLaneChangeRight;
            lc_context_.type = LaneChangeTriggerType::kStick;
            last_lc_proposal_.trigger_time = stamp;
            LOG(WARNING) << std::fixed << std::setprecision(5)
                         << "[HMI][[cached]] stick [Right] triggered in "
                         << bp_.cfg().function().stick_lane_change_in_seconds()
                         << " s. Trigger time " << lc_context_.trigger_time
                         << " and absolute action time: "
                         << lc_context_.desired_operation_time << ". Cd alc.";
          }
        }
      }
      // 3b-v. 主动变道提案生效
      else {
        if (last_lc_proposal_.valid &&
            map_adapter_.IsLaneConsistent(last_lc_proposal_.ego_lane_id,
                                          ego_lane_id_) &&
            stamp > last_lc_proposal_.trigger_time &&
            last_lc_proposal_.lat != LateralBehavior::kLaneKeeping) {
          // 检查提案方向没有被禁止
          if ((last_lc_proposal_.lat == LateralBehavior::kLaneChangeLeft &&
               !task.lc_info.forbid_lane_change_left) ||
              (last_lc_proposal_.lat == LateralBehavior::kLaneChangeRight &&
               !task.lc_info.forbid_lane_change_right)) {
            // 激活主动变道
            lc_context_.completed = false;
            lc_context_.trigger_when_appropriate = false;
            lc_context_.trigger_time = stamp;
            lc_context_.desired_operation_time =
                last_lc_proposal_.trigger_time +
                last_lc_proposal_.operation_at_seconds;
            lc_context_.ego_lane_id = last_lc_proposal_.ego_lane_id;
            lc_context_.lat = last_lc_proposal_.lat;
            lc_context_.type = LaneChangeTriggerType::kActive;
            last_lc_proposal_.trigger_time = stamp;
            if (last_lc_proposal_.lat == LateralBehavior::kLaneChangeLeft) {
              LOG(WARNING) << std::fixed << std::setprecision(5)
                           << "[HMI][[Active]] [Left] triggered in "
                           << last_lc_proposal_.operation_at_seconds
                           << " s. Trigger time " << lc_context_.trigger_time
                           << " and absolute action time: "
                           << lc_context_.desired_operation_time << ". Cd alc.";
            } else {
              LOG(WARNING) << std::fixed << std::setprecision(5)
                           << "[HMI][[Active]] [Right] triggered in "
                           << last_lc_proposal_.operation_at_seconds
                           << " s. Trigger time " << lc_context_.trigger_time
                           << " and absolute action time: "
                           << lc_context_.desired_operation_time << ". Cd alc.";
            }
          }
        }
      }
    }
  }  // if under control

  // ========================================================================
  // 4. 周期结束清理
  // ========================================================================
  // 任何变道提案只持续一个周期
  last_lc_proposal_.valid = false;
  // 保存当前任务为下一周期的比较基准
  last_task_ = task;
}

// ============================================================================
//  快照保存与行为构建
// ============================================================================

/// @brief 保存EUDM规划结果到快照
///
/// 从EudmPlanner复制所有核心结果数据到Snapshot结构体，
/// 包括优胜序列ID、仿真结果、代价、轨迹、行为、时间戳和时间开销。
///
/// @param snapshot [out] 输出的快照结构体
void EudmManager::SaveSnapshot(Snapshot* snapshot) {
  snapshot->valid = true;
  snapshot->plan_state = bp_.plan_state();
  snapshot->original_winner_id = bp_.winner_id();
  snapshot->processed_winner_id = bp_.winner_id();  // 初始值等于原始ID，后续可能被Reselect修改
  snapshot->action_script = bp_.action_script();
  snapshot->sim_res = bp_.sim_res();
  snapshot->risky_res = bp_.risky_res();
  snapshot->sim_info = bp_.sim_info();
  snapshot->final_cost = bp_.final_cost();
  snapshot->progress_cost = bp_.progress_cost();
  snapshot->tail_cost = bp_.tail_cost();
  snapshot->forward_trajs = bp_.forward_trajs();
  snapshot->forward_lat_behaviors = bp_.forward_lat_behaviors();
  snapshot->forward_lon_behaviors = bp_.forward_lon_behaviors();
  snapshot->surround_trajs = bp_.surround_trajs();

  snapshot->plan_stamp = map_adapter_.map()->time_stamp();
  snapshot->time_cost = bp_.time_cost();
}

/// @brief 从快照构建语义行为输出
///
/// 提取重选后的优胜序列对应的行为信息：
/// 1. 横向行为：序列第一个动作的横向方向
/// 2. 纵向行为：序列第一个动作的纵向方向
/// 3. 自车的前向仿真轨迹（仅优胜序列的）
/// 4. 周围车辆的仿真轨迹（仅优胜序列的）
/// 5. 规划时的自车状态
/// 6. 参考车道
///
/// @param behavior [out] 输出的语义行为结构
void EudmManager::ConstructBehavior(common::SemanticBehavior* behavior) {
  if (not last_snapshot_.valid) return;
  int selected_seq_id = last_snapshot_.processed_winner_id;

  // 构建周围车辆轨迹的最终版本（仅包含优胜序列的轨迹）
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_final;
  surround_trajs_final.emplace_back(
      last_snapshot_.surround_trajs[selected_seq_id]);

  // 提取行为信息
  behavior->lat_behavior =
      last_snapshot_.forward_lat_behaviors[selected_seq_id].front();
  behavior->lon_behavior =
      last_snapshot_.forward_lon_behaviors[selected_seq_id].front();
  behavior->forward_trajs = vec_E<vec_E<common::Vehicle>>{
      last_snapshot_.forward_trajs[selected_seq_id]};
  behavior->forward_behaviors = std::vector<LateralBehavior>{
      last_snapshot_.forward_lat_behaviors[selected_seq_id].front()};
  behavior->surround_trajs = surround_trajs_final;
  behavior->state = last_snapshot_.plan_state;
  behavior->ref_lane = last_snapshot_.ref_lane;
}

// ============================================================================
//  参考速度评估
// ============================================================================

/// @brief 评估参考速度——基于道路曲率的安全速度约束
///
/// ## 算法
///
/// 1. 计算安全制动距离：
///    t_forward = plan_state.velocity / a_comfort (使用舒适制动减速度)
///    s_forward = min(max(20, t_forward * velocity), lane.end())
///
/// 2. 在参考车道上，从当前位置向前以resolution=0.2m采样：
///    对每个采样点查询该处的曲率(curvature)和曲率变化率(cc)：
///      v_max_by_curvature = sqrt(a_lat_max / |curvature|)
///      v_ref = min(v_ref, v_max_by_curvature)
///
/// 3. 最终参考速度：
///    ref_vel = floor(min(max(v_ref, 0), user_desired_vel))
///
/// ## 设计意图
/// 确保车辆在任何参考车道曲率下都有足够的安全裕度：
/// 最大侧向加速度 a_lat_max = v^2 * |curvature|
/// => 最大安全速度 v_max = sqrt(a_lat_max / |curvature|)
///
/// @param task 当前任务
/// @param ref_vel [out] 输出的参考速度（向下取整）
ErrorType EudmManager::EvaluateReferenceVelocity(
    const planning::eudm::Task& task, decimal_t* ref_vel) {
  // 参考车道无效时直接使用用户期望速度
  if (!last_snapshot_.ref_lane.IsValid()) {
    *ref_vel = task.user_desired_vel;
    return kSuccess;
  }

  // 获取当前Frenet状态
  common::StateTransformer stf(last_snapshot_.ref_lane);
  common::FrenetState current_fs;
  stf.GetFrenetStateFromState(last_snapshot_.plan_state, &current_fs);

  decimal_t c, cc;
  decimal_t v_max_by_curvature;
  decimal_t v_ref = kInf;

  // 舒适制动减速度（用于计算前向搜索距离）
  decimal_t a_comfort = bp_.cfg().sim().ego().lon().limit().soft_brake();
  // 制动时间：速度 / 舒适制动减速度
  decimal_t t_forward = last_snapshot_.plan_state.velocity / a_comfort;
  // 前向搜索距离：max(20m, 制动距离), 不超过车道末端
  decimal_t s_forward =
      std::min(std::max(20.0, t_forward * last_snapshot_.plan_state.velocity),
               last_snapshot_.ref_lane.end());
  decimal_t resolution = 0.2;  // 采样分辨率0.2m

  // 沿车道向前采样曲率
  for (decimal_t s = current_fs.vec_s[0]; s < current_fs.vec_s[0] + s_forward;
       s += resolution) {
    if (last_snapshot_.ref_lane.GetCurvatureByArcLength(s, &c, &cc) ==
        kSuccess) {
      // 由侧向加速度约束计算最大安全过弯速度
      v_max_by_curvature =
          sqrt(bp_.cfg().sim().ego().lat().limit().acc() / fabs(c));
      // 取所有采样点中的最小速度（最严格约束）
      v_ref = v_max_by_curvature < v_ref ? v_max_by_curvature : v_ref;
    }
  }

  // 最终参考速度 = floor(min(max(v_ref, 0), 用户期望速度))
  *ref_vel = std::floor(std::min(std::max(v_ref, 0.0), task.user_desired_vel));

  LOG(WARNING) << "[Eudm][Desired]User ref vel: " << task.user_desired_vel
               << ", final ref vel: " << *ref_vel;
  return kSuccess;
}

// ============================================================================
//  上下文感知重选
// ============================================================================

/// @brief [核心函数] 根据变道上下文从快照中重选最优动作序列
///
/// EUDM原始选出的代价最低序列可能不符合当前的变道上下文。
/// 此函数根据变道状态对候选序列进行筛选，选出符合上下文约束的
/// 最优序列。
///
/// ## 重选规则
///
/// 根据变道上下文的状态分三种情况：
///
/// ### 情况1: 变道已完成（lc_context_.completed == true）
/// 只选择"保持车道"的序列（lat_behavior == kLaneKeeping）
/// 不选择任何变道序列，防止自动变道。
///
/// ### 情况2: 变道进行中但尚未到达执行时间（stamp < desired_operation_time）
/// 只选择"保持车道"的序列。
/// 在等待变道时机期间，应保持当前车道。
///
/// ### 情况3: 变道进行中且已到达执行时间（stamp >= desired_operation_time）
/// 选择"保持车道"或"匹配变道方向"的序列。
/// 此时应该准备执行或正在执行变道操作。
///
/// ## 选择策略
///
/// 在符合条件的序列中，选择final_cost最小的。
/// 只考虑仿真成功（sim_res == true）的序列。
///
/// ## 错误处理
///
/// 如果找不到任何符合条件的序列（例如所有变道序列都仿真失败），
/// 返回kWrongStatus。此时上一层（Run方法）会提前退出，
/// 使用上一周期的规划结果。
///
/// @param stamp 当前时间戳
/// @param snapshot 完整仿真结果快照
/// @param new_seq_id [out] 重选后的序列ID
ErrorType EudmManager::ReselectByContext(const decimal_t stamp,
                                         const Snapshot& snapshot,
                                         int* new_seq_id) {
  int selected_seq_id;
  int num_seqs = snapshot.action_script.size();
  bool find_match = false;
  decimal_t cost = kInf;

  for (int i = 0; i < num_seqs; i++) {
    // 跳过仿真失败的序列
    if (!snapshot.sim_res[i]) continue;

    // 分析该序列的横向行为类型
    common::LateralBehavior lat_behavior;
    decimal_t operation_at_seconds;
    bool is_cancel_behavior;
    bp_.ClassifyActionSeq(snapshot.action_script[i], &operation_at_seconds,
                          &lat_behavior, &is_cancel_behavior);

    // 根据变道上下文状态进行筛选
    if ((lc_context_.completed &&
         lat_behavior == common::LateralBehavior::kLaneKeeping) ||
        (!lc_context_.completed && stamp < lc_context_.desired_operation_time &&
         lat_behavior == common::LateralBehavior::kLaneKeeping) ||
        (!lc_context_.completed &&
         stamp >= lc_context_.desired_operation_time &&
         (lat_behavior == lc_context_.lat ||
          lat_behavior == common::LateralBehavior::kLaneKeeping))) {
      find_match = true;
      // 选择代价最低的匹配序列
      if (snapshot.final_cost[i] < cost) {
        cost = snapshot.final_cost[i];
        selected_seq_id = i;
      }
    }
  }

  if (!find_match) {
    return kWrongStatus;
  }
  *new_seq_id = selected_seq_id;
  return kSuccess;
}

// ============================================================================
//  EUDM主运行流程
// ============================================================================

/// @brief [核心函数] 执行一次完整的EUDM规划生命周期
///
/// 这是EUDM系统的最高层调度函数，编排五个规划阶段：
///
/// ## 五阶段流水线
///
/// ```
/// [I]   Prepare
///      -> 设置地图数据
///      -> 获取期望动作
///      -> 处理变道上下文
///      -> 评估参考速度
///      -> 配置变道信息
///  [II]  RunOnce
///      -> EudmPlanner::RunOnce()
///      -> 多线程并行仿真 + 代价评估 + 优胜选择
/// [III]  Summarize
///      -> SaveSnapshot(snapshot)
///      -> 复制所有结果数据到快照
///  [IV]  Reselect
///      -> ReselectByContext(snapshot)
///      -> 根据变道上下文重选最优序列
///  [V]   Update
///      -> 获取参考车道
///      -> 存储快照
///      -> 生成变道提案
///      -> 更新规划上下文
/// ```
///
/// ## 时间日志
///
/// 每个阶段的耗时都会被记录并输出：
/// - Prepare time
/// - RunOnce time
/// - Sum & Reselect time
/// - Lane fitting & update time
/// - Total run time
///
/// 这有助于定位性能瓶颈。
///
/// ## 错误恢复
///
/// 如果任何阶段失败（返回kWrongStatus），函数立即返回错误。
/// 此时last_snapshot_保持不变，系统使用上一周期的规划结果。
///
/// @param stamp 当前时间戳
/// @param map_ptr 语义地图管理器共享指针
/// @param task EUDM规划任务
/// @return kSuccess表示全部阶段成功完成
ErrorType EudmManager::Run(
    const decimal_t stamp,
    const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,
    const planning::eudm::Task& task) {
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]******************** RUN START: " << stamp
               << "******************";
  static TicToc eudm_timer;
  eudm_timer.tic();

  // ========================================================================
  // [I] Prepare阶段：准备地图和上下文信息
  // ========================================================================
  static TicToc prepare_timer;
  prepare_timer.tic();
  if (Prepare(stamp, map_ptr, task) != kSuccess) {
    return kWrongStatus;
  }
  auto t_prepare = prepare_timer.toc();
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]Prepare time cost " << t_prepare << " ms";

  // ========================================================================
  // [II] RunOnce阶段：执行核心EUDM仿真与评估
  // ========================================================================
  static TicToc runonce_timer;
  runonce_timer.tic();
  if (bp_.RunOnce() != kSuccess) {
    LOG(WARNING) << "[Eudm][Fatal]BP runonce failed.";
    return kWrongStatus;
  }
  auto t_runonce = runonce_timer.toc();
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]RunOnce time cost " << t_runonce << " ms";

  // ========================================================================
  // [III] Summarize阶段：保存规划结果
  // [IV] Reselect阶段： 上下文感知重选
  // ========================================================================
  static TicToc sum_reselect_timer;
  sum_reselect_timer.tic();

  Snapshot snapshot;
  SaveSnapshot(&snapshot);

  if (ReselectByContext(stamp, snapshot, &snapshot.processed_winner_id) !=
      kSuccess) {
    LOG(WARNING) << "[Eudm][Fatal]Reselect failed.";
    return kWrongStatus;
  }

  LOG(WARNING) << "[Eudm]original id " << snapshot.original_winner_id
               << " reselect : " << snapshot.processed_winner_id;

  // 输出重选后的优胜序列详情
  {
    std::ostringstream line_info;
    line_info << "[Eudm][Output]Reselected <if_risky:"
              << snapshot.risky_res[snapshot.processed_winner_id] << ">[";
    for (auto& a : snapshot.action_script[snapshot.processed_winner_id]) {
      line_info << DcpTree::RetLonActionName(a.lon);
    }
    line_info << "|";
    for (auto& a : snapshot.action_script[snapshot.processed_winner_id]) {
      line_info << DcpTree::RetLatActionName(a.lat);
    }
    line_info << "]";
    // 输出前向轨迹中每个点的信息
    for (auto& v : snapshot.forward_trajs[snapshot.processed_winner_id]) {
      line_info << std::fixed << std::setprecision(5) << "<"
                << v.state().time_stamp - stamp << "," << v.state().velocity
                << "," << v.state().acceleration << "," << v.state().curvature
                << ">";
    }
    LOG(WARNING) << line_info.str();
  }
  auto t_sum_reselect = sum_reselect_timer.toc();
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]Sum & Reselect time cost " << t_sum_reselect << " ms";

  // ========================================================================
  // [V] Update阶段：获取参考车道、存储快照、生成变道提案、更新上下文
  // ========================================================================
  static TicToc lane_timer;
  lane_timer.tic();

  // 获取参考车道（用于后续的参考速度评估和可视化）
  if (map_adapter_.map()->GetRefLaneForStateByBehavior(
          snapshot.plan_state, std::vector<int>(),
          snapshot.forward_lat_behaviors[snapshot.processed_winner_id].front(),
          250.0, 20.0, true, &(snapshot.ref_lane)) != kSuccess) {
    return kWrongStatus;
  }

  // 保存快照为最新规划结果
  last_snapshot_ = snapshot;

  // 生成系统主动变道提案
  GenerateLaneChangeProposal(stamp, task);

  // 更新规划上下文（为下一周期的重规划做准备）
  context_.is_valid = true;
  context_.seq_start_time = stamp;
  context_.action_seq = snapshot.action_script[snapshot.processed_winner_id];

  auto t_lane = lane_timer.toc();
  LOG(WARNING) << "[Eudm]Fit reflane & update cost: " << t_lane << " ms";

  // 汇总时间日志
  auto t_sum = t_prepare + t_runonce + t_sum_reselect + t_lane;
  auto t_eudmrun = eudm_timer.toc();
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]Sum of time: " << t_sum
               << " ms, diff: " << t_eudmrun - t_sum << " ms";
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]******************** RUN FINISH: " << stamp << " +"
               << t_eudmrun << " ms ******************";
  return kSuccess;
}

// ============================================================================
//  重规划期望动作获取
// ============================================================================

/// @brief 从重规划上下文中获取当前应执行的动作
///
/// 根据当前时间和上次规划起始时间的差值（time_since_last_plan），
/// 以及动作序列中每个动作的累计时间，确定当前应执行哪个动作。
///
/// ## 算法
///
/// ```
/// t_aggre = 0
/// for each action in context_.action_seq:
///   t_aggre += action.t
///   if time_since_last_plan + epsilon < t_aggre:
///     desired_action = action
///     desired_action.t = t_aggre - time_since_last_plan  // 剩余时间
///     return true
/// return false  // 所有动作都已过期
/// ```
///
/// 例如：context_.action_seq = [(M,K,1.0), (A,K,2.0), (M,L,2.0)]
/// time_since_last_plan = 1.5s
/// - t_aggre after action0: 1.0 (1.5+eps > 1.0, 继续)
/// - t_aggre after action1: 3.0 (1.5+eps < 3.0, 找到!)
/// desired_action = (A,K, 3.0-1.5=1.5)  // 加速+保持, 还剩1.5秒
///
/// @param current_time 当前时间戳
/// @param desired_action [out] 输出的期望动作
/// @return true表示成功获取，false表示上下文无效或所有动作已过期
bool EudmManager::GetReplanDesiredAction(const decimal_t current_time,
                                         DcpAction* desired_action) {
  if (!context_.is_valid) return false;

  decimal_t time_since_last_plan = current_time - context_.seq_start_time;
  if (time_since_last_plan < -kEPS) return false;  // 时间倒退，异常

  decimal_t t_aggre = 0.0;
  bool find_match_action = false;
  int action_seq_len = context_.action_seq.size();

  for (int i = 0; i < action_seq_len; ++i) {
    t_aggre += context_.action_seq[i].t;
    if (time_since_last_plan + kEPS < t_aggre) {
      // 当前时间点落在第i个动作的持续期内
      *desired_action = context_.action_seq[i];
      // 剩余时间 = 动作结束时间 - 当前时间
      desired_action->t = t_aggre - time_since_last_plan;
      find_match_action = true;
      break;
    }
  }

  // 所有动作都已经过期
  if (!find_match_action) {
    return false;
  }
  return true;
}

// ============================================================================
//  公共接口
// ============================================================================

/// @brief 重置规划上下文（清空状态）
///
/// 在系统重置或退出自动驾驶模式时调用，清除所有累积的规划上下文。
void EudmManager::Reset() { context_.is_valid = false; }

/// @brief 获取EudmPlanner引用
EudmPlanner& EudmManager::planner() { return bp_; }

}  // namespace planning
