/**
 * @file eudm_manager.h
 * @author EPSILON Autonomous Driving Team
 * @brief EUDM管理器——完整的EUDM规划生命周期管理
 * @version 0.1
 * @date 2019-07-07
 * @copyright Copyright (c) 2019
 *
 * ## 设计原理
 *
 * 由于EUDM规划器本身**完全无状态**（每个规划周期独立运行），
 * EudmManager作为状态机包装器，负责：
 *
 * 1. **跟踪规划上下文**：维护上次规划结果的快照（Snapshot），
 *    包括优胜的动作序列、轨迹、代价等
 * 2. **管理变道生命周期**：追踪完整变道操作的状态变迁，
 *    处理用户拨杆信号和系统主动变道请求
 * 3. **桥接任务与规划器**：将上游输入（语义地图、用户指令）转换为
 *    EUDM规划器需要的格式
 * 4. **重新选择**：基于变道上下文对原始优胜结果进行重选
 *    （例如：如果系统正在进行变道操作，则筛除非变道的候选序列）
 * 5. **参考速度评估**：基于道路曲率约束确定安全的参考速度
 *
 * ## 变道触发机制
 *
 * 支持两种变道触发方式：
 * - **kStick（拨杆触发）**：驾驶员通过拨杆请求变道
 * - **kActive（系统主动触发）**：EUDM系统根据场景判断需要变道，
 *   在满足一致性条件（连续N帧same请求）后触发
 *
 * ## 主要工作流程（Run方法）
 *
 * 1. **Prepare阶段**：设置地图、获取plan context、处理变道上下文
 * 2. **RunOnce阶段**：调用EudmPlanner执行核心仿真和评估
 * 3. **Summarize阶段**：保存仿真结果到快照（Snapshot）
 * 4. **Reselect阶段**：根据变道上下文重选最优序列
 * 5. **Update阶段**：更新规划上下文，生成变道提案
 *
 * @see eudm_planner.h
 * @see eudm_itf.h
 */

#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_MANAGER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_MANAGER_H_

#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/eudm_planner.h"
#include "eudm_planner/map_adapter.h"

namespace planning {

/// @class EudmManager
/// @brief EUDM规划生命周期管理器
///
/// 该管理器的核心职责是实现"带状态的规划"：
/// EUDM本身是无状态的单周期规划，但自动驾驶需要连续运行、处理
/// 变道的完整生命周期。Manager提供状态追踪、上下文感知的重选、
/// 变道触发/取消、参考速度评估等功能。
class EudmManager {
 public:
  using DcpLatAction = planning::DcpTree::DcpLatAction;
  using DcpLonAction = planning::DcpTree::DcpLonAction;
  using DcpAction = planning::DcpTree::DcpAction;
  using LateralBehavior = common::LateralBehavior;
  using LongitudinalBehavior = common::LongitudinalBehavior;
  using CostStructure = planning::EudmPlanner::CostStructure;

  /// @enum LaneChangeTriggerType
  /// @brief 变道触发类型
  enum class LaneChangeTriggerType {
    kStick = 0,  ///< 用户通过拨杆信号触发的变道请求
    kActive       ///< 系统主动发起的变道请求（基于EUDM场景评估）
  };

  /// @struct ReplanningContext
  /// @brief 重规划上下文：记录上一次规划的优胜动作序列
  ///
  /// 用于在下一个规划周期中确定"当前正在执行"的动作（ongoing_action），
  /// 从而保证规划的时间连续性。
  struct ReplanningContext {
    bool is_valid = false;                   ///< 上下文是否有效（首个周期无效）
    decimal_t seq_start_time;                ///< 动作序列开始执行的时间戳
    std::vector<DcpAction> action_seq;       ///< 上次规划的优胜动作序列
  };

  /// @struct ActivateLaneChangeRequest
  /// @brief 系统主动变道请求记录
  ///
  /// 每次EUDM运行后，如果优胜序列包含变道行为，会产生一条请求。
  /// 系统需要积累足够多的**连续一致**请求才能正式触发主动变道。
  struct ActivateLaneChangeRequest {
    decimal_t trigger_time;                  ///< 该请求产生的时间
    decimal_t desired_operation_time;        ///< 期望执行变道操作的时间
    int ego_lane_id;                         ///< 产生请求时的自车车道ID
    LateralBehavior lat = LateralBehavior::kLaneKeeping;  ///< 请求的变道方向
  };

  /// @struct LaneChangeProposal
  /// @brief 变道提案：确认的变道操作计划
  ///
  /// 一旦active requests积累足够（达到consistent_min_num_frame），
  /// 就会产生一个LaneChangeProposal，包含确认的变道时间表。
  struct LaneChangeProposal {
    bool valid = false;                      ///< 提案是否有效
    decimal_t trigger_time = 0.0;            ///< 提案触发时间
    decimal_t operation_at_seconds = 0.0;    ///< 从触发到操作的相对时间
    int ego_lane_id;                         ///< 提案产生时的车道ID
    LateralBehavior lat = LateralBehavior::kLaneKeeping;  ///< 变道方向
  };

  /// @struct LaneChangeContext
  /// @brief 变道上下文：跟踪当前变道操作的完整状态
  ///
  /// 该结构体管理变道操作的整个生命周期：
  /// - completed=false: 正在进行变道（包括等待执行时机）
  /// - completed=true: 没有进行中的变道（可以接受新的变道请求）
  ///
  /// trigger_when_appropriate: 当用户请求变道但当前条件不满足
  /// （例如被禁止或时机不合适），标记为"等合适时机再触发"。
  struct LaneChangeContext {
    bool completed = true;                          ///< 当前变道操作是否已完成
    bool trigger_when_appropriate = false;          ///< 是否等待合适时机触发缓存的变道请求
    decimal_t trigger_time = 0.0;                   ///< 变道触发的时间戳
    decimal_t desired_operation_time = 0.0;         ///< 期望执行变道操作的时间戳
    int ego_lane_id = 0;                            ///< 触发变道时的自车车道ID
    LateralBehavior lat = LateralBehavior::kLaneKeeping;  ///< 变道方向
    LaneChangeTriggerType type;                     ///< 触发类型（拨杆/主动）
  };

  /// @struct Snapshot
  /// @brief EUDM规划结果的完整快照
  ///
  /// 保存一次完整的EUDM规划周期的所有结果数据，包括：
  /// - 原始和重选后的优胜序列
  /// - 所有序列的仿真结果、代价、轨迹
  /// - 规划时间戳和时间开销
  struct Snapshot {
    bool valid = false;                                            ///< 快照是否包含有效数据
    int original_winner_id;                                        ///< EUDM原始优胜序列ID
    int processed_winner_id;                                       ///< 经Reselect处理后最终的优胜序列ID
    common::State plan_state;                                      ///< 规划时的自车状态
    std::vector<std::vector<DcpAction>> action_script;             ///< 所有候选动作序列
    std::vector<bool> sim_res;                                     ///< 各序列仿真成功标志
    std::vector<bool> risky_res;                                   ///< 各序列风险标志
    std::vector<std::string> sim_info;                             ///< 各序列仿真信息
    std::vector<decimal_t> final_cost;                             ///< 各序列最终代价
    std::vector<std::vector<CostStructure>> progress_cost;         ///< 各序列每层代价
    std::vector<CostStructure> tail_cost;                          ///< 各序列尾部代价
    vec_E<vec_E<common::Vehicle>> forward_trajs;                   ///< 各序列自车轨迹
    std::vector<std::vector<LateralBehavior>> forward_lat_behaviors;   ///< 各序列每层横向行为
    std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors; ///< 各序列每层纵向行为
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs; ///< 各序列周围车轨迹
    common::Lane ref_lane;                                         ///< 参考车道

    double plan_stamp = 0.0;                                       ///< 规划时间戳
    double time_cost = 0.0;                                        ///< 规划时间开销（ms）
  };

  EudmManager() {}

  /// @brief 初始化管理器
  ///
  /// 初始化EudmPlanner、地图适配器、工作频率，配置glog日志系统。
  ///
  /// @param config_path EUDM配置文件路径
  /// @param work_rate 规划工作频率（Hz）
  void Init(const std::string& config_path, const decimal_t work_rate);

  /// @brief 执行一次完整的EUDM规划生命周期
  ///
  /// 规划流程（5阶段）：
  /// 1. **Prepare**：设置地图数据、获取期望动作、处理变道上下文、
  ///    评估参考速度、配置变道信息
  /// 2. **RunOnce**：调用EudmPlanner::RunOnce()执行核心仿真和评估
  /// 3. **Summarize**：将仿真结果保存到Snapshot（SaveSnapshot）
  /// 4. **Reselect**：根据变道上下文重选最优序列（ReselectByContext）
  /// 5. **Update**：获取参考车道、更新规划上下文、生成变道提案
  ///
  /// @param stamp 当前时间戳
  /// @param map_ptr 语义地图管理器共享指针
  /// @param task EUDM规划任务描述
  ErrorType Run(
      const decimal_t stamp,
      const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,
      const planning::eudm::Task& task);

  /// @brief 重置规划上下文（清空replanning context）
  void Reset();

  /// @brief 从最新快照构建语义行为输出
  ///
  /// 根据重选后的优胜序列ID，提取对应的横向/纵向行为、
  /// 前向轨迹和周围车辆轨迹。
  ///
  /// @param behavior [out] 输出的语义行为结构
  void ConstructBehavior(common::SemanticBehavior* behavior);

  /// @brief 获取EudmPlanner引用
  EudmPlanner& planner();

  int original_winner_id() const { return last_snapshot_.original_winner_id; }
  int processed_winner_id() const { return last_snapshot_.processed_winner_id; }
  std::shared_ptr<semantic_map_manager::SemanticMapManager> map() {
    return map_adapter_.map();
  }

 private:
  /// @brief 获取最近的未来决策时间点
  ///
  /// 将时间点对齐到layer_time的整数倍，确保变道操作在
  /// 决策周期的边界上触发。
  ///
  /// @param stamp 参考时间点
  /// @param delta 从参考点向前的最小偏移
  /// @return 最近的未来决策点时间戳
  decimal_t GetNearestFutureDecisionPoint(const decimal_t& stamp,
                                          const decimal_t& delta);

  /// @brief 检查当前是否适合执行指定的横向行为
  ///
  /// 检查条件：在上次规划的快照中，是否有足够多的安全序列
  /// 在其第1-3层包含了指定的变道方向（配合加速或维持速度）。
  /// 如果最安全的几个序列都包含该变道方向，则认为时机合适。
  ///
  /// @param lat 待检查的横向行为（左变道或右变道）
  /// @return true表示时机合适，适合执行变道
  bool IsTriggerAppropriate(const LateralBehavior& lat);

  /// @brief 准备阶段：设置地图、提取规划上下文、处理变道上下文和参考速度
  ///
  /// 具体操作：
  /// 1. 设置地图适配器
  /// 2. 从规划上下文中获取期望的重规划动作
  /// 3. 处理变道上下文（若正在进行变道，锁定横向方向）
  /// 4. 评估参考速度（基于曲率约束）
  /// 5. 设置变道信息（推荐变道标志）
  ///
  /// @param stamp 当前时间戳
  /// @param map_ptr 语义地图管理器共享指针
  /// @param task EUDM规划任务
  ErrorType Prepare(
      const decimal_t stamp,
      const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,
      const planning::eudm::Task& task);

  /// @brief 评估参考速度：基于道路曲率的安全速度约束
  ///
  /// 在前方道路上采样曲率，计算每个点的最大安全过弯速度：
  ///   v_max = sqrt(a_lat_max / |curvature|)
  /// 取所有采样点的最小值，并与用户期望速度比较，取较小值。
  ///
  /// @param task 当前任务
  /// @param ref_vel [out] 输出的参考速度
  ErrorType EvaluateReferenceVelocity(const planning::eudm::Task& task,
                                      decimal_t* ref_vel);

  /// @brief 从重规划上下文中获取待执行（desired）动作
  ///
  /// 根据当前时间与上次规划起始时间的差值，确定"当前应该执行
  /// 序列中的哪个动作"。剩余时间 = 序列累计时间 - 已过时间。
  ///
  /// @param current_time 当前时间戳
  /// @param desired_action [out] 输出的期望动作
  /// @return true表示成功获取，false表示上下文无效
  bool GetReplanDesiredAction(const decimal_t current_time,
                              DcpAction* desired_action);

  /// @brief 保存EUDM仿真结果到快照结构体
  ///
  /// 从EudmPlanner复制所有结果数据到Snapshot，用于后续的状态追踪。
  ///
  /// @param snapshot [out] 输出的快照
  void SaveSnapshot(Snapshot* snapshot);

  /// @brief 根据变道上下文从快照中重选最优动作序列
  ///
  /// 核心逻辑：
  /// - 若没有进行中的变道（lc_context_.completed=true），
  ///   只选择保持车道的序列
  /// - 若正在进行变道且尚未到达期望操作时间，
  ///   选择保持车道（等待变道时机）的序列
  /// - 若正在进行变道且已到达期望操作时间，
  ///   选择保持车道或匹配变道方向的序列
  ///
  /// 选出的序列必须在仿真中成功且代价最低。
  ///
  /// @param stamp 当前时间戳
  /// @param snapshot 完整仿真结果快照
  /// @param new_seq_id [out] 重选后的序列ID
  ErrorType ReselectByContext(const decimal_t stamp, const Snapshot& snapshot,
                              int* new_seq_id);

  /// @brief 根据新收到的Task更新变道上下文状态
  ///
  /// 处理以下状态变迁：
  /// - 自动驾驶模式激活/取消：重置变道上下文
  /// - 拨杆信号变化：触发/取消用户变道请求
  /// - 主动变道自动取消条件：超时、禁止信号、用户反向信号
  /// - 缓存触发（trigger_when_appropriate）：当条件满足时自动触发
  /// - 主动变道提案激活
  ///
  /// @param stamp 当前时间戳
  /// @param task 新的任务
  void UpdateLaneChangeContextByTask(const decimal_t stamp,
                                     const planning::eudm::Task& task);

  /// @brief 生成系统主动变道提案
  ///
  /// 检查当前EUDM结果中优胜序列是否包含变道行为，如果包含
  /// 则产生一条ActivateLaneChangeRequest。当连续积累足够数量
  /// 的一致请求后，生成正式的LaneChangeProposal。
  ///
  /// 触发条件：
  /// - 主动变道功能启用
  /// - 自动驾驶系统处于控制状态
  /// - 没有正在进行的变道操作
  /// - 满足冷启动间隔
  /// - 满足速度范围约束
  ///
  /// @param stamp 当前时间戳
  /// @param task 当前任务
  ErrorType GenerateLaneChangeProposal(const decimal_t& stamp,
                                       const planning::eudm::Task& task);

  EudmPlanner bp_;                          ///< EUDM行为规划器实例
  EudmPlannerMapAdapter map_adapter_;       ///< 地图适配器（依赖注入到bp_）
  decimal_t work_rate_{20.0};               ///< 规划工作频率（Hz），默认20Hz

  int ego_lane_id_;                         ///< 自车当前车道ID
  ReplanningContext context_;               ///< 重规划上下文
  Snapshot last_snapshot_;                  ///< 最近一次规划的完整快照
  planning::eudm::Task last_task_;          ///< 最近一次处理的任务（用于检测变化）
  LaneChangeContext lc_context_;            ///< 当前变道操作上下文
  LaneChangeProposal last_lc_proposal_;     ///< 最近一次变道提案
  std::vector<ActivateLaneChangeRequest> preliminary_active_requests_; ///< 积累的主动变道请求队列
};

}  // namespace planning

#endif
