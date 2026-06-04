/**
 * @file behavior_planner.h
 * @brief MPDM（多策略决策）行为规划器头文件
 *
 * 本文件定义了 EPSILON 自动驾驶系统中上层行为规划模块的核心类 BehaviorPlanner。
 *
 * ## 模块定位
 * 行为规划器（Behavior Planner）位于分层规划架构的高层，负责决定自车的宏观驾驶行为：
 *   - 车道保持（Lane Keeping）
 *   - 向左变道（Lane Change Left）
 *   - 向右变道（Lane Change Right）
 *
 * ## MPDM 算法概述
 * MPDM = Multi-Policy Decision Making（多策略决策），是一种基于前向仿真与代价评估
 * 的行为决策算法。其核心思想是：
 *   1. 枚举候选行为：根据当前车道拓扑，枚举 1~3 个可行的横向行为
 *   2. 多智能体前向仿真：对每个候选行为，在 ~4s 时间窗口内以 0.4s 步长进行前向推演，
 *      所有交通参与者使用 IDM 跟驰模型交互
 *   3. 代价评估：对每条仿真轨迹计算综合代价（效率 + 安全 + 动作代价）
 *   4. 最小代价选择：选取综合代价最低的行为作为最终决策
 *
 * ## 自动驾驶等级支持
 *   - L2 级：直接接受 HMI（人机接口）命令，如摇杆变道/调速
 *   - L3 级：HMI 命令作为建议，MPDM 算法进行最终决策（可覆盖 HMI）
 *
 * ## 关键数据结构
 *   - Behavior: 语义行为（横向行为 + 期望速度 + 参考车道 + 前向轨迹）
 *   - LateralBehavior: 横向行为枚举（车道保持/左变道/右变道）
 *   - 状态机: UpdateEgoBehavior 防止不合逻辑的行为跳变
 */
#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_BEHAVIOR_PLANNER_H_
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_BEHAVIOR_PLANNER_H_

#include <memory>
#include <string>

#include "behavior_planner/map_interface.h"
#include "common/basics/basics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/state/state.h"
#include "route_planner/route_planner.h"

#include "forward_simulator/multimodal_forward.h"
#include "forward_simulator/onlane_forward_simulation.h"
namespace planning {

/**
 * @class BehaviorPlanner
 * @brief MPDM 行为规划器主类
 *
 * 继承自 Planner 基类，实现了基于多策略前向仿真的行为决策算法。
 *
 * ## 运行流程（RunOnce）：
 *   1. 获取自车所在车道 ID
 *   2. 获取自车状态
 *   3. 运行路线规划器（RoutePlanner）
 *   4. 通过车道 ID 判断当前横向行为
 *   5. UpdateEgoBehavior: 状态机更新行为（防止非法跳变）
 *   6. 若 L3 级别，运行 MPDM 算法进行决策
 *   7. 构建最终参考车道（ConstructReferenceLane）
 *
 * ## MPDM 子流程（RunMpdm -> MultiBehaviorJudge）：
 *   1. 获取关键车辆集合
 *   2. 枚举候选行为（车道保持 + 可选变道）
 *   3. 为每辆车构建 <车辆, 参考车道> 配对
 *   4. SimulateEgoBehavior: 对每个候选行为进行前向仿真
 *      - 优先使用 MultiAgentSimForward（多智能体交互仿真）
 *      - 失败回退到 OpenloopSimForward（开环仿真）
 *   5. EvaluateMultiPolicyTrajs: 评估所有候选轨迹
 *      - EvaluateSinglePolicyTraj: 单轨迹评估（效率 + 安全 + 动作代价）
 *   6. 选择最小代价行为，并对期望速度做限幅
 *   7. HMI 锁定检查：若 lock_to_hmi_ 为真且 HMI 行为在可行集中，则优先使用
 */
class BehaviorPlanner : public Planner {
 public:
  /// 状态类型别名
  using State = common::State;
  /// 车道类型别名
  using Lane = common::Lane;
  /// 语义行为类型别名（包含横向行为、期望速度、参考车道、前向轨迹等）
  using Behavior = common::SemanticBehavior;
  /// 横向行为枚举类型（车道保持/左变道/右变道）
  using LateralBehavior = common::LateralBehavior;

  /// 返回规划器名称
  std::string Name() override;

  /// 初始化规划器（创建 RoutePlanner，初始化行为状态）
  ErrorType Init(const std::string config) override;

  /// 单次规划循环入口，执行完整的行为决策流程
  ErrorType RunOnce() override;

  /// 设置地图接口（依赖注入，解耦地图数据源）
  void set_map_interface(BehaviorPlannerMapItf* itf);

  /**
   * @brief 设置用户期望速度（HMI 输入）
   * @param desired_vel 用户指定的期望速度 [m/s]
   * @note 仅在 L2+ 级别生效
   */
  void set_user_desired_velocity(const decimal_t desired_vel);

  /**
   * @brief L2 级别的人机接口变道命令
   * @param hmi_behavior HMI 指定的横向行为
   * @note L2 级别直接执行；L3 级别作为 MPDM 的建议输入
   */
  void set_hmi_behavior(const LateralBehavior& hmi_behavior);

  /**
   * @brief 设置自动驾驶等级
   * @param level 自动驾驶等级（2 = L2 / 3 = L3）
   * @note L2: HMI 直接控制；L3: MPDM 算法决策
   */
  void set_autonomous_level(int level);

  /**
   * @brief 设置前向仿真时间步长
   * @param sim_resolution 仿真步长 [s]，默认 0.4s
   */
  void set_sim_resolution(const decimal_t sim_resolution);

  /**
   * @brief 设置前向仿真时间窗口
   * @param sim_horizon 仿真时间范围 [s]，默认 4.0s
   */
  void set_sim_horizon(const decimal_t sim_horizon);

  /// 设置是否使用仿真状态
  void set_use_sim_state(bool use_sim_state);

  /**
   * @brief 设置激进等级
   * @param level 激进程度，影响 IDM 参数和代价函数权重
   */
  void set_aggressive_level(int level);

  /**
   * @brief 运行路线规划器
   * @param nearest_lane_id 离自车最近的车道 ID
   * @note 为 MPDM 提供导航路径信息
   */
  ErrorType RunRoutePlanner(const int nearest_lane_id);

  /**
   * @brief 运行 MPDM 多策略决策主流程
   * @return kSuccess 如果成功选出最优行为
   * @note 调用 MultiBehaviorJudge 完成候选枚举、前向仿真、代价评估全流程
   */
  ErrorType RunMpdm();

  /// 获取当前决策出的语义行为
  Behavior behavior() const;

  /// 获取用户设定期望速度
  decimal_t user_desired_velocity() const;

  /// 获取参考期望速度（受曲率限制后的安全速度）
  decimal_t reference_desired_velocity() const;

  /// 获取当前自动驾驶等级
  int autonomous_level() const;

  /// 获取所有候选行为的前向仿真轨迹（用于可视化）
  vec_E<vec_E<common::Vehicle>> forward_trajs() const;

  /// 获取所有候选行为列表（用于可视化）
  std::vector<LateralBehavior> forward_behaviors() const;

 protected:
  /**
   * @brief 根据横向行为构建参考车道
   * @param lat_behavior 目标横向行为
   * @param[out] lane 构建出的参考车道
   * @note 根据行为确定目标车道 ID，从地图获取采样点，拟合成平滑车道
   */
  ErrorType ConstructReferenceLane(const LateralBehavior& lat_behavior,
                                   Lane* lane);

  /**
   * @brief 从离散采样点拟合成 Lane 对象
   * @param samples 车道中心线采样点序列
   * @param[out] lane 拟合后的车道
   * @note 使用三次样条插值，生成 20 段分段多项式
   */
  ErrorType ConstructLaneFromSamples(const vec_E<Vecf<2>>& samples, Lane* lane);

  /**
   * @brief MPDM 多行为决策核心函数
   *
   * 完整执行 MPDM 算法的全部步骤：
   *   1. 获取关键车辆信息
   *   2. 枚举候选横向行为（1~3 个）
   *   3. 为每辆车构建参考车道
   *   4. 对每个候选行为进行前向仿真（SimulateEgoBehavior）
   *   5. 对所有仿真轨迹进行代价评估（EvaluateMultiPolicyTrajs）
   *   6. 选择最小代价行为，对期望速度限幅
   *   7. HMI 锁定逻辑检查
   *
   * @param previous_desired_vel 上一帧的期望速度
   * @param[out] mpdm_behavior MPDM 选出的最优横向行为
   * @param[out] actual_desired_velocity MPDM 计算出的最优期望速度
   */
  ErrorType MultiBehaviorJudge(const decimal_t previous_desired_vel,
                               LateralBehavior* mpdm_behavior,
                               decimal_t* actual_desired_velocity);

  /**
   * @brief 获取指定行为的潜在目标车道 ID 列表
   *
   * @param source_lane_id 源车道 ID
   * @param beh 目标横向行为
   * @param[out] candidate_lane_ids 候选车道 ID 列表
   * @note 车道保持返回子车道；变道返回目标车道及其子车道
   */
  ErrorType GetPotentialLaneIds(const int source_lane_id,
                                const LateralBehavior& beh,
                                std::vector<int>* candidate_lane_ids);

  /// 更新自车当前所在车道 ID，并刷新所有候选行为的车道 ID 列表
  ErrorType UpdateEgoLaneId(const int new_ego_lane_id);

  /**
   * @brief 根据车辆位置对应的车道 ID 推断当前横向行为
   *
   * @param ego_lane_id_by_pos 根据自车位置查询到的车道 ID
   * @param[out] behavior_by_lane_id 推断出的横向行为
   * @note 通过与潜在车道 ID 列表匹配来判断车辆正处于何种行为状态
   */
  ErrorType JudgeBehaviorByLaneId(const int ego_lane_id_by_pos,
                                  LateralBehavior* behavior_by_lane_id);

  /**
   * @brief 行为状态机：根据观测行为更新系统行为状态
   *
   * 防止不合逻辑的行为跳变，状态转移规则：
   *   - 车道保持 + 观测到车道保持 -> 保持
   *   - 车道保持 + 观测到非保持 -> 标记为 Undefined（异常）
   *   - 变道中 + 观测到车道保持 -> 仍处于变道过程中
   *   - 变道中 + 观测到同侧变道 -> 变道完成，切回车道保持
   *   - 变道中 + 观测到 Undefined -> 取消变道，切回车道保持
   *
   * @param behavior_by_lane_id 根据车辆位置观测到的行为
   */
  ErrorType UpdateEgoBehavior(const LateralBehavior& behavior_by_lane_id);

  /**
   * @brief 多智能体交互前向仿真
   *
   * 在每个仿真步长内：
   *   1. 对每辆车（包括自车），查找其参考车道上的前车
   *   2. 使用 IDM 跟驰模型计算加速度并推进状态
   *   3. 检查是否与前车碰撞（碰撞则仿真失败）
   *   4. 所有车辆同时更新，体现交互效应
   *
   * 这是 MPDM 中的核心仿真方法，所有交通参与者互相影响。
   *
   * @param ego_id 自车 ID
   * @param semantic_vehicle_set 包含自车和周围车辆的全集
   * @param[out] traj 自车的仿真轨迹
   * @param[out] surround_trajs 周围车辆的仿真轨迹（按 ID 索引）
   */
  ErrorType MultiAgentSimForward(
      const int ego_id, const common::SemanticVehicleSet& semantic_vehicle_set,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

  /**
   * @brief 开环前向仿真（回退方案）
   *
   * 周围车辆独立行驶不反应自车行为，自车沿参考车道用 IDM 行驶。
   * 当 MultiAgentSimForward 失败时（如碰撞、数值错误）作为降级方案使用。
   *
   * @param ego_semantic_vehicle 自车语义车辆（含参考车道）
   * @param agent_vehicles 周围车辆集合（不含自车）
   * @param[out] traj 自车的仿真轨迹
   * @param[out] surround_trajs 周围车辆的仿真轨迹
   */
  ErrorType OpenloopSimForward(
      const common::SemanticVehicle& ego_semantic_vehicle,
      const common::SemanticVehicleSet& agent_vehicles,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

  /**
   * @brief 对单个候选行为执行完整的仿真流程
   *
   * 执行步骤：
   *   1. 构建自车在该行为下的参考车道
   *   2. 尝试 MultiAgentSimForward（多智能体交互仿真）
   *   3. 若失败，回退到 OpenloopSimForward（开环仿真）
   *
   * @param ego_vehicle 自车当前状态
   * @param ego_behavior 候选的横向行为
   * @param semantic_vehicle_set 周围车辆集合
   * @param[out] traj 仿真得到的前向轨迹
   * @param[out] surround_trajs 周围车辆的仿真轨迹
   */
  ErrorType SimulateEgoBehavior(
      const common::Vehicle& ego_vehicle, const LateralBehavior& ego_behavior,
      const common::SemanticVehicleSet& semantic_vehicle_set,
      vec_E<common::Vehicle>* traj,
      std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs);

  /**
   * @brief 对多个候选策略的仿真轨迹进行代价评估与最优选择
   *
   * 遍历每个候选行为的仿真结果，调用 EvaluateSinglePolicyTraj 计算代价，
   * 选取代价最小的行为作为胜出行为。
   *
   * @param valid_behaviors 有效候选行为列表
   * @param valid_forward_trajs 对应的自车前向轨迹列表
   * @param valid_surround_trajs 对应的周围车辆轨迹列表
   * @param[out] winner_behavior 最小代价的胜出行为
   * @param[out] winner_forward_traj 胜出行为对应的前向轨迹
   * @param[out] winner_score 胜出行为的代价分数
   * @param[out] desired_vel 胜出行为对应的期望速度
   */
  ErrorType EvaluateMultiPolicyTrajs(
      const std::vector<LateralBehavior>& valid_behaviors,
      const vec_E<vec_E<common::Vehicle>>& valid_forward_trajs,
      const vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>&
          valid_surround_trajs,
      LateralBehavior* winner_behavior,
      vec_E<common::Vehicle>* winner_forward_traj, decimal_t* winner_score,
      decimal_t* desired_vel);

  /**
   * @brief 评估单条候选策略轨迹的综合代价
   *
   * 代价函数包含三部分：
   *   - cost_efficiency（效率代价）:
   *       分为自车到期望速度的偏差 + 前车限速导致的效率损失
   *       = 0.5 * (|v_ego_terminal - v_desired| / 10.0 + distance_residual_ratio * |Δv| / max(2.0, d_lead))
   *   - cost_safety（安全代价）:
   *       对每辆周围车辆，检查轨迹对碰撞并使用膨化车辆计算碰撞速度差
   *       碰撞代价值 = 0.01 * |v_ego - v_other| * 0.5
   *   - cost_action（动作代价）:
   *       车道保持 = 0.0，变道 = 0.5（惩罚频繁变道）
   *
   * 总代价 = cost_action + cost_safety + cost_efficiency
   *
   * @param behavior 候选横向行为
   * @param forward_traj 自车的前向仿真轨迹
   * @param surround_traj 周围车辆的仿真轨迹（按 ID 索引）
   * @param[out] score 综合代价（越低越好）
   * @param[out] desired_vel 该轨迹对应的期望速度
   */
  ErrorType EvaluateSinglePolicyTraj(
      const LateralBehavior& behaivor,
      const vec_E<common::Vehicle>& forward_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_traj,
      decimal_t* score, decimal_t* desired_vel);

  /**
   * @brief 评估两条轨迹之间的安全代价
   *
   * 对轨迹的每个时间步：
   *   1. 将两车膨化（各方向+1m 安全距离）
   *   2. 检查膨化矩形是否碰撞
   *   3. 若碰撞，代价 += 0.01 * |速度差| * 0.5
   *
   * @param traj_a 轨迹 A（通常为自车轨迹）
   * @param traj_b 轨迹 B（通常为周围车辆轨迹）
   * @param[out] cost 累积安全代价
   */
  ErrorType EvaluateSafetyCost(const vec_E<common::Vehicle>& traj_a,
                               const vec_E<common::Vehicle>& traj_b,
                               decimal_t* cost);

  /**
   * @brief 从轨迹中获取期望速度
   *
   * 遍历轨迹所有状态点，找法向加速度最大点对应的速度作为期望速度，
   * 本质上是在轨迹中选出最保守（最受曲率约束）的速度值。
   *
   * @param vehicle_vec 车辆轨迹序列
   * @param[out] vel 期望速度
   */
  ErrorType GetDesiredVelocityOfTrajectory(
      const vec_E<common::Vehicle> vehicle_vec, decimal_t* vel);

  /// 地图接口指针（依赖注入），解耦地图数据来源
  BehaviorPlannerMapItf* map_itf_{nullptr};

  /// 当前语义行为状态（横向行为 + 期望速度 + 参考车道 + 前向轨迹）
  Behavior behavior_;

  /// 路线规划器指针
  planning::RoutePlanner* p_route_planner_{nullptr};

  /// 用户设定的期望速度 [m/s]，默认 5.0
  decimal_t user_desired_velocity_{5.0};

  /// 参考期望速度，由 ConstructReferenceLane 根据曲率限制计算
  decimal_t reference_desired_velocity_{5.0};

  /// 自动驾驶等级：2 = L2（HMI 直控），3 = L3（MPDM 决策）
  int autonomous_level_{3};

  /// 前向仿真时间步长 [s]，默认 0.4
  decimal_t sim_resolution_{0.4};

  /// 前向仿真时间窗口 [s]，默认 4.0
  decimal_t sim_horizon_{4.0};

  /// 激进等级，影响 IDM 模型参数和代价权重
  int aggressive_level_{3};

  /// IDM 跟驰模型参数集
  planning::OnLaneForwardSimulation::Param sim_param_;

  /// 是否在导航中使用仿真状态
  bool use_sim_state_ = true;

  /// HMI 锁定标志：true 时 MPDM 优先选择 HMI 指定的行为
  bool lock_to_hmi_ = false;

  /// HMI 指定的横向行为，默认车道保持
  LateralBehavior hmi_behavior_ = LateralBehavior::kLaneKeeping;

  /// 自车当前所在的车道 ID
  int ego_lane_id_{kInvalidLaneId};

  /// 自车 ID
  int ego_id_;

  /// 从当前车道出发，左变道可达的潜在车道 ID 列表
  std::vector<int> potential_lcl_lane_ids_;

  /// 从当前车道出发，右变道可达的潜在车道 ID 列表
  std::vector<int> potential_lcr_lane_ids_;

  /// 从当前车道出发，车道保持可达的潜在车道 ID 列表（子车道）
  std::vector<int> potential_lk_lane_ids_;

  // 调试/可视化用数据
  /// 所有候选行为的前向仿真轨迹集合
  vec_E<vec_E<common::Vehicle>> forward_trajs_;

  /// 所有候选行为列表
  std::vector<LateralBehavior> forward_behaviors_;

  /// 所有候选行为对应的周围车辆轨迹集合
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_;
};

}  // namespace planning

#endif  // _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_BEHAVIOR_PLANNER_H_
