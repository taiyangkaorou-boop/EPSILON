/**
 * @file eudm_planner.h
 * @author EPSILON Autonomous Driving Team
 * @brief EUDM（高效不确定性感知决策）规划器核心头文件
 * @version 0.1
 * @date 2019-07-07
 * @copyright Copyright (c) 2019
 *
 * 本文件定义了EudmPlanner类，它是EUDM行为规划系统的核心引擎。
 *
 * ## EUDM算法概述
 *
 * EUDM（Efficient Uncertainty-aware Decision Making，高效不确定性感知决策）
 * 是EPSILON自动驾驶系统中比MPDM（多策略决策）更高级的行为规划方案。
 * 其核心思想是：
 *
 * 1. **动作序列生成**：通过DCP-Tree（离散-连续规划树）生成候选动作序列。
 *    每层包含横向动作（保持/左变道/右变道）× 纵向动作（维持/加速/减速）= 12种组合。
 *
 * 2. **周围车辆行为不确定性建模**：与MPDM不同，EUDM显式地建模周围车辆的
 *    行为不确定性。通过概率分布（ProbDistOfLatBehaviors）描述周围车辆
 *    的横向行为概率（例如80%保持车道、20%左变道），并基于这些分布模拟
 *    多种"假设场景"（what-if scenarios）。
 *
 * 3. **代价函数三要素**：
 *    - 效率代价（Efficiency）：自车速度相对于期望速度的偏差，考虑前车影响
 *    - 安全代价（Safety）：基于RSS（责任敏感安全）检查 + 占位碰撞风险
 *    - 导航代价（Navigation）：偏好保持在规划路径上的程度
 *
 * 4. **多线程并行仿真**：使用多线程并行评估所有候选动作序列的前向仿真。
 *
 * ## 工作流程
 * 1. RunOnce() -> 获取自车状态和周围语义车辆
 * 2. RunEudm() -> 并行仿真所有动作序列
 * 3. 评估每个序列的代价，选择最优者
 * 4. 输出优胜动作序列及其对应的行为
 *
 * ## 涉及的关键数据结构
 * - ForwardSimEgoAgent: 前向仿真中自车的状态封装
 * - ForwardSimAgent: 前向仿真中周围车辆的状态封装
 * - CostStructure: 三层代价结构（效率/安全/导航）
 *
 * @see dcp_tree.h DCP-Tree动作序列生成
 * @see eudm_manager.h EUDM管理器（负责完整的规划生命周期）
 */
#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_

#include <algorithm>
#include <memory>
#include <string>
#include <thread>

#include "common/basics/basics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/mobil/mobil_model.h"
#include "common/state/state.h"
#include "eudm_config.pb.h"
#include "eudm_planner/dcp_tree.h"
#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/map_interface.h"
#include "forward_simulator/onlane_forward_simulation.h"

namespace planning {

/// @class EudmPlanner
/// @brief EUDM（高效不确定性感知决策）行为规划器的核心类
///
/// 负责执行完整的前向仿真循环：生成候选动作序列、多线程并行仿真、
/// 代价评估和优胜者选择。EUDM相较于MPDM的核心改进在于显式建模
/// 周围车辆行为的不确定性，并通过概率分布进行多场景仿真。
///
/// 继承自Planner基类，实现了自动驾驶系统的标准规划器接口。
class EudmPlanner : public Planner {
 public:
  /// @name 类型别名定义
  /// @{
  using State = common::State;                        ///< 车辆状态（位置/速度/加速度/曲率等）
  using Lane = common::Lane;                          ///< 车道数据结构
  using Behavior = common::SemanticBehavior;           ///< 语义行为（横向+纵向行为组合）
  using LateralBehavior = common::LateralBehavior;    ///< 横向行为枚举（保持/左变道/右变道）
  using LongitudinalBehavior = common::LongitudinalBehavior;  ///< 纵向行为枚举（维持/加速/减速）
  using DcpAction = DcpTree::DcpAction;               ///< DCP-Tree动作（横向×纵向的组合）
  using DcpLonAction = DcpTree::DcpLonAction;          ///< DCP纵向动作类型
  using DcpLatAction = DcpTree::DcpLatAction;          ///< DCP横向动作类型
  using Cfg = planning::eudm::Config;                  ///< Protobuf配置类型
  using LaneChangeInfo = planning::eudm::LaneChangeInfo;  ///< 变道信息结构
  /// @}

  /// @enum LatSimMode
  /// @brief 横向仿真模式枚举
  ///
  /// 描述一个完整的横向动作序列属于哪种模式：
  /// - kAlwaysLaneKeep: 全程保持车道（如 KKKKK）
  /// - kKeepThenChange: 先保持后变道（如 KKKLL）
  /// - kAlwaysLaneChange: 立即变道（如 LLLLL）
  /// - kChangeThenCancel: 变道后取消（如 LLKKK，即变道失败回退）
  enum class LatSimMode {
    kAlwaysLaneKeep = 0,    ///< 全程保持车道，不做任何变道操作
    kKeepThenChange,         ///< 延迟变道：先保持数个时间层后再变道
    kAlwaysLaneChange,       ///< 立即变道：从第一个时间层就开始变道操作
    kChangeThenCancel         ///< 变道后取消：变道完成后因某种原因取消，回退到原行为
  };

  // * 在前向仿真中，自车智能体和周围车辆智能体具有不同的属性，
  // * 因此设计了两种不同的结构体

  /// @struct ForwardSimEgoAgent
  /// @brief 前向仿真中自车智能体的完整状态封装
  ///
  /// 该结构体分为四个更新层级：
  /// - **常量级别**：横向范围，在仿真期间不变
  /// - **场景级别**：仿真参数、横向序列模式，在每轮仿真开始时设置
  /// - **层级级别**：当前动作对应的纵向/横向行为、关联车道和坐标系，每层更新
  /// - **步长级别**：车辆状态，每仿真步更新
  struct ForwardSimEgoAgent {
    // * 常量级别：仿真期间不变的参数
    decimal_t lat_range;     ///< 自车横向搜索范围，用于查找前车/后车

    // * 场景级别：每个动作序列开始时设置
    OnLaneForwardSimulation::Param sim_param;  ///< 前向仿真参数（IDM参数、转向控制、运动学约束）

    LatSimMode seq_lat_mode;                  ///< 该序列的横向动作模式（全程保持/延迟变道/立即变道/变道取消）
    common::LateralBehavior lat_behavior_longterm{LateralBehavior::kUndefined};  ///< 长期横向行为倾向
    common::LateralBehavior seq_lat_behavior;  ///< 序列整体的横向行为（保持/左变道/右变道）
    bool is_cancel_behavior;                   ///< 是否为变道后取消的行为序列
    decimal_t operation_at_seconds{0.0};       ///< 从序列开始到执行横向操作的时间间隔

    // * 层级级别：每个动作层（时间层）开始时设置
    common::LongitudinalBehavior lon_behavior{LongitudinalBehavior::kMaintain};  ///< 当前层面的纵向行为
    common::LateralBehavior lat_behavior{LateralBehavior::kUndefined};          ///< 当前层面的横向行为

    common::Lane current_lane;                ///< 自车当前所在车道
    common::StateTransformer current_stf;     ///< 当前车道的Frenet-SL坐标变换器
    common::Lane target_lane;                 ///< 目标车道（变道时为目标车道，保持时为当前车道）
    common::StateTransformer target_stf;      ///< 目标车道的Frenet-SL坐标变换器
    common::Lane longterm_lane;               ///< 长期目标车道（序列结束后的期望车道）
    common::StateTransformer longterm_stf;    ///< 长期目标车道的Frenet-SL坐标变换器

    /// @brief 目标间隙的前后车辆ID
    /// target_gap_ids(0) = 前车ID（-1表示不存在），target_gap_ids(1) = 后车ID（-1表示不存在）
    /// 间隙ID在每层固定，但间隙的具体车辆在每步可能变化
    Vec2i target_gap_ids;

    // * 步长级别：每个仿真步更新
    common::Vehicle vehicle;                  ///< 当前仿真步的车辆状态
  };

  /// @struct ForwardSimAgent
  /// @brief 前向仿真中单个周围车辆智能体的状态封装
  ///
  /// 包含周围车辆的ID、运动状态、仿真参数和关键的**行为不确定性建模**
  /// ——横向行为概率分布（lat_probs），这是EUDM区别于MPDM的核心特征。
  struct ForwardSimAgent {
    int id = kInvalidAgentId;               ///< 车辆唯一标识符
    common::Vehicle vehicle;                 ///< 当前车辆状态

    // * 纵向仿真参数
    OnLaneForwardSimulation::Param sim_param;  ///< IDM等纵向控制器参数

    // * 横向行为不确定性建模（EUDM核心特征）
    common::ProbDistOfLatBehaviors lat_probs;  ///< 横向行为概率分布，例如{保持:80%, 左变道:20%}
    common::LateralBehavior lat_behavior{LateralBehavior::kUndefined};  ///< 当前采样到的横向行为

    common::Lane lane;                       ///< 车辆所在车道
    common::StateTransformer stf;            ///< 车辆所在车道的Frenet-SL坐标变换器

    // * 其他参数
    decimal_t lat_range;                     ///< 横向搜索范围
  };

  /// @struct ForwardSimAgentSet
  /// @brief 周围车辆前向仿真智能体的集合
  ///
  /// 使用unordered_map存储，key为车辆ID，value为ForwardSimAgent。
  /// 用于一次前向仿真中所有周围车辆的批量管理。
  struct ForwardSimAgentSet {
    std::unordered_map<int, ForwardSimAgent> forward_sim_agents;  ///< ID到智能体的映射
  };

  /// @struct EfficiencyCost
  /// @brief 效率代价结构
  ///
  /// 效率代价衡量自车速度与期望速度的接近程度，以及前车对速度的影响。
  /// - ego_to_desired_vel: 自车速度到期望速度的差距
  /// - leading_to_desired_vel: 前车速度对自车期望速度的阻碍程度
  struct EfficiencyCost {
    decimal_t ego_to_desired_vel = 0.0;       ///< 自车与期望速度之间的代价分量
    decimal_t leading_to_desired_vel = 0.0;   ///< 因前车阻挡产生的速度代价分量
    /// @brief 效率代价平均值
    decimal_t ave() const {
      return (ego_to_desired_vel + leading_to_desired_vel) / 2.0;
    }
  };

  /// @struct SafetyCost
  /// @brief 安全代价结构
  ///
  /// 安全代价由两部分组成：
  /// - rss: 基于RSS（责任敏感安全）检查的安全代价
  /// - occu_lane: 基于占位/碰撞风险的额外安全代价
  struct SafetyCost {
    decimal_t rss = 0.0;       ///< RSS安全检查产生的不安全代价
    decimal_t occu_lane = 0.0; ///< 占位/碰撞风险代价（如变道到被禁止的车道）
    decimal_t ave() const { return (rss + occu_lane) / 2.0; }
  };

  /// @struct NavigationCost
  /// @brief 导航代价结构
  ///
  /// 衡量行为序列与导航意图（规划路径）的一致性。
  /// - 变道操作本身有固定代价（优先保持当前车道）
  /// - 但当系统推荐变道时，变道代价可变为负值（奖励）
  /// - 取消变道操作有额外惩罚
  struct NavigationCost {
    decimal_t lane_change_preference = 0.0;    ///< 变道偏好代价（正值=惩罚变道，负值=鼓励变道）
    decimal_t ave() const { return lane_change_preference; }
  };

  /// @struct CostStructure
  /// @brief 完整的代价结构体，包含效率/安全/导航三维代价
  ///
  /// 每个动作层会产生一个CostStructure，用于记录该层的仿真结果代价。
  /// - valid_sample_index_ub: 关联到微观动作的有效采样索引上界
  /// - weight: 本层的权重（默认等于持续时间，通过折扣因子可以衰减后续层）
  /// - 总代价 ≈ (efficiency.ave() + safety.ave() + navigation.ave()) * weight
  struct CostStructure {
    // * 使用该索引将代价关联到微观动作
    int valid_sample_index_ub;         ///< 有效采样索引的上界，关联到具体的前向轨迹点

    // * 效率
    EfficiencyCost efficiency;          ///< 效率代价
    // * 安全
    SafetyCost safety;                  ///< 安全代价
    // * 导航
    NavigationCost navigation;          ///< 导航代价
    decimal_t weight = 1.0;            ///< 本代价层的权重（通常为持续时间×折扣因子）

    /// @brief 该层综合代价的加权平均值
    decimal_t ave() const {
      return (efficiency.ave() + safety.ave() + navigation.ave()) * weight;
    }

    /// @brief 流输出操作符，用于日志打印代价详情
    friend std::ostream& operator<<(std::ostream& os,
                                    const CostStructure& cost) {
      os << std::fixed;
      os << std::fixed;
      os << std::setprecision(3);
      os << "(efficiency: "
         << "ego (" << cost.efficiency.ego_to_desired_vel << ") + leading ("
         << cost.efficiency.leading_to_desired_vel << "), safety: ("
         << cost.safety.rss << "," << cost.safety.occu_lane
         << "), navigation: " << cost.navigation.lane_change_preference << ")";
      return os;
    }
  };

  /// @name Planner标准接口
  /// @{

  /// @brief 返回规划器名称
  std::string Name() override;

  /// @brief 初始化规划器：读取配置文件，创建DCP-Tree，初始化RSS和仿真参数
  /// @param config 配置文件路径
  ErrorType Init(const std::string config) override;

  /// @brief 执行一次EUDM规划循环
  ///
  /// 核心流程：
  /// 1. 从地图接口获取自车状态和周围语义车辆
  /// 2. 预筛除不合理的动作序列（如左变道后立即右变道）
  /// 3. 调用RunEudm()执行多线程并行仿真
  /// 4. 输出优胜动作序列和对应的代价
  ErrorType RunOnce() override;
  /// @}

  /// @brief 设置地图接口指针（依赖注入）
  void set_map_interface(EudmPlannerMapItf* itf);

  /// @brief 设置自车期望速度（由上游管理器或人机交互指定）
  void set_desired_velocity(const decimal_t desired_vel);

  /// @brief 设置变道信息（禁止变道标志、推荐变道标志等）
  void set_lane_change_info(const LaneChangeInfo& lc_info);

  /// @brief 执行EUDM核心仿真与评估逻辑
  ///
  /// 详细流程：
  /// 1. 从地图接口获取周围语义车辆（包含行为概率分布）
  /// 2. 将语义车辆转换为前向仿真智能体（ForwardSimAgent）
  /// 3. 获取DCP-Tree生成的所有候选动作序列
  /// 4. 准备多线程容器（sim_res_, risky_res_等结果数组）
  /// 5. 为每个动作序列创建独立线程，调用SimulateActionSequence进行前向仿真
  /// 6. 等待所有线程完成
  /// 7. 调用EvaluateMultiThreadSimResults评估并选择最优动作序列
  ErrorType RunEudm();

  /// @brief 获取当前的语义行为（横向+纵向）
  Behavior behavior() const;

  /// @brief 获取优胜动作序列
  std::vector<DcpAction> winner_action_seq() const {
    return winner_action_seq_;
  }

  /// @brief 获取当前期望速度
  decimal_t desired_velocity() const;

  /// @brief 获取所有前向仿真轨迹
  vec_E<vec_E<common::Vehicle>> forward_trajs() const { return forward_trajs_; }

  /// @brief 获取优胜序列的索引ID
  int winner_id() const;

  /// @brief 获取本次规划的时间开销（毫秒）
  decimal_t time_cost() const;

  /// @brief 获取每个动作序列的仿真成功/失败标志
  std::vector<bool> sim_res() const {
    std::vector<bool> ret;
    for (auto& r : sim_res_) {
      if (r == 0) {
        ret.push_back(false);
      } else {
        ret.push_back(true);
      }
    }
    return ret;
  }

  /// @brief 获取每个动作序列是否有风险（RSS不安全）
  std::vector<bool> risky_res() const {
    std::vector<bool> ret;
    for (auto& r : risky_res_) {
      if (r == 0) {
        ret.push_back(false);
      } else {
        ret.push_back(true);
      }
    }
    return ret;
  }

  /// @brief 获取每个序列的仿真信息字符串（包含失败原因等调试信息）
  std::vector<std::string> sim_info() const { return sim_info_; }

  /// @brief 获取每个序列的最终代价
  std::vector<decimal_t> final_cost() const { return final_cost_; }

  /// @brief 获取每个序列每层（每个动作层）的过程代价
  std::vector<std::vector<CostStructure>> progress_cost() const {
    return progress_cost_;
  }

  /// @brief 获取每个序列的尾部代价（当前未使用，预留字段）
  std::vector<CostStructure> tail_cost() const { return tail_cost_; }

  /// @brief 获取每个序列每层的横向行为
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors() const {
    return forward_lat_behaviors_;
  }

  /// @brief 获取每个序列每层的纵向行为
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors() const {
    return forward_lon_behaviors_;
  }

  /// @brief 获取每个序列中周围车辆的仿真轨迹
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs()
      const {
    return surround_trajs_;
  }

  /// @brief 获取规划时的自车状态
  common::State plan_state() { return ego_vehicle_.state(); }

  /// @brief 获取DCP-Tree生成的所有动作序列（动作脚本）
  std::vector<std::vector<DcpAction>> action_script() {
    return dcp_tree_ptr_->action_script();
  }

  /// @brief 获取配置引用
  const Cfg& cfg() const { return cfg_; }

  /// @brief 获取地图接口指针
  EudmPlannerMapItf* map_itf() const;

  /// @brief 更新DCP-Tree的当前进行中的动作，重新生成动作序列
  /// @param ongoing_action 当前正在执行的动作
  void UpdateDcpTree(const DcpAction& ongoing_action);

  /// @brief 分析动作序列的类型
  ///
  /// 识别动作序列属于哪种模式：
  /// - 纯保持车道
  /// - 变道（左/右），并找出变道操作的起始时间
  /// - 变道后取消
  ///
  /// @param action_seq 输入的动作序列
  /// @param operation_at_seconds [out] 从序列开始到执行横向操作的时间
  /// @param lat_behavior [out] 该序列的横向行为类型
  /// @param is_cancel_operation [out] 是否为变道后取消的操作
  ErrorType ClassifyActionSeq(const std::vector<DcpAction>& action_seq,
                              decimal_t* operation_at_seconds,
                              common::LateralBehavior* lat_behavior,
                              bool* is_cancel_operation) const;

 private:
  /// @brief 从Protobuf文本配置文件读取EUDM规划器配置
  /// @param config_path 配置文件路径
  ErrorType ReadConfig(const std::string config_path);

  /// @brief 从Protobuf配置中提取前向仿真参数
  /// @param cfg 仿真配置详情（包含IDM参数、横向控制参数、运动学约束）
  /// @param sim_param [out] 输出仿真参数结构体
  ErrorType GetSimParam(const planning::eudm::ForwardSimDetail& cfg,
                        OnLaneForwardSimulation::Param* sim_param);

  /// @brief 获取给定车道执行指定横向行为后可能到达的车道ID列表
  ///
  /// 例如从车道A左变道，返回车道A左侧相邻车道及其子车道的ID列表。
  ///
  /// @param source_lane_id 源车道ID
  /// @param beh 横向行为（保持/左变道/右变道）
  /// @param candidate_lane_ids [out] 候选车道ID列表
  ErrorType GetPotentialLaneIds(const int source_lane_id,
                                const LateralBehavior& beh,
                                std::vector<int>* candidate_lane_ids) const;

  /// @brief 更新自车车道ID
  ErrorType UpdateEgoLaneId(const int new_ego_lane_id);

  /// @brief 根据自车位置对应的车道ID判断横向行为类型
  ///
  /// 通过查看该车道ID属于保持/左变道/右变道的哪个候选列表中判断行为。
  ErrorType JudgeBehaviorByLaneId(const int ego_lane_id_by_pos,
                                  LateralBehavior* behavior_by_lane_id);

  /// @brief 根据车道ID推断的行为更新自车行为
  ErrorType UpdateEgoBehavior(const LateralBehavior& behavior_by_lane_id);

  /// @brief 将DcpAction（离散动作）转换为横向和纵向行为枚举
  /// @param action DCP动作
  /// @param lat [out] 横向行为
  /// @param lon [out] 纵向行为
  ErrorType TranslateDcpActionToLonLatBehavior(const DcpAction& action,
                                               LateralBehavior* lat,
                                               LongitudinalBehavior* lon) const;

  /// @brief 将语义车辆集合转换为前向仿真智能体集合
  ///
  /// 关键操作：
  /// 1. 提取每个语义车辆的运动状态和位置
  /// 2. **保留横向行为概率分布（lat_probs）**——这是EUDM建模行为不确定性的核心
  /// 3. 设置IDM仿真参数（如果车辆减速，使用当前减速度估计期望速度）
  ///
  /// @param surrounding_semantic_vehicles 周围语义车辆集合
  /// @param forward_sim_agents [out] 前向仿真智能体集合
  ErrorType GetSurroundingForwardSimAgents(
      const common::SemanticVehicleSet& surrounding_semantic_vehicles,
      ForwardSimAgentSet* forward_sim_agents) const;

  // * 仿真控制循环

  /// @brief 仿真单个动作序列（作为多线程的入口函数）
  ///
  /// 每个动作序列在一个独立线程中执行。内部调用SimulateScenario进行逐层仿真。
  /// 仿真结果写入多线程容器（sim_res_[seq_id]等）。
  ///
  /// @param ego_vehicle 自车初始状态
  /// @param surrounding_fsagents 周围车辆仿真智能体
  /// @param action_seq 待仿真的动作序列
  /// @param seq_id 该动作序列的索引（用于写入对应的结果容器）
  ErrorType SimulateActionSequence(
      const common::Vehicle& ego_vehicle,
      const ForwardSimAgentSet& surrounding_fsagents,
      const std::vector<DcpAction>& action_seq, const int& seq_id);

  /// @brief 仿真一个场景（子序列），逐层执行每个动作
  ///
  /// 这是前向仿真的核心函数。执行以下步骤：
  /// 1. 初始化自车和周围车辆的轨迹容器
  /// 2. 设置场景级别的仿真配置（纵向参数、序列模式等）
  /// 3. **逐层循环**：对动作序列中的每个动作：
  ///    a. UpdateSimSetupForLayer: 更新该层的Frenet坐标系、车道、间隙车辆
  ///    b. SimulateSingleAction: 执行该层的多步前向仿真
  ///    c. StrictSafetyCheck: 严格安全碰撞检查（几何碰撞）
  ///    d. CostFunction: 计算该层的代价（效率/安全/导航）
  ///    e. 检查横向动作是否完成，若完成则更新后续动作序列
  /// 4. 合并所有层的结果到输出容器
  ///
  /// @param ego_vehicle 自车初始车辆状态
  /// @param surrounding_fsagents 周围车辆仿真智能体
  /// @param action_seq 动作序列
  /// @param seq_id 主序列索引
  /// @param sub_seq_id 子序列索引（当前固定为0）
  /// @param sub_sim_res [out] 子序列仿真成功标志
  /// @param sub_risky_res [out] 子序列风险标志
  /// @param sub_sim_info [out] 子序列仿真信息字符串
  /// @param sub_progress_cost [out] 子序列每层的过程代价
  /// @param sub_tail_cost [out] 子序列尾部代价
  /// @param sub_forward_trajs [out] 子序列前向轨迹
  /// @param sub_forward_lat_behaviors [out] 子序列每层横向行为
  /// @param sub_forward_lon_behaviors [out] 子序列每层纵向行为
  /// @param sub_surround_trajs [out] 子序列周围车辆轨迹
  ErrorType SimulateScenario(
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
          sub_surround_trajs);

  /// @brief 仿真单个动作（一个时间层），执行多步前向仿真
  ///
  /// 对单个动作时间层进行N步仿真（步长由sim_time_resolution决定）。
  /// 每步依次仿真自车和所有周围车辆。
  ///
  /// @param action 当前的DCP动作
  /// @param ego_fsagent_this_layer 当前层的自车仿真智能体
  /// @param surrounding_fsagents_this_layer 当前层的周围车辆仿真智能体
  /// @param ego_traj_multisteps [out] 自车在该层的多步轨迹
  /// @param surround_trajs_multisteps [out] 周围车辆在该层的多步轨迹
  ErrorType SimulateSingleAction(
      const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent_this_layer,
      const ForwardSimAgentSet& surrounding_fsagents_this_layer,
      vec_E<common::Vehicle>* ego_traj_multisteps,
      std::unordered_map<int, vec_E<common::Vehicle>>*
          surround_trajs_multisteps);

  // * 评估函数

  /// @brief 代价函数：计算一个动作层的三维代价（效率/安全/导航）
  ///
  /// 代价函数的核心逻辑：
  ///
  /// **效率代价（Efficiency）**：
  /// - 自车速度低于期望速度时，惩罚 = unit_cost * |v_ego - v_desired|
  /// - 自车速度高于期望+容忍阈值时，惩罚 = unit_cost * |v_ego - v_desired - threshold|
  /// - 前车影响：当前车导致自车无法达到期望速度时，按残留距离比例惩罚
  ///
  /// **安全代价（Safety）**：
  /// - RSS检查：对每对（自车，周围车）执行RSS安全检查
  ///   - TooFast: 自车太快，惩罚与超速量指数相关
  ///   - TooSlow: 自车太慢，惩罚与落后量指数相关
  /// - 占位代价：如果自车选择了被禁止的变道方向，施加额外惩罚
  ///
  /// **导航代价（Navigation）**：
  /// - 变道操作有固定代价（优先保持当前车道）
  /// - 若系统推荐变道，变道代价为负（奖励）
  /// - 取消变道有额外惩罚
  /// - 延迟执行推荐变道有额外惩罚
  ///
  /// @param action 当前动作
  /// @param ego_fsagent 自车仿真智能体
  /// @param other_fsagent 周围车辆仿真智能体集合
  /// @param ego_traj 自车本层轨迹
  /// @param surround_trajs 周围车辆本层轨迹
  /// @param verbose 是否详细输出
  /// @param cost [out] 输出代价结构
  /// @param is_risky [out] 是否存在RSS风险
  /// @param risky_ids [out] 产生风险的车辆ID集合
  ErrorType CostFunction(
      const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent,
      const ForwardSimAgentSet& other_fsagent,
      const vec_E<common::Vehicle>& ego_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
      bool verbose, CostStructure* cost, bool* is_risky,
      std::set<int>* risky_ids);

  /// @brief 严格安全检查：对自车和周围车辆的每对同时刻状态做几何碰撞检测
  ///
  /// 使用地图接口的碰撞检测功能，先对车辆模型按照安全膨胀参数进行膨胀，
  /// 然后逐对逐时刻检查是否碰撞。
  ///
  /// @param ego_traj 自车轨迹
  /// @param surround_trajs 周围车辆轨迹
  /// @param is_safe [out] 是否安全（无碰撞）
  /// @param collided_id [out] 若碰撞，对方的车辆ID
  ErrorType StrictSafetyCheck(
      const vec_E<common::Vehicle>& ego_traj,
      const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
      bool* is_safe, int* collided_id);

  /// @brief 评估两辆车轨迹之间的RSS安全状态
  ///
  /// 使用RssChecker对轨迹A和轨迹B的每对状态进行RSS安全检查。
  /// 如果违反RSS安全条件，根据违规类型（太快/太慢）计算安全代价。
  ///
  /// @param traj_a 车辆A的轨迹
  /// @param traj_b 车辆B的轨迹
  /// @param cost [out] RSS安全代价
  /// @param is_rss_safe [out] 是否RSS安全
  /// @param risky_id [out] 若存在风险，对方的ID
  ErrorType EvaluateSafetyStatus(const vec_E<common::Vehicle>& traj_a,
                                 const vec_E<common::Vehicle>& traj_b,
                                 decimal_t* cost, bool* is_rss_safe,
                                 int* risky_id);

  /// @brief 评估单个策略轨迹序列的总代价
  ///
  /// 将过程代价（progress_cost）和尾部代价（tail_cost）累加。
  ///
  /// @param progress_cost 每层的过程代价列表
  /// @param tail_cost 尾部代价
  /// @param action_seq 动作序列
  /// @param score [out] 输出的总代价分数
  ErrorType EvaluateSinglePolicyTrajs(
      const std::vector<CostStructure>& progress_cost,
      const CostStructure& tail_cost, const std::vector<DcpAction>& action_seq,
      decimal_t* score);

  /// @brief 评估所有线程的仿真结果，选择最优动作序列
  ///
  /// 对所有成功仿真的动作序列，调用EvaluateSinglePolicyTrajs计算总代价，
  /// 选择代价最小的序列作为优胜者。
  ///
  /// @param winner_id [out] 优胜序列的索引
  /// @param winner_cost [out] 优胜序列的代价
  ErrorType EvaluateMultiThreadSimResults(int* winner_id,
                                          decimal_t* winner_cost);

  // * 仿真辅助函数

  /// @brief 为整个场景（动作序列）设置仿真配置
  ///
  /// 主要操作：
  /// 1. 调用ClassifyActionSeq分析序列类型
  /// 2. 根据序列类型设置LatSimMode（全程保持/延迟变道/立即变道/变道取消）
  /// 3. **根据第二个动作的纵向类型设置IDM期望速度**（加速+gap、减速-gap、维持不变）
  /// 4. 调整IDM的侵略性参数（纵向侵略比影响最小间距和期望时距）
  ///
  /// @param action_seq 动作序列
  /// @param ego_fsagent [in/out] 自车仿真智能体
  ErrorType UpdateSimSetupForScenario(const std::vector<DcpAction>& action_seq,
                                      ForwardSimEgoAgent* ego_fsagent) const;

  /// @brief 为当前动作层设置仿真配置
  ///
  /// 主要操作：
  /// 1. 转换当前动作为横向/纵向行为
  /// 2. 获取当前车道、目标车道、长期车道的Frenet坐标系
  /// 3. **变道场景**：查找目标车道上的前后间隙车辆
  /// 4. **启用时**：执行严格RSS预检查，筛除明显不合理的变道动作
  ///
  /// @param action 当前动作
  /// @param other_fsagent 周围车辆智能体
  /// @param ego_fsagent [in/out] 自车仿真智能体
  ErrorType UpdateSimSetupForLayer(const DcpAction& action,
                                   const ForwardSimAgentSet& other_fsagent,
                                   ForwardSimEgoAgent* ego_fsagent) const;

  /// @brief 根据动作更新自车的横向和纵向行为
  ErrorType UpdateEgoBehaviorsUsingAction(
      const DcpAction& action, ForwardSimEgoAgent* ego_fsagent) const;

  /// @brief 检查横向动作（变道）是否已经完成
  ///
  /// 通过比较自车当前位置对应的车道ID和动作参考车道ID来判断。
  /// 若当前车道ID与目标车道方向一致，则认为变道完成。
  ///
  /// @param cur_state 自车当前状态
  /// @param action_ref_lane_id 动作开始时的参考车道ID
  /// @param lat_behavior 横向行为方向
  /// @param current_lane_id [out] 当前位置对应的车道ID
  /// @return true表示横向动作已完成
  bool CheckIfLateralActionFinished(const common::State& cur_state,
                                    const int& action_ref_lane_id,
                                    const LateralBehavior& lat_behavior,
                                    int* current_lane_id) const;

  /// @brief 在横向动作完成后更新后续动作序列
  ///
  /// 当变道完成时，需要修改后续动作序列中的横向动作：
  /// - LLLLL（全变道） -> LLKKK（后续改保持），因为已经完成变道
  /// - LLKKK -> LLRRR（后续改反向变道），因为正常变道完成后习惯会回正
  /// - RRRRR -> RRKKK 同理
  /// - RRKKK -> RRLLL 同理
  ///
  /// @param cur_idx 当前已完成动作的索引
  /// @param action_seq [in/out] 待更新的动作序列
  ErrorType UpdateLateralActionSequence(
      const int cur_idx, std::vector<DcpAction>* action_seq) const;

  /// @brief 准备多线程容器：按动作序列数量分配结果存储空间
  ///
  /// 将所有结果数组（sim_res_, risky_res_, final_cost_等）resize到
  /// n_sequence的大小，确保多线程写入不冲突。
  ///
  /// @param n_sequence 动作序列数量
  ErrorType PrepareMultiThreadContainers(const int n_sequence);

  /// @brief 将动作的持续时间分解为多个仿真时间步长
  ///
  /// 例如action.t = 2.3s, step_resolution = 0.2s，
  /// 则返回 [0.1, 0.2, 0.2, ..., 0.2]（首个步长吸收余数）
  ///
  /// @param action DCP动作
  /// @param dt_steps [out] 时间步长列表
  ErrorType GetSimTimeSteps(const DcpAction& action,
                            std::vector<decimal_t>* dt_steps) const;

  /// @brief 自车前向仿真单步（一个时间步长）
  ///
  /// 根据自车的横向行为选择不同的仿真模式：
  /// - **车道保持（LaneKeeping）**：使用PropagateOnceAdvancedLK，
  ///   仅考虑自车所在车道的前车进行IDM纵向控制
  /// - **变道（LaneChangeLeft/Right）**：使用PropagateOnceAdvancedLC，
  ///   同时考虑当前车道前车、目标车道前车/后车的间隙约束，
  ///   以及可能的后车避让策略（RSS检查+虚拟屏障）
  ///
  /// @param ego_fsagent 自车仿真智能体
  /// @param all_sim_vehicles 所有仿真车辆集合
  /// @param sim_time_step 仿真时间步长
  /// @param state_out [out] 仿真后的状态
  ErrorType EgoAgentForwardSim(const ForwardSimEgoAgent& ego_fsagent,
                               const common::VehicleSet& all_sim_vehicles,
                               const decimal_t& sim_time_step,
                               common::State* state_out) const;

  /// @brief 周围车辆前向仿真单步
  ///
  /// 使用IDM模型沿车道进行纵向仿真。查找前车作为IDM输入。
  /// 注意：当前版本周围车辆的横向行为固定为车道保持（忽略lat_probs采样），
  /// 未来版本可能实现基于概率分布的随机采样。
  ///
  /// @param fsagent 周围车辆仿真智能体
  /// @param all_sim_vehicles 所有仿真车辆集合
  /// @param sim_time_step 仿真时间步长
  /// @param state_out [out] 仿真后的状态
  ErrorType SurroundingAgentForwardSim(
      const ForwardSimAgent& fsagent,
      const common::VehicleSet& all_sim_vehicles,
      const decimal_t& sim_time_step, common::State* state_out) const;

  // * 地图接口
  EudmPlannerMapItf* map_itf_{nullptr};       ///< 地图接口（纯虚接口，支持依赖注入）

  // * 动作
  DcpTree* dcp_tree_ptr_;                      ///< DCP-Tree指针，负责生成候选动作序列

  // * 配置与设置
  Cfg cfg_;                                    ///< EUDM Protobuf配置
  LaneChangeInfo lc_info_;                     ///< 当前变道信息（禁止/推荐标志）
  decimal_t desired_velocity_{5.0};            ///< 自车期望速度（m/s）
  decimal_t sim_time_total_ = 0.0;             ///< 总仿真时长（所有层时间之和）
  std::set<int> pre_deleted_seq_ids_;          ///< 预筛除的动作序列ID（逻辑不一致的序列）
  int ego_lane_id_{kInvalidLaneId};            ///< 自车当前车道ID
  std::vector<int> potential_lcl_lane_ids_;    ///< 左变道可能到达的车道ID列表
  std::vector<int> potential_lcr_lane_ids_;    ///< 右变道可能到达的车道ID列表
  std::vector<int> potential_lk_lane_ids_;     ///< 保持车道可能到达的车道ID列表

  // * RSS安全相关
  common::Lane rss_lane_;                      ///< RSS检查使用的参考车道
  common::StateTransformer rss_stf_;           ///< RSS参考车道的Frenet变换器
  common::RssChecker::RssConfig rss_config_;   ///< 标准RSS安全配置
  common::RssChecker::RssConfig rss_config_strict_as_front_;  ///< 作为前车时的严格RSS配置
  common::RssChecker::RssConfig rss_config_strict_as_rear_;   ///< 作为后车时的严格RSS配置

  // * 仿真参数
  OnLaneForwardSimulation::Param ego_sim_param_;   ///< 自车仿真参数（IDM + 横向控制 + 运动学）
  OnLaneForwardSimulation::Param agent_sim_param_; ///< 周围车辆仿真参数

  // * 运行时状态
  decimal_t time_stamp_;                       ///< 当前规划的时间戳
  int ego_id_;                                  ///< 自车ID
  common::Vehicle ego_vehicle_;                 ///< 自车完整状态

  // * 仿真结果

  int winner_id_ = 0;                                    ///< 优胜动作序列的索引
  decimal_t winner_score_ = 0.0;                         ///< 优胜动作序列的代价分数
  std::vector<DcpAction> winner_action_seq_;             ///< 优胜动作序列
  std::vector<int> sim_res_;                             ///< 每个序列的仿真成功标志（1=成功,0=失败）
  std::vector<int> risky_res_;                           ///< 每个序列的风险标志（1=有风险,0=安全）
  std::vector<std::string> sim_info_;                    ///< 每个序列的仿真信息（失败原因等）
  std::vector<decimal_t> final_cost_;                    ///< 每个序列的最终总代价
  std::vector<std::vector<CostStructure>> progress_cost_; ///< 每个序列每层的代价结构
  std::vector<CostStructure> tail_cost_;                 ///< 每个序列的尾部代价
  vec_E<vec_E<common::Vehicle>> forward_trajs_;          ///< 每个序列的自车前向轨迹
  std::vector<std::vector<LateralBehavior>> forward_lat_behaviors_;  ///< 每个序列每层横向行为
  std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors_; ///< 每个序列每层纵向行为
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs_; ///< 每个序列周围车辆轨迹
  decimal_t time_cost_ = 0.0;                            ///< 本次规划循环的时间开销（ms）
};

}  // namespace planning

#endif  // _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_PLANNER_H_
