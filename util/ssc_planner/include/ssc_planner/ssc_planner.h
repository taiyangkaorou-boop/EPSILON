/**
 * @file ssc_planner.h
 * @author HKUST Aerial Robotics Group
 * @brief SSC（时空语义走廊）规划器 —— EPSILON 系统中的低层运动规划模块
 *
 * [概述]
 * SscPlanner 是 EPSILON 自动驾驶系统中负责将高层行为决策转化为可执行轨迹
 * 的低层运动规划器。它采用"时空语义走廊 + QP优化"的算法框架：
 *   1. 将自车状态、参考车道及障碍物信息转换至 Frenet 坐标系
 *      (s = 沿车道的纵向距离, d = 相对车道中心的横向偏移)
 *   2. 构建 SSC Map —— (s, d, t) 三维时空占据栅格地图
 *   3. 沿初始参考轨迹膨胀生成时空走廊 —— 一组在 s-d-t 空间中无碰撞的立方体序列
 *   4. 通过二次规划(QP)在走廊约束内拟合光滑的 Bezier 样条轨迹
 *   5. 低速场景下使用基于横向依赖的 primitive 轨迹作为备选
 *
 * [输入]
 *   - 自车状态 (State) 及车辆参数
 *   - 参考车道 (Lane) 与行为决策 (LateralBehavior)
 *   - 障碍物占据栅格地图 (GridMap2D)
 *   - 多行为前向仿真轨迹 (forward_trajs + surround_forward_trajs)
 *
 * [输出]
 *   - Frenet 坐标系下的 Bezier 样条轨迹 (FrenetBezierTrajectory),
 *     可供 PurePursuit + IDM 控制器跟踪执行
 *   - 低速场景下的备选 primitive 轨迹 (FrenetPrimitiveTrajectory)
 *
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_SEARCH_H_
#define _UTIL_SSC_PLANNER_INC_SSC_SEARCH_H_

#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "common/basics/basics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/primitive/frenet_primitive.h"
#include "common/spline/spline_generator.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "common/trajectory/frenet_bezier_traj.h"
#include "common/trajectory/frenet_primitive_traj.h"
#include "ssc_config.pb.h"
#include "ssc_planner/map_interface.h"
#include "ssc_planner/ssc_map.h"

namespace planning {

/// @class SscPlanner
/// @brief 时空语义走廊（Spatio-Temporal Semantic Corridor）规划器
///
/// 该类是 EPSILON 系统中低层运动规划的核心实现。它接收来自语义地图管理器的
/// 环境信息（自车状态、参考车道、障碍物、前向轨迹），通过以下流程生成安全
/// 且光滑的可执行轨迹：
///
/// 算法流程：
///   Step 1. 坐标变换 (StateTransformForInputData):
///           将全局笛卡尔坐标系下的所有状态和点转换到 Frenet 坐标系
///   Step 2. SSC Map 构建 (ConstructSscMap):
///           在 (s, d, t) 三维空间中建立占据栅格地图，填充静态和动态障碍物
///   Step 3. 时空走廊构建 (ConstructCorridorUsingInitialTrajectory):
///           沿每条前向轨迹膨胀生成无碰撞的时空立方体序列
///   Step 4. QP 轨迹优化 (RunQpOptimization):
///           在走廊约束内通过二次规划拟合光滑 Bezier 样条，最小化 jerk 与加速度
///   Step 5. 轨迹选择 (UpdateTrajectoryWithCurrentBehavior):
///           根据当前行为决策选择对应的优化轨迹作为输出
///
/// 速度阈值说明：
///   - 高速 (> low_speed_threshold): 横向运动与纵向运动可解耦 (is_lateral_independent_=true)
///   - 低速: 使用 lateral-dependent primitive 轨迹
class SscPlanner : public Planner {
 public:
  using ObstacleMapType = uint8_t;   ///< 障碍物地图数据类型
  using SscMapDataType = uint8_t;    ///< SSC地图数据类型

  using Lane = common::Lane;                             ///< 车道类型
  using State = common::State;                           ///< 全局状态类型
  using Vehicle = common::Vehicle;                       ///< 车辆类型
  using LateralBehavior = common::LateralBehavior;       ///< 横向行为类型 (车道保持/左变道/右变道)
  using FrenetState = common::FrenetState;               ///< Frenet坐标系状态类型
  using FrenetTrajectory = common::FrenetTrajectory;     ///< Frenet轨迹基类
  using FrenetPrimitive = common::FrenetPrimitive;       ///< Frenet primitive 轨迹
  using FrenetBezierTrajectory = common::FrenetBezierTrajectory;   ///< Frenet Bezier样条轨迹
  using FrenetPrimitiveTrajectory = common::FrenetPrimitiveTrajectory; ///< Primitive轨迹
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>; ///< 2D占据栅格地图

  typedef common::BezierSpline<5, 2> BezierSpline;       ///< 5阶2维 Bezier样条

  SscPlanner() = default;

  // =========================================================================
  // 设置器 (Setters)
  // =========================================================================

  /// @brief 设置地图接口（用于从语义地图管理器获取环境信息）
  /// @param map_itf 指向地图接口的指针，负责提供自车状态、障碍物等信息
  /// @return 错误码
  ErrorType set_map_interface(SscPlannerMapItf* map_itf);

  /// @brief 设置规划起始状态（用于闭环仿真中指定初始状态）
  /// @param state 全局坐标系下的初始状态
  /// @return 错误码
  ErrorType set_initial_state(const State& state);

  // =========================================================================
  // 获取器 (Getters)
  // =========================================================================

  /// @brief 获取 SSC 三维时空地图指针
  SscMap* p_ssc_map() const { return p_ssc_map_; }

  /// @brief 获取自车在 Frenet 坐标系下的当前状态
  FrenetState ego_frenet_state() const { return ego_frenet_state_; }

  /// @brief 获取自车在全局坐标系下的车辆信息
  Vehicle ego_vehicle() const { return ego_vehicle_; }

  /// @brief 获取各行为下的全局坐标系前向仿真轨迹
  vec_E<vec_E<Vehicle>> forward_trajs() const { return forward_trajs_; }

  /// @brief 获取各行为对应的 QP 优化 Bezier 样条轨迹 (Frenet坐标)
  vec_E<BezierSpline> qp_trajs() const { return qp_trajs_; }

  /// @brief 获取时间原点 (规划起始时刻的时间戳)
  decimal_t time_origin() const { return time_origin_; }

  /// @brief 获取周围车辆在 Frenet 坐标系下的预测轨迹
  std::unordered_map<int, vec_E<common::FsVehicle>> sur_vehicle_trajs_fs()
      const {
    return sur_vehicle_trajs_fs_;
  }

  /// @brief 获取各行为下的周围车辆 Frenet 坐标系预测轨迹集合
  vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>>
  surround_forward_trajs_fs() const {
    return surround_forward_trajs_fs_;
  };

  /// @brief 获取各行为下的自车前向 Frenet 仿真轨迹
  vec_E<vec_E<common::FsVehicle>> forward_trajs_fs() const {
    return forward_trajs_fs_;
  }

  /// @brief 获取自车在 Frenet 坐标系下的轮廓顶点
  vec_E<Vec2f> ego_vehicle_contour_fs() const {
    return fs_ego_vehicle_.vertices;
  }

  /// @brief 获取自车的 Frenet 车辆对象（含状态与轮廓）
  common::FsVehicle fs_ego_vehicle() const { return fs_ego_vehicle_; }

  /// @brief 获取当前输出轨迹
  /// @return 高速时返回 Bezier 样条轨迹，低速时返回 primitive 轨迹
  std::unique_ptr<FrenetTrajectory> trajectory() const {
    if (!is_lateral_independent_) {
      return std::unique_ptr<FrenetPrimitiveTrajectory>(
          new FrenetPrimitiveTrajectory(low_spd_alternative_traj_));
    }
    return std::unique_ptr<FrenetBezierTrajectory>(
        new FrenetBezierTrajectory(trajectory_));
  }

  /// @brief 获取坐标变换器（全局坐标 <-> Frenet 坐标）
  common::StateTransformer state_transformer() const { return stf_; }

  /// @brief 获取最近一次规划的总耗时（毫秒）
  decimal_t time_cost() const { return time_cost_; }

  /// @brief 获取初始 Frenet 状态
  common::FrenetState initial_frenet_state() const {
    return initial_frenet_state_;
  }

  // =========================================================================
  // 核心接口 (Planner基类虚函数实现)
  // =========================================================================

  /// @brief 返回规划器名称 "ssc_planner"
  std::string Name() override;

  /// @brief 初始化规划器：加载 protobuf 配置文件，创建 SscMap 实例
  /// @param config_path protobuf 文本格式配置文件的路径
  /// @return 错误码
  ErrorType Init(const std::string config_path) override;

  /// @brief 执行一次完整的规划循环（所有算法步骤）
  ///
  /// 流程：
  ///   1. 从地图接口读取环境信息（自车、车道、障碍物、前向轨迹）
  ///   2. 坐标变换至 Frenet 坐标系
  ///   3. 重置 SSC Map 并构建时空占据地图
  ///   4. 为每条前向轨迹构建时空走廊
  ///   5. 运行 QP 优化生成光滑轨迹
  ///   6. 按当前行为选择最终输出轨迹
  ///
  /// @return 错误码，kSuccess 表示规划成功
  ErrorType RunOnce() override;

 private:
  /// @brief MVP-6 候选轨迹风险暴露统计
  /// @note 该结构只用于 QP 成功后的候选评价/日志/CSV。默认配置下不会影响轨迹选择。
  struct RiskExposureMetrics {
    int candidate_index = -1;                  ///< qp_trajs_ / valid_behaviors_ 中的候选索引
    LateralBehavior behavior{LateralBehavior::kUndefined};  ///< 候选轨迹对应的横向行为
    size_t sample_count = 0;                   ///< Bezier 轨迹采样点数量
    size_t high_risk_hits = 0;                 ///< 风险值超过阈值的采样点数量
    decimal_t exposure_sum = 0.0;              ///< 风险暴露总和 Σ Risk(s,d,t)
    decimal_t exposure_mean = 0.0;             ///< 平均风险暴露
    decimal_t exposure_max = 0.0;              ///< 最大单点风险
    decimal_t risk_score = 0.0;                ///< 加权风险软代价 λ_risk * exposure_sum
  };

  /// @brief 从 protobuf 文本文件读取配置
  ErrorType ReadConfig(const std::string config_path);

  /// @brief 检查时空走廊的连续性
  /// 验证前后相邻立方体的时间上下界是否对齐（前一立方体 t_ub == 后一立方体 t_lb）
  /// @param cubes 时空语义立方体序列
  /// @return kSuccess 表示走廊合法可优化
  ErrorType CorridorFeasibilityCheck(
      const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& cubes);

  /// @brief 批量坐标变换主函数
  ///
  /// 三个阶段：
  ///   Stage I:   将全局状态和点打包成向量
  ///   Stage II:  多线程/单线程执行坐标变换
  ///   Stage III: 从变换结果中恢复 Frenet 状态和点坐标
  ErrorType StateTransformForInputData();

  /// @brief 运行 QP 优化生成 Bezier 样条轨迹
  ///
  /// 详细流程：
  ///   1. 从 SscMap 获取各行为的时空走廊
  ///   2. 对每条有效走廊：提取起始约束（位置、速度、加速度）和终止约束
  ///   3. 调用 SplineGenerator 在走廊约束内拟合 Bezier 样条
  ///   4. 低速场景下额外生成 primitive 备选轨迹
  ///   5. 记录所有成功生成的轨迹及其对应的行为标签
  ///
  /// @return 错误码
  ErrorType RunQpOptimization();

  /// @brief 验证最终轨迹的基本合法性
  /// 检查起点位置/速度与初始状态的一致性，以及全轨迹曲率不超过阈值
  /// @param traj 待验证的 Frenet 轨迹
  /// @return 错误码
  ErrorType ValidateTrajectory(const FrenetTrajectory& traj);

  /// @brief OpenMP 多线程坐标变换
  /// 用 4 线程并行执行状态到 Frenet 坐标和点到 Frenet 坐标的转换
  ErrorType StateTransformUsingOpenMp(const vec_E<State>& global_state_vec,
                                      const vec_E<Vec2f>& global_point_vec,
                                      vec_E<FrenetState>* frenet_state_vec,
                                      vec_E<Vec2f>* fs_point_vec) const;

  /// @brief 单线程坐标变换
  ErrorType StateTransformSingleThread(const vec_E<State>& global_state_vec,
                                       const vec_E<Vec2f>& global_point_vec,
                                       vec_E<FrenetState>* frenet_state_vec,
                                       vec_E<Vec2f>* fs_point_vec) const;

  /// @brief 根据当前自车行为从有效轨迹列表中选择匹配的轨迹
  ///
  /// 匹配优先级：
  ///   1. 精确匹配当前 ego_behavior_
  ///   2. 若无精确匹配，回退到 LaneKeeping 行为
  /// @return 错误码，找不到匹配行为时失败
  ErrorType UpdateTrajectoryWithCurrentBehavior();

  /// @brief 按原始 SSC 规则查找 baseline 候选轨迹索引
  /// @return qp_trajs_ 中的候选索引；找不到精确行为且无 LaneKeeping 回退时返回 -1
  /// @note MVP-6 将原有两级选择逻辑抽出，保证风险重选关闭时行为完全一致。
  int FindBaselineTrajectoryIndex() const;

  /// @brief 计算单条 QP 候选 Bezier 轨迹的风险暴露
  /// @param candidate_index qp_trajs_ / valid_behaviors_ 中的候选索引
  /// @return 风险暴露统计；若采样失败则 sample_count 为 0，风险默认为 0
  /// @note MVP-6: 只读 p_ssc_map_ 的 risk grid，不修改 QP 轨迹、走廊或地图。
  RiskExposureMetrics ComputeRiskExposureForCandidate(
      const int candidate_index) const;

  /// @brief 选择风险软代价最低的候选轨迹
  /// @param baseline_index 原始 SSC 行为选择得到的候选索引
  /// @param metrics 所有 QP 成功候选的风险暴露统计
  /// @return 最终候选索引；默认返回 baseline_index
  /// @note 只有 enable_risk_exposure_reselect=true 且风险优势超过切换裕度时才会改变选择。
  int SelectRiskAwareTrajectoryIndex(
      const int baseline_index,
      const std::vector<RiskExposureMetrics>& metrics) const;

  /// @brief 将 MVP-6 候选轨迹风险暴露结果追加写入 CSV
  /// @param metrics 所有候选轨迹风险暴露统计
  /// @param baseline_index 原始 SSC 选择的候选索引
  /// @param selected_index MVP-6 最终选择的候选索引
  /// @note CSV 仅用于论文实验分析；写入失败只打日志，不中断规划。
  void AppendRiskExposureMetricsToCsv(
      const std::vector<RiskExposureMetrics>& metrics,
      const int baseline_index, const int selected_index) const;

  /// @brief 判断 MVP-6 风险暴露评价是否需要启用
  /// @return true 表示需要保存 risk grid 快照并计算候选轨迹 exposure
  /// @note 评价开关或重选开关任一打开时都需要计算；默认二者关闭，不增加 baseline 开销。
  bool IsRiskExposureEvaluationEnabled() const;

  // =========================================================================
  // 成员变量
  // =========================================================================

  /// 自车在全局坐标系下的车辆信息
  Vehicle ego_vehicle_;
  /// 自车当前的横向行为决策 (LaneKeeping / LeftLaneChange / RightLaneChange)
  LateralBehavior ego_behavior_;
  /// 自车在 Frenet 坐标系下的当前状态
  FrenetState ego_frenet_state_;
  /// 局部导航参考车道（用于 StateTransformer 初始化）
  Lane nav_lane_local_;
  /// 规划的时间原点（自车当前时间戳）
  decimal_t time_origin_{0.0};

  /// 规划起始状态（用于闭环仿真中 override 自车当前状态）
  State initial_state_;
  /// 是否已设置外部初始状态
  bool has_initial_state_ = false;

  /// 初始 Frenet 状态
  common::FrenetState initial_frenet_state_;

  /// 2D 障碍物占据栅格地图（全局笛卡尔坐标）
  GridMap2D grid_map_;
  /// 障碍物栅格集合（全局笛卡尔坐标）
  std::set<std::array<decimal_t, 2>> obstacle_grids_;
  /// 各行为下的自车前向仿真轨迹（全局笛卡尔坐标）
  vec_E<vec_E<Vehicle>> forward_trajs_;
  /// 各前向轨迹对应的行为标签
  std::vector<LateralBehavior> forward_behaviors_;
  /// 各行为下的周围车辆前向仿真轨迹 [behavior_idx][vehicle_id] = trajectory
  vec_E<std::unordered_map<int, vec_E<Vehicle>>> surround_forward_trajs_;
  /// 周围车辆多模态预测轨迹（全局坐标），仅用于 risk grid
  MultiModalSurroundingTrajectories multimodal_surround_trajs_;

  /// 障碍物栅格的 Frenet 坐标
  vec_E<Vec2f> obstacle_grids_fs_;

  // --- 优化用初始解 ---
  /// 自车在 Frenet 坐标系下的完整信息（状态 + 轮廓）
  common::FsVehicle fs_ego_vehicle_;
  /// 各行为下的自车前向仿真轨迹（Frenet 坐标）
  vec_E<vec_E<common::FsVehicle>> forward_trajs_fs_;
  /// 周围车辆的 Frenet 轨迹（单组）
  std::unordered_map<int, vec_E<common::FsVehicle>> sur_vehicle_trajs_fs_;
  /// 周围车辆当前确定性预测轨迹的存在概率（key=车辆ID, value=argmax行为概率）
  std::unordered_map<int, decimal_t> surround_traj_existence_probs_;
  /// 周围车辆多模态预测轨迹（Frenet 坐标），仅用于 risk grid
  MultiModalSurroundingFsTrajectories multimodal_surround_trajs_fs_;
  /// 各行为下的周围车辆 Frenet 轨迹集合
  vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>>
      surround_forward_trajs_fs_;

  /// QP 优化生成的 Bezier 样条轨迹集合（每行为一条）
  vec_E<BezierSpline> qp_trajs_;
  /// 低速备选 primitive 轨迹集合
  vec_E<FrenetPrimitive> primitive_trajs_;
  /// 生成成功的行为标签列表
  std::vector<LateralBehavior> valid_behaviors_;
  /// 各行为对应的时空走廊
  vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> corridors_;
  /// 各行为的参考状态列表
  vec_E<vec_E<common::FrenetState>> ref_states_list_;
  /// 每个前向行为对应的 risk grid 快照
  /// @note ConstructSscMap() 按行为循环复用同一个 SscMap，当前地图只保留最后一次风险图；
  ///       MVP-6 因此需要在每个行为构图后保存快照，避免候选 exposure 使用错位风险场。
  std::vector<RiskGridMap3D> behavior_risk_grid_snapshots_;
  /// 每个 QP 成功候选对应的 risk grid 快照，索引与 qp_trajs_ / valid_behaviors_ 对齐
  std::vector<RiskGridMap3D> candidate_risk_grid_snapshots_;

  /// 是否横向独立（高速模式，横向与纵向解耦）
  bool is_lateral_independent_ = true;
  /// 最终选择的 Bezier 样条轨迹
  FrenetBezierTrajectory trajectory_;
  /// 低速备选 primitive 轨迹
  FrenetPrimitiveTrajectory low_spd_alternative_traj_;
  /// 最终选择的时空走廊
  vec_E<common::SpatioTemporalSemanticCubeNd<2>> final_corridor_;
  /// 最终选择的参考状态列表
  vec_E<common::FrenetState> final_ref_states_;

  /// 状态变换器（全局坐标 <-> Frenet 坐标）
  common::StateTransformer stf_;

  // --- 地图相关 ---
  /// 地图接口指针（解耦对语义地图管理器的直接依赖）
  SscPlannerMapItf* map_itf_;
  /// 地图接口是否有效
  bool map_valid_ = false;
  /// SSC 三维时空地图指针
  SscMap* p_ssc_map_;

  /// 当前规划帧的时间戳
  decimal_t stamp_ = 0.0;
  /// 最近一次规划的总耗时（毫秒）
  decimal_t time_cost_ = 0.0;

  /// protobuf 配置对象，包含规划器和地图的所有参数
  planning::ssc::Config cfg_;
  /// MVP-6 风险暴露 CSV 是否已经确认/写入 header
  mutable bool risk_exposure_csv_header_written_ = false;
  /// MVP-6 风险暴露 CSV 规划周期计数器
  mutable size_t risk_exposure_cycle_count_ = 0;
};

}  // namespace planning

#endif
