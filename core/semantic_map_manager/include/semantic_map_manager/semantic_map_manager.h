/**
 * @file semantic_map_manager.h
 * @author EPSILON Autonomous Driving Group
 * @brief EPSILON 自动驾驶系统的核心世界模型——语义地图管理器
 *
 * @details
 * SemanticMapManager 是整个 EPSILON 规划栈的"世界模型中枢"，负责以下核心功能：
 *
 * 第1层 - 数据聚合（通过 UpdateSemanticMap 入口）：
 *   - 接收并存储自车状态（ego_vehicle_）
 *   - 接收完整车道网络（whole_lane_net_）与局部车道网络（surrounding_lane_net_）
 *   - 接收障碍物栅格地图（obstacle_map_）与栅格集合（obstacle_grids_）
 *   - 接收周围车辆集合（surrounding_vehicles_）
 *
 * 第2层 - 语义增强与推理：
 *   - 构建语义车道集合（semantic_lane_set_），将原始车道数据转化为带语义标注的车道，
 *     包含换道可用性、拓扑关系、行为类型等信息
 *   - 构建局部车道快速查找表（local_lanes_ / LUT），将离散的segment车道拼接为
 *     延展连续的大车道，加速下游查表运算
 *   - 构建语义车辆集合（semantic_surrounding_vehicles_），为每辆周车附加
 *     最近车道ID、横向行为概率、参考车道等语义信息
 *
 * 第3层 - 行为预测与安全检查：
 *   - 基于规则的横向行为预测（NaiveRuleBasedLateralBehaviorPrediction）：
 *     利用Frenet坐标系下的横向偏移和横向速度判断换道意图
 *   - MOBIL 规则换道行为预测（MobilRuleBasedBehaviorPrediction）：
 *     评估换道对本车和周围车辆的影响，决定是否换道
 *   - 开环轨迹预测（OpenloopTrajectoryPrediction）：基于IDM跟车模型，
 *     预测周车在车道参考线上的未来轨迹，用于碰撞检测
 *   - RSS 安全性检查（通过 rss_checker_ 成员）
 *
 * 第4层 - 关键车辆筛选（UpdateKeyVehicles）：
 *   - 从周围车辆中筛选出与自车规划相关的"关键车辆"（key_vehicles_），
 *     基于车道拓扑距离和车速自适应裁剪范围
 *
 * 第5层 - 查询接口（供所有上层 planner 调用）：
 *   - 碰撞检测（CheckCollisionUsingState / GlobalPosition）
 *   - 最近车道查询（GetNearestLaneIdUsingState）
 *   - 拓扑可达性判定（IsTopologicallyReachable）
 *   - 前车/后车查询（GetLeadingVehicleOnLane / GetFollowingVehicleOnLane）
 *   - 限速与交通信号查询（GetSpeedLimit / GetTrafficStoppingState）
 *
 * 该类的输出数据被 behavior_planner、eudm_planner、ssc_planner 等所有上层规划器
 * 作为公共输入消费，是整个规划栈的"单一事实来源（Single Source of Truth）"。
 *
 * @version 0.1
 * @date 2019-03-20
 *
 * @copyright Copyright (c) 2019
 */

#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_SEMANTIC_MAP_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_SEMANTIC_MAP_H_

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <list>
#include <memory>
#include <set>
#include <thread>
#include <unordered_set>
#include <vector>

#include "common/basics/semantics.h"
#include "common/basics/shapes.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/math/calculations.h"
#include "common/mobil/mobil_behavior_prediction.h"
#include "common/rss/rss_checker.h"
#include "motion_predictor/onlane_fs_predictor.h"
#include "semantic_map_manager/config_loader.h"
#include "semantic_map_manager/traffic_signal_manager.h"
#include "vehicle_model/idm_model.h"

namespace semantic_map_manager {

/// @class SemanticMapManager
/// @brief EPSILON 自动驾驶系统的核心世界模型管理器
///
/// 该类负责从传感器/仿真器原始数据构建统一的语义世界表示，
/// 并进行行为预测、关键车辆筛选等推理，为所有上层规划器提供统一的查询接口。
/// 它扮演着"场景理解"这一关键角色，将几何级别的感知数据提升为语义级别的环境表示。
class SemanticMapManager {
 public:
  using ObstacleMapType = uint8_t;   ///< 障碍物地图单元类型（0=空闲, 100=占据, 255=未知）
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;  ///< 二维栅格地图类型别名
  using State = common::State;       ///< 车辆状态类型别名（位置、速度、加速度、航向角等）
  using Lane = common::Lane;          ///< 车道类型别名
  using LateralBehavior = common::LateralBehavior;  ///< 横向行为类型别名（车道保持/左换道/右换道）
  using SemanticLane = common::SemanticLane;        ///< 语义车道类型别名

  SemanticMapManager() {}
  /// @brief 通过智能体配置文件路径构造（从json配置加载全部参数）
  /// @param id 自车ID
  /// @param agent_config_path 智能体配置JSON文件路径
  SemanticMapManager(const int &id, const std::string &agent_config_path);

  /// @brief 通过显式参数构造（用于非文件配置的场景）
  /// @param id 自车ID
  /// @param surrounding_search_radius 周围车辆搜索半径（米）
  /// @param enable_openloop_prediction 是否启用开环轨迹预测
  /// @param use_right_hand_axis 是否使用右手坐标系
  SemanticMapManager(const int &id, const decimal_t surrounding_search_radius,
                     bool enable_openloop_prediction, bool use_right_hand_axis);
  ~SemanticMapManager() {}

  // ======================== 第5层：查询接口 ========================

  /// @brief 检查给定全局坐标位置是否与障碍物地图发生碰撞
  /// @param p_w 世界坐标系下的二维位置
  /// @param res [输出] 是否碰撞
  ErrorType CheckCollisionUsingGlobalPosition(const Vec2f &p_w,
                                              bool *res) const;

  /// @brief 获取全局坐标位置的障碍物地图值
  /// @param p_w 世界坐标系下的二维位置
  /// @param res [输出] 障碍物地图值
  ErrorType GetObstacleMapValueUsingGlobalPosition(const Vec2f &p_w,
                                                   ObstacleMapType *res);

  /// @brief 检查给定车辆参数和状态是否与障碍物地图/开环预测轨迹碰撞
  /// @param vehicle_param 车辆几何参数（长、宽、轴距等）
  /// @param state 待检查的车辆状态
  /// @param res [输出] 是否碰撞
  /// @note 同时检查静态障碍物（栅格地图）和动态障碍物（周车开环预测轨迹）
  ErrorType CheckCollisionUsingStateAndVehicleParam(
      const common::VehicleParam &vehicle_param, const common::State &state,
      bool *res);

  /// @brief 检查两个车辆状态是否发生碰撞（基于有向包围盒OBB相交判定）
  /// @param param_a 车辆A的几何参数
  /// @param state_a 车辆A的状态
  /// @param param_b 车辆B的几何参数
  /// @param state_b 车辆B的状态
  /// @param res [输出] 是否碰撞
  ErrorType CheckCollisionUsingState(const common::VehicleParam &param_a,
                                     const common::State &state_a,
                                     const common::VehicleParam &param_b,
                                     const common::State &state_b, bool *res);

  /// @brief [未实现] 检查状态向量中的自碰撞
  ErrorType CheckCollisionUsingStateVec(
      const vec_E<common::State> state_vec) const;

  /// @brief 获取所有语义车道到给定3自由度状态的距离信息
  /// @param state 三维状态 (x, y, theta)
  /// @param res [输出] 按距离排序的元组集合，每个元组为 (距离, 弧长, 角度差, 车道ID)
  /// @note 距离计算：投影点到车道参考线的最近距离 + 角度差约束
  ErrorType GetDistanceToLanesUsing3DofState(
      const Vec3f &state,
      std::set<std::tuple<decimal_t, decimal_t, decimal_t, int>> *res) const;

  // ======================== 第1层：数据聚合 ========================

  /// @brief 更新语义地图的核心入口函数（主数据管道）
  ///
  /// 处理流程（按调用顺序）：
  ///   1. 存储时间戳及所有传感数据
  ///   2. UpdateSemanticLaneSet() —— 构建语义车道集合
  ///   3. UpdateLocalLanesAndFastLut() —— 构建局部车道快速查找表（可选，取决于配置）
  ///   4. UpdateSemanticVehicles() —— 为周围车辆注入语义信息
  ///   5. UpdateKeyVehicles() —— 筛选关键车辆
  ///   6. OpenloopTrajectoryPrediction() —— 预测周车轨迹（可选，取决于配置）
  ///   7. SaveMapToLog() —— 记录数据到日志（可选，取决于配置）
  ///
  /// @param time_stamp 当前时间戳（秒）
  /// @param ego_vehicle 自车完整信息
  /// @param whole_lane_net 完整车道网络（所有车道段）
  /// @param surrounding_lane_net 局部车道网络（自车周围的车道段）
  /// @param obstacle_map 障碍物栅格地图
  /// @param obstacle_grids 障碍物占据的栅格世界坐标集合
  /// @param surrounding_vehicles 周围车辆集合（距离在搜索半径内）
  ErrorType UpdateSemanticMap(
      const double &time_stamp, const common::Vehicle &ego_vehicle,
      const common::LaneNet &whole_lane_net,
      const common::LaneNet &surrounding_lane_net,
      const common::GridMapND<ObstacleMapType, 2> &obstacle_map,
      const std::set<std::array<decimal_t, 2>> &obstacle_grids,
      const common::VehicleSet &surrounding_vehicles);

  /// @brief 获取给定状态相对于导航路径的最近车道ID
  /// @param state 三维状态 (x, y, theta)
  /// @param navi_path 导航路径（车道ID序列，用于约束搜索范围，可为空）
  /// @param id [输出] 最近车道ID
  /// @param distance [输出] 到最近车道的距离（米）
  /// @param arc_len [输出] 在最近车道上的弧长位置（米）
  /// @note 选择策略：先在 nearest_lane_range_ 内筛选，优先取角度差最小的车道；
  ///        若前向/后向角度差均大于90度，则退化为取最近的车道
  ErrorType GetNearestLaneIdUsingState(const Vec3f &state,
                                       const std::vector<int> &navi_path,
                                       int *id, decimal_t *distance,
                                       decimal_t *arc_len) const;

  // ======================== 第3层：行为预测 ========================

  /// @brief 朴素规则横向行为预测
  ///
  /// 基于Frenet坐标系下的横向偏移(d)和横向速度(dd)判断换道意图：
  ///   - 若车辆明显偏向左侧且正在左移，且左侧车道换道可用 -> 左换道
  ///   - 若车辆明显偏向右侧且正在右移，且右侧车道换道可用 -> 右换道
  ///   - 否则 -> 车道保持
  ///
  /// @param vehicle 目标车辆
  /// @param nearest_lane_id 目标车辆的最近车道ID
  /// @param lat_probs [输出] 横向行为概率分布（硬分类，某行为概率为1.0其余为0.0）
  /// @note 阈值参数：横向距离阈值0.4m，横向速度阈值0.35m/s
  ErrorType NaiveRuleBasedLateralBehaviorPrediction(
      const common::Vehicle &vehicle, const int nearest_lane_id,
      common::ProbDistOfLatBehaviors *lat_probs);

  /// @brief MOBIL规则换道行为预测
  ///
  /// 基于MOBIL（Minimizing Overall Braking Induced by Lane changes）模型：
  ///   1. 为每种横向行为（保持/左换/右换）构建对应的参考车道
  ///   2. 寻找各参考车道上的前车和后车
  ///   3. 通过IDM模型计算换道对目标车及周围车辆的影响（加速增益/减速度惩罚）
  ///   4. 产生连续的换道行为概率分布（而非硬分类）
  ///
  /// @param vehicle 目标车辆
  /// @param nearby_vehicles 附近车辆集合
  /// @param res [输出] MOBIL评估后的横向行为概率分布
  ErrorType MobilRuleBasedBehaviorPrediction(
      const common::Vehicle &vehicle, const common::VehicleSet &nearby_vehicles,
      common::ProbDistOfLatBehaviors *res);

  /// @brief 对单一车辆进行开环轨迹预测（基于IDM跟车模型和参考车道）
  /// @param vehicle 目标车辆
  /// @param lane 参考车道
  /// @param t_pred 预测总时长（秒），默认5.0s
  /// @param t_step 预测步长（秒），默认0.2s
  /// @param traj [输出] 预测的状态轨迹序列
  /// @note 使用 OnLaneFsPredictor::GetPredictedTrajectory 实现
  ErrorType TrajectoryPredictionForVehicle(const common::Vehicle &vehicle,
                                           const common::Lane &lane,
                                           const decimal_t &t_pred,
                                           const decimal_t &t_step,
                                           vec_E<common::State> *traj);

  /// @brief 判断给定车道ID是否能通过拓扑连接到达导航路径中的任意节点
  /// @param lane_id 起始车道ID
  /// @param path 目标导航路径（车道ID序列）
  /// @param num_lane_changes [输出] 最少需要的换道次数
  /// @param res [输出] 是否拓扑可达
  /// @note 使用BFS搜索，最大展开节点数限制 20；同时考虑child（前向）和左右换道通道
  ErrorType IsTopologicallyReachable(const int lane_id,
                                     const std::vector<int> &path,
                                     int *num_lane_changes, bool *res) const;

  /// @brief 根据给定横向行为获取对应的参考车道
  ///
  /// 流程：
  ///   1. GetNearestLaneIdUsingState 获取最近车道
  ///   2. GetTargetLaneId 根据行为获取目标车道ID
  ///   3. 若启用快速LUT则在 local_lanes_ 中直接查表
  ///   4. 否则通过 GetLocalLaneSamplesByState 采样并拟合车道
  ///
  /// @param state 车辆状态
  /// @param navi_path 导航路径
  /// @param behavior 横向行为
  /// @param max_forward_len 最大前向长度（米）
  /// @param max_back_len 最大后向长度（米）
  /// @param is_high_quality 是否使用高质量样条拟合
  /// @param lane [输出] 参考车道
  ErrorType GetRefLaneForStateByBehavior(const common::State &state,
                                         const std::vector<int> &navi_path,
                                         const LateralBehavior &behavior,
                                         const decimal_t &max_forward_len,
                                         const decimal_t &max_back_len,
                                         const bool is_high_quality,
                                         common::Lane *lane) const;

  /// @brief 根据当前车道ID和横向行为获取目标车道ID
  /// @param lane_id 当前车道ID
  /// @param behavior 横向行为
  /// @param target_lane_id [输出] 目标车道ID
  ErrorType GetTargetLaneId(const int lane_id, const LateralBehavior &behavior,
                            int *target_lane_id) const;

  /// @brief 获取给定状态在指定车道附近（含前后扩展）的采样点
  /// @param state 车辆状态
  /// @param lane_id 目标车道ID
  /// @param navi_path 导航路径（用于在分叉处选择路径）
  /// @param max_reflane_dist 最大前向参考距离
  /// @param max_backward_dist 最大后向参考距离
  /// @param samples [输出] 沿车道前向后向扩展后的采样点序列
  ErrorType GetLocalLaneSamplesByState(const common::State &state,
                                       const int lane_id,
                                       const std::vector<int> &navi_path,
                                       const decimal_t max_reflane_dist,
                                       const decimal_t max_backward_dist,
                                       vec_Vecf<2> *samples) const;

  /// @brief 获取参考车道上给定状态的前方车辆（最近的前车）
  /// @param ref_lane 参考车道
  /// @param ref_state 参考状态
  /// @param vehicle_set 候选车辆集合
  /// @param lat_range 横向搜索范围（米），默认约2.2m
  /// @param leading_vehicle [输出] 前车
  /// @param distance_residual_ratio [输出] 距离剩余比例
  /// @note 沿车道弧长方向逐步搜索，每次前进 resolution 长度（lat_range/1.4），
  ///        最大搜索120m
  ErrorType GetLeadingVehicleOnLane(const common::Lane &ref_lane,
                                    const common::State &ref_state,
                                    const common::VehicleSet &vehicle_set,
                                    const decimal_t &lat_range,
                                    common::Vehicle *leading_vehicle,
                                    decimal_t *distance_residual_ratio) const;

  /// @brief 获取参考车道上给定状态的后方车辆（最近的后车）
  /// @param ref_lane 参考车道
  /// @param ref_state 参考状态
  /// @param vehicle_set 候选车辆集合
  /// @param lat_range 横向搜索范围（米），默认约2.2m
  /// @param following_vehicle [输出] 后车
  /// @note 向后搜索，最大搜索距离 = min(ref_fs.s - lane.begin(), 100.0)
  ErrorType GetFollowingVehicleOnLane(const common::Lane &ref_lane,
                                      const common::State &ref_state,
                                      const common::VehicleSet &vehicle_set,
                                      const decimal_t &lat_range,
                                      common::Vehicle *following_vehicle) const;

  /// @brief 一次性获取参考车道上前车和后车的Frenet状态
  /// @note 组合调用 GetLeadingVehicleOnLane + GetFollowingVehicleOnLane，
  ///        并将结果转换为Frenet坐标系
  ErrorType GetLeadingAndFollowingVehiclesFrenetStateOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
      common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
      bool *has_following_vehicle, common::Vehicle *following_vehicle,
      common::FrenetState *following_fs) const;

  /// @brief 将当前语义地图数据保存到日志文件
  /// @note 输出 CSV 格式：时间戳, 车辆ID, x, y, 速度, 加速度, 航向角, 曲率, 方向盘转角
  ErrorType SaveMapToLog();

  /// @brief 判断局部车道是否包含给定的段车道ID
  /// @param local_lane_id 局部车道ID
  /// @param seg_lane_id 段车道ID
  /// @return 是否包含
  bool IsLocalLaneContainsLane(const int &local_lane_id,
                               const int &seg_lane_id) const;

  /// @brief 获取给定状态和车道的限速值（委托给 TrafficSignalManager）
  ErrorType GetSpeedLimit(const State &state, const Lane &lane,
                          decimal_t *speed_limit) const;

  /// @brief 获取给定状态和车道的交通停车状态（委托给 TrafficSignalManager）
  ErrorType GetTrafficStoppingState(const State &state, const Lane &lane,
                                    State *stopping_state) const;

  /// @brief 获取自车最近的lane ID
  ErrorType GetEgoNearestLaneId(int *ego_lane_id) const;

  // ======================== 数据访问器（Getter / Setter）========================

  inline double time_stamp() const { return time_stamp_; }

  inline int ego_id() const { return ego_id_; }

  inline common::Vehicle ego_vehicle() const { return ego_vehicle_; }

  inline common::GridMapND<ObstacleMapType, 2> obstacle_map() const {
    return obstacle_map_;
  }
  inline common::GridMapND<ObstacleMapType, 2> *obstacle_map_ptr() {
    return &obstacle_map_;
  }
  inline std::set<std::array<decimal_t, 2>> obstacle_grids() const {
    return obstacle_grids_;
  }
  inline common::VehicleSet surrounding_vehicles() const {
    return surrounding_vehicles_;
  }
  /// @brief 获取关键车辆集合（从周围车辆中筛选出对自车规划重要的车辆）
  inline common::VehicleSet key_vehicles() const { return key_vehicles_; }
  inline common::LaneNet whole_lane_net() const { return whole_lane_net_; }
  inline common::LaneNet surrounding_lane_net() const {
    return surrounding_lane_net_;
  }
  inline common::SemanticLaneSet semantic_lane_set() const {
    return semantic_lane_set_;
  }
  inline const common::SemanticLaneSet *semantic_lane_set_cptr() const {
    const common::SemanticLaneSet *ptr = &semantic_lane_set_;
    return ptr;
  }
  inline common::SemanticBehavior ego_behavior() const { return ego_behavior_; }
  inline common::SemanticVehicleSet semantic_surrounding_vehicles() const {
    return semantic_surrounding_vehicles_;
  }
  inline common::SemanticVehicleSet semantic_key_vehicles() const {
    return semantic_key_vehicles_;
  }
  inline AgentConfigInfo agent_config_info() const {
    return agent_config_info_;
  }
  inline std::vector<int> key_vehicle_ids() const { return key_vehicle_ids_; }

  /// @brief 获取观测不确定性车辆ID列表（由跟踪噪声注入模块产生）
  inline std::vector<int> uncertain_vehicle_ids() const {
    return uncertain_vehicle_ids_;
  }

  /// @brief 获取所有周车的开环预测轨迹（key=车辆ID, value=状态序列）
  inline std::unordered_map<int, vec_E<common::State>> openloop_pred_trajs()
      const {
    return openloop_pred_trajs_;
  }

  inline vec_E<common::SpeedLimit> RetTrafficInfoSpeedLimit() const {
    return traffic_singal_manager_.speed_limit_list();
  }

  inline vec_E<common::TrafficLight> RetTrafficInfoTrafficLight() const {
    return traffic_singal_manager_.traffic_light_list();
  }

  /// @brief 获取局部车道集合（拼接后的连续长车道，用于快速查找）
  inline std::unordered_map<int, common::Lane> local_lanes() const {
    return local_lanes_;
  }

  // Setter 函数 —— 用于从外部注入数据
  inline void set_ego_id(const int &in) { ego_id_ = in; }
  inline void set_obstacle_map(
      const common::GridMapND<ObstacleMapType, 2> &in) {
    obstacle_map_ = in;
  }
  inline void set_obstacle_grids(const std::set<std::array<decimal_t, 2>> &in) {
    obstacle_grids_ = in;
  }
  inline void set_ego_vehicle(const common::Vehicle &in) { ego_vehicle_ = in; }
  inline void set_surrounding_vehicles(const common::VehicleSet &in) {
    surrounding_vehicles_ = in;
  }
  inline void set_whole_lane_net(const common::LaneNet &in) {
    whole_lane_net_ = in;
  }
  inline void set_surrounding_lane_net(const common::LaneNet &in) {
    surrounding_lane_net_ = in;
  }
  inline void set_semantic_lane_set(const common::SemanticLaneSet &in) {
    semantic_lane_set_ = in;
  }
  inline void set_ego_behavior(const common::SemanticBehavior &in) {
    ego_behavior_ = in;
  }
  inline void set_uncertain_vehicle_ids(
      const std::vector<int> &uncertain_vehicle_ids) {
    uncertain_vehicle_ids_ = uncertain_vehicle_ids;
  }

 private:
  // ======================== 第2层：语义增强 ========================

  /// @brief 更新语义车道集合
  /// @details 将 surrounding_lane_net_ 中的原始车道数据转换为 SemanticLane，
  ///          并校验左右换道通道、父子拓扑关系的一致性：
  ///          - 若左侧换道目标不在语义集合中，则标记左换道不可用
  ///          - 若右侧换道目标不在语义集合中，则标记右换道不可用
  ///          - 清理无效的父子车道ID引用
  ErrorType UpdateSemanticLaneSet();

  /// @brief 构建局部车道快速查找表（Fast LUT）
  ///
  /// 以自车当前车道为根（含左、左左、右、右右共最多5条根车道），
  /// 分别向前、向后递归展开车道段路径，然后将前后路径拼装成连续长车道，
  /// 并构建双向索引：
  ///   - local_to_segment_lut_：local_id -> {segment_id 序列}
  ///   - segment_to_local_lut_：segment_id -> {local_id 集合}
  ///
  /// @note 前向最小展开长度 = local_lane_length_forward_ (250m)
  ///        后向最小展开长度 = local_lane_length_backward_ (150m)
  ErrorType UpdateLocalLanesAndFastLut();

  /// @brief 更新语义车辆集合
  ///
  /// 对 surrounding_vehicles_ 中的每辆车：
  ///   1. 查找其最近车道ID
  ///   2. 进行朴素规则横向行为预测
  ///   3. 取最大概率行为作为当前横向行为
  ///   4. 根据行为构建对应的参考车道（前向长度 = max(车速*10, 50m)，后向长度 = 10m）
  ErrorType UpdateSemanticVehicles();

  /// @brief 更新关键车辆集合
  ///
  /// 筛选策略：
  ///   1. 从自车所在车道及左右相邻车道出发，沿length方向展开若干后继/前驱车道段，
  ///      构建"关键车道ID集合"
  ///   2. 对于每辆周车，检查其最近车道是否在关键车道集合中
  ///   3. 若周车在自车前方（弧长差值 >= 0）且在自适应规划范围内 -> 加入关键车辆
  ///      自适应范围 = clamp(velocity * t_comfort + 100, 30, 170)
  ///   4. 若周车在自车后方但有安全余量（s_margin） -> 加入关键车辆
  ErrorType UpdateKeyVehicles();

  /// @brief 对所有语义周围车辆执行开环轨迹预测
  /// @details 遍历 semantic_surrounding_vehicles_，对每辆车调用
  ///          TrajectoryPredictionForVehicle，使用 pred_time_=5.0, pred_step_=0.2
  ErrorType OpenloopTrajectoryPrediction();

  /// @brief 计算车道网络上两点间的距离（Dijkstra-like图搜索，未完整实现）
  ErrorType GetDistanceOnLaneNet(const int &lane_id_0,
                                 const decimal_t &arc_len_0,
                                 const int &lane_id_1,
                                 const decimal_t &arc_len_1,
                                 decimal_t *dist) const;

  /// @brief 在车道上均匀采样
  /// @param lane 目标车道
  /// @param s0 起始弧长
  /// @param s1 终止弧长
  /// @param step 采样步长
  /// @param samples [输出] 采样点序列
  /// @param accum_dist [输出] 累计采样距离
  ErrorType SampleLane(const common::Lane &lane, const decimal_t &s0,
                       const decimal_t &s1, const decimal_t &step,
                       vec_E<Vecf<2>> *samples, decimal_t *accum_dist) const;

  /// @brief 递归获取前向车道ID路径（达到最小长度后停止）
  /// @param node_id 当前车道节点ID
  /// @param node_length 当前车道段长度
  /// @param aggre_length 已累计的长度
  /// @param path_to_node 到达当前节点之前的路径
  /// @param all_paths [输出] 所有满足最小长度的前向路径集合
  void GetAllForwardLaneIdPathsWithMinimumLengthByRecursion(
      const decimal_t &node_id, const decimal_t &node_length,
      const decimal_t &aggre_length, const std::vector<int> &path_to_node,
      std::vector<std::vector<int>> *all_paths);

  /// @brief 递归获取后向车道ID路径（达到最小长度后停止，结果自动反转）
  /// @note 与Forward版本的区别：最终路径会反转，使得路径从最远的父车道指向当前根车道
  void GetAllBackwardLaneIdPathsWithMinimumLengthByRecursion(
      const decimal_t &node_id, const decimal_t &node_length,
      const decimal_t &aggre_length, const std::vector<int> &path_to_node,
      std::vector<std::vector<int>> *all_paths);

  /// @brief 通过车道ID序列构建局部连续长车道
  /// @param state 车辆状态（用于定位弧长）
  /// @param lane_ids 车道ID序列（前后拼接后的完整路径）
  /// @param max_reflane_dist 前向参考距离
  /// @param max_backward_dist 后向参考距离
  /// @param is_high_quality 是否高质量样条拟合
  /// @param lane [输出] 拼接并裁剪后的连续车道
  ErrorType GetLocalLaneUsingLaneIds(const common::State &state,
                                     const std::vector<int> &lane_ids,
                                     const decimal_t max_reflane_dist,
                                     const decimal_t max_backward_dist,
                                     const bool &is_high_quality,
                                     common::Lane *lane);

  /// @brief 通过采样点拟合车道
  /// @param samples 采样点序列
  /// @param is_high_quality 若为true则使用分段样条拟合（20段），否则使用默认拟合方法
  /// @param lane [输出] 拟合后的车道
  ErrorType GetLaneBySampledPoints(const vec_Vecf<2> &samples,
                                   const bool &is_high_quality,
                                   common::Lane *lane) const;

  // ======================== 成员变量 ========================

  double time_stamp_{0.0};      ///< 当前数据的时间戳

  decimal_t pred_time_ = 5.0;   ///< 开环轨迹预测总时长（秒）
  decimal_t pred_step_ = 0.2;   ///< 开环轨迹预测步长（秒）

  decimal_t nearest_lane_range_ = 1.5;  ///< 最近车道搜索的距离阈值（米）
  decimal_t lane_range_ = 10.0;         ///< 车道搜索的全局距离范围（米）

  decimal_t max_distance_to_lane_ = 2.0;  ///< 车辆关联到车道的最大容许距离（米）

  // ======================== 快速查找表（Fast LUT）相关 ========================

  bool has_fast_lut_ = false;  ///< 快速查找表是否已构建
  /// @brief 局部车道集合：key=local_lane_id, value=拼接后的连续长车道
  std::unordered_map<int, common::Lane> local_lanes_;
  /// @brief 局部车道到段车道的映射：local_lane_id -> {segment_lane_id 序列}
  std::unordered_map<int, std::vector<int>> local_to_segment_lut_;
  /// @brief 段车道到局部车道的映射：segment_lane_id -> {local_lane_id 集合}
  std::unordered_map<int, std::set<int>> segment_to_local_lut_;

  decimal_t local_lane_length_forward_ = 250.0;   ///< 局部车道前向展开长度（米）
  decimal_t local_lane_length_backward_ = 150.0;  ///< 局部车道后向展开长度（米）

  // ======================== 智能体配置 ========================

  int ego_id_;                     ///< 自车ID
  std::string agent_config_path_;  ///< 智能体配置文件路径
  AgentConfigInfo agent_config_info_;  ///< 智能体配置信息结构体
  bool use_right_hand_axis_ = true;    ///< 是否使用右手坐标系
  /// @brief 是否仅用于高速公路类简单车道结构（单个方向无分叉）
  bool is_simple_lane_structure_ = false;

  // ======================== 核心数据成员 ========================

  common::Vehicle ego_vehicle_;             ///< 自车完整信息（状态+参数）
  GridMap2D obstacle_map_;                  ///< 障碍物栅格地图（0=空闲, 100=占据, 255=未知）
  std::set<std::array<decimal_t, 2>> obstacle_grids_;  ///< 被障碍物占据的栅格世界坐标
  // * 周围车辆通过搜索半径构建
  common::VehicleSet surrounding_vehicles_;          ///< 周围车辆集合
  // * 周围车辆的语义版本
  common::SemanticVehicleSet semantic_surrounding_vehicles_;  ///< 语义版周围车辆集合
  // * 基于筛选策略的关键车道车辆
  // * 关键车辆是周围车辆的子集
  common::VehicleSet key_vehicles_;                ///< 关键车辆集合（周围车辆的子集）
  // * 带语义的关键车辆
  common::SemanticVehicleSet semantic_key_vehicles_;  ///< 带语义的关键车辆集合
  std::vector<int> key_vehicle_ids_;               ///< 关键车辆的ID列表
  std::vector<int> uncertain_vehicle_ids_;         ///< 观测不确定性车辆ID列表

  common::LaneNet whole_lane_net_;          ///< 完整车道网络
  common::LaneNet surrounding_lane_net_;    ///< 局部车道网络（自车周围的子集）
  common::SemanticLaneSet semantic_lane_set_;  ///< 语义车道集合
  common::SemanticBehavior ego_behavior_;      ///< 自车语义行为

  // * 开环预测仅用于在线运动规划器的碰撞检查
  /// @brief 周车开环预测轨迹：vehicle_id -> 状态序列
  std::unordered_map<int, vec_E<common::State>> openloop_pred_trajs_;

  TicToc global_timer_;                 ///< 全局计时器
  TrafficSignalManager traffic_singal_manager_;  ///< 交通信号管理器（限速、红绿灯等）
  ConfigLoader *p_config_loader_;       ///< 配置加载器指针

  common::RssChecker rss_checker_;      ///< RSS（责任敏感安全）检查器
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_SEMANTIC_MAP_H_
