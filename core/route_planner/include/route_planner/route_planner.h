/**
 * @file route_planner.h
 * @author EPSILON Autonomous Driving Group
 * @brief 导航路径规划器——在车道网络中生成从起点到目的地的车道ID序列
 *
 * @details
 * RoutePlanner 是 EPSILON 规划栈的"导航层"，负责在车道拓扑网络上
 * 生成一条从当前车道出发的宏观路径（车道ID序列）。
 *
 * 核心职责：
 *   1. 接收车道网络（lane_net_）和自车最近车道ID（nearest_lane_id_）
 *   2. 通过沿 child_id 链随机展开（kRandomExpansion 模式），生成一条
 *      总长度超过 navi_path_max_length_（默认200m）的车道路径
 *   3. 将路径中的车道采样点拟合为一条连续的长车道（navi_lane_）
 *   4. 维护导航状态（kReadyToGo -> kInProgress -> kFinished），
 *      支持在到达目标后自动重启（if_restart_ = true）
 *
 * 导航模式（NaviMode）：
 *   - kRandomExpansion：沿子车道（child_id）链随机展开（当前唯一实现）
 *     在每个节点处，从所有可用的子车道中随机选择一条继续前进，
 *     直到累计路径长度达到 navi_path_max_length_
 *   - kAssignedTarget：指定目标车道的导航（未完整实现）
 *
 * 状态机（NaviStatus）：
 *   - kReadyToGo：就绪，开始新的路径规划
 *   - kInProgress：路径生成中，持续检查进度
 *   - kFinished：路径已完成，等待重启
 *
 * 输出：
 *   - navi_path_：车道ID序列（std::vector<int>）
 *   - navi_lane_：从车道ID序列拟合的连续 Lane 对象
 *   - navi_cur_arc_len_：自车在导航车道上的当前弧长位置
 *
 * @note 该导航路径被语义地图管理器和所有下游规划器（behavior_planner、
 *        eudm_planner）消费，用于约束横向行为选择和局部车道构建。
 *
 * @version 0.1
 * @date 2019-03-20
 */

#ifndef _CORE_ROUTE_PLANNER_INC_ROUTE_PLANNER_H_
#define _CORE_ROUTE_PLANNER_INC_ROUTE_PLANNER_H_

#include <memory>
#include <random>
#include <set>
#include <string>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/state/state.h"

namespace planning {

/// @class RoutePlanner
/// @brief 导航路径规划器——在车道拓扑网络中生成宏观导航路径
///
/// 继承自 Planner 基类，遵循统一规划器接口。
/// 在车道网络中使用随机展开策略生成车道ID序列路径，
/// 作为下游规划器的全局导航指引。
class RoutePlanner : public Planner {
 public:
  /// @brief 导航模式枚举
  enum NaviMode {
    kRandomExpansion,  ///< 沿子车道链随机展开（主要模式，已实现）
    kAssignedTarget    ///< 指定目标车道（预留，未实现）
  };

  /// @brief 导航状态机
  enum NaviStatus {
    kReadyToGo,   ///< 就绪，开始生成新路径
    kInProgress,  ///< 路径执行中，持续检查进度
    kFinished     ///< 路径完成
  };

  std::string Name() override;            ///< 返回规划器名称
  ErrorType Init(const std::string config) override;  ///< 初始化规划器
  ErrorType RunOnce() override;           ///< 执行一步规划（在回调中被周期性调用）

  // ======================== Setter ========================

  void set_navi_mode(const NaviMode& mode) { navi_mode_ = mode; };
  void set_ego_state(const common::State& state) { ego_state_ = state; }
  /// @brief 设置自车最近车道ID（由上游语义地图管理器计算）
  void set_nearest_lane_id(const int& id) { nearest_lane_id_ = id; }
  /// @brief 设置车道网络（同时设置 if_get_lane_net_ 标志）
  void set_lane_net(const common::LaneNet& lane_net) {
    lane_net_ = lane_net;
    if_get_lane_net_ = true;
  }

  // ======================== Getter ========================

  std::vector<int> navi_path() const { return navi_path_; }  ///< 导航车道ID序列
  common::Lane navi_lane() const { return navi_lane_; }      ///< 从路径拟合的连续长车道
  bool if_get_lane_net() const { return if_get_lane_net_; }  ///< 是否已设置车道网络
  decimal_t navi_cur_arc_len() const { return navi_cur_arc_len_; }  ///< 自车在导航车道上的弧长

 private:
  /// @brief 获取指定车道的子车道ID列表（用于随机展开）
  ErrorType GetChildLaneIds(const int lane_id, std::vector<int>* child_ids);

  /// @brief 通过随机展开生成导航路径（核心算法）
  ///
  /// 算法：
  ///   1. 以 nearest_lane_id_ 为起点
  ///   2. 循环获取当前车道的子车道列表
  ///   3. 从中随机选择一个子车道加入路径
  ///   4. 累计车道长度，直到 >= navi_path_max_length_
  ///   5. 从路径中所有车道段的采样点拟合连续长车道（navi_lane_）
  ///   6. 计算自车在导航车道上的弧长位置
  ErrorType GetNaviPathByRandomExpansion();

  /// @brief 检查是否到达目标车道（即 nearest_lane_id_ == navi_path_ 最后一个车道）
  bool CheckIfArriveTargetLane();

  /// @brief 检查导航进度——将状态机推进到 kFinished
  /// @note 当前实现直接标记为 kFinished（简化版，原本应检查进度比例）
  ErrorType CheckNaviProgress();

  /// @brief 随机展开模式的主循环（状态机驱动）
  ErrorType NaviLoopRandomExpansion();

  /// @brief 指定目标模式的主循环（未实现）
  ErrorType NaviLoopAssignedTarget();

  // ======================== 数据成员 ========================

  common::LaneNet lane_net_;      ///< 车道网络（拓扑+几何）
  common::State ego_state_;       ///< 自车状态
  int nearest_lane_id_;           ///< 自车最近车道ID

  NaviStatus navi_status_ = kReadyToGo;  ///< 当前导航状态
  NaviMode navi_mode_ = kRandomExpansion; ///< 当前导航模式

  bool if_restart_ = true;         ///< 是否在到达目标后自动重启规划
  bool if_get_lane_net_ = false;   ///< 是否已设置车道网络

  // decimal_t navi_path_max_length_ = 1500;  // 原始值（已弃用）
  decimal_t navi_path_max_length_ = 200;         ///< 导航路径最大长度（米）
  decimal_t navi_start_arc_length_{0.0};         ///< 自车在导航车道上的起始弧长
  // decimal_t navi_tail_remain_ = 200;           // 原始值（已弃用）
  decimal_t navi_path_length_{0.0};              ///< 导航路径总长度
  decimal_t navi_cur_arc_len_{0.0};              ///< 自车在导航车道上的当前弧长

  std::vector<int> navi_path_;     ///< 导航车道ID序列
  common::Lane navi_lane_;         ///< 从路径拟合的连续长车道

  std::random_device rd_gen_;      ///< 随机数生成器（用于随机展开）
};

}  // namespace planning

#endif  //_CORE_ROUTE_PLANNER_INC_ROUTE_PLANNER_H_
