/**
 * @file dcp_tree.cc
 * @author EPSILON Autonomous Driving Team
 * @brief DCP-Tree（离散-连续规划树）实现
 * @version 0.1
 * @date 2019-07-07
 * @copyright Copyright (c) 2019
 *
 * 本文件实现了DcpTree类的核心功能——基于当前进行中的动作
 * 生成所有候选动作序列（action_script）。
 *
 * ## 动作序列生成算法
 *
 * 算法以"当前进行中的动作"（ongoing_action）作为所有序列的第一层（根节点）。
 * 对于每种纵向动作（Maintain/Accel/Decel），生成对应的一组动作序列：
 *
 * 1. 序列第一层：使用ongoing_action的lat方向 + 当前纵向动作
 * 2. 从第1层（索引1）到第tree_height-1层：
 *    - 在该层位置尝试每一个与ongoing_action.lat不同的横向方向
 *    - 若在某层选择了新方向，后续所有层都沿用该方向（一次性决策）
 *    - 若在该位置仍使用ongoing的lat方向，继续向下一层分支
 * 3. 最终生成一个全ongoing方向序列（无变道序列）和多个变道序列
 *
 * ### 具体示例
 *
 * ongoing_action = (lon=Maintain, lat=Keep), tree_height=5, layer_time=1s
 *
 * 生成的序列（以M为纵向、K为保持、L为左变道、R为右变道）：
 * - MKKKK (全程保持，各层分支后但保留ongoing方向的结果)
 * - MKLLL (第1层保持, 第2层开始全左变道)
 * - MKRRR (第1层保持, 第2层开始全右变道)
 * - MMKKK ... (类似，纵向为Maintain省略中间层的变化)
 *
 * 总共会生成 (3种纵向) * ((tree_height-1)*2 + 1) 个序列。
 * 例如tree_height=5时: 3 * (4*2 + 1) = 27个序列。
 */

#include "eudm_planner/dcp_tree.h"

namespace planning {

/// @brief 构造函数：全层统一时长
///
/// 创建DCP树后立即调用GenerateActionScript()生成初始动作脚本。
DcpTree::DcpTree(const int& tree_height, const decimal_t& layer_time)
    : tree_height_(tree_height), layer_time_(layer_time) {
  last_layer_time_ = layer_time_;
  GenerateActionScript();
}

/// @brief 构造函数：最后一层独立时长
///
/// 当规划时域不是layer_time的整数倍时使用此构造函数，
/// 通过last_layer_time调整最后一层的长度以精确控制总时域。
DcpTree::DcpTree(const int& tree_height, const decimal_t& layer_time,
                 const decimal_t& last_layer_time)
    : tree_height_(tree_height),
      layer_time_(layer_time),
      last_layer_time_(last_layer_time) {
  GenerateActionScript();
}

/// @brief 更新动作脚本——当ongoing_action变化时重新生成
ErrorType DcpTree::UpdateScript() { return GenerateActionScript(); }

/// @brief 辅助函数：将动作a重复n次追加到序列末尾
///
/// @param seq_in 已有的动作序列
/// @param a 要追加的动作
/// @param n 追加次数
/// @return 扩展后的新序列（原有序列+ n个动作a）
std::vector<DcpTree::DcpAction> DcpTree::AppendActionSequence(
    const std::vector<DcpAction>& seq_in, const DcpAction& a,
    const int& n) const {
  std::vector<DcpAction> seq = seq_in;
  for (int i = 0; i < n; ++i) {
    seq.push_back(a);
  }
  return seq;
}

/// @brief 生成所有候选动作序列的核心算法
///
/// 算法详解：
/// ```
/// 对于每种纵向动作 lon in {Maintain, Accelerate, Decelerate}:
///   初始化序列 ongoing_action_seq = [(lon, ongoing_action.lat, t)]
///   对于每个树层 h in [1, tree_height_):
///     对于每个不同的横向动作 lat in {Keep, ChangeLeft, ChangeRight}:
///       如果 lat != ongoing_action.lat（即不同于根动作的横向方向）:
///         创建分支序列 = 当前序列 + 重复(lon, lat, layer_time) * (tree_height_-h)
///         将该分支序列加入action_script_
///     将(lon, ongoing_action.lat, layer_time)追加到ongoing_action_seq
///   将ongoing_action_seq（全ongoing方向序列）加入action_script_
/// 将所有序列的最后一层时间覆盖为last_layer_time_
/// ```
///
/// 序列数量 = 3种纵向 * (1个全保持序列 + (tree_height_-1) * 2个变道分支)
ErrorType DcpTree::GenerateActionScript() {
  action_script_.clear();
  std::vector<DcpAction> ongoing_action_seq;
  // 遍历三种纵向动作类型：维持、加速、减速
  for (int lon = 0; lon < static_cast<int>(DcpLonAction::MAX_COUNT); lon++) {
    ongoing_action_seq.clear();
    // 第一层：纵向=当前lon类型，横向=ongoing_action的横向，时间=ongoing_action的剩余时间
    ongoing_action_seq.push_back(
        DcpAction(DcpLonAction(lon), ongoing_action_.lat, ongoing_action_.t));

    // 从第1层（索引1）开始分支
    for (int h = 1; h < tree_height_; ++h) {
      // 尝试每个可能与ongoing方向不同的横向动作
      for (int lat = 0; lat < static_cast<int>(DcpLatAction::MAX_COUNT);
           lat++) {
        if (lat != static_cast<int>(ongoing_action_.lat)) {
          // 在位置h选择了新的横向方向，后续所有层都沿用该方向
          auto actions = AppendActionSequence(
              ongoing_action_seq,
              DcpAction(DcpLonAction(lon), DcpLatAction(lat), layer_time_),
              tree_height_ - h);  // 剩余层数都用此lat
          action_script_.push_back(actions);
        }
      }
      // 在位置h继续使用ongoing的横向方向，进入下一层循环
      ongoing_action_seq.push_back(
          DcpAction(DcpLonAction(lon), ongoing_action_.lat, layer_time_));
    }
    // 将全ongoing横向方向的序列（无变道序列）加入脚本
    action_script_.push_back(ongoing_action_seq);
  }
  // 将所有序列的最后一层时间覆盖为last_layer_time_，
  // 用于精确控制规划总时域
  for (auto& action_seq : action_script_) {
    action_seq.back().t = last_layer_time_;
  }
  return kSuccess;
}

/// @brief 计算规划时域（总仿真时间）
///
/// 返回第一个序列（action_script_[0]）的所有层时间之和。
/// 所有序列的总时长相同。
decimal_t DcpTree::planning_horizon() const {
  if (action_script_.empty()) return 0.0;
  decimal_t planning_horizon = 0.0;
  for (const auto& a : action_script_[0]) {
    planning_horizon += a.t;
  }
  return planning_horizon;
}

}  // namespace planning
