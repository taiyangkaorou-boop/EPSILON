/**
 * @file dcp_tree.h
 * @author EPSILON Autonomous Driving Team
 * @brief DCP-Tree（离散-连续规划树）——EUDM规划器的动作序列生成器
 * @version 0.1
 * @date 2019-07-07
 * @copyright Copyright (c) 2019
 *
 * ## DCP-Tree 概述
 *
 * DCP-Tree（Discrete-Continuous Planning Tree）是EUDM系统中负责任成候选
 * 动作序列的核心组件。它构建一个层次化、离散-连续混合的动作规划树，
 * 用于生成所有可能的高层行为序列。
 *
 * ## 树的拓扑结构
 *
 * 树的每一层代表一个固定时长的时间片（layer_time），整个树的深度
 * 由tree_height决定，规划时域 = tree_height * layer_time（最后一层
 * 可以有独立的时长last_layer_time）。
 *
 * ### 每层包含的候选动作组合
 * - **横向动作（DcpLatAction）**：车道保持(K)、左变道(L)、右变道(R) —— 3种
 * - **纵向动作（DcpLonAction）**：维持速度(M)、加速(A)、减速(D) —— 3种（代码中MAX_COUNT=3，但实际定义只有3个值）
 * - **每层组合数** = 横向 × 纵向 = 3 × 3 = 9种（不含重复组合优化）
 *
 * ### 动作序列生成策略
 * 序列从"当前正在执行的动作"（ongoing_action）开始，后续层中的横向动作
 * 只能从与ongoing_action.lat不同的方向中选择（避免同时向左又向右）。
 * 纵向动作在整个序列中保持一致（第一层选定的lon用于所有层）。
 *
 * ### 示例
 * ongoing_action = (lon=Maintain, lat=Keep), tree_height=3, layer_time=1s
 * 可能生成的部分脚本：
 * - MKK (1s维持速度+保持, 1s维持速度+左变道, 1s维持速度+保持)
 * - MKR (1s维持速度+保持, 1s维持速度+右变道, 1s维持速度+保持)
 * - AKK (1s加速+保持, ...)
 * - DKL (1s减速+保持, ...)
 *
 * @see eudm_planner.h EudmPlanner使用DcpTree生成的动作序列进行仿真评估
 */

#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_TREE_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_TREE_H_

#include <map>
#include <memory>
#include <string>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"

namespace planning {

/// @class DcpTree
/// @brief DCP-Tree（离散-连续规划树）——生成候选动作序列的层次化树结构
///
/// 该树是EUDM行为规划器的前端组件，负责根据当前正在执行的动作
/// 生成所有可能的候选动作序列（action_script），供EUDM进行
/// 多线程并行仿真和评估。
///
/// 树的结构为：根节点 = 当前进行中的动作，每层子树 = 该层所有可能的新动作。
/// 共生成 (横向动作数-1) × (tree_height-1) + 1 个动作序列（每种纵向动作下）。
///
/// ## 关键设计
/// - 纵向行为在整个序列中保持一致性（一个序列只有一种lon_action）
/// - 横向行为可以变化，但第一次出现不同方向后，后续层会连续使用该方向
/// - 最后一层可以有独立的时长，用于精确控制规划时域
class DcpTree {
 public:
  using LateralBehavior = common::LateralBehavior;

  /// @enum DcpLonAction
  /// @brief DCP纵向动作枚举
  ///
  /// 纵向动作控制车辆的加速/减速/维持行为，
  /// 实际效果通过调整IDM模型的期望速度和参数实现。
  enum class DcpLonAction {
    kMaintain = 0,   ///< 维持当前速度（IDM期望速度=当前速度）
    kAccelerate,      ///< 加速（IDM期望速度=当前速度+加速gap，减小最小间距和期望时距）
    kDecelerate,      ///< 减速（IDM期望速度=当前速度-减速gap）
    MAX_COUNT = 3     ///< 纵向动作总数（用于循环遍历）
  };

  /// @enum DcpLatAction
  /// @brief DCP横向动作枚举
  ///
  /// 横向动作控制车辆的车道变更意图。
  /// 在实际仿真中，横向动作影响目标车道的选择和车辆的前向仿真模式。
  enum class DcpLatAction {
    kLaneKeeping = 0,     ///< 保持当前车道
    kLaneChangeLeft,      ///< 向左变道
    kLaneChangeRight,     ///< 向右变道
    MAX_COUNT = 3         ///< 横向动作总数（用于循环遍历）
  };

  /// @struct DcpAction
  /// @brief DCP动作结构：横向+纵向+持续时间 的三元组
  ///
  /// 每个DcpAction定义了在一个时间层内自车的意图：
  /// - 纵向意图（加速/减速/维持）
  /// - 横向意图（保持/左变道/右变道）
  /// - 持续时间（该意图持续的秒数）
  ///
  /// 注意：lon和lat都是"意图"而非物理量，它们的实际物理效果
  /// 取决于前向仿真器如何使用这些意图来调整控制器参数。
  struct DcpAction {
    DcpLonAction lon = DcpLonAction::kMaintain;    ///< 纵向动作意图
    DcpLatAction lat = DcpLatAction::kLaneKeeping; ///< 横向动作意图
    decimal_t t = 0.0;                              ///< 该动作的持续时间（秒）

    /// @brief 流输出操作符，格式：(lon: 0, lat: 1, t: 2.000)
    friend std::ostream& operator<<(std::ostream& os, const DcpAction& action) {
      os << "(lon: " << static_cast<int>(action.lon)
         << ", lat: " << static_cast<int>(action.lat) << ", t: " << action.t
         << ")";
      return os;
    }

    DcpAction() {}
    DcpAction(const DcpLonAction& lon_, const DcpLatAction& lat_,
              const decimal_t& t_)
        : lon(lon_), lat(lat_), t(t_) {}
  };

  /// @brief 构造函数：全层使用统一时长
  /// @param tree_height 树的层数（包括根节点层），即动作序列长度
  /// @param layer_time 每层的时间长度（秒），最后一层也将使用该值
  DcpTree(const int& tree_height, const decimal_t& layer_time);

  /// @brief 构造函数：指定最后一层独立时长
  /// @param tree_height 树的层数
  /// @param layer_time 前N-1层的时间长度（秒）
  /// @param last_layer_time 最后一层的时间长度（秒），用于精确控制规划时域
  DcpTree(const int& tree_height, const decimal_t& layer_time,
          const decimal_t& last_layer_time);

  ~DcpTree() = default;

  /// @brief 设置当前正在进行中的动作
  ///
  /// 该动作将作为所有生成的动作序列的根动作（第一层）。
  /// 后续层的横向动作不能与该动作的横向方向冲突。
  void set_ongoing_action(const DcpAction& a) { ongoing_action_ = a; }

  /// @brief 获取所有生成的动作序列（动作脚本）
  ///
  /// 返回二维数组：action_script_[i][j] = 第i个序列的第j个动作
  std::vector<std::vector<DcpAction>> action_script() const {
    return action_script_;
  }

  /// @brief 获取规划时域（所有层时间之和，即第一个序列的总时长）
  decimal_t planning_horizon() const;

  /// @brief 获取树的层数
  int tree_height() const { return tree_height_; }

  /// @brief 获取每层的仿真时间
  decimal_t sim_time_per_layer() const { return layer_time_; }

  /// @brief 更新动作脚本（重新生成所有动作序列）
  ///
  /// 当ongoing_action变化时需要调用此函数。
  /// 内部调用GenerateActionScript()重新生成。
  ErrorType UpdateScript();

  /// @brief 返回纵向动作的简称字符串
  /// @return "M"（维持）、"A"（加速）、"D"（减速）、"Null"（无效）
  static std::string RetLonActionName(const DcpLonAction a) {
    std::string a_str;
    switch (a) {
      case DcpLonAction::kMaintain: {
        a_str = std::string("M");
        break;
      }
      case DcpLonAction::kAccelerate: {
        a_str = std::string("A");
        break;
      }
      case DcpLonAction::kDecelerate: {
        a_str = std::string("D");
        break;
      }
      default: {
        a_str = std::string("Null");
        break;
      }
    }
    return a_str;
  }

  /// @brief 返回横向动作的简称字符串
  /// @return "K"（保持）、"L"（左变道）、"R"（右变道）、"Null"（无效）
  static std::string RetLatActionName(const DcpLatAction a) {
    std::string a_str;
    switch (a) {
      case DcpLatAction::kLaneKeeping: {
        a_str = std::string("K");
        break;
      }
      case DcpLatAction::kLaneChangeLeft: {
        a_str = std::string("L");
        break;
      }
      case DcpLatAction::kLaneChangeRight: {
        a_str = std::string("R");
        break;
      }
      default: {
        a_str = std::string("Null");
        break;
      }
    }
    return a_str;
  }

 private:
  /// @brief 生成所有候选动作序列（action_script_）
  ///
  /// 核心生成算法：
  /// 1. 遍历所有纵向动作类型（Maintain/Accel/Decel）
  /// 2. 对于每种纵向动作，构建以此为纵向基础的动作序列
  /// 3. 序列第一层使用ongoing_action的横向方向
  /// 4. 对于树的每一层（从1到tree_height-1），尝试每个不同于ongoing的横向动作：
  ///    - 如果该层使用了不同的横向方向，后续所有层都沿用该方向
  ///    - 如果该层继续使用ongoing的横向方向，继续向下一层分支
  /// 5. 将所有序列的最后一层时间覆盖为last_layer_time_
  ErrorType GenerateActionScript();

  /// @brief 将动作a重复n次附加到序列seq_in末尾
  ///
  /// 用于批量扩展动作序列。例如：
  /// AppendActionSequence(KK, L, 3) => KKLLL
  ///
  /// @param seq_in 输入序列
  /// @param a 要追加的动作
  /// @param n 追加次数
  /// @return 扩展后的新序列
  std::vector<DcpAction> AppendActionSequence(
      const std::vector<DcpAction>& seq_in, const DcpAction& a,
      const int& n) const;

  int tree_height_ = 5;               ///< 树的层数（动作序列长度），默认5层
  decimal_t layer_time_ = 1.0;        ///< 每层的默认仿真时间（秒）
  decimal_t last_layer_time_ = 1.0;   ///< 最后一层的仿真时间（秒），可独立设置
  DcpAction ongoing_action_;           ///< 当前正在执行的动作（根动作）
  std::vector<std::vector<DcpAction>> action_script_;  ///< 所有生成的动作序列集合
};
}  // namespace planning

#endif  //  _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_TREE_H_
