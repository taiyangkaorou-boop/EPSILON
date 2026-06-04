/**
 * @file route_planner.cc
 * @author EPSILON Autonomous Driving Group
 * @brief RoutePlanner 类实现——导航路径规划
 *
 * @details
 * 该文件实现了导航路径规划器的所有核心逻辑：
 *
 * 1. 状态机驱动（NaviLoopRandomExpansion）：
 *    - kReadyToGo：触发 GetNaviPathByRandomExpansion 生成路径
 *    - kInProgress：调用 CheckNaviProgress 监控进度
 *    - kFinished：若 if_restart_ 为 true，自动回转到 kReadyToGo
 *
 * 2. 路径生成（GetNaviPathByRandomExpansion）：
 *    - 从 nearest_lane_id_ 出发，沿 child_id 链随机选择子车道
 *    - 每步随机在可用子车道中选择一个，累计路径长度
 *    - 达到 navi_path_max_length_（200m）或遇到无子车道时停止
 *    - 将所有车道的采样点提取出来，拟合为一条连续的长车道（navi_lane_）
 *    - 计算自车在导航车道上的弧长位置
 *
 * 3. 随机展开策略：
 *    - 使用 std::random_device 作为随机源
 *    - 在 n_child 个可用子车道中用 floor(random / (max/n)) 获得随机索引
 *    - 这种朴素策略可能导致路径一直向同一方向延伸，不检查目的地
 *
 * @note 当前 kAssignedTarget 模式未实现，所有逻辑走 kRandomExpansion
 */

#include "route_planner/route_planner.h"

namespace planning {

/// @brief 返回规划器名称
std::string RoutePlanner::Name() {
  return std::string("Generic route planner");
}

/// @brief 初始化规划器——重置为就绪状态
ErrorType RoutePlanner::Init(const std::string config) {
  navi_status_ = kReadyToGo;
  return kSuccess;
}

/// @brief 执行一步规划——状态机驱动的主入口
///
/// 根据 navi_mode_ 选择对应的状态机循环：
///   - kRandomExpansion -> NaviLoopRandomExpansion
///   - kAssignedTarget -> NaviLoopAssignedTarget（未实现）
///
/// 执行后检查 navi_lane_ 是否有效，若无效则返回 kWrongStatus
/// 并打印错误信息。
ErrorType RoutePlanner::RunOnce() {
  switch (navi_mode_) {
    case kRandomExpansion:
      NaviLoopRandomExpansion();
      break;
    case kAssignedTarget:
      NaviLoopAssignedTarget();
      break;
    default:
      assert(false);
  }
  // ~ 若导航车道无效，通知调用方
  if (!navi_lane_.IsValid()) {
    printf("[RP]Err - fail to output valid navi lane.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

/// @brief 随机展开模式的主循环——状态机驱动
///
/// 状态转换：
///   kReadyToGo -> GetNaviPathByRandomExpansion -> kInProgress
///   kInProgress -> CheckNaviProgress -> kFinished（推进）
///   kFinished -> if_restart_ -> kReadyToGo（循环）
ErrorType RoutePlanner::NaviLoopRandomExpansion() {
  switch (navi_status_) {
    case kReadyToGo:
      // 就绪状态：生成新的导航路径
      if (GetNaviPathByRandomExpansion() != kSuccess) {
        printf("[RP]Err - fail to find navi path.\n");
        break;
      }
      navi_status_ = kInProgress;
      break;

    case kInProgress:
      // 执行中：检查进度
      if (CheckNaviProgress() != kSuccess) {
        printf("[RP]Err - InProgress but fail to check navi progress.\n");
        printf("[RP]Switching back to kReadyToGo.\n");
        navi_status_ = kReadyToGo;  // 进度检查失败，回退到就绪
      }
      break;

    case kFinished:
      // 已完成：可选重启
      printf("[RP]Finish trip.\n");
      if (if_restart_) {
        printf("[RP]Restart mission.\n");
        navi_status_ = kReadyToGo;  // 自动重启，开始新路径
      }
      break;

    default:
      assert(false);
  }
  return kSuccess;
}

/// @brief 指定目标模式的主循环（未实现）
ErrorType RoutePlanner::NaviLoopAssignedTarget() { return kSuccess; }

/// @brief 通过随机展开生成导航路径——核心算法
///
/// 算法详细步骤：
///   1. 清空 navi_path_ 并推入 nearest_lane_id_ 作为起点
///   2. 循环获取当前车道 cur_id 的子车道列表
///   3. 若无子车道（n_child == 0），停止展开
///   4. 使用随机数生成器在 n_child 个候选中随机选择一个子车道
///   5. 将该子车道加入 navi_path_，累计其长度到 dist_acc
///   6. 重复步骤2-5直到 dist_acc >= navi_path_max_length_ (200m)
///   7. 收集 navi_path_ 中所有车道段的采样点
///   8. 从采样点拟合连续的长车道 navi_lane_
///   9. 计算自车在导航车道上的弧长位置
///      - navi_start_arc_length_：起始弧长
///      - navi_cur_arc_len_：当前弧长
///      - navi_path_length_：导航路径的有效总长度
///
/// @note 随机选择算法：
///       int rand_num = floor(rd_gen_() / (rd_gen_.max() / n_child))
///       此方法在 n_child 不能整除 rd_gen_.max() 时有轻微偏斜，
///       但在这个非均匀的导航场景下可接受。
ErrorType RoutePlanner::GetNaviPathByRandomExpansion() {
  navi_path_.clear();
  navi_path_.push_back(nearest_lane_id_);  // 起点 = 自车最近车道

  decimal_t dist_acc = 0;
  int cur_id = nearest_lane_id_;

  // 沿 child_id 链随机展开
  while (dist_acc < navi_path_max_length_) {
    std::vector<int> child_ids;
    GetChildLaneIds(cur_id, &child_ids);
    int n_child = static_cast<int>(child_ids.size());

    // ~ 若无子车道，停止展开（防止除零错误）
    if (n_child == 0) break;

    // 随机选择一个子车道
    int rand_num = std::floor(rd_gen_() / (rd_gen_.max() / n_child));
    int rand_id = child_ids[rand_num];

    dist_acc += lane_net_.lane_set.at(rand_id).length;
    navi_path_.push_back(rand_id);
    cur_id = rand_id;
  }

  // 收集所有车道段的采样点
  vec_Vecf<2> raw_samples;
  for (const auto &id : navi_path_) {
    // 第一个车道从第0点开始，后续车道从第1点开始（避免重复点）
    if (raw_samples.empty() &&
        (int)lane_net_.lane_set.at(id).lane_points.size() > 0) {
      raw_samples.push_back(lane_net_.lane_set.at(id).lane_points[0]);
    }
    for (int i = 1; i < (int)lane_net_.lane_set.at(id).lane_points.size();
         ++i) {
      raw_samples.push_back(lane_net_.lane_set.at(id).lane_points[i]);
    }
  }

  // 从采样点拟合连续长车道
  if (common::LaneGenerator::GetLaneBySamplePoints(raw_samples, &navi_lane_) !=
      kSuccess) {
    printf("[RP]Err - fail to fitting lane with %d samples.\n",
           static_cast<int>(raw_samples.size()));
    return kWrongStatus;
  }

  // 计算自车在导航车道上的弧长位置
  Vec2f pos(ego_state_.vec_position(0), ego_state_.vec_position(1));
  if (navi_lane_.GetArcLengthByVecPosition(pos, &navi_start_arc_length_) !=
      kSuccess) {
    printf("[RP]Err - fail to locate ego state on navi lane.\n");
    return kWrongStatus;
  }
  navi_lane_.GetArcLengthByVecPosition(pos, &navi_cur_arc_len_);

  // 计算有效导航长度：车道终点 - 自车起始弧长
  navi_path_length_ = navi_lane_.end() - navi_start_arc_length_;
  if (navi_path_length_ < 0.0) {
    printf("[PubgLane]Err - fail to get legal navi path length.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

/// @brief 检查导航进度——当前实现直接标记为 kFinished
///
/// @note 简化实现：不论自车实际到达何处，直接标记完成。
///        原始代码中有一个被注释掉的进度比例检查逻辑：
///        ratio = (navi_cur_arc_len_ - navi_start_arc_length_) / navi_path_length_
///        该逻辑本应在 ratio >= 1.0 时标记完成，但被注释替换为直接kFinished。
ErrorType RoutePlanner::CheckNaviProgress() {
  if (!navi_lane_.IsValid()) return kWrongStatus;
  if (navi_path_length_ < 0.0) return kWrongStatus;

  // 原始进度比例逻辑（已注释）：
  // decimal_t ratio =
  //     (navi_cur_arc_len_ - navi_start_arc_length_) / navi_path_length_;
  // std::cout << "[PubgLane] Navi progress ratio: " << ratio << std::endl;
  // if (ratio >= 0.0) {
  //   navi_status_ = kFinished;
  // }

  navi_status_ = kFinished;
  return kSuccess;
}

/// @brief 检查是否到达目标车道
///
/// 比较 nearest_lane_id_ 与 navi_path_ 的最后一个车道ID。
///
/// @note 在 kRandomExpansion 模式下，目标车道就是路径终点，
///        因此这个检查等价于判断自车是否到达路径末端。
bool RoutePlanner::CheckIfArriveTargetLane() {
  std::cout << "[rp] Nearest = " << nearest_lane_id_ << std::endl;
  if (nearest_lane_id_ == *navi_path_.rbegin()) {
    return true;
  } else {
    return false;
  }
}

/// @brief 获取指定车道的子车道ID列表
///
/// 直接从 lane_net_.lane_set 中查找 lane_id 对应的车道，
/// 并复制其 child_id 向量。
///
/// @param lane_id 源车道ID
/// @param child_ids [输出] 子车道ID列表
ErrorType RoutePlanner::GetChildLaneIds(const int lane_id,
                                        std::vector<int> *child_ids) {
  auto it = lane_net_.lane_set.find(lane_id);
  if (it == lane_net_.lane_set.end()) {
    return kWrongStatus;
  } else {
    // ~ 注意这里是直接赋值（assign），将 child_id 复制到输出
    child_ids->assign(it->second.child_id.begin(), it->second.child_id.end());
  }
  return kSuccess;
}

}  // namespace planning
