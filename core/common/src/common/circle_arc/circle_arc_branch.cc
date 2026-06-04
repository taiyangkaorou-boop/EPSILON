/**
 * @file circle_arc_branch.cc
 * @brief 圆弧分支类的实现文件
 *
 * 本文件实现了EPSILON自动驾驶规划系统中的CircleArcBranch类，
 * 它从单个起始状态生成一组圆弧轨迹分支（不同曲率和弧长的组合）。
 *
 * ==================== 核心功能 ====================
 *
 * CircleArcBranch 表示从同一起始状态出发的一组轨迹候选：
 * - 每条轨迹是一段恒定曲率的圆弧（或直线，曲率=0）
 * - 不同的曲率和弧长组合形成"轨迹分支"（如转向不同方向或行驶不同距离）
 *
 * 典型应用场景：
 * - 运动原语生成：对连续动作空间进行离散采样
 * - 局部路径规划：生成待评估的候选轨迹集合
 * - 前向模拟：模拟车辆在一定时间后的可能状态
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/circle_arc/circle_arc_branch.h"

namespace common {

/*
 * RetFinalStates - 返回所有分支圆弧的终点状态
 *
 * 遍历所有生成的圆弧轨迹，收集每条圆弧的终点状态。
 * 这些终点状态可以用于后续的路径扩展或可达状态分析。
 *
 * @param p_states 输出参数，所有分支终点状态的向量
 */
void CircleArcBranch::RetFinalStates(std::vector<Vec3f> *p_states) const {
  for (const auto &arc : circle_arc_vec_) {
    p_states->emplace_back(arc.final_state());
  }
}

/*
 * RetAllSampledStates - 返回所有分支圆弧的采样状态
 *
 * 对每条圆弧按0.2弧长步长进行均匀采样，收集所有采样点的状态。
 * 用于可视化和后续的轨迹分析。
 *
 * @param p_states 输出参数，所有分支所有采样点的状态向量
 */
void CircleArcBranch::RetAllSampledStates(std::vector<Vec3f> *p_states) const {
  for (const auto &arc : circle_arc_vec_) {
    std::vector<Vec3f> samples;
    arc.GetSampledStates(0.2, &samples);  // 以0.2弧长步长采样
    for (const auto &state : samples) {
      p_states->emplace_back(state);
    }
  }
}

/*
 * CalculateCircleArcBranch - 根据曲率和弧长向量生成所有圆弧分支
 *
 * 算法流程：
 * 1. 检查曲率向量和弧长向量的长度是否一致
 * 2. 对每一对 (curvature, arc_length) 调用 CircleArc 构造函数
 * 3. 将生成的圆弧对象存入 circle_arc_vec_
 *
 * 第i个圆弧分支使用 curvature_vec_[i] 和 length_vec_[i] 作为参数，
 * 所有分支共享同一起始状态 start_state_。
 *
 * 例如：curvature_vec_ = {-0.1, 0.0, 0.1}, length_vec_ = {10, 10, 10}
 * 会生成三个分支：左转圆弧、直行、右转圆弧，各长10米。
 */
void CircleArcBranch::CalculateCircleArcBranch() {
  int n_arcs = curvature_vec_.size();  // 分支数量 = 曲率向量的长度
  if ((int)length_vec_.size() != n_arcs) {
    std::cerr << "[CircleArcBranch] ERROR - Size of vec are not equal"
              << std::endl;
    assert(false);  // 曲率向量和弧长向量的长度必须一致
  }
  for (int i = 0; i < n_arcs; ++i) {
    CircleArc arc(start_state_, curvature_vec_[i], length_vec_[i]);
    circle_arc_vec_.emplace_back(arc);
  }
}

}  // namespace common
