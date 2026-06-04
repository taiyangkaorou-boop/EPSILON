/**
 * @file circle_arc_branch.h
 * @brief 圆弧分支（Circle Arc Branch）
 *
 * 圆弧分支是以给定起始状态为根节点，通过组合不同的曲率和弧长，
 * 生成一棵圆弧运动树的结构。这对应于自动驾驶中的"前向仿真"概念：
 * 从当前车辆状态出发，考虑不同的转向角（曲率）和行驶时间（弧长），
 * 预测车辆可能到达的未来状态集合。
 *
 * 分支结构：
 *   - 起始状态（根）：当前的 (x, y, theta)
 *   - 曲率集合：如 [-0.2, -0.1, 0, 0.1, 0.2] 对应左转、直行、右转
 *   - 弧长集合：如 [10, 20, 30] 对应不同预测时长
 *   - 结果：曲率数 x 弧长数 条圆弧
 *
 * 在本项目中的应用：
 *   - EUDM 规划器的前向仿真：生成自车及周围车辆的未来可能轨迹
 *   - 行为预测：预测周围车辆未来数秒内的可能运动状态
 *   - 可视化：在 RViz 中展示车辆的可能行驶路径
 *
 * @author ZHANG Lu (lzhangbz@connect.ust.hk)
 * @date
 */

#ifndef _CORE_COMMON_INC_COMMON_CIRCLE_ARC_BRANCH_H_
#define _CORE_COMMON_INC_COMMON_CIRCLE_ARC_BRANCH_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/circle_arc/circle_arc.h"

namespace common {

/**
 * @class CircleArcBranch
 * @brief 圆弧分支集合
 *
 * 从同一起始状态出发，生成一组对应不同曲率和弧长的圆弧。
 * 每对 (curvature, arc_length) 生成一条独立的 CircleArc。
 */
class CircleArcBranch {
 public:
  /// 默认构造函数
  CircleArcBranch();

  /**
   * @brief 带参构造函数
   *
   * 从起始状态、曲率向量和弧长向量创建圆弧分支。
   * 曲率向量和弧长向量的笛卡尔积构成所有生成的圆弧。
   *
   * @param start_state 起始状态 (x, y, theta)
   * @param curvature_vec 曲率向量 [1/m]，例如 {-0.2, -0.1, 0, 0.1, 0.2}
   * @param length_vec 弧长向量 [m]，例如 {10, 20, 30}
   *
   * @note 生成的圆弧总数 = curvature_vec.size() * length_vec.size()
   */
  CircleArcBranch(const Vec3f &start_state,
                  const std::vector<double> &curvature_vec,
                  const std::vector<double> length_vec)
      : start_state_(start_state),
        curvature_vec_(curvature_vec),
        length_vec_(length_vec) {
    CalculateCircleArcBranch();  // 构造时立即计算所有圆弧
  }

  /// 析构函数
  ~CircleArcBranch() {}

  /// @name 属性访问器
  /// @{
  inline Vec3f start_state() const { return start_state_; }              ///< 起始状态
  inline std::vector<double> curvature_vec() const { return curvature_vec_; }  ///< 曲率向量
  inline std::vector<double> length_vec() const { return length_vec_; }        ///< 弧长向量
  inline std::vector<CircleArc> circle_arc_vec() const {
    return circle_arc_vec_;
  }   ///< 生成的圆弧集合
  /// @}

  /**
   * @brief 返回所有圆弧的终止状态
   *
   * 可用于获取预测车辆在各圆弧末端的位置和朝向。
   *
   * @param[out] p_states 输出的终止状态向量，每个元素为 (x, y, theta)
   */
  void RetFinalStates(std::vector<Vec3f> *p_states) const;

  /**
   * @brief 返回所有圆弧的均匀采样状态
   *
   * 对所有圆弧进行离散采样，合并为一个状态向量。
   * 常用于可视化或批量碰撞检测。
   *
   * @param[out] p_states 输出的全部采样状态向量
   */
  void RetAllSampledStates(std::vector<Vec3f> *p_states) const;

 private:
  /**
   * @brief 计算圆弧分支
   *
   * 遍历 curvature_vec_ x length_vec_ 的笛卡尔积，
   * 为每一对 (curvature, length) 创建一个 CircleArc 对象。
   */
  void CalculateCircleArcBranch();

  Vec3f start_state_;                     ///< 起始状态 (x, y, theta)
  std::vector<double> curvature_vec_;     ///< 曲率集合 [1/m]
  std::vector<double> length_vec_;        ///< 弧长集合 [m]
  std::vector<CircleArc> circle_arc_vec_; ///< 生成的圆弧对象集合
};

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_CIRCLE_ARC_BRANCH_H_
