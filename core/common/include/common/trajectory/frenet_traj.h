/**
 * @file frenet_traj.h
 * @brief Frenet 坐标系下的轨迹抽象基类
 *
 * Frenet 坐标系是自动驾驶轨迹规划中的核心概念。它以道路参考线为基准，
 * 使用纵向弧长 s 和横向偏移 d 来描述车辆位置，替代传统的笛卡尔坐标 (x, y)。
 *
 * Frenet 坐标系的优势：
 *   - 沿道路方向和横向偏移自然解耦，便于分别规划纵向速度曲线和横向换道曲线
 *   - 道路曲率被隐含在参考线中，简化了曲率约束的处理
 *   - 符合人类驾驶的直觉（沿着路走，左右变道）
 *
 * FrenetTrajectory 继承自 Trajectory，在其基础上增加了：
 *   - GetFrenetState(t)：直接获取 Frenet 坐标系下的状态
 *   - Jerk()：获取纵向和横向的 jerk（加加速度），用于舒适性评估
 *
 * 该基类被 FrenetBezierTrajectory 和 FrenetPrimitiveTrajectory 继承。
 *
 * @author EPSILON Autonomous Driving Team
 * @date 2026
 */

#ifndef _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_TRAJECTORY_H__
#define _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_TRAJECTORY_H__

#include "common/basics/config.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/trajectory/trajectory.h"

namespace common {

/**
 * @class FrenetTrajectory
 * @brief Frenet 坐标系轨迹抽象基类
 *
 * 继承 Trajectory，增加 Frenet 坐标系特有的接口。
 * 所有基于 Frenet 坐标系的轨迹类型都从此类派生。
 */
class FrenetTrajectory : public Trajectory {
 public:
  /// 虚析构函数
  virtual ~FrenetTrajectory() = default;

  // 继承自 Trajectory 的接口
  virtual ErrorType GetState(const decimal_t& t, State* state) const = 0;

  /**
   * @brief 获取指定时刻的 Frenet 坐标系状态
   *
   * 返回包含纵向弧长 s、横向偏移 d 及其各阶导数（速度、加速度）
   * 的 FrenetState 对象。
   *
   * @param[in] t 查询时刻 [s]
   * @param[out] fs 输出的 Frenet 状态对象
   * @return ErrorType kSuccess 表示查询成功
   */
  virtual ErrorType GetFrenetState(const decimal_t& t,
                                   FrenetState* fs) const = 0;

  // 继承自 Trajectory 的时间范围接口
  virtual decimal_t begin() const = 0;
  virtual decimal_t end() const = 0;
  virtual bool IsValid() const = 0;

  // 继承自 Trajectory 的优化接口
  virtual std::vector<decimal_t> variables() const = 0;
  virtual void set_variables(const std::vector<decimal_t>& variables) = 0;

  // ========== Frenet 特有接口 ==========

  /**
   * @brief 获取轨迹的 jerk（加加速度）
   *
   * jerk 是加速度的导数，反映加速度变化的速率，
   * 是衡量驾驶舒适性的重要指标。jerk 越小，乘坐越舒适。
   *
   * @param[out] j_lon 纵向 jerk（s 的三阶导数）[m/s^3]
   * @param[out] j_lat 横向 jerk（d 的三阶导数）[m/s^3]
   *
   * @note 在轨迹评估中，jerk 通常作为舒适性代价项出现在目标函数中
   */
  virtual void Jerk(decimal_t* j_lon, decimal_t* j_lat) const = 0;
};

}  // namespace common

#endif
