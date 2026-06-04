/**
 * @file trajectory.h
 * @brief 轨迹抽象基类
 *
 * 定义了 EPSILON 自动驾驶系统中所有轨迹类型的统一抽象接口。
 * 轨迹是规划器输出的核心数据结构，描述了车辆在未来一段时间内
 * 的期望运动状态（位置、速度、加速度、航向角等）随时间的变化。
 *
 * 轨迹类层次结构：
 *   Trajectory (抽象基类)
 *   ├── FrenetTrajectory (Frenet 坐标系轨迹)
 *   │   ├── FrenetBezierTrajectory (贝塞尔曲线 Frenet 轨迹)
 *   │   └── FrenetPrimitiveTrajectory (基于运动基元的 Frenet 轨迹)
 *   └── [其他轨迹类型]
 *
 * 轨迹接口的核心功能：
 *   - GetState(t)：获取指定时刻的车辆状态
 *   - begin()/end()：获取轨迹的有效时间范围
 *   - IsValid()：检查轨迹是否有效（如是否通过碰撞检测）
 *   - variables()/set_variables()：获取/设置轨迹的可优化变量
 *     （用于基于优化的轨迹改进）
 *
 * @author EPSILON Autonomous Driving Team
 * @date 2026
 */

#ifndef _CORE_COMMON_INC_COMMON_TRAJECTORY_TRAJECTORY_H__
#define _CORE_COMMON_INC_COMMON_TRAJECTORY_TRAJECTORY_H__

#include "common/basics/config.h"
#include "common/state/state.h"

namespace common {

/**
 * @class Trajectory
 * @brief 轨迹抽象基类
 *
 * 所有轨迹类型的公共接口。派生类必须实现所有纯虚函数以提供
 * 完整的轨迹查询和修改能力。
 */
class Trajectory {
 public:
  /// 虚析构函数
  virtual ~Trajectory() = default;

  /**
   * @brief 获取指定时刻的车辆状态（笛卡尔坐标系）
   *
   * 在时间 t 处对轨迹进行采样，返回对应的车辆状态向量，
   * 包括位置 (x, y)、速度、加速度、航向角、曲率等信息。
   *
   * @param[in] t 查询时刻 [s]，应在 [begin(), end()] 范围内
   * @param[out] state 输出的车辆状态对象
   * @return ErrorType kSuccess 表示查询成功，kWrongStatus 表示 t 超出范围
   */
  virtual ErrorType GetState(const decimal_t& t, State* state) const = 0;

  /**
   * @brief 获取轨迹的起始时刻
   *
   * @return decimal_t 轨迹参数化的起始时间戳 [s]
   */
  virtual decimal_t begin() const = 0;

  /**
   * @brief 获取轨迹的结束时刻
   *
   * @return decimal_t 轨迹参数化的结束时间戳 [s]
   */
  virtual decimal_t end() const = 0;

  /**
   * @brief 检查轨迹是否有效
   *
   * 有效的轨迹应满足：无碰撞、满足动力学约束、满足交通规则等。
   *
   * @return true 轨迹有效，false 轨迹无效（不可用于执行）
   */
  virtual bool IsValid() const = 0;

  // ========== 优化相关接口 ==========

  /**
   * @brief 获取轨迹的可优化变量（返回副本）
   *
   * 将轨迹的内部表示（如多项式系数）导出为浮点数向量，
   * 供外部优化器（如 QP 求解器）进行进一步优化。
   *
   * @return std::vector<decimal_t> 优化变量的副本
   *
   * @note 变量的具体含义取决于派生类的具体实现
   */
  virtual std::vector<decimal_t> variables() const = 0;

  /**
   * @brief 设置轨迹的可优化变量
   *
   * 将优化后的变量向量写回轨迹的内部表示，
   * 实现轨迹的"热启动"或增量优化。
   *
   * @param variables 优化后的变量向量
   *
   * @note 调用者需确保变量维度与轨迹类型匹配
   */
  virtual void set_variables(const std::vector<decimal_t>& variables) = 0;
};

}  // namespace common

#endif
