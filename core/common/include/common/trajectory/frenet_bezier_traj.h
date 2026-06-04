/**
 * @file frenet_bezier_traj.h
 * @brief 基于贝塞尔（Bezier）曲线的 Frenet 轨迹
 *
 * 该文件定义了使用贝塞尔样条曲线在 Frenet 坐标系下参数化轨迹的实现类。
 * 贝塞尔曲线具有以下特性，使其特别适合自动驾驶轨迹规划：
 *   - 凸包性质：曲线始终位于控制点的凸包内，便于进行碰撞检测
 *   - 变差缩减性：曲线不会比控制多边形产生更多的波动
 *   - 端点插值：曲线精确通过首尾控制点，方便满足起止状态约束
 *   - C^n 连续性：高阶贝塞尔曲线保证位置、速度、加速度的连续
 *
 * 在本项目中，FrenetBezierTrajectory 是 SSC 规划器（SscPlanner）的
 * 主要输出格式。SSC 规划器通过采样换道时间和纵向行驶时间，然后使用
 * 贝塞尔曲线生成平滑的换道轨迹。
 *
 * 轨迹结构：
 *   - 纵向（s 方向）：对应贝塞尔曲线的一个维度，描述沿道路弧长的行进
 *   - 横向（d 方向）：对应贝塞尔曲线的另一个维度，描述相对参考线的横向偏移
 *   - 内部包含 StateTransformer，用于在 Frenet 坐标和笛卡尔坐标之间转换
 *
 * @author EPSILON Autonomous Driving Team
 * @date 2026
 */

#ifndef _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_BEZIER_TRAJ_H__
#define _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_BEZIER_TRAJ_H__

#include "common/basics/config.h"
#include "common/spline/bezier.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "common/trajectory/frenet_traj.h"

namespace common {

/**
 * @class FrenetBezierTrajectory
 * @brief 贝塞尔 Frenet 轨迹类
 *
 * 使用二维贝塞尔样条曲线在 Frenet 坐标系中表示轨迹。
 * 第一维对应纵向 (s)，第二维对应横向 (d)。
 *
 * @note 类型定义：BezierTrajectory = BezierSpline<TrajectoryDegree, TrajectoryDim>
 */
class FrenetBezierTrajectory : public FrenetTrajectory {
 public:
  /// 贝塞尔轨迹类型别名：TrajectoryDegree 阶贝塞尔样条，TrajectoryDim 维
  using BezierTrajectory = BezierSpline<TrajectoryDegree, TrajectoryDim>;

  /// 默认构造函数，创建无效轨迹（is_valid_ = false）
  FrenetBezierTrajectory() {}

  /**
   * @brief 从贝塞尔样条和状态转换器构造有效轨迹
   *
   * @param bezier_spline 已配置的贝塞尔样条曲线（包含完整的 (s, d) 参数化）
   * @param stf 状态转换器，用于 Frenet 坐标 <-> 笛卡尔坐标的转换
   *
   * @note 构造后 is_valid_ 自动设为 true
   */
  FrenetBezierTrajectory(const BezierTrajectory& bezier_spline,
                         const StateTransformer& stf)
      : bezier_spline_(bezier_spline), stf_(stf), is_valid_(true) {}

  /// @name 时间范围接口
  /// @{
  decimal_t begin() const override { return bezier_spline_.begin(); }
  decimal_t end() const override { return bezier_spline_.end(); }
  /// @}

  bool IsValid() const override { return is_valid_; }

  /**
   * @brief 获取指定时刻的笛卡尔坐标系状态
   *
   * 先在 Frenet 坐标系下采样贝塞尔曲线获取 (s, d, s', d', s'', d'')，
   * 然后通过 StateTransformer 转换为笛卡尔坐标系的 State。
   * 速度会被钳制为非负值。
   *
   * @param[in] t 查询时刻 [s]
   * @param[out] state 输出的笛卡尔状态
   * @return ErrorType kSuccess 或 kWrongStatus（超出范围）
   */
  ErrorType GetState(const decimal_t& t, State* state) const override {
    if (t < begin() - kEPS || t > end() + kEPS) return kWrongStatus;
    common::FrenetState fs;
    if (GetFrenetState(t, &fs) != kSuccess) {
      return kWrongStatus;
    }
    if (stf_.GetStateFromFrenetState(fs, state) != kSuccess) {
      return kWrongStatus;
    }
    state->velocity = std::max(0.0, state->velocity);  // 速度不允许为负
    return kSuccess;
  }

  /**
   * @brief 获取指定时刻的 Frenet 坐标系状态
   *
   * 对贝塞尔样条的 s 和 d 维度分别进行 0 阶（位置）、1 阶（速度）、2 阶（加速度）
   * 采样，组装为 FrenetState 对象。
   *
   * @param[in] t 查询时刻 [s]
   * @param[out] fs 输出的 Frenet 状态
   * @return ErrorType kSuccess 或 kWrongStatus
   */
  ErrorType GetFrenetState(const decimal_t& t, FrenetState* fs) const override {
    if (t < begin() - kEPS || t > end() + kEPS) return kWrongStatus;
    Vecf<2> pos, vel, acc;
    bezier_spline_.evaluate(t, 0, &pos);   // 第 0 阶：位置 (s, d)
    bezier_spline_.evaluate(t, 1, &vel);   // 第 1 阶：速度 (s', d')
    bezier_spline_.evaluate(t, 2, &acc);   // 第 2 阶：加速度 (s'', d'')
    // 默认使用 kInitWithDt 模式（以时间 t 为参数）
    fs->time_stamp = t;
    fs->Load(Vec3f(pos[0], vel[0], acc[0]), Vec3f(pos[1], vel[1], acc[1]),
             common::FrenetState::kInitWithDt);
    // 如果 ds 不可用（速度为零或接近零），回退到 kInitWithDs 模式
    if (!fs->is_ds_usable) {
      fs->Load(Vec3f(pos[0], 0.0, 0.0), Vec3f(pos[1], 0.0, 0.0),
               FrenetState::kInitWithDs);
    }
    return kSuccess;
  }

  std::vector<decimal_t> variables() const override {
    // TODO: 待实现 — 将贝塞尔控制点导出为优化变量
    return std::vector<decimal_t>();
  }

  void set_variables(const std::vector<decimal_t>& variables) override {
    // TODO: 待实现 — 从优化变量重建贝塞尔控制点
  }

  virtual void Jerk(decimal_t* j_lon, decimal_t* j_lat) const override {
    // TODO: 待实现 — 计算贝塞尔曲线的三阶导数
  }

 private:
  BezierTrajectory bezier_spline_;  ///< 贝塞尔样条轨迹（s 和 d 两个维度）
  StateTransformer stf_;            ///< Frenet <-> 笛卡尔坐标转换器
  bool is_valid_ = false;           ///< 轨迹有效性标志
};

}  // namespace common

#endif
