/**
 * @file frenet_primitive_traj.h
 * @brief 基于运动基元（Motion Primitive）的 Frenet 轨迹
 *
 * 该文件定义了使用运动基元（FrenetPrimitive）在 Frenet 坐标系下
 * 参数化轨迹的实现类。运动基元是一种预定义的短时运动模式，通过
 * 组合多个基元可以生成复杂的轨迹。
 *
 * 与 FrenetBezierTrajectory 不同，本类使用五次多项式在纵向 (s) 和
 * 横向 (d) 上分别参数化，因此能够精确满足位置、速度和加速度的
 * 起止边界条件。
 *
 * 内部表示：
 *   - poly_s：五次多项式 Poly<5>，描述纵向参数化 s(t)
 *   - poly_d：五次多项式 Poly<5>，描述横向参数化 d(t)
 *   - 两个多项式的系数（各 6 个，共 12 个）构成可优化变量
 *
 * 在本项目中，此类被 EUDM 规划器（EudmPlanner）广泛使用，
 * 用于表示采样生成的轨迹候选项，并通过 variables()/set_variables()
 * 接口配合优化器进行轨迹平滑改进。
 *
 * @author EPSILON Autonomous Driving Team
 * @date 2026
 */

#ifndef _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_PRIMITIVE_TRAJ_H__
#define _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_PRIMITIVE_TRAJ_H__

#include "common/basics/config.h"
#include "common/primitive/frenet_primitive.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "common/trajectory/frenet_traj.h"

namespace common {

/**
 * @class FrenetPrimitiveTrajectory
 * @brief 基于运动基元的 Frenet 轨迹类
 *
 * 封装一个 FrenetPrimitive 运动基元，提供完整的 Trajectory 接口。
 * 内部使用两个五次多项式分别描述纵向和横向的参数化。
 */
class FrenetPrimitiveTrajectory : public FrenetTrajectory {
 public:
  /// 默认构造函数，创建无效轨迹
  FrenetPrimitiveTrajectory() {}

  /**
   * @brief 从运动基元和状态转换器构造有效轨迹
   *
   * @param primitive 已构建的 FrenetPrimitive 运动基元
   * @param stf 状态转换器，用于 Frenet <-> 笛卡尔坐标转换
   */
  FrenetPrimitiveTrajectory(const FrenetPrimitive& primitive,
                            const StateTransformer& stf)
      : primitive_(primitive), stf_(stf), is_valid_(true) {}

  /// @name 时间范围接口
  /// @{
  decimal_t begin() const override { return primitive_.begin(); }
  decimal_t end() const override { return primitive_.end(); }
  /// @}

  bool IsValid() const override { return is_valid_; }

  /**
   * @brief 获取指定时刻的笛卡尔坐标系状态
   *
   * 通过基元获取 Frenet 状态，再转换为笛卡尔状态。
   * 速度被钳制为非负值以确保物理可行性。
   *
   * @param[in] t 查询时刻 [s]
   * @param[out] state 输出的笛卡尔车辆状态
   * @return ErrorType
   */
  ErrorType GetState(const decimal_t& t, State* state) const override {
    if (t < begin() - kEPS || t > end() + kEPS) return kWrongStatus;
    FrenetState fs;
    if (primitive_.GetFrenetState(t, &fs) != kSuccess) {
      return kWrongStatus;
    }

    if (stf_.GetStateFromFrenetState(fs, state) != kSuccess) {
      return kWrongStatus;
    }
    state->velocity = std::max(0.0, state->velocity);  // 速度钳制为非负
    return kSuccess;
  }

  /**
   * @brief 获取指定时刻的 Frenet 坐标系状态
   *
   * 直接委托给内部的 FrenetPrimitive 对象。
   *
   * @param[in] t 查询时刻 [s]
   * @param[out] fs 输出的 Frenet 状态
   * @return ErrorType
   */
  ErrorType GetFrenetState(const decimal_t& t, FrenetState* fs) const override {
    if (t < begin() - kEPS || t > end() + kEPS) return kWrongStatus;
    if (primitive_.GetFrenetState(t, fs) != kSuccess) {
      return kWrongStatus;
    }
    return kSuccess;
  }

  /**
   * @brief 导出轨迹的优化变量（12 维向量）
   *
   * 将 s 和 d 两个方向五次多项式的系数（各 6 个，共 12 个）
   * 拼接为一个向量返回。前 6 个为 poly_s 的系数（从低次到高次），
   * 后 6 个为 poly_d 的系数。
   *
   * @return 12 维优化变量向量
   */
  std::vector<decimal_t> variables() const override {
    std::vector<decimal_t> variables(12);  // 2 个五次多项式 = 12 个系数
    Vecf<6> coeff_s = primitive_.poly_s().coeff();
    Vecf<6> coeff_d = primitive_.poly_d().coeff();
    for (int i = 0; i < 6; i++) {
      variables[i] = coeff_s[i];       // 前 6 个：纵向多项式系数
      variables[i + 6] = coeff_d[i];   // 后 6 个：横向多项式系数
    }
    return variables;
  }

  /**
   * @brief 从优化变量重建轨迹参数
   *
   * 将 12 维优化变量向量解析回两个五次多项式的系数，
   * 并更新内部的 FrenetPrimitive 对象。
   *
   * @param variables 12 维优化变量向量
   */
  void set_variables(const std::vector<decimal_t>& variables) override {
    assert(variables.size() == 12);  // 必须恰好 12 个变量
    Vecf<6> coeff_s, coeff_d;
    for (int i = 0; i < 6; i++) {
      coeff_s[i] = variables[i];       // 从优化变量恢复纵向多项式系数
      coeff_d[i] = variables[6 + i];   // 从优化变量恢复横向多项式系数
    }
    Polynomial<5> poly_s, poly_d;
    poly_s.set_coeff(coeff_s);
    poly_d.set_coeff(coeff_d);
    primitive_.set_poly_s(poly_s);  // 更新基元的纵向多项式
    primitive_.set_poly_d(poly_d);  // 更新基元的横向多项式
  }

  /**
   * @brief 获取轨迹的 jerk（加加速度）
   *
   * 委托给内部 FrenetPrimitive 的 GetJ 方法。
   *
   * @param[out] j_lon 纵向 jerk [m/s^3]
   * @param[out] j_lat 横向 jerk [m/s^3]
   */
  virtual void Jerk(decimal_t* j_lon, decimal_t* j_lat) const override {
    primitive_.GetJ(j_lon, j_lat);
  }

 private:
  FrenetPrimitive primitive_;  ///< 内部运动基元（五次多项式在 s 和 d 上的参数化）
  StateTransformer stf_;       ///< Frenet <-> 笛卡尔坐标转换器
  bool is_valid_ = false;      ///< 轨迹有效性标志
};

}  // namespace common

#endif
