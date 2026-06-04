/**
 * @file frenet_primitive.cc
 * @brief Frenet坐标系运动原语的实现文件
 *
 * 本文件实现了EPSILON自动驾驶规划系统中的 FrenetPrimitive 类，
 * 它描述在Frenet坐标系中一条由起止状态定义的轨迹片段（运动原语）。
 *
 * ==================== FrenetPrimitive 核心概念 ====================
 *
 * FrenetPrimitive 表示Frenet坐标系中两个状态之间的连接轨迹：
 *   - 纵向(s方向)：通常使用加加速度最优（Jerk-Optimal）的五次多项式
 *   - 横向(d方向)：同样是五次多项式，但有两种参数化方式
 *
 * ==================== 两种横向参数化模式 ====================
 *
 * 1. 横向时间独立模式 (is_lateral_independent_ = true)：
 *    d(t) 直接参数化为时间的函数：d(t), d'(t), d''(t)
 *    横向多项式的时间范围与纵向一致（duration_ = T）。
 *
 * 2. 横向空间依赖模式 (is_lateral_independent_ = false)：
 *    d(s) 参数化为纵向弧长 s 的函数：d(s), d'(s), d''(s)
 *    横向多项式的定义域为 [s0, s1]，即沿纵向的弧长增量。
 *    这意味着横向偏移是沿"走了多远"来定义的，而非时间。
 *
 * ==================== 纵向-横向解耦的规划框架 ====================
 *
 * 在Frenet框架下，轨迹规划被解耦为两个一维问题：
 *   纵向(s): 决定"走多快"（速度剖面）
 *   横向(d): 决定"走哪条路"（变道/保持）
 *
 * FrenetPrimitive 同时包含这两个方向的多项式，从而
 * 完全定义了一段Frenet轨迹。
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/primitive/frenet_primitive.h"

namespace common {

/*
 * Connect - 在Frenet坐标系中连接两个状态，生成轨迹原语
 *
 * 对于纵向(s)和横向(d)分别构造加加速度最优的五次多项式。
 *
 * 纵向连接：使用 poly_s_.GetJerkOptimalConnection 生成 s(t) 多项式
 *   满足起止点约束 s(0)=s0, s'(0)=v0, s''(0)=a0, s(T)=s1, s'(T)=v1, s''(T)=a1
 *
 * 横向连接有两种模式：
 *   - 时间独立模式：d(t) 多项式，范围 t∈[0, T]
 *   - 空间依赖模式：d(s) 多项式，范围 s∈[s0, s1]
 *
 * 空间依赖模式中的特殊处理：
 *   当纵向位移 s1-s0 < kSmallDistanceThreshold_ 时，
 *   使用虚拟大距离(100.0)构造横向多项式，避免数值问题。
 *
 * @param fs0 起始Frenet状态（包含纵向和横向的状态信息）
 * @param fs1 终点Frenet状态
 * @param stamp 起始时间戳
 * @param T 时间长度（秒）
 * @param is_lateral_independent 横向是否时间独立（true=时间参数化, false=空间参数化）
 * @return kSuccess
 */
ErrorType FrenetPrimitive::Connect(const FrenetState& fs0,
                                   const FrenetState& fs1,
                                   const decimal_t stamp, const decimal_t T,
                                   bool is_lateral_independent) {
  is_lateral_independent_ = is_lateral_independent;

  // 纵向(s): 加加速度最优五次多项式时间连接
  poly_s_.GetJerkOptimalConnection(fs0.vec_s(0), fs0.vec_s(1), fs0.vec_s(2),
                                   fs1.vec_s(0), fs1.vec_s(1), fs1.vec_s(2), T);

  if (is_lateral_independent) {
    // 模式1: 横向时间独立 — d(t)，使用时间T
    poly_d_.GetJerkOptimalConnection(fs0.vec_dt(0), fs0.vec_dt(1),
                                     fs0.vec_dt(2), fs1.vec_dt(0),
                                     fs1.vec_dt(1), fs1.vec_dt(2), T);
  } else {
    // 模式2: 横向空间依赖 — d(s)，使用纵向弧长增量
    if (fs1.vec_s[0] - fs0.vec_s[0] < kSmallDistanceThreshold_) {
      // 纵向位移过小，使用虚拟大距离避免数值奇异性
      const decimal_t virtual_large_distance = 100.0;
      poly_d_.GetJerkOptimalConnection(
          fs0.vec_ds(0), fs0.vec_ds(1), fs0.vec_ds(2), fs1.vec_ds(0),
          fs1.vec_ds(1), fs1.vec_ds(2), virtual_large_distance);
    } else {
      poly_d_.GetJerkOptimalConnection(
          fs0.vec_ds(0), fs0.vec_ds(1), fs0.vec_ds(2), fs1.vec_ds(0),
          fs1.vec_ds(1), fs1.vec_ds(2), fs1.vec_s[0] - fs0.vec_s[0]);
    }
  }

  stamp_ = stamp;
  fs0_ = fs0;
  duration_ = T;
  fs1_ = fs1;
  return kSuccess;
}

/*
 * Propagate - 从起始状态和恒定控制量传播生成轨迹原语
 *
 * 与 Connect 不同，Propagate 使用恒定控制量（纵向和横向的加速度）
 * 来生成轨迹，而非连接两个状态。
 *
 * 使用三次多项式（jerk=0，加速度恒定）:
 *   纵向: s(t) = s0 + v0*t + 0.5*u[0]*t^2  (u[0]=a_s)
 *   横向: d(t) = d0 + vd0*t + 0.5*u[1]*t^2  (u[1]=a_d)
 *
 * @param fs0 起始Frenet状态
 * @param u 控制量向量 [纵向加速度, 横向加速度]
 * @param stamp 起始时间戳
 * @param T 传播时间长度
 * @return kSuccess
 */
ErrorType FrenetPrimitive::Propagate(const FrenetState& fs0, const Vecf<2>& u,
                                     const decimal_t stamp, const decimal_t T) {
  is_lateral_independent_ = true;
  Vecf<6> coeff;
  // 纵向三次多项式系数 (jerk=0):
  //   0.5*a_s * t^2 + v0 * t + s0
  coeff << 0.0, 0.0, 0.0, u[0], fs0.vec_s[1], fs0.vec_s[0];
  poly_s_.set_coeff(coeff);
  // 横向三次多项式系数 (jerk=0):
  //   0.5*a_d * t^2 + vd0 * t + d0
  coeff << 0.0, 0.0, 0.0, u[1], fs0.vec_dt[1], fs0.vec_dt[0];
  poly_d_.set_coeff(coeff);
  duration_ = T;
  stamp_ = stamp;
  fs0_ = fs0;
  GetFrenetState(stamp + T, &fs1_);  // 预计算终点状态
  return kSuccess;
}

/*
 * GetFrenetState - 在指定全局时间获取Frenet状态
 *
 * 将全局时间 t_global 转换为原语内的相对时间 t = t_global - stamp_，
 * 然后计算该时刻的Frenet状态。
 *
 * 时间独立模式（dt模式）:
 *   直接使用 poly_s_(t) 和 poly_d_(t) 求值
 *
 * 空间依赖模式（ds模式）:
 *   纵向 s = poly_s_(t, 0)，然后计算相对弧长增量 st = s - s0
 *   横向 d = poly_d_(st, 0)，使用弧长增量 st 而非时间 t 来求值
 *
 * @param t_global 全局时间戳
 * @param fs 输出参数，计算得到的Frenet状态
 * @return kSuccess；kWrongStatus 持续时间为零
 */
ErrorType FrenetPrimitive::GetFrenetState(const decimal_t t_global,
                                          FrenetState* fs) const {
  if (duration_ < kEPS) return kWrongStatus;
  auto t = t_global - stamp_;  // 转换为相对时间
  if (is_lateral_independent_) {
    // dt模式：纵向和横向都是时间的函数
    fs->Load(Vecf<3>(poly_s_.evaluate(t, 0), poly_s_.evaluate(t, 1),
                     poly_s_.evaluate(t, 2)),
             Vecf<3>(poly_d_.evaluate(t, 0), poly_d_.evaluate(t, 1),
                     poly_d_.evaluate(t, 2)),
             FrenetState::kInitWithDt);
    fs->time_stamp = t_global;
  } else {
    // ds模式：横向是纵向弧长s的函数
    auto s = poly_s_.evaluate(t, 0);          // 当前纵向位置
    auto st = s - fs0_.vec_s[0];              // 相对于起始点的弧长增量
    fs->Load(Vecf<3>(s, poly_s_.evaluate(t, 1), poly_s_.evaluate(t, 2)),
             Vecf<3>(poly_d_.evaluate(st, 0),   // 用st而非t来求横向多项式
                     poly_d_.evaluate(st, 1),
                     poly_d_.evaluate(st, 2)),
             FrenetState::kInitWithDs);
    fs->time_stamp = t_global;
  }
  return kSuccess;
}

/*
 * GetFrenetStateSamples - 在时间范围内均匀采样Frenet状态
 *
 * 从 begin()+offset 到 end()，以 step 为步长均匀采样。
 * 采样数量预估 = (end - begin - offset) / step + 10（包含余量）。
 *
 * @param step 采样时间步长
 * @param offset 起始偏移（跳过前offset秒的样本）
 * @param fs_vec 输出参数，采样状态向量
 * @return kSuccess
 */
ErrorType FrenetPrimitive::GetFrenetStateSamples(
    const decimal_t step, const decimal_t offset,
    vec_E<FrenetState>* fs_vec) const {
  FrenetState fs;
  fs_vec->clear();
  int num_samples_esti =
      static_cast<int>((end() - begin() - offset) / step) + 10;
  fs_vec->reserve(num_samples_esti);
  for (decimal_t t = begin() + offset; t < end(); t += step) {
    if (GetFrenetState(t, &fs) == kSuccess) {
      fs_vec->push_back(fs);
    }
  }
  return kSuccess;
}

/*
 * GetJ - 获取纵向和横向的加加速度（Jerk）平方积分代价
 *
 * 加加速度平方积分: J = integral_{0}^{T} (d³p/dt³)² dt
 *
 * 纵向J: poly_s_.J(duration_, 3) — 三阶导数的平方积分
 * 横向J:
 *   - dt模式: poly_d_.J(duration_, 3)
 *   - ds模式: poly_d_.J(fs1_.vec_s[0] - fs0_.vec_s[0], 3)
 *             （因为横向多项式的自变量是弧长s而非时间t）
 *
 * @param c_s 输出参数，纵向加加速度代价
 * @param c_d 输出参数，横向加加速度代价
 * @return kSuccess
 */
ErrorType FrenetPrimitive::GetJ(decimal_t* c_s, decimal_t* c_d) const {
  *c_s = poly_s_.J(duration_, 3);
  if (is_lateral_independent_) {
    *c_d = poly_d_.J(duration_, 3);
  } else {
    *c_d = poly_d_.J(fs1_.vec_s[0] - fs0_.vec_s[0], 3);
  }
  return kSuccess;
}

/*
 * lateral_T - 获取横向多项式的有效时间/空间范围
 *
 * dt模式: 返回时间长度 duration_
 * ds模式: 返回纵向弧长增量 fs1_.vec_s[0] - fs0_.vec_s[0]
 *
 * @return 横向多项式的参数域长度
 */
decimal_t FrenetPrimitive::lateral_T() const {
  if (is_lateral_independent_) {
    return duration_;
  } else {
    return fs1_.vec_s[0] - fs0_.vec_s[0];
  }
}

/*
 * longitudial_T - 获取纵向多项式的有效时间范围
 *
 * 纵向总是时间参数化的，因此返回 duration_。
 *
 * @return 纵向多项式的持续时间
 */
decimal_t FrenetPrimitive::longitudial_T() const { return duration_; }

// 获取终点Frenet状态
FrenetState FrenetPrimitive::fs1() const { return fs1_; }

// 获取起点Frenet状态
FrenetState FrenetPrimitive::fs0() const { return fs0_; }

}  // namespace common
