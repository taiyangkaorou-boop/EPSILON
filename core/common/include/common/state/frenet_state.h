/**
 * @file frenet_state.h
 * @brief Frenet 坐标系下的车辆状态表示
 *
 * 本文件定义了 EPSILON 自动驾驶规划系统中 Frenet 坐标系下的状态结构体。
 * Frenet 坐标系是自动驾驶轨迹规划中最核心的坐标系统之一，它将车辆运动
 * 分解为沿参考线方向（s 分量）和垂直于参考线方向（d 分量）两个独立的分量。
 *
 * Frenet 坐标系基本概念：
 *   - s 坐标: 沿参考线（通常是车道中心线）的弧长距离，表示车辆沿道路前进的程度
 *   - d 坐标: 车辆偏离参考线的横向距离，正值表示位于参考线左侧（以行进方向为准）
 *
 * 与笛卡尔坐标系的对比：
 *   笛卡尔坐标系 (x, y) 描述车辆在平面中的绝对位置，但在结构化道路场景中，
 *   沿路-横向的分解方式更为自然。Frenet 坐标系将非线性的道路几何转化为
 *   正交的 (s, d) 空间，使得轨迹规划问题可以在近似解耦的纵、横向空间中
 *   分别求解，大幅降低规划问题的复杂度。
 *
 * 两种参数化模式：
 *   - 关于时间 t 的参数化 (vec_dt): 高速模式，此时 d 分量的导数对 t 求导。
 *     这是默认模式，适用于中高速场景。
 *   - 关于弧长 s 的参数化 (vec_ds): 低速模式，此时 d 分量的导数对 s 求导。
 *     当车速较低时（接近零），dt 参数化会出现数值奇异性，ds 参数化更为稳定。
 */

#ifndef _COMMON_INC_COMMON_STATE_FRENET_STATE_H__
#define _COMMON_INC_COMMON_STATE_FRENET_STATE_H__

namespace common {

/**
 * @struct FrenetState
 * @brief Frenet 坐标系下的车辆运动状态结构体
 *
 * 该类封装了车辆在 Frenet 空间中的完整状态，包括沿参考线方向（s 分量）
 * 和横向偏离方向（d 分量）的位置、速度和加速度信息。Frenet 坐标系将
 * 非线性的道路几何转化为正交坐标，使得轨迹规划可以分解为纵向和横向
 * 两个相对独立的问题。
 *
 * vec_s 的索引约定（与 vec_dt、vec_ds 一致）：
 *   - 索引 0: 位置值（s 或 d）
 *   - 索引 1: 一阶导数值（速度，ds/dt 或 dd/dt 或 dd/ds）
 *   - 索引 2: 二阶导数值（加速度，d^2s/dt^2 或 d^2d/dt^2 或 d^2d/ds^2）
 *
 * @note 该结构体支持两种 d 分量的参数化方式：
 *   - kInitWithDt: d 分量关于时间 t 求导（高速模式，默认推荐）
 *   - kInitWithDs: d 分量关于弧长 s 求导（低速模式，避免数值奇异性）
 *   两种参数化之间可以通过 Load() 函数自动转换。
 */
struct FrenetState {
  /**
   * @enum InitType
   * @brief Frenet 状态初始化类型枚举
   *
   * 指定传入的 d 分量向量是以时间 t 还是弧长 s 为自变量进行求导。
   */
  enum InitType {
    kInitWithDt,  ///< d 分量关于时间 t 求导（vec_dt 模式），适用于中高速场景
    kInitWithDs   ///< d 分量关于弧长 s 求导（vec_ds 模式），适用于低速场景
  };

  /// 时间戳，记录该 Frenet 状态对应的时刻（秒）
  decimal_t time_stamp{0.0};

  /// s 分量的状态向量：[s, ds/dt, d^2s/dt^2]，分别表示沿参考线的弧长位置、速度和加速度
  Vecf<3> vec_s{Vecf<3>::Zero()};

  /// d 分量关于时间 t 的状态向量：[d, dd/dt, d^2d/dt^2]，即横向偏移及其对时间的导数
  Vecf<3> vec_dt{Vecf<3>::Zero()};

  /// d 分量关于弧长 s 的状态向量：[d, dd/ds, d^2d/ds^2]，即横向偏移及其对弧长的导数
  Vecf<3> vec_ds{Vecf<3>::Zero()};

  /// 标记 vec_ds 是否可用的标志位（当车速接近零时 vec_ds 不可用）
  bool is_ds_usable = true;

  /// 默认构造函数，所有向量初始化为零
  FrenetState() {}

  /**
   * @brief 加载 Frenet 状态并自动完成 dt/ds 两种参数化之间的转换
   *
   * 当以 vec_dt 模式（kInitWithDt）初始化时，该函数自动计算对应的 vec_ds 值：
   *   - ds = dt                          （零阶不变）
   *   - d' = dt' / s'                    （链式法则：dd/ds = dd/dt / ds/dt）
   *   - d'' = (dt'' - d' * s'') / (s')^2 （链式法则的二阶推广）
   *   当 s' 接近零时（kEPS 阈值），vec_ds 不可用，is_ds_usable 设为 false。
   *
   * 当以 vec_ds 模式（kInitWithDs）初始化时，该函数自动计算对应的 vec_dt 值：
   *   - dt = ds                          （零阶不变）
   *   - dt' = s' * ds'                   （链式法则：dd/dt = dd/ds * ds/dt）
   *   - dt'' = ds'' * (s')^2 + ds' * s'' （链式法则的二阶推广）
   *
   * @param s s 分量的状态向量 [s, s', s'']
   * @param d d 分量的状态向量，其导数含义取决于 type 参数
   * @param type 初始化类型，指定 d 向量的导数是对 t 求导还是对 s 求导
   */
  void Load(const Vecf<3>& s, const Vecf<3>& d, const InitType& type) {
    vec_s = s;
    if (type == kInitWithDt) {
      // === dt 模式：d 中存储的是关于时间的导数 ===
      vec_dt = d;
      vec_ds[0] = vec_dt[0];          // 零阶保持一致
      if (fabs(vec_s[1]) > kEPS) {
        // 链式法则一阶：dd/ds = (dd/dt) / (ds/dt)
        vec_ds[1] = vec_dt[1] / vec_s[1];
        // 链式法则二阶：d^2d/ds^2 = (d^2d/dt^2 - (dd/ds) * d^2s/dt^2) / (ds/dt)^2
        vec_ds[2] = (vec_dt[2] - vec_ds[1] * vec_s[2]) / (vec_s[1] * vec_s[1]);
        is_ds_usable = true;
      } else {
        // 车速过小，ds 参数化不可用
        vec_ds[1] = 0.0;
        vec_ds[2] = 0.0;
        is_ds_usable = false;
      }
    } else if (type == kInitWithDs) {
      // === ds 模式：d 中存储的是关于弧长的导数 ===
      vec_ds = d;
      vec_dt[0] = vec_ds[0];          // 零阶保持一致
      // 链式法则一阶：dd/dt = (dd/ds) * (ds/dt)
      vec_dt[1] = vec_s[1] * vec_ds[1];
      // 链式法则二阶：d^2d/dt^2 = (d^2d/ds^2) * (ds/dt)^2 + (dd/ds) * (d^2s/dt^2)
      vec_dt[2] = vec_ds[2] * vec_s[1] * vec_s[1] + vec_ds[1] * vec_s[2];
      is_ds_usable = true;
    } else {
      // 未知初始化类型，触发断言错误
      assert(false);
    }
  }

  /**
   * @brief 通过直接指定 s、dt、ds 三个向量构造 Frenet 状态
   * @param s s 分量的状态向量 [s, s', s'']
   * @param dt d 分量关于时间 t 的导数向量 [d, dd/dt, d^2d/dt^2]
   * @param ds d 分量关于弧长 s 的导数向量 [d, dd/ds, d^2d/ds^2]
   * @note 此构造函数不做自动转换，调用者需确保 dt 和 ds 的一致性
   */
  FrenetState(const Vecf<3>& s, const Vecf<3>& dt, const Vecf<3>& ds) {
    vec_s = s;
    vec_dt = dt;
    vec_ds = ds;
  }

  /**
   * @brief 打印 Frenet 状态详情到标准输出
   * @note 当 is_ds_usable 为 false 时，会额外输出警告信息
   */
  void print() const {
    printf("frenet state stamp: %lf.\n", time_stamp);
    printf("-- vec_s: (%lf, %lf, %lf).\n", vec_s[0], vec_s[1], vec_s[2]);
    printf("-- vec_dt: (%lf, %lf, %lf).\n", vec_dt[0], vec_dt[1], vec_dt[2]);
    printf("-- vec_ds: (%lf, %lf, %lf).\n", vec_ds[0], vec_ds[1], vec_ds[2]);
    if (!is_ds_usable) printf("-- warning: ds not usable.\n");
  }
};

}  // namespace common

#endif
