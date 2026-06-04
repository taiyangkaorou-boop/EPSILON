/**
 * @file state_transformer.h
 * @brief 笛卡尔坐标与 Frenet 坐标之间的状态转换器
 *
 * 本文件定义了 EPSILON 自动驾驶规划系统中的坐标转换器 StateTransformer，
 * 负责在笛卡尔坐标系 (x, y, theta, curvature, velocity, ...) 和 Frenet 坐标系
 * (s, ds/dt, d^2s/dt^2, d, dd/dt, d^2d/dt^2) 之间进行双向状态转换。
 *
 * 坐标转换的核心依赖于一条参考车道线（Lane 对象），该车道线由样条曲线
 * 表示。转换过程包括以下关键操作：
 *
 *   1. 笛卡尔 -> Frenet 变换（GetFrenetStateFromState）:
 *      - 在参考线上寻找车辆位置对应的最近投影点（最近弧长 s）
 *      - 计算投影点处的位置、切向、法向等信息
 *      - 将笛卡尔状态投影到 Frenet 空间的 (s, d) 坐标上
 *      - 通过链式法则计算 s 和 d 各阶导数
 *      - 注意：该函数在参考线上使用有限采样策略进行投影，可能引入约 1cm 的位置误差，
 *        但计算效率很高，单次转换耗时约 0.03ms
 *
 *   2. Frenet -> 笛卡尔变换（GetStateFromFrenetState）:
 *      - 根据 s 坐标在参考线上求值，获得投影点的位置、切向、法向和曲率
 *      - 将 Frenet 的 d 分量投影回笛卡尔空间
 *      - 通过链式法则和曲率补偿计算笛卡尔空间的速度、加速度和朝向角
 *
 * 该转换器是规划系统中轨迹规划与车辆状态估计之间的桥梁，因为规划通常在
 * Frenet 空间中进行（便于纵向和横向解耦），而车辆的实际状态和控制指令
 * 则需要在笛卡尔空间中进行表述。
 */

#ifndef _COMMON_INC_COMMON_STATE_STATE_TRANSFORMER_H__
#define _COMMON_INC_COMMON_STATE_STATE_TRANSFORMER_H__

#include "common/basics/basics.h"
#include "common/basics/config.h"
#include "common/lane/lane.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"

namespace common {

/**
 * @class StateTransformer
 * @brief 笛卡尔坐标系与 Frenet 坐标系之间的双向状态转换器
 *
 * 该转换器持有一条参考车道线（Lane 对象），并基于该车道线提供
 * 笛卡尔状态 (x, y, theta, velocity, ...) 与 Frenet 状态
 * (s, s', s'', d, d', d'') 之间的相互转换功能。
 *
 * 核心转换方法：
 *   - GetFrenetStateFromState:  笛卡尔 -> Frenet 的单点转换
 *   - GetStateFromFrenetState:  Frenet -> 笛卡尔的单点转换
 *   - GetFrenetStateVectorFromStates:     笛卡尔 -> Frenet 的批量转换
 *   - GetStateVectorFromFrenetStates:     Frenet -> 笛卡尔的批量转换
 *   - GetFrenetPointFromPoint:           笛卡尔点 -> Frenet 点的几何转换
 *   - GetFrenetPointVectorFromPoints:    笛卡尔点 -> Frenet 点的批量几何转换
 *
 * @note 在批量转换中，由于每个点都可能在参考线上找到不同的最近投影弧长，
 *       转换结果是相互独立的，不同于简单的坐标变换。
 */
class StateTransformer {
 public:
  /// 默认构造函数，不绑定任何参考线
  StateTransformer() {}

  /**
   * @brief 使用指定的车道线对象构造转换器
   * @param lane 作为参考线的 Lane 对象，所有坐标转换都将基于该参考线进行
   */
  StateTransformer(const Lane& lane) { lane_ = lane; }

  /**
   * @brief 将 Frenet 状态转换为笛卡尔状态
   * @param fs 输入的 Frenet 坐标状态 (s, s', s'', d, d', d'')
   * @param s 输出的笛卡尔坐标状态 (x, y, theta, curvature, velocity, ...)
   * @return ErrorType 转换是否成功
   *
   * 转换步骤：
   *   1. 在参考线 s 处求位置、切向量、法向量和曲率
   *   2. 笛卡尔位置 = 参考线位置 + d * 法向量（d 为横向偏移）
   *   3. 通过链式法则和曲率补偿计算速度、加速度和朝向角
   */
  ErrorType GetStateFromFrenetState(const FrenetState& fs, State* s) const;

  /**
   * @brief 将笛卡尔状态转换为 Frenet 状态
   * @param s 输入的笛卡尔坐标状态 (x, y, theta, velocity, ...)
   * @param fs 输出的 Frenet 坐标状态 (s, s', s'', d, d', d'')
   * @return ErrorType 转换是否成功
   *
   * @note 该函数采用有限采样策略在参考线上寻找最近投影点，可能导致约 1cm 的
   *       位置误差，但计算效率很高（单次转换约 0.03ms）。
   */
  ErrorType GetFrenetStateFromState(const State& s, FrenetState* fs) const;

  /**
   * @brief 批量将笛卡尔状态向量转换为 Frenet 状态向量
   * @param state_vec 输入的笛卡尔状态向量
   * @param fs_vec 输出的 Frenet 状态向量
   * @return ErrorType 转换是否成功
   */
  ErrorType GetFrenetStateVectorFromStates(const vec_E<State> state_vec,
                                           vec_E<FrenetState>* fs_vec) const;

  /**
   * @brief 批量将 Frenet 状态向量转换为笛卡尔状态向量
   * @param fs_vec 输入的 Frenet 状态向量
   * @param state_vec 输出的笛卡尔状态向量
   * @return ErrorType 转换是否成功
   */
  ErrorType GetStateVectorFromFrenetStates(const vec_E<FrenetState>& fs_vec,
                                           vec_E<State>* state_vec) const;

  /**
   * @brief 将笛卡尔坐标点转换为 Frenet 坐标点（纯几何变换，不含运动学量）
   * @param s 输入的笛卡尔坐标点 (x, y)
   * @param fs 输出的 Frenet 坐标点 (s, d)
   * @return ErrorType 转换是否成功
   */
  ErrorType GetFrenetPointFromPoint(const Vec2f& s, Vec2f* fs) const;

  /**
   * @brief 批量将笛卡尔坐标点转换为 Frenet 坐标点
   * @param s 输入的笛卡尔坐标点向量
   * @param fs 输出的 Frenet 坐标点向量
   * @return ErrorType 转换是否成功
   */
  ErrorType GetFrenetPointVectorFromPoints(const vec_E<Vec2f>& s,
                                           vec_E<Vec2f>* fs) const;

  /**
   * @brief 检查转换器是否有效（绑定的参考线是否有效）
   * @return true 表示参考线有效，可进行坐标转换
   */
  bool IsValid() const { return lane_.IsValid(); }

  /// 调试用打印函数（当前为空实现）
  void print() {}

 private:
  /// 内部持有的参考车道线对象，所有坐标转换都基于该参考线
  Lane lane_;
};

}  // namespace common

#endif
