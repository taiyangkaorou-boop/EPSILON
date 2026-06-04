#ifndef _VEHICLE_MODEL_INC_VEHIDLE_MODEL_CTX_IDM_MODEL_H__
#define _VEHICLE_MODEL_INC_VEHIDLE_MODEL_CTX_IDM_MODEL_H__

/// @file        ctx_idm_model.h
/// @brief       上下文感知智能驾驶员模型（Context-Aware IDM），专为换道场景设计
/// @details     该模型在标准IDM的基础上扩展了对目标车道状态（target lane state）的感知能力。
///              标准IDM只考虑当前车道的前方车辆，而CtxIDM同时考虑：
///              - 当前车道前方车辆（前车）：用于安全跟车
///              - 目标车道的参考状态（目标状态）：用于引导换道过程中的速度调整
///
///              模型的核心思想是：车辆速度同时受到两个"吸引力"的影响：
///              1. IDM安全跟车加速度（基于前车）
///              2. 目标跟踪加速度（基于目标车道参考状态），公式为：
///                 v_ref = v_target + k_s * (s_target - s)
///                 a_track = k_v * (v_ref - v)
///
///              最终加速度取a_track（当前实现中IDM加速度被计算但未被直接使用），
///              并通过限幅保护（[-1.0, 1.0] m/s^2）确保运动平滑。
///              该模型主要用于换道决策时的前向仿真，评估换道后的速度收益。

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>

#include "common/basics/basics.h"
#include "common/idm/intelligent_driver_model.h"
#include "common/state/state.h"

using namespace boost::placeholders;

namespace simulator {

/// @class ContextIntelligentDriverModel
/// @brief 上下文感知IDM类，在换道场景中同时考虑前车和目标的参考状态
/// @details 相比标准IDM增加了目标车道状态（s_target, v_target），
///          以及上下文控制参数（k_s 位置增益, k_v 速度增益）。
///          状态空间为6维：自车位置(s,v)、前车位置(s_front,v_front)、目标状态(s_target,v_target)。
///
///          加速度计算逻辑：
///          1. 计算IDM期望加速度 acc_idm（基于前车）
///          2. 计算目标跟踪加速度 acc_track：
///             v_ref = v_target + k_s * (s_target - s)
///             acc_track = k_v * (v_ref - v)
///          3. 对 acc_track 限幅在 [-1.0, 1.0]
///          4. 最终使用 acc_track 作为车辆加速度
class ContextIntelligentDriverModel {
 public:
  /// @brief IDM参数类型别名
  using IdmParam = common::IntelligentDriverModel::Param;
  /// @brief IDM状态类型别名
  using IdmState = common::IntelligentDriverModel::State;

  /// @struct CtxParam
  /// @brief 上下文控制参数，控制车辆向目标状态收敛的速率
  struct CtxParam {
    decimal_t k_s = 0.5;                ///< 位置误差反馈增益（1/s），控制对s误差的响应强度
    decimal_t k_v = 2.0 * k_s;          ///< 速度误差反馈增益（1/s），通常设置为k_s的2倍以保证收敛

    CtxParam() = default;
    /// @brief 构造函数，显式设置反馈增益
    /// @param _k_s 位置误差增益
    /// @param _k_v 速度误差增益
    CtxParam(const decimal_t &_k_s, const decimal_t &_k_v)
        : k_s(_k_s), k_v(_k_v) {}
  };

  /// @struct CtxIdmState
  /// @brief CtxIDM的扩展状态结构体，包含自车、前车和目标的纵向状态
  struct CtxIdmState {
    decimal_t s{0.0};        ///< 自车纵向位置（m），通常使用Frenet坐标系的s坐标（后轴中心）
    decimal_t v{0.0};        ///< 自车纵向速度（m/s）
    decimal_t s_front{0.0};  ///< 当前车道前方车辆（前车）的纵向位置（m）
    decimal_t v_front{0.0};  ///< 前车的纵向速度（m/s）
    decimal_t s_target{0.0}; ///< 目标车道参考位置的纵向坐标（m），用于引导速度调整
    decimal_t v_target{0.0}; ///< 目标车道参考速度（m/s）
  };

  /// @brief 默认构造函数，使用默认参数
  ContextIntelligentDriverModel();

  /// @brief 使用指定参数构造CtxIDM模型
  /// @param idm_parm IDM参数，用于安全跟车加速度计算
  /// @param ctx_param 上下文参数，控制向目标车道状态收敛的行为
  ContextIntelligentDriverModel(const IdmParam &idm_parm,
                                const CtxParam &ctx_param);

  ~ContextIntelligentDriverModel();

  /// @brief 获取当前扩展状态（包含目标车道信息）
  const CtxIdmState &state(void) const;

  /// @brief 设置扩展状态
  /// @param state 包含自车、前车和目标状态的完整6维状态
  void set_state(const CtxIdmState &state);

  /// @brief 执行一个时间步的CtxIDM仿真
  /// @param dt 时间步长（秒）
  void Step(double dt);

  /// @brief 内部状态类型定义（6维数组），需设置为public供odeint访问
  /// @details 内部状态索引：
  ///          0: 自车位置 s, 1: 自车速度 v,
  ///          2: 前车位置 s_front, 3: 前车速度 v_front,
  ///          4: 目标位置 s_target, 5: 目标速度 v_target
  typedef boost::array<double, 6> InternalState;

  /// @brief ODE系统函数对象（functor），供odeint库调用
  /// @param x 当前内部状态向量（6维）
  /// @param dxdt 输出：状态导数向量（6维）
  /// @param t 当前时间（本模型中为dt，用于限幅保护）
  /// @details 计算规则：
  ///          ds/dt = v（位置积分自车速度）
  ///          dv/dt = acc_track（目标跟踪加速度，限幅[-1.0, 1.0]）
  ///          ds_front/dt = v_front（前车位置积分前车速度）
  ///          dv_front/dt = 0（假设前车匀速）
  ///          ds_target/dt = v_target（目标位置积分目标速度）
  ///          dv_target/dt = 0（假设目标状态匀速）
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

 private:
  /// @brief 将外部CtxIdmState同步到内部状态数组
  void UpdateInternalState(void);

  InternalState internal_state_;   ///< odeint内部使用的状态数组（6维）
  IdmParam idm_param_;             ///< 标准IDM参数
  CtxParam ctx_param_;             ///< 上下文控制参数（k_s, k_v）
  CtxIdmState state_;              ///< 外部可见的扩展状态
};
}  // namespace simulator

#endif  // _VEHICLE_MODEL_INC_VEHIDLE_MODEL_CTX_IDM_MODEL_H__
