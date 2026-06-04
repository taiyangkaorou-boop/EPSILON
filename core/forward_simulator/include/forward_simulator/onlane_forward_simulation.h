#ifndef _CORE_FORWARD_SIMULATOR_INC_ONLANE_FORWARD_SIMULATOR_H_
#define _CORE_FORWARD_SIMULATOR_INC_ONLANE_FORWARD_SIMULATOR_H_

/// @file        onlane_forward_simulation.h
/// @brief       基于车道的前向仿真引擎（On-Lane Forward Simulation）
/// @details     该文件是EPSILON前向仿真系统的核心组件，提供车辆在车道坐标系下的
///              一步前向传播（PropagateOnce）功能。前向仿真引擎将横向控制
///              （纯追踪Pure Pursuit）和纵向控制（IDM/CtxIDM）结合，通过
///              IdealSteerModel进行物理约束下的运动学仿真。
///
///              功能模块：
///              1. GetTargetStateOnTargetLane()：计算目标车道上的期望纵向状态
///                 （用于换道场景中确定自车在目标车道中的合适位置和速度）
///              2. PropagateOnce()：基于当前车道的基本前向传播
///              3. PropagateOnceAdvancedLK()：带横向偏移的高级车道保持前向传播
///              4. PropagateOnceAdvancedLC()：换道场景的高级前向传播
///              5. GetIdmEquivalentVehicleLength()：计算IDM等效车辆长度
///
///              仿真流程（两步法）：
///              第一步 - 横向（Steer）：纯追踪控制器根据参考路径和预瞄距离计算目标转角
///              第二步 - 纵向（Velocity）：IDM模型根据前车状态计算目标速度
///              第三步 - 运动学：IdealSteerModel在物理约束下积分到目标状态

#include <algorithm>

#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "vehicle_model/controllers/ctx_idm_velocity_controller.h"
#include "vehicle_model/controllers/idm_velocity_controller.h"
#include "vehicle_model/controllers/pure_pursuit_controller.h"
#include "vehicle_model/ideal_steer_model.h"

namespace planning {

/// @class OnLaneForwardSimulation
/// @brief 基于车道参考线的前向仿真引擎类
/// @details 该类为纯静态方法的集合，不维护内部状态。所有仿真参数通过Param结构体传入。
///          核心思想是将横向控制（纯追踪）和纵向控制（IDM）解耦后分别计算，
///          再将结果输入带物理约束的车辆模型进行一步状态传播。
///
///          重要设计决策：
///          - 使用Frenet坐标系（StateTransformer）将笛卡尔空间状态转换到车道曲线坐标系，
///            简化了横向（d坐标）和纵向（s坐标）的解耦控制
///          - 转向计算失败时使用当前转角（保持），并可选择自动减速到0
///            （auto_decelerate_if_lat_failed），保证安全性
///          - 当没有前车时构造虚拟前车，距离与当前速度成正比，模拟自由流状态
///          - IDM使用等效车辆长度，通过投影车辆角点到Frenet坐标系来考虑曲率效应
class OnLaneForwardSimulation {
 public:
  using Lane = common::Lane;
  using State = common::State;
  using FrenetState = common::FrenetState;
  using VehicleControlSignal = common::VehicleControlSignal;
  using Vehicle = common::Vehicle;
  using CtxParam = simulator::ContextIntelligentDriverModel::CtxParam;

  /// @struct Param
  /// @brief 前向仿真的参数配置结构体
  /// @details 包含IDM纵向参数、纯追踪横向参数和车辆物理约束参数。
  ///          这些参数控制仿真中的驾驶行为（激进/保守）和运动学限制。
  struct Param {
    // === IDM纵向跟随参数 ===
    simulator::IntelligentDriverModel::Param idm_param;

    // === 纯追踪横向控制参数 ===
    decimal_t steer_control_gain = 1.5;               ///< 预瞄距离与速度的比例系数，越大预瞄越远
    decimal_t steer_control_max_lookahead_dist = 50.0; ///< 最大预瞄距离（m），上限保护
    decimal_t steer_control_min_lookahead_dist = 3.0;  ///< 最小预瞄距离（m），低速下保证基本跟踪能力

    // === 车辆物理约束参数 ===
    decimal_t max_lat_acceleration_abs = 1.5;   ///< 最大侧向加速度绝对值（m/s^2），1.5约为优质公路乘客舒适上限
    decimal_t max_lat_jerk_abs = 3.0;            ///< 最大侧向加加速度绝对值（m/s^3）
    decimal_t max_curvature_abs = 0.33;          ///< 最大曲率绝对值（1/m），对应约3m最小转弯半径
    decimal_t max_lon_acc_jerk = 5.0;            ///< 最大纵向加速加加速度（m/s^3）
    decimal_t max_lon_brake_jerk = 5.0;          ///< 最大纵向制动加加速度（m/s^3）
    decimal_t max_steer_angle_abs = 45.0 / 180.0 * kPi; ///< 最大前轮转角（rad），45度对应典型乘用车
    decimal_t max_steer_rate = 0.39;             ///< 最大前轮转角速率（rad/s），约22.3 deg/s
    bool auto_decelerate_if_lat_failed = true;   ///< 横向控制失败时是否自动减速（保证安全）
  };

  /// @brief 计算目标车道上的期望纵向目标状态（用于换道场景）
  /// @param stf_target 目标车道的状态转换器（提供Frenet<->笛卡尔坐标转换）
  /// @param ego_vehicle 自车信息（位置、速度、尺寸参数）
  /// @param gap_front_vehicle 目标车道上的前方车辆（-1表示不存在）
  /// @param gap_rear_vehicle 目标车道上的后方车辆（-1表示不存在）
  /// @param param 仿真参数
  /// @param target_state 输出：目标车道上的期望笛卡尔状态
  /// @return kSuccess 表示计算成功，kWrongStatus表示参数/状态异常
  /// @details 该函数计算自车在目标车道上应该占据的合理纵向位置和速度。
  ///          考虑了三种情况：
  ///          - 前后车都存在：取两车中间位置，速度受前后车约束
  ///          - 仅前车存在：不超过前车安全距离，速度不超过前车相关量
  ///          - 仅后车存在：不低于后车安全距离，速度不低于后车相关量
  ///          位置计算基于IDM的最小间距和期望时距参数。
  static ErrorType GetTargetStateOnTargetLane(
      const common::StateTransformer& stf_target,
      const common::Vehicle& ego_vehicle,
      const common::Vehicle& gap_front_vehicle,
      const common::Vehicle& gap_rear_vehicle, const Param& param,
      common::State* target_state) {
    // 将自车状态转换到目标车道的Frenet坐标系
    common::FrenetState ego_fs;
    if (kSuccess !=
        stf_target.GetFrenetStateFromState(ego_vehicle.state(), &ego_fs)) {
      return kWrongStatus;
    }

    // 提取IDM安全距离参数
    decimal_t time_headaway = param.idm_param.kDesiredHeadwayTime;
    decimal_t min_spacing = param.idm_param.kMinimumSpacing;

    // --- 处理前车 ---
    bool has_front = false;
    common::FrenetState front_fs;
    decimal_t s_ref_front = -1;  // 前车尾部位置的s坐标
    decimal_t s_thres_front = -1; // 自车不应超过的s上限（前车安全距离边界）
    if (gap_front_vehicle.id() != -1 &&
        kSuccess == stf_target.GetFrenetStateFromState(
                        gap_front_vehicle.state(), &front_fs)) {
      has_front = true;
      // 前车后缘s坐标 = 前车中心s - (车身长度/2 - 后轴到车尾距离)
      s_ref_front =
          front_fs.vec_s[0] - (gap_front_vehicle.param().length() / 2.0 -
                               gap_front_vehicle.param().d_cr());
      // 自车s上限 = 前车后缘 - 最小间距 - 期望时距 * 自车速度
      s_thres_front = s_ref_front - min_spacing -
                      time_headaway * ego_vehicle.state().velocity;
    }

    // --- 处理后车 ---
    bool has_rear = false;
    common::FrenetState rear_fs;
    decimal_t s_ref_rear = -1;  // 后车头部位置的s坐标
    decimal_t s_thres_rear = -1; // 自车不应低于的s下限（后车安全距离边界）
    if (gap_rear_vehicle.id() != -1 &&
        kSuccess == stf_target.GetFrenetStateFromState(gap_rear_vehicle.state(),
                                                       &rear_fs)) {
      has_rear = true;
      // 后车前缘s坐标 = 后车中心s + 车身长度/2 + 后轴到车尾距离
      s_ref_rear = rear_fs.vec_s[0] + gap_rear_vehicle.param().length() / 2.0 +
                   gap_rear_vehicle.param().d_cr();
      // 自车s下限 = 后车前缘 + 最小间距 + 期望时距 * 后车速度
      s_thres_rear = s_ref_rear + min_spacing +
                     time_headaway * gap_rear_vehicle.state().velocity;
    }

    decimal_t desired_s = ego_fs.vec_s[0];
    decimal_t desired_v = ego_vehicle.state().velocity;

    // 速度调整参数
    decimal_t k_v = 0.1;      // 速度偏差反馈系数，用于从位置误差推算参考速度
    decimal_t p_v_ego = 0.1;  // 自车期望速度的平滑系数
    decimal_t dv_lb = -3.0;   // 速度修正量下限（m/s）
    decimal_t dv_ub = 5.0;    // 速度修正量上限（m/s）

    // 计算自车趋向期望速度的过渡速度（平滑趋近）
    decimal_t ego_desired_vel =
        ego_vehicle.state().velocity +
        (param.idm_param.kDesiredVelocity - ego_vehicle.state().velocity) *
            p_v_ego;

    if (has_front && has_rear) {
      // 情况1：前后车都存在
      if (s_ref_front < s_ref_rear) {
        return kWrongStatus;  // 前车在后车后面，不可能的情况
      }

      // 取前后车之间的中点作为理想位置
      decimal_t ds = fabs(s_ref_front - s_ref_rear);
      decimal_t s_star = s_ref_rear + ds / 2.0 - ego_vehicle.param().d_cr();

      // 边界裁剪：安全距离边界不能超过几何中点
      s_thres_front = std::max(s_star, s_thres_front);
      s_thres_rear = std::min(s_star, s_thres_rear);

      // 期望位置夹在前后车安全距离之间
      desired_s =
          std::min(std::max(s_thres_rear, ego_fs.vec_s[0]), s_thres_front);

      // 从前车安全边界计算参考速度上限
      decimal_t s_err_front = s_thres_front - ego_fs.vec_s[0];
      decimal_t v_ref_front =
          std::max(0.0, gap_front_vehicle.state().velocity +
                            truncate(s_err_front * k_v, dv_lb, dv_ub));

      // 从后车安全边界计算参考速度下限
      decimal_t s_err_rear = s_thres_rear - ego_fs.vec_s[0];
      decimal_t v_ref_rear =
          std::max(0.0, gap_rear_vehicle.state().velocity +
                            truncate(s_err_rear * k_v, dv_lb, dv_ub));

      // 期望速度夹在后车下限和前车上限之间
      desired_v = std::min(std::max(v_ref_rear, ego_desired_vel), v_ref_front);

    } else if (has_front) {
      // 情况2：仅前车存在
      desired_s = std::min(ego_fs.vec_s[0], s_thres_front);

      decimal_t s_err_front = s_thres_front - ego_fs.vec_s[0];
      decimal_t v_ref_front =
          std::max(0.0, gap_front_vehicle.state().velocity +
                            truncate(s_err_front * k_v, dv_lb, dv_ub));
      desired_v = std::min(ego_desired_vel, v_ref_front);

    } else if (has_rear) {
      // 情况3：仅后车存在
      desired_s = std::max(ego_fs.vec_s[0], s_thres_rear);

      decimal_t s_err_rear = s_thres_rear - ego_fs.vec_s[0];
      decimal_t v_ref_rear =
          std::max(0.0, gap_rear_vehicle.state().velocity +
                            truncate(s_err_rear * k_v, dv_lb, dv_ub));
      desired_v = std::max(v_ref_rear, ego_desired_vel);
    }

    // 构造目标Frenet状态并转换回笛卡尔坐标
    common::FrenetState target_fs;
    target_fs.Load(Vecf<3>(desired_s, desired_v, 0.0), Vecf<3>(0.0, 0.0, 0.0),
                   common::FrenetState::kInitWithDs);

    if (kSuccess !=
        stf_target.GetStateFromFrenetState(target_fs, target_state)) {
      return kWrongStatus;
    }

    return kSuccess;
  }

  /// @brief 高级车道保持前向传播（带横向偏移）
  /// @param stf 当前车道的状态转换器
  /// @param ego_vehicle 自车信息
  /// @param leading_vehicle 前方车辆（id=-1表示不存在）
  /// @param lat_track_offset 横向跟踪偏移量（Frenet d坐标偏移），正值向车道左侧偏移
  /// @param dt 时间步长（秒）
  /// @param param 仿真参数
  /// @param desired_state 输出：dt时刻后的期望笛卡尔状态
  /// @return kSuccess 表示仿真成功
  /// @details 与基本PropagateOnce的区别在于引入了lat_track_offset参数，
  ///          允许车辆跟踪车道线的横向偏移位置（如车道中心的左/右偏移），
  ///          用于实现避让、靠边停车等需要偏离车道中心线的场景。
  static ErrorType PropagateOnceAdvancedLK(
      const common::StateTransformer& stf, const common::Vehicle& ego_vehicle,
      const Vehicle& leading_vehicle, const decimal_t& lat_track_offset,
      const decimal_t& dt, const Param& param, State* desired_state) {
    common::State current_state = ego_vehicle.state();
    decimal_t wheelbase_len = ego_vehicle.param().wheel_base();
    auto sim_param = param;

    // ===== 第一步：计算横向控制（转向角）=====
    bool steer_calculation_failed = false;
    common::FrenetState current_fs;
    if (stf.GetFrenetStateFromState(current_state, &current_fs) != kSuccess ||
        current_fs.vec_s[1] < -kEPS) {
      // 自车Frenet状态无效或处于倒车状态
      steer_calculation_failed = true;
    }

    decimal_t steer, velocity;
    if (!steer_calculation_failed) {
      // 预瞄距离 = min(max(min_lookahead, v * gain), max_lookahead)
      // 即与速度成正比，但有上下限保护
      decimal_t approx_lookahead_dist =
          std::min(std::max(param.steer_control_min_lookahead_dist,
                            current_state.velocity * param.steer_control_gain),
                   param.steer_control_max_lookahead_dist);
      // 计算期望转角，同时传入横向偏移量lat_track_offset
      if (CalcualateSteer(stf, current_state, current_fs, wheelbase_len,
                          Vec2f(approx_lookahead_dist, lat_track_offset),
                          &steer) != kSuccess) {
        steer_calculation_failed = true;
      }
    }

    // 转向失败时保持当前转角，并可选择减速
    steer = steer_calculation_failed ? current_state.steer : steer;
    decimal_t sim_vel = param.idm_param.kDesiredVelocity;
    if (param.auto_decelerate_if_lat_failed && steer_calculation_failed) {
      sim_vel = 0.0;  // 横向控制失败时安全减速至停止
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0, sim_vel);

    // ===== 第二步：计算纵向控制（速度）=====
    common::FrenetState leading_fs;
    if (leading_vehicle.id() == kInvalidAgentId ||
        stf.GetFrenetStateFromState(leading_vehicle.state(), &leading_fs) !=
            kSuccess) {
      // 无前车：使用虚拟前车的自由流IDM速度计算
      CalcualateVelocityUsingIdm(current_state.velocity, dt, sim_param,
                                 &velocity);
    } else {
      // 有前车：计算IDM等效车辆长度（考虑曲率效应）
      decimal_t eqv_vehicle_len;
      GetIdmEquivalentVehicleLength(stf, ego_vehicle, leading_vehicle,
                                    leading_fs, &eqv_vehicle_len);
      sim_param.idm_param.kVehicleLength = eqv_vehicle_len;

      // 使用IDM计算跟车速度
      CalcualateVelocityUsingIdm(
          current_fs.vec_s[0], current_state.velocity, leading_fs.vec_s[0],
          leading_vehicle.state().velocity, dt, sim_param, &velocity);
    }

    // ===== 第三步：通过车辆运动学模型传播状态 =====
    CalculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          sim_param, desired_state);
    return kSuccess;
  }

  /// @brief 换道场景高级前向传播
  /// @param stf_current 当前车道的状态转换器
  /// @param stf_target 目标车道的状态转换器
  /// @param ego_vehicle 自车信息
  /// @param current_leading_vehicle 当前车道上的前方车辆
  /// @param gap_front_vehicle 目标车道上的前方车辆（汇入间隙前车）
  /// @param gap_rear_vehicle 目标车道上的后方车辆（汇入间隙后车）
  /// @param lat_track_offset 横向跟踪偏移量
  /// @param dt 时间步长（秒）
  /// @param param 仿真参数
  /// @param desired_state 输出：dt时刻后的期望笛卡尔状态
  /// @return kSuccess 表示仿真成功
  /// @details 换道仿真的核心逻辑：
  ///          1. 横向控制：预瞄点设置在目标车道上（而非当前车道），
  ///             引导车辆向目标车道转向
  ///          2. 纵向控制：使用CtxIDM，同时考虑当前前车和目标车道参考状态，
  ///             使车辆在换道过程中平衡安全性和换道意图
  static ErrorType PropagateOnceAdvancedLC(
      const common::StateTransformer& stf_current,
      const common::StateTransformer& stf_target,
      const common::Vehicle& ego_vehicle,
      const Vehicle& current_leading_vehicle, const Vehicle& gap_front_vehicle,
      const Vehicle& gap_rear_vehicle, const decimal_t& lat_track_offset,
      const decimal_t& dt, const Param& param, State* desired_state) {
    common::State current_state = ego_vehicle.state();
    decimal_t wheelbase_len = ego_vehicle.param().wheel_base();
    auto sim_param = param;

    decimal_t steer, velocity;

    // ===== 第一步：计算横向控制 =====
    // 注意：换道时的纯追踪参考线是目标车道
    bool steer_calculation_failed = false;
    common::FrenetState ego_on_tarlane_fs;
    if (stf_target.GetFrenetStateFromState(current_state, &ego_on_tarlane_fs) !=
            kSuccess ||
        ego_on_tarlane_fs.vec_s[1] < -kEPS) {
      steer_calculation_failed = true;
    }

    if (!steer_calculation_failed) {
      decimal_t approx_lookahead_dist =
          std::min(std::max(param.steer_control_min_lookahead_dist,
                            current_state.velocity * param.steer_control_gain),
                   param.steer_control_max_lookahead_dist);
      // 使用目标车道的转换器计算转角（引导向目标车道转向）
      if (CalcualateSteer(stf_target, current_state, ego_on_tarlane_fs,
                          wheelbase_len,
                          Vec2f(approx_lookahead_dist, lat_track_offset),
                          &steer) != kSuccess) {
        steer_calculation_failed = true;
      }
    }
    steer = steer_calculation_failed ? current_state.steer : steer;
    decimal_t sim_vel = param.idm_param.kDesiredVelocity;
    if (param.auto_decelerate_if_lat_failed && steer_calculation_failed) {
      sim_vel = 0.0;
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0, sim_vel);

    // ===== 第二步：计算纵向控制 =====
    // 计算目标车道上的期望纵向状态（位置+速度）
    common::State target_state;
    if (kSuccess != GetTargetStateOnTargetLane(
                        stf_target, ego_vehicle, gap_front_vehicle,
                        gap_rear_vehicle, sim_param, &target_state)) {
      target_state = current_state;  // 失败时退化为当前状态
    }
    // 将目标状态投影到当前车道坐标系中
    common::FrenetState target_on_curlane_fs;
    if (stf_current.GetFrenetStateFromState(
            target_state, &target_on_curlane_fs) != kSuccess) {
    }

    // 获取自车在当前车道坐标系下的Frenet状态
    common::FrenetState ego_on_curlane_fs;
    if (stf_current.GetFrenetStateFromState(current_state,
                                            &ego_on_curlane_fs) != kSuccess) {
    }

    // 上下文参数：设置较温和的收敛增益
    simulator::ContextIntelligentDriverModel::CtxParam ctx_param(0.4, 0.8);

    common::FrenetState current_leading_fs;
    if (current_leading_vehicle.id() == kInvalidAgentId ||
        stf_current.GetFrenetStateFromState(current_leading_vehicle.state(),
                                            &current_leading_fs) != kSuccess) {
      // 无前车：使用虚拟前车的CtxIDM（前车用虚拟距离，目标用计算值）
      CalcualateVelocityUsingCtxIdm(
          ego_on_tarlane_fs.vec_s[0], current_state.velocity,
          target_on_curlane_fs.vec_s[0], target_state.velocity, dt, sim_param,
          ctx_param, &velocity);
    } else {
      // 有前车：使用真实前车的CtxIDM
      decimal_t eqv_vehicle_len;
      GetIdmEquivalentVehicleLength(stf_current, ego_vehicle,
                                    current_leading_vehicle, current_leading_fs,
                                    &eqv_vehicle_len);
      sim_param.idm_param.kVehicleLength = eqv_vehicle_len;

      CalcualateVelocityUsingCtxIdm(
          ego_on_tarlane_fs.vec_s[0], current_state.velocity,
          current_leading_fs.vec_s[0], current_leading_vehicle.state().velocity,
          target_on_curlane_fs.vec_s[0], target_state.velocity, dt, sim_param,
          ctx_param, &velocity);
    }

    // ===== 第三步：运动学传播 =====
    CalculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          sim_param, desired_state);
    return kSuccess;
  }

  /// @brief 基本前向传播（车道居中保持，lat_track_offset=0）
  /// @param stf 车道的状态转换器
  /// @param ego_vehicle 自车信息
  /// @param leading_vehicle 前方车辆
  /// @param dt 时间步长（秒）
  /// @param param 仿真参数
  /// @param desired_state 输出：dt时刻后的期望状态
  /// @return kSuccess 表示仿真成功
  /// @details 这是最基本的车道保持前向仿真，使用lat_track_offset=0
  ///          （即跟踪车道中心线）。内部逻辑与PropagateOnceAdvancedLK相同，
  ///          仅横向偏移置为0。
  static ErrorType PropagateOnce(const common::StateTransformer& stf,
                                 const common::Vehicle& ego_vehicle,
                                 const Vehicle& leading_vehicle,
                                 const decimal_t& dt, const Param& param,
                                 State* desired_state) {
    common::State current_state = ego_vehicle.state();
    decimal_t wheelbase_len = ego_vehicle.param().wheel_base();
    auto sim_param = param;

    // ===== 第一步：计算横向控制 =====
    bool steer_calculation_failed = false;
    common::FrenetState current_fs;
    if (stf.GetFrenetStateFromState(current_state, &current_fs) != kSuccess ||
        current_fs.vec_s[1] < -kEPS) {
      steer_calculation_failed = true;
    }

    decimal_t steer, velocity;
    if (!steer_calculation_failed) {
      decimal_t approx_lookahead_dist =
          std::min(std::max(param.steer_control_min_lookahead_dist,
                            current_state.velocity * param.steer_control_gain),
                   param.steer_control_max_lookahead_dist);
      // 横向偏移为0，即跟踪车道中心线
      if (CalcualateSteer(stf, current_state, current_fs, wheelbase_len,
                          Vec2f(approx_lookahead_dist, 0.0),
                          &steer) != kSuccess) {
        steer_calculation_failed = true;
      }
    }

    steer = steer_calculation_failed ? current_state.steer : steer;
    decimal_t sim_vel = param.idm_param.kDesiredVelocity;
    if (param.auto_decelerate_if_lat_failed && steer_calculation_failed) {
      sim_vel = 0.0;
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0, sim_vel);

    // ===== 第二步：计算纵向控制 =====
    common::FrenetState leading_fs;
    if (leading_vehicle.id() == kInvalidAgentId ||
        stf.GetFrenetStateFromState(leading_vehicle.state(), &leading_fs) !=
            kSuccess) {
      // 无前车：自由流
      CalcualateVelocityUsingIdm(current_state.velocity, dt, sim_param,
                                 &velocity);
    } else {
      // 有前车：计算等效车辆长度后跟车
      decimal_t eqv_vehicle_len;
      GetIdmEquivalentVehicleLength(stf, ego_vehicle, leading_vehicle,
                                    leading_fs, &eqv_vehicle_len);
      sim_param.idm_param.kVehicleLength = eqv_vehicle_len;

      CalcualateVelocityUsingIdm(
          current_fs.vec_s[0], current_state.velocity, leading_fs.vec_s[0],
          leading_vehicle.state().velocity, dt, sim_param, &velocity);
    }

    // ===== 第三步：运动学传播 =====
    CalculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          sim_param, desired_state);
    return kSuccess;
  }

  /// @brief 定速/定转角前向传播（重载版本，不计算新的控制量）
  /// @param desired_vel 保持的期望速度（m/s）（实际未使用，保留接口兼容性）
  /// @param ego_vehicle 自车信息
  /// @param dt 时间步长（秒）
  /// @param param 仿真参数
  /// @param desired_state 输出：dt时刻后的期望状态
  /// @return kSuccess 表示仿真成功
  /// @details 直接使用当前速度和转角作为控制输入进行运动学传播，
  ///          不对转向和速度进行重新计算。适用于车辆在无参考线场景中
  ///          以恒定速度和转向运动的情况。
  static ErrorType PropagateOnce(const decimal_t& desired_vel,
                                 const common::Vehicle& ego_vehicle,
                                 const decimal_t& dt, const Param& param,
                                 State* desired_state) {
    common::State current_state = ego_vehicle.state();
    decimal_t wheelbase_len = ego_vehicle.param().wheel_base();
    decimal_t steer = current_state.steer;
    decimal_t velocity = current_state.velocity;
    // 直接使用当前状态进行物理约束下的传播
    CalculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          param, desired_state);
    return kSuccess;
  }

 private:
  /// @brief 计算IDM等效车辆长度（考虑曲率道路上车辆角点的投影效应）
  /// @param stf 状态转换器
  /// @param ego_vehicle 自车
  /// @param leading_vehicle 前车
  /// @param leading_fs 前车的Frenet状态
  /// @param eqv_vehicle_len 输出：IDM使用的等效车辆长度（m）
  /// @return kSuccess 表示计算成功
  /// @details 与原始IDM不同，本实现仍以后轴中心为车辆位置参考点。
  ///          为了在弯道上准确计算自车与前车之间的净间距，
  ///          需要将前车角点投影到Frenet坐标系中，
  ///          取最接近自车的角点s坐标来计算等效长度：
  ///          eqv_len = 自车后轴到前保险杠的距离 + 前车最近角点到后轴的距离
  ///          这考虑了弯道上车辆相对姿态的影响。
  static ErrorType GetIdmEquivalentVehicleLength(
      const common::StateTransformer& stf, const common::Vehicle& ego_vehicle,
      const common::Vehicle& leading_vehicle,
      const common::FrenetState& leading_fs, decimal_t* eqv_vehicle_len) {
    // 获取前车保险杠的两个角点坐标
    std::array<Vec2f, 2> leading_pts;
    leading_vehicle.RetBumperVertices(&leading_pts);

    // 将角点投影到Frenet坐标系，收集所有s坐标
    Vec2f fs_pt1, fs_pt2;
    std::vector<decimal_t> s_vec;
    if (kSuccess == stf.GetFrenetPointFromPoint(leading_pts[0], &fs_pt1)) {
      s_vec.push_back(fs_pt1(0));
    }
    if (kSuccess == stf.GetFrenetPointFromPoint(leading_pts[1], &fs_pt2)) {
      s_vec.push_back(fs_pt2(0));
    }
    s_vec.push_back(leading_fs.vec_s(0)); // 也包含前车后轴中心s
    // 取最小的s值（离自车最近的角点）
    decimal_t s_nearest_vtx = *(std::min_element(s_vec.begin(), s_vec.end()));

    // 前车后轴到最近角点的距离
    decimal_t len_rb2r = fabs(leading_fs.vec_s(0) - s_nearest_vtx);
    // 等效车长 = 自车后轴到前保险杠 + 前车最近角点到后轴
    *eqv_vehicle_len = ego_vehicle.param().length() / 2.0 +
                       ego_vehicle.param().d_cr() + len_rb2r;

    return kSuccess;
  }

  /// @brief 使用纯追踪算法计算期望前轮转角
  /// @param stf 状态转换器
  /// @param current_state 当前笛卡尔状态
  /// @param current_fs 当前Frenet状态
  /// @param wheelbase_len 轴距（m）
  /// @param lookahead_offset (预瞄纵向距离, 预瞄横向偏移)
  /// @param steer 输出：期望前轮转角（rad）
  /// @return kSuccess 表示计算成功
  /// @details 计算流程：
  ///          1. 在Frenet坐标下构造预瞄点：(s + lookahead_s, d + lookahead_offset)
  ///          2. 将预瞄点转换到笛卡尔坐标系
  ///          3. 计算预瞄距离 = 预瞄点到当前车位置的欧氏距离
  ///          4. 计算角度偏差 = 车辆当前航向与预瞄点方向的夹角
  ///          5. 调用PurePursuitControl计算期望转角
  static ErrorType CalcualateSteer(const common::StateTransformer& stf,
                                   const State& current_state,
                                   const FrenetState& current_fs,
                                   const decimal_t& wheelbase_len,
                                   const Vec2f& lookahead_offset,
                                   decimal_t* steer) {
    // 构造Frenet坐标系下的预瞄目标点
    common::FrenetState dest_fs;
    dest_fs.Load(Vecf<3>(lookahead_offset(0) + current_fs.vec_s[0], 0.0, 0.0),
                 Vecf<3>(lookahead_offset(1), 0.0, 0.0),
                 common::FrenetState::kInitWithDs);

    // 将预瞄点转换到笛卡尔坐标系
    State dest_state;
    if (stf.GetStateFromFrenetState(dest_fs, &dest_state) != kSuccess) {
      return kWrongStatus;
    }

    // 计算预瞄距离（笛卡尔空间中的欧氏距离）
    decimal_t look_ahead_dist =
        (dest_state.vec_position - current_state.vec_position).norm();
    // 计算从当前位置到预瞄点的方向角
    decimal_t cur_to_dest_angle =
        vec2d_to_angle(dest_state.vec_position - current_state.vec_position);
    // 计算车辆航向与预瞄方向的角度偏差
    decimal_t angle_diff =
        normalize_angle(cur_to_dest_angle - current_state.angle);
    // 纯追踪公式计算期望转角
    control::PurePursuitControl::CalculateDesiredSteer(
        wheelbase_len, angle_diff, look_ahead_dist, steer);
    return kSuccess;
  }

  /// @brief 使用IDM计算期望速度（有前车版本）
  /// @param current_pos 自车Frenet s坐标
  /// @param current_vel 自车速度（m/s）
  /// @param leading_pos 前车Frenet s坐标
  /// @param leading_vel 前车速度（m/s）
  /// @param dt 时间步长（秒）
  /// @param param 仿真参数
  /// @param velocity 输出：期望速度（m/s）
  /// @return kSuccess 表示计算成功
  /// @details 注意：不能用Frenet状态的速度直接给IDM模型，
  ///          因为在弯道上Frenet速度可能大于车体真实速度
  ///          （参考StateTransformer的实现）。
  static ErrorType CalcualateVelocityUsingIdm(
      const decimal_t& current_pos, const decimal_t& current_vel,
      const decimal_t& leading_pos, const decimal_t& leading_vel,
      const decimal_t& dt, const Param& param, decimal_t* velocity) {
    decimal_t leading_vel_fin = leading_vel;
    if (leading_vel < 0) {
      leading_vel_fin = 0;  // 前车速度为非负
    }
    return control::IntelligentVelocityControl::CalculateDesiredVelocity(
        param.idm_param, current_pos, leading_pos, current_vel, leading_vel_fin,
        dt, velocity);
  }

  /// @brief 使用IDM计算期望速度（无前车版本，使用虚拟前车模拟自由流）
  /// @param current_vel 自车速度（m/s）
  /// @param dt 时间步长（秒）
  /// @param param 仿真参数
  /// @param velocity 输出：期望速度（m/s）
  /// @return kSuccess 表示计算成功
  /// @details 构造一个虚拟前车，其距离与自车速度成正比：
  ///          virtual_leading_dist = 100 + 100 * current_vel
  ///          这使得高速行驶时虚拟前车更远，低速时更近，
  ///          模拟驾驶员在不同速度下的自由流跟车策略。
  static ErrorType CalcualateVelocityUsingIdm(const decimal_t& current_vel,
                                              const decimal_t& dt,
                                              const Param& param,
                                              decimal_t* velocity) {
    const decimal_t virtual_leading_dist = 100.0 + 100.0 * current_vel;
    return control::IntelligentVelocityControl::CalculateDesiredVelocity(
        param.idm_param, 0.0, 0.0 + virtual_leading_dist, current_vel,
        current_vel, dt, velocity);
  }

  /// @brief 使用CtxIDM计算换道场景期望速度（有前车版本）
  /// @param current_pos 自车Frenet s坐标（在目标车道坐标系中）
  /// @param current_vel 自车速度（m/s）
  /// @param leading_pos 前车Frenet s坐标
  /// @param leading_vel 前车速度（m/s）
  /// @param target_pos 目标车道参考位置s坐标
  /// @param target_vel 目标车道参考速度（m/s）
  /// @param dt 时间步长（秒）
  /// @param param 仿真参数
  /// @param ctx_param 上下文控制参数
  /// @param velocity 输出：期望速度（m/s）
  /// @return kSuccess 表示计算成功
  static ErrorType CalcualateVelocityUsingCtxIdm(
      const decimal_t& current_pos, const decimal_t& current_vel,
      const decimal_t& leading_pos, const decimal_t& leading_vel,
      const decimal_t& target_pos, const decimal_t& target_vel,
      const decimal_t& dt, const Param& param, const CtxParam& ctx_param,
      decimal_t* velocity) {
    decimal_t leading_vel_fin = leading_vel;
    if (leading_vel < 0) {
      leading_vel_fin = 0;
    }
    return control::ContextIntelligentVelocityControl::CalculateDesiredVelocity(
        param.idm_param, ctx_param, current_pos, leading_pos, target_pos,
        current_vel, leading_vel_fin, target_vel, dt, velocity);
  }

  /// @brief 使用CtxIDM计算换道场景期望速度（无前车版本）
  /// @details 与无前车IDM相同，构造虚拟前车（距离 = 100 + 100 * current_vel）
  static ErrorType CalcualateVelocityUsingCtxIdm(
      const decimal_t& current_pos, const decimal_t& current_vel,
      const decimal_t& target_pos, const decimal_t& target_vel,
      const decimal_t& dt, const Param& param, const CtxParam& ctx_param,
      decimal_t* velocity) {
    const decimal_t virtual_leading_pos =
        current_pos + 100.0 + 100.0 * current_vel;
    return control::ContextIntelligentVelocityControl::CalculateDesiredVelocity(
        param.idm_param, ctx_param, current_pos, virtual_leading_pos,
        target_pos, current_vel, current_vel, target_vel, dt, velocity);
  }

  /// @brief 通过IdealSteerModel进行一步运动学状态传播
  /// @param current_state 当前车辆状态
  /// @param steer 目标前轮转角（rad，已经过横向控制器计算）
  /// @param velocity 目标纵向速度（m/s，已经过IDM计算）
  /// @param wheelbase_len 轴距（m）
  /// @param dt 时间步长（秒）
  /// @param param 仿真参数（包含物理约束）
  /// @param state 输出：dt时刻后的车辆状态
  /// @return kSuccess 表示计算成功
  /// @details 创建IdealSteerModel实例，设置当前状态和控制量（steer+velocity），
  ///          执行一步，将物理约束限幅后的状态作为结果返回。
  ///          时间戳更新为 current_state.time_stamp + dt。
  static ErrorType CalculateDesiredState(const State& current_state,
                                         const decimal_t steer,
                                         const decimal_t velocity,
                                         const decimal_t wheelbase_len,
                                         const decimal_t dt, const Param& param,
                                         State* state) {
    simulator::IdealSteerModel model(
        wheelbase_len, param.idm_param.kAcceleration,
        param.idm_param.kHardBrakingDeceleration, param.max_lon_acc_jerk,
        param.max_lon_brake_jerk, param.max_lat_acceleration_abs,
        param.max_lat_jerk_abs, param.max_steer_angle_abs, param.max_steer_rate,
        param.max_curvature_abs);
    model.set_state(current_state);
    // 设置高层控制量（steer和velocity），模型内部会进行物理限幅
    model.set_control(simulator::IdealSteerModel::Control(steer, velocity));
    model.Step(dt);
    *state = model.state();
    state->time_stamp = current_state.time_stamp + dt;
    return kSuccess;
  }
};

}  // namespace planning

#endif  // _CORE_FORWARD_SIMULATOR_INC_ONLANE_FORWARD_SIMULATOR_H_
