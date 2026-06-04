#ifndef _CORE_MOTION_PREDICTOR_INC_ONLANE_FORWARD_SIMULATION_PREDICTOR_H_
#define _CORE_MOTION_PREDICTOR_INC_ONLANE_FORWARD_SIMULATION_PREDICTOR_H_

/// @file        onlane_fs_predictor.h
/// @brief       基于前向仿真的运动预测器（On-Lane Forward Simulation Predictor）
/// @details     该预测器通过反复调用前向仿真引擎（OnLaneForwardSimulation::PropagateOnce）
///              来预测周围车辆在给定时间内的运动轨迹。预测器接收一条车道参考线和一辆车，
///              以固定的时间步长（t_step）进行多次前向传播，生成预测时间段（t_pred）
///              内的状态序列。
///
///              预测逻辑分为两种模式：
///              1. 有效车道模式：如果车道参考线有效，使用完整的IDM+纯追踪+运动学模型
///              2. 无效车道模式：如果无参考线信息，使用定速定转角直接传播（恒速假设）
///
///              在EPSILON中，该预测器用于运动规划模块评估周围车辆的预期行为，
///              为决策和轨迹规划提供环境预测支持。

#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"
#include "forward_simulator/onlane_forward_simulation.h"

namespace planning {

/// @class OnLaneFsPredictor
/// @brief 基于前向仿真的运动预测器类
/// @details 该类通过迭代调用前向仿真，预测车辆未来的状态序列。
///          核心方法GetPredictedTrajectory()根据给定的预测时长(t_pred)
///          和采样步长(t_step)，生成 num_step = ceil(t_pred/t_step) 个预测状态点。
///
///          预测结果包含起始状态在内共 num_step+1 个状态点：
///          pred_states[0] = 车辆当前状态
///          pred_states[1] = t_step 时刻的预测状态
///          pred_states[n] = n*t_step 时刻的预测状态
///
///          重要假设：
///          - 在有效车道模式下，假设车辆前方无其他车辆（传入空的Vehicle对象作为前车）
///          - 车辆期望速度等于当前速度（sim_param.idm_param.kDesiredVelocity = current_vel）
///            即假设车辆在预测时间内保持当前速度意图
class OnLaneFsPredictor {
 public:
  using Lane = common::Lane;
  using State = common::State;
  using VehicleControlSignal = common::VehicleControlSignal;
  using Vehicle = common::Vehicle;

  OnLaneFsPredictor() {}
  ~OnLaneFsPredictor();

  /// @brief 获取预测轨迹（核心接口）
  /// @param lane 车道参考线，提供Frenet坐标系转换和路径跟踪基准
  /// @param vehicle 被预测的车辆（包含当前状态和物理参数）
  /// @param t_pred 预测总时长（秒），预测覆盖的时间范围
  /// @param t_step 时间步长（秒），预测状态的采样间隔
  /// @param pred_states 输出：预测状态序列，包含首尾共 ceil(t_pred/t_step)+1 个状态点
  /// @return kSuccess 表示预测成功，kWrongStatus表示传播失败
  /// @details 迭代流程：
  ///          1. 计算预测步数 num_step = round(t_pred / t_step)
  ///          2. 设置IDM期望速度 = 车辆当前速度（恒速意图假设）
  ///          3. 插入初始状态到结果列表
  ///          4. 循环 num_step 次：
  ///             a. 如果有有效车道线：调用 PropagateOnce(stf, ...) 进行完整仿真
  ///             b. 如果无车道线：调用 PropagateOnce(vel, ...) 进行定速传播
  ///             c. 将结果状态加入列表，更新车辆状态为预测值
  ///          5. 返回完整轨迹
  static ErrorType GetPredictedTrajectory(const Lane& lane,
                                          const Vehicle& vehicle,
                                          const decimal_t& t_pred,
                                          const decimal_t& t_step,
                                          vec_E<State>* pred_states) {
    pred_states->clear();
    int num_step = std::round(t_pred / t_step);
    State desired_state;
    // 假定车辆在预测时段内保持当前速度意图
    decimal_t desired_vel = vehicle.state().velocity;
    planning::OnLaneForwardSimulation::Param sim_param;
    sim_param.idm_param.kDesiredVelocity = desired_vel;
    pred_states->push_back(vehicle.state());  // 插入初始状态（t=0）
    common::Vehicle v_in = vehicle;
    common::StateTransformer stf = common::StateTransformer(lane);
    for (int i = 0; i < num_step; ++i) {
      if (lane.IsValid()) {
        // 有效车道线：使用完整的前向仿真（IDM+纯追踪+运动学）
        // 传入空的Vehicle对象作为前车，即假设前方无车，自由流状态
        if (planning::OnLaneForwardSimulation::PropagateOnce(
                stf, v_in, common::Vehicle(), t_step, sim_param,
                &desired_state) != kSuccess) {
          return kWrongStatus;
        }
      } else {
        // 无效车道线：使用恒速恒转角直接传播（无参考线可跟踪）
        if (planning::OnLaneForwardSimulation::PropagateOnce(
                desired_vel, v_in, t_step,
                planning::OnLaneForwardSimulation::Param(),
                &desired_state) != kSuccess) {
          return kWrongStatus;
        }
      }
      pred_states->push_back(desired_state);
      v_in.set_state(desired_state);  // 更新车辆状态用于下一次传播
    }
    return kSuccess;
  }

 private:
};

}  // namespace planning

#endif  // _CORE_MOTION_PREDICTOR_INC_ONLANE_FORWARD_SIMULATION_PREDICTOR_H_
