#ifndef _CORE_FORWARD_SIMULATOR_MULTIMODAL_FORWARD_H__
#define _CORE_FORWARD_SIMULATOR_MULTIMODAL_FORWARD_H__

/// @file        multimodal_forward.h
/// @brief       多模态前向仿真参数查找表（Multi-Modal Forward Parameter Lookup）
/// @details     该文件为前向仿真提供不同激进程度（Aggressiveness Level）下的参数预配置。
///              在自动驾驶的多模态预测或规划中，需要对不同驾驶风格（从保守到激进）
///              进行前向仿真以覆盖可能的未来轨迹。
///
///              激进程度等级（1~5级）：
///              等级1 - 极度保守：大时距、大间距、低加速度
///              等级2 - 保守：    中小时距、中等减速度
///              等级3 - 中性：    标准时距、标准加速度
///              等级4 - 激进：    小时距、高加速度
///              等级5 - 极度激进：非常小时距、非常小间距、高加速度
///
///              主要调整的参数：
///              - kDesiredHeadwayTime（期望时距）：越小越激进，跟车越紧
///              - kMinimumSpacing（最小间距）：越小越激进，停车间距越小
///              - kAcceleration（加速度）：越大越激进，起步/加速更快
///              - kComfortableBrakingDeceleration（舒适减速度）：越大越激进，制动越强
///              - steer_control_gain（转向增益）：控制预瞄距离与速度的比例

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"

#include "forward_simulator/onlane_forward_simulation.h"

namespace planning {

/// @class MultiModalForward
/// @brief 多模态前向仿真参数查询类
/// @details 提供不同激进程度等级下的前向仿真参数查询。
///          通过传入激进程度等级（1~5），查找并填充对应的OnLaneForwardSimulation::Param。
///
///          使用示例：
///          @code
///          OnLaneForwardSimulation::Param param;
///          MultiModalForward::ParamLookUp(3, &param);  // 填入中性驾驶风格参数
///          OnLaneForwardSimulation::PropagateOnce(stf, ego, leading, dt, param, &state);
///          @endcode
class MultiModalForward {
 public:
  using Lane = common::Lane;
  using VehicleSet = common::VehicleSet;
  using GridMap = common::GridMapND<uint8_t, 2>;
  using State = common::State;
  typedef int AggressivenessLevel;  ///< 激进程度等级，1最保守，5最激进

  /// @brief 根据激进程度等级查找并填充仿真参数
  /// @param agg_level 激进程度等级（1~5），合法值范围外触发assert
  /// @param param 输出：填充好的OnLaneForwardSimulation仿真参数
  /// @return kSuccess 表示查找成功
  /// @details 各等级下的参数变化趋势：
  ///          - 等级1：时距2.0s, 最小间距2.5m, 加速度1.0m/s^2, 舒适减速度1.0m/s^2
  ///          - 等级2：时距1.7s, 最小间距2.5m, 加速度1.0m/s^2, 舒适减速度1.67m/s^2
  ///          - 等级3：时距1.5s, 最小间距2.5m, 加速度2.0m/s^2, 舒适减速度3.0m/s^2
  ///          - 等级4：时距1.0s, 最小间距1.5m, 加速度2.0m/s^2, 舒适减速度3.0m/s^2
  ///          - 等级5：时距0.5s, 最小间距1.0m, 加速度2.0m/s^2, 舒适减速度3.0m/s^2
  ///          转向增益在所有等级中保持一致（2.0）。
  static ErrorType ParamLookUp(const AggressivenessLevel& agg_level,
                               OnLaneForwardSimulation::Param* param) {
    switch (agg_level) {
      case 1:  // 极度保守
        param->idm_param.kDesiredHeadwayTime = 2.0;
        param->idm_param.kMinimumSpacing = 2.5;
        param->idm_param.kAcceleration = 1.0;
        param->idm_param.kComfortableBrakingDeceleration = 1.0;
        param->steer_control_gain = 2.0;
        break;
      case 2:  // 保守
        param->idm_param.kDesiredHeadwayTime = 1.7;
        param->idm_param.kMinimumSpacing = 2.5;
        param->idm_param.kAcceleration = 1.0;
        param->idm_param.kComfortableBrakingDeceleration = 1.67;
        param->steer_control_gain = 2.0;
        break;
      case 3:  // 中性
        param->idm_param.kDesiredHeadwayTime = 1.5;
        param->idm_param.kMinimumSpacing = 2.5;
        param->idm_param.kAcceleration = 2.0;
        param->idm_param.kComfortableBrakingDeceleration = 3.0;
        param->steer_control_gain = 2.0;
        break;
      case 4:  // 激进
        param->idm_param.kDesiredHeadwayTime = 1.0;
        param->idm_param.kMinimumSpacing = 1.5;
        param->idm_param.kAcceleration = 2.0;
        param->idm_param.kComfortableBrakingDeceleration = 3.0;
        param->steer_control_gain = 2.0;
        break;
      case 5:  // 极度激进
        param->idm_param.kDesiredHeadwayTime = 0.5;
        param->idm_param.kMinimumSpacing = 1.0;
        param->idm_param.kAcceleration = 2.0;
        param->idm_param.kComfortableBrakingDeceleration = 3.0;
        param->steer_control_gain = 2.0;
        break;
      default:
        assert(false);  // 非法激进等级
        break;
    }
    return kSuccess;
  }

 private:
};

}  // namespace planning

#endif
