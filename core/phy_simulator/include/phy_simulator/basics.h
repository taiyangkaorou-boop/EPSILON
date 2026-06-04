/**
 * @file basics.h
 * @author HKUST Aerial Robotics Group
 * @brief phy_simulator 模块辅助类型定义和公共头文件汇总
 *
 * @version 0.1
 * @date 2019-03-18
 *
 * @copyright Copyright (c) 2019
 *
 * 本文件是 phy_simulator 模块的基础头文件汇总，充当模块内部代码的公共前置声明。
 * 它集中引入了以下依赖：
 *   - common/basics/basics.h     —— 基础类型（Vec3f、Point、Polygon 等）
 *   - common/basics/semantics.h  —— 语义层类型（VehicleSet、LaneNet、ObstacleSet 等）
 *   - common/state/free_state.h  —— 自由状态类型（FreeState，无约束的状态表示）
 *   - common/state/state.h       —— 车辆状态类型（State，包含位置、速度、加速度等）
 *   - Eigen 几何库 —— 用于矩阵/向量运算（Eigen::Vector2d 等）
 *
 * 本头文件自身不定义新的类型，仅作为模块内其他文件的统一 include 入口。
 * 其他头文件（如 phy_simulator.h、arena_loader.h）通过包含本文件来获取所需的基础类型，
 * 避免了在每个文件中重复书写 include 语句，便于维护统一的依赖关系。
 *
 * 注意：ros_adapter.h 和 visualizer.h 有独立的 ROS 依赖路径，
 * 不通过本文件引入 ROS 相关依赖。
 */
#ifndef _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_BASICS_H_
#define _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_BASICS_H_

#include <assert.h>
#include <iostream>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"

namespace phy_simulator {}  // namespace phy_simulator

#endif  // _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_BASICS_H_
