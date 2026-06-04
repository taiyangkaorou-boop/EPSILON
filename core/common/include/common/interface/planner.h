/**
 * @file planner.h
 * @brief 规划器抽象基类（Abstract Base Class）
 *
 * 定义了 EPSILON 自动驾驶系统中所有规划器的统一接口。这是一个典型的
 * 策略模式（Strategy Pattern）设计——所有具体的规划器实现（如行为规划器、
 * 运动规划器、轨迹规划器等）都继承自该基类，并实现其纯虚函数接口。
 *
 * 规划器的生命周期：
 *   1. 构造 Planner 对象
 *   2. 调用 Init(config) 进行初始化（加载配置参数、设置数据结构等）
 *   3. 在每个规划周期，调用 RunOnce() 执行一次规划计算
 *   4. 通过 Name() 获取规划器标识（用于日志和调试）
 *
 * 当前继承该基类的规划器包括：
 *   - BehaviorPlanner：行为规划器，负责高层行为决策（保持/换道/加速/减速）
 *   - EudmPlanner：EU-DM（Efficient Uncertainty-aware Decision Making）规划器，
 *     基于不确定性感知的决策方法
 *   - SscPlanner：SSC（Sampling-based Stochastic Control）规划器，
 *     基于采样的随机控制方法
 *
 * @note 这是一个纯抽象类（接口），不可直接实例化
 * @note 所有派生类必须实现 Name(), Init(), RunOnce() 三个纯虚函数
 *
 * @author EPSILON Autonomous Driving Team
 * @date 2026
 */

#ifndef _COMMON_INC_COMMON_INTERFACE_PLANNER_H__
#define _COMMON_INC_COMMON_INTERFACE_PLANNER_H__

#include <string>

#include "common/basics/basics.h"

namespace planning {

/**
 * @class Planner
 * @brief 规划器抽象基类
 *
 * 所有路径规划器、运动规划器和行为规划器的统一抽象接口。
 * 派生类通过重写虚函数实现具体的规划算法。
 */
class Planner {
 public:
  /// 默认构造函数
  Planner() = default;

  /// 虚析构函数，确保派生类对象能被正确释放
  virtual ~Planner() = default;

  /**
   * @brief 返回规划器的名称标识
   *
   * 每个具体的规划器实现应返回唯一的名称字符串，
   * 用于日志输出、性能分析和调试。
   *
   * @return std::string 规划器名称（如 "BehaviorPlanner"、"EudmPlanner" 等）
   */
  virtual std::string Name() = 0;

  /**
   * @brief 初始化规划器
   *
   * 加载配置文件、初始化数据结构、注册回调函数等。
   * 在规划器首次运行前必须调用一次。
   *
   * @param config 配置文件路径或配置字符串（格式由具体实现决定）
   * @return ErrorType 初始化状态，kSuccess 表示成功
   *
   * @note 如果初始化失败，后续的 RunOnce() 调用结果不确定
   */
  virtual ErrorType Init(const std::string config) = 0;

  /**
   * @brief 执行一次完整的规划计算
   *
   * 这是规划器的主循环入口。在每个规划周期（通常与传感器更新频率同步），
   * 系统调用此函数执行一次规划迭代，包括：
   *   - 读取最新传感器数据和环境信息
   *   - 进行行为决策或轨迹规划
   *   - 输出规划结果（轨迹、控制指令等）
   *
   * @return ErrorType 执行状态，kSuccess 表示规划成功
   *
   * @note 该函数的典型执行周期为 100ms 或更短，取决于具体实现
   */
  virtual ErrorType RunOnce() = 0;
};

}  // namespace planning

#endif
