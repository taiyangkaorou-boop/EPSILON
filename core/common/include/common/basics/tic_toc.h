/**
 * @file tic_toc.h
 * @author HKUST Aerial Robotics Group
 * @brief EPSILON 自动驾驶决策规划系统 — 高精度计时器工具类
 *
 * @details
 * 本文件实现了基于 std::chrono 的轻量级计时器类 TicToc。
 * 参考 MATLAB 中 tic/toc 的命名和使用方式：
 *   - tic() 记录起始时间点
 *   - toc() 返回自最近一次 tic() 以来的时间差（毫秒）
 *
 * 使用场景：
 *   - 代码段性能分析（性能基准测试）
 *   - 函数运行时间统计和日志记录
 *   - 实时性约束验证（确保规划周期满足时间预算）
 *
 * 此外提供静态方法 TimePointToDouble()，将 chrono 时间点转换为双精度浮点数，
 * 便于与其他模块的时间戳系统对接。
 *
 * @note 时间单位为毫秒（ms）。TicToc 在构造时自动调用 tic()，创建即开始计时。
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_BASICS_TIC_TOC_H_
#define _COMMON_INC_COMMON_BASICS_TIC_TOC_H_

#include <chrono>
#include <cstdlib>
#include <ctime>

/**
 * @class TicToc
 * @brief 高精度计时器 — 基于 std::chrono::system_clock 的秒表功能
 *
 * @details
 * 典型的 RAII 风格计时器：
 * @code
 *   TicToc timer;           // 构造即开始计时
 *   // ... 被测试代码 ...
 *   double ms = timer.toc(); // 获取耗时（毫秒）
 * @endcode
 */
class TicToc {
 public:
  /// @brief 构造函数，构造时自动开始计时
  TicToc() { tic(); }

  /**
   * @brief 开始/重置计时（记录当前时间点）
   * @note 时间单位为毫秒（ms）
   */
  void tic() { start = std::chrono::system_clock::now(); }

  /**
   * @brief 获取自最近一次 tic() 以来的耗时
   * @return double 时间差，单位为毫秒（ms）
   */
  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;  // ~ 秒转毫秒
  }

  /**
   * @brief 将 chrono::system_clock::time_point 转换为自 epoch 以来的秒数（双精度）
   *
   * @details
   * 用于将 chrono 内部的时间点格式转换为统一的 double 时间戳，
   * 便于跨模块时间同步和数据持久化。
   *
   * @param t chrono 时间点
   * @return double 自 epoch 以来的秒数
   */
  static double TimePointToDouble(
      const std::chrono::system_clock::time_point& t) {
    auto tt = std::chrono::duration<double>(t.time_since_epoch());
    return tt.count();
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start, end;  ///< 起始/终止时间点
};

#endif
