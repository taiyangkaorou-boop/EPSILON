/**
 * @file tool_func.h
 * @author HKUST Aerial Robotics Group
 * @brief EPSILON 自动驾驶决策规划系统 — 通用工具函数头文件
 *
 * @details
 * 本文件定义了 EPSILON 中所有模块共享的通用工具函数。
 * 这些函数不依赖于特定的业务逻辑，提供以下能力：
 *   - 字符串处理：SplitString（字符串分割，用于解析配置文件/数据流）
 *   - 组合生成：GetAllCombinations / GetResultInVector（全组合枚举，用于候选决策生成）
 *   - 格式化输出：GetStringByValueWithPrecision（带精度的数值转字符串）
 *   - 范围向量生成：GetRangeVector（生成等差数列，用于参数空间采样）
 *
 * 函数分为两类：
 *   - 非模板函数：声明在头文件，实现在对应的 .cc 文件中
 *   - 模板函数：直接在头文件中实现（header-only）
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_BASICS_TOOL_FUNC_H__
#define _COMMON_INC_COMMON_BASICS_TOOL_FUNC_H__

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "common/basics/basics.h"

namespace common {

/**
 * @brief 按指定分隔符分割字符串
 *
 * @details
 * 将输入字符串 s 按照分隔符 c 分割为多个子串，存入向量 v 中。
 * 连续的多个分隔符不会产生空字符串（即不保留空白段）。
 *
 * 使用示例：
 * @code
 *   std::vector<std::string> parts;
 *   SplitString("lane1,lane2,lane3", ",", &parts);
 *   // parts = {"lane1", "lane2", "lane3"}
 * @endcode
 *
 * @param s 输入字符串
 * @param c 分隔符（可以是单字符或多字符）
 * @param v [out] 分割结果向量
 */
void SplitString(const std::string& s, const std::string& c,
                 std::vector<std::string>* v);

/**
 * @brief 递归辅助函数 — 生成向量组合的中间结果
 *
 * @details
 * 这是 GetAllCombinations 的内部递归辅助函数。
 * 使用回溯法（Backtracking）生成所有可能的组合：
 *   从第 N 层开始，依次将当前层的每个候选值加入临时结果 tmp，
 *   递归进入下一层 (N+1)，到达最后一层时将 tmp 存入 tmp_result。
 *
 * @param vec 输入向量组（每组是一个候选值列表）
 * @param N 当前递归层级
 * @param tmp 当前构造中的组合
 * @param tmp_result [out] 已完成的组合列表
 */
void GetResultInVector(const std::vector<std::vector<int>>& vec, const int& N,
                       std::vector<int>* tmp,
                       std::vector<std::vector<int>>* tmp_result);

/**
 * @brief 生成输入向量组的所有组合（笛卡尔积）
 *
 * @details
 * 给定多组候选值，生成所有可能的"从每组选一个值"的组合。
 * 即计算多个集合的笛卡尔积。
 *
 * 使用示例：
 * @code
 *   std::vector<std::vector<int>> input = {{1, 2}, {3, 4}, {5}};
 *   std::vector<std::vector<int>> result;
 *   GetAllCombinations(input, &result);
 *   // result = {{1,3,5}, {1,4,5}, {2,3,5}, {2,4,5}}
 * @endcode
 *
 * 在 EPSILON 中的应用：
 *   - 不同行为（换道/保持） x 不同候选轨迹 = 所有候选决策组合
 *   - 多目标的最优组合搜索
 *
 * @param vec_in 输入向量组（外层:组, 内层:候选值）
 * @param res [out] 所有组合结果
 */
void GetAllCombinations(const std::vector<std::vector<int>>& vec_in,
                        std::vector<std::vector<int>>* res);

/**
 * @brief 将数值转换为指定精度的字符串
 *
 * @details
 * 使用 std::ostringstream + std::fixed + std::setprecision 实现格式化输出。
 * 用于日志记录和可视化标注的数值显示。
 *
 * @tparam T 数值类型（int, double, float 等支持 << 操作符的类型）
 * @param val 数值
 * @param pre 小数精度（小数点后位数）
 * @return std::string 格式化后的字符串
 */
template <typename T>
std::string GetStringByValueWithPrecision(const T& val, const int& pre) {
  std::ostringstream os;
  os << std::fixed;
  os << std::setprecision(pre);
  os << val;
  return os.str();
}

/**
 * @brief 生成等差数列向量 [lb, ub) 或 [lb, ub]（包含上界）
 *
 * @details
 * 从下界 lb 开始，以 step 为步长生成等差数列。
 * 若 if_inc_tail 为 true，则额外将 ub 附加到结果末尾（保证上界被包含）。
 *
 * 使用示例：
 * @code
 *   std::vector<double> samples;
 *   GetRangeVector(0.0, 1.0, 0.3, true, &samples);
 *   // samples = {0.0, 0.3, 0.6, 0.9, 1.0}
 * @endcode
 *
 * 在 EPSILON 中的应用：
 *   - 速度/加速度采样空间生成
 *   - 参数网格化搜索
 *   - 时间离散化
 *
 * @tparam T 数值类型
 * @param lb 下界
 * @param ub 上界
 * @param step 步长
 * @param if_inc_tail 是否包含上界 ub
 * @param vec [out] 生成的等差数列向量
 */
template <typename T>
void GetRangeVector(const T& lb, const T& ub, const T& step,
                    const bool& if_inc_tail, std::vector<T>* vec) {
  vec->clear();
  // ~ 使用 ceil 和 kEPS 避免浮点舍入导致最后一步丢失
  int num = std::ceil((ub - lb - kEPS) / step);
  for (int i = 0; i < num; ++i) {
    vec->push_back(lb + i * step);
  }
  // ~ 若需要包含上界，将 ub 直接加入末尾
  if (if_inc_tail) {
    vec->push_back(ub);
  }
}

}  // namespace common

#endif  // _COMMON_INC_COMMON_BASICS_TOOL_FUNC_H__
