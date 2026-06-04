/**
 * @file tool_func.cc
 * @author HKUST Aerial Robotics Group
 * @brief 通用工具函数的实现文件
 *
 * 本文件实现了EPSILON自动驾驶规划系统中通用的工具函数，
 * 主要包括字符串处理函数和组合数学相关的函数。
 *
 * 核心功能包括：
 * - SplitString: 根据分隔符将字符串拆分为子字符串向量
 * - GetAllCombinations: 计算多个集合的笛卡尔积（所有可能的组合）
 *   递归遍历输入向量的每个元素，生成所有可能的组合
 *
 * 这些工具函数在整个规划系统中被广泛使用，用于配置解析、
 * 参数组合枚举等场景。
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/basics/tool_func.h"

namespace common {

/*
 * SplitString - 根据分隔符拆分字符串
 *
 * 算法：使用双指针法在字符串中查找分隔符出现的位置，
 * 将分隔符之间的子字符串提取出来放入结果向量中。
 *
 * 例如：SplitString("a,b,c", ",", &v) → v = ["a", "b", "c"]
 *
 * @param s 待拆分的输入字符串
 * @param c 分隔符字符串（可以是一个或多个字符）
 * @param v 输出参数，存储拆分结果的字符串向量
 */
void SplitString(const std::string& s, const std::string& c,
                 std::vector<std::string>* v) {
  std::string::size_type pos1, pos2;
  pos2 = s.find(c);      // 查找第一个分隔符的位置
  pos1 = 0;              // 当前子串的起始位置
  while (std::string::npos != pos2) {  // 当找到分隔符时继续循环
    v->push_back(s.substr(pos1, pos2 - pos1));  // 提取 pos1 到 pos2 之间的子串

    pos1 = pos2 + c.size();  // 更新下一个子串起始位置（跳过分隔符）
    pos2 = s.find(c, pos1);  // 继续查找下一个分隔符
  }
  // 处理最后一个子串（分隔符之后的部分）
  if (pos1 != s.length()) v->push_back(s.substr(pos1));
}

/*
 * GetResultInVector - 递归获取所有组合的内部辅助函数
 *
 * 使用深度优先搜索（DFS）遍历所有可能的组合：
 * 1. 从第N组元素开始，依次将每个元素加入临时结果
 * 2. 如果还没有处理完所有组（N < vec.size() - 1），递归处理下一组
 * 3. 如果已经处理完所有组，将当前临时结果作为一个完整组合保存
 * 4. 回溯：移除当前元素，尝试下一个元素
 *
 * 例如：
 *   输入 vec = [[1,2], [3,4], [5]]
 *   输出 result = [[1,3,5], [1,4,5], [2,3,5], [2,4,5]]
 *
 * @param vec 输入的分组元素向量，每个子向量代表一组可选的元素
 * @param N 当前递归深度（当前处理的分组索引）
 * @param tmp 临时存储当前正在构建的单个组合
 * @param tmp_result 输出参数，存储所有完整组合的向量
 */
void GetResultInVector(const std::vector<std::vector<int>>& vec, const int& N,
                       std::vector<int>* tmp,
                       std::vector<std::vector<int>>* tmp_result) {
  // 遍历第N组的所有可能元素
  for (int i = 0; i < vec[N].size(); ++i) {
    tmp->push_back(vec[N][i]);  // 选择第N组的第i个元素
    if (N < vec.size() - 1) {
      // 还有下一组未处理，递归处理
      GetResultInVector(vec, N + 1, tmp, tmp_result);
    } else {
      // 已处理完所有组，保存当前组合
      std::vector<int> one_result;
      for (int i = 0; i < tmp->size(); ++i) {
        one_result.push_back(tmp->at(i));
      }
      tmp_result->push_back(one_result);
    }
    tmp->pop_back();  // 回溯：移除当前元素，准备尝试下一个元素
  }
}

/*
 * GetAllCombinations - 计算多个集合的笛卡尔积（所有可能的组合）
 *
 * 给定一个包含多个可选值集合的向量 vec_in，其中每个子向量代表
 * 对应位置的可选值，计算所有可能的取值组合。
 *
 * 这是 GetResultInVector 的入口函数，初始化递归搜索。
 *
 * @param vec_in 输入的分组元素向量
 * @param res 输出参数，所有可能的组合
 */
void GetAllCombinations(const std::vector<std::vector<int>>& vec_in,
                        std::vector<std::vector<int>>* res) {
  std::vector<int> tmp_vec;
  GetResultInVector(vec_in, 0, &tmp_vec, res);  // 从第0组开始递归搜索
}

}  // namespace common
