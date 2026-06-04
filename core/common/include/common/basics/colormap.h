/**
 * @file colormap.h
 * @author HKUST Aerial Robotics Group
 * @brief EPSILON 自动驾驶决策规划系统 — 颜色映射工具头文件
 *
 * @details
 * 本文件定义了 EPSILON 可视化系统中使用的颜色类型和颜色映射函数。
 * 主要用于在可视化界面（如 RViz 或基于 OpenCV 的自定义渲染）中
 * 根据数据值（速度、代价、概率等）映射到对应的颜色。
 *
 * 颜色体系：
 *   - ColorARGB：ARGB 颜色类型（Alpha + RGB），浮点范围 [0, 1]
 *   - cmap：命名颜色表（通过字符串名称获取颜色）
 *   - jet_map：Jet 颜色映射表（蓝 -> 青 -> 绿 -> 黄 -> 红，用于热力图）
 *   - autumn_map：Autumn 颜色映射表（红 -> 黄渐变）
 *
 * 颜色映射流程：
 *   1. 将数据值归一化到 [min, max] 范围
 *   2. 通过查找表（LUT）或连续映射函数将归一化值映射到颜色
 *   3. 返回 ColorARGB 对象用于渲染
 *
 * @note ColorARGB 的 a (alpha) 分量默认为 0.0（不透明时为 1.0）。
 *       set_a() 方法返回新对象而非修改当前对象（函数式风格），用于链式调用。
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_BASICS_COLORMAP_H__
#define _COMMON_INC_COMMON_BASICS_COLORMAP_H__

#include <map>

#include "common/basics/basics.h"

namespace common {

/**
 * @struct ColorARGB
 * @brief ARGB 颜色表示 — 四个浮点分量范围均为 [0, 1]
 *
 * @details
 * 采用 ARGB 顺序（Alpha / Red / Green / Blue），与 OpenCV 的 Scalar BGR 顺序不同。
 * 在使用时需注意颜色通道顺序的转换。
 *
 * 用例：
 * @code
 *   ColorARGB red(1.0, 1.0, 0.0, 0.0);  // 不透明红色
 *   ColorARGB semi_blue = red.set_a(0.5);  // 半透明蓝色
 * @endcode
 */
struct ColorARGB {
  decimal_t a;  ///< Alpha 通道（透明度），0.0=完全透明, 1.0=完全不透明
  decimal_t r;  ///< 红色通道，范围 [0, 1]
  decimal_t g;  ///< 绿色通道，范围 [0, 1]
  decimal_t b;  ///< 蓝色通道，范围 [0, 1]

  /**
   * @brief 完整参数构造函数
   * @param A Alpha 值
   * @param R 红色值
   * @param G 绿色值
   * @param B 蓝色值
   */
  ColorARGB(decimal_t A, decimal_t R, decimal_t G, decimal_t B)
      : a(A), r(R), g(G), b(B) {}

  /// @brief 默认构造函数（全透明黑色）
  ColorARGB() : a(0.0), r(0.0), g(0.0), b(0.0) {}

  /**
   * @brief 设置 Alpha 值并返回新对象（函数式风格，不修改原对象）
   * @param _a 新的 Alpha 值
   * @return ColorARGB 新的颜色对象
   */
  ColorARGB set_a(const decimal_t _a) { return ColorARGB(_a, r, g, b); }
};

/// @brief 命名颜色表：通过字符串名称（如 "red", "blue"）获取颜色
extern std::map<std::string, ColorARGB> cmap;
/// @brief Jet 颜色映射表：蓝 -> 青 -> 绿 -> 黄 -> 红（热力图标准配色）
extern std::map<decimal_t, ColorARGB> jet_map;
/// @brief Autumn 颜色映射表：红 -> 黄渐变
extern std::map<decimal_t, ColorARGB> autumn_map;

/**
 * @brief 通过查找表（LUT）将数值映射为颜色
 *
 * @details
 * 在离散化的颜色查找表中，通过二分查找找到 val 对应的颜色。
 * 若 val 超出表的最小/最大键范围，则返回最接近的边界颜色。
 *
 * @param val 输入数值
 * @param m 颜色映射表（key=数值阈值, value=对应颜色）
 * @return ColorARGB 映射后的颜色
 */
ColorARGB GetColorByValue(const decimal_t val,
                          const std::map<decimal_t, ColorARGB>& m);

/**
 * @brief 通过连续映射函数将数值映射为 Jet 颜色
 *
 * @details
 * Jet 颜色映射是科学可视化中最常用的配色方案之一：
 *   最小值 -> 蓝色
 *   中间值 -> 绿色
 *   最大值 -> 红色
 *   中间过渡为青色和黄色
 *
 * 映射公式将 val 归一化到 [min, max] 区间后通过分段函数
 * 计算 RGB 值，实现光滑的颜色过渡。
 *
 * @param val 输入数值
 * @param max 数值范围最大值
 * @param min 数值范围最小值
 * @return ColorARGB Jet 颜色
 */
ColorARGB GetJetColorByValue(const decimal_t val, const decimal_t max,
                             const decimal_t min);

}  // namespace common

#endif
