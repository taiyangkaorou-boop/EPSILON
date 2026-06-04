/**
 * @file colormap.cc
 * @author HKUST Aerial Robotics Group
 * @brief 颜色映射表的实现文件
 *
 * 本文件实现了EPSILON自动驾驶规划系统中用于可视化的颜色映射功能。
 * 提供预定义的命名颜色表、jet颜色渐变色表、autumn颜色渐变色表，
 * 以及根据标量值进行颜色插值查询的工具函数。
 *
 * 核心功能包括：
 * - 命名颜色表（cmap）：提供常用颜色的名称到ARGB颜色的映射
 *   （如black, white, red, green, blue, yellow等）
 * - Jet渐变色表（jet_map）：从蓝色(0.0)过渡到青色、绿色、黄色、红色(1.0)
 *   ，常用于科学可视化
 * - Autumn渐变色表（autumn_map）：从黄色(1.0)过渡到红色(0.0)，
 *   常用于表示衰减或危险程度
 * - GetColorByValue: 在离散颜色映射表中按值查找颜色
 * - GetJetColorByValue: 在指定的[min, max]范围内进行jet颜色插值，
 *   支持连续渐变
 *
 * ColorARGB格式：使用 (alpha, red, green, blue) 四通道，
 * 每个通道取值范围 [0.0, 1.0]
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/basics/colormap.h"

namespace common {

/*
 * 命名颜色映射表
 * 键为颜色名称字符串，值为 ColorARGB(alpha, red, green, blue)
 * 每个通道范围 [0.0, 1.0]，alpha=1.0 表示完全不透明
 */
std::map<std::string, ColorARGB> cmap{
    {"black", ColorARGB(1.0, 0.0, 0.0, 0.0)},         // 纯黑
    {"white", ColorARGB(1.0, 1.0, 1.0, 1.0)},         // 纯白
    {"red", ColorARGB(1.0, 1.0, 0.0, 0.0)},            // 纯红
    {"hot pink", ColorARGB(1.0, 1.0, 0.44, 0.70)},     // 热粉红
    {"green", ColorARGB(1.0, 0.0, 1.0, 0.0)},          // 纯绿
    {"blue", ColorARGB(1.0, 0.0, 0.0, 1.0)},           // 纯蓝
    {"aqua marine", ColorARGB(1.0, 0.5, 1.0, 0.83)},   // 浅水绿
    {"yellow", ColorARGB(1.0, 1.0, 1.0, 0.0)},         // 纯黄
    {"cyan", ColorARGB(1.0, 0.0, 1.0, 1.0)},           // 青色
    {"magenta", ColorARGB(1.0, 1.0, 0.0, 1.0)},        // 品红
    {"violet", ColorARGB(1.0, 0.93, 0.43, 0.93)},      // 紫罗兰
    {"orange red", ColorARGB(1.0, 1.0, 0.275, 0.0)},   // 橙红
    {"orange", ColorARGB(1.0, 1.0, 0.65, 0.0)},        // 橙色
    {"dark orange", ColorARGB(1.0, 1.0, 0.6, 0.0)},    // 深橙色
    {"gold", ColorARGB(1.0, 1.0, 0.84, 0.0)},          // 金色
    {"green yellow", ColorARGB(1.0, 0.5, 1.0, 0.0)},   // 黄绿色
    {"forest green", ColorARGB(1.0, 0.13, 0.545, 0.13)}, // 森林绿
    {"spring green", ColorARGB(1.0, 0.0, 1.0, 0.5)},   // 春绿色
    {"sky blue", ColorARGB(1.0, 0.0, 0.749, 1.0)},     // 天蓝色
    {"medium orchid", ColorARGB(1.0, 0.729, 0.333, 0.827)}, // 中兰紫
    {"grey", ColorARGB(1.0, 0.5, 0.5, 0.5)}};          // 灰色

/*
 * Jet颜色渐变色表
 * jet渐变色是MATLAB风格的冷暖渐变方案：
 *   0.0  → 深蓝色
 *   0.2  → 蓝色
 *   0.4  → 青色
 *   0.6  → 绿色
 *   0.8  → 黄色
 *   1.0  → 红色
 *
 * 常用于表示物理量从低到高的渐变。这里使用离散采样点表示。
 */
std::map<decimal_t, ColorARGB> jet_map{
    {0.0, ColorARGB(1.0, 0.0, 0.0, 0.6667)},     // 深蓝
    {0.1, ColorARGB(1.0, 0.0, 0.0, 1.0000)},     // 蓝
    {0.2, ColorARGB(1.0, 0.0, 0.3333, 1.0000)},  // 蓝青过渡
    {0.3, ColorARGB(1.0, 0.0, 0.6667, 1.0000)},  // 浅青
    {0.4, ColorARGB(1.0, 0.0, 1.0000, 1.0000)},  // 青色
    {0.5, ColorARGB(1.0, 0.3333, 1.0000, 0.6667)}, // 青绿过渡
    {0.6, ColorARGB(1.0, 0.6667, 1.0000, 0.3333)}, // 绿黄过渡
    {0.7, ColorARGB(1.0, 1.0000, 1.0000, 0.0)},   // 黄色
    {0.8, ColorARGB(1.0, 1.0000, 0.6667, 0.0)},   // 黄橙过渡
    {0.9, ColorARGB(1.0, 1.0000, 0.3333, 0.0)},   // 橙红过渡
    {1.0, ColorARGB(1.0, 1.0000, 0.0, 0.0)}};    // 红色

/*
 * Autumn颜色渐变色表
 * autumn渐变色是一种从黄色到红色的暖色调渐变：
 *   1.0  → 红色（表示高值/危险）
 *   0.0  → 黄色（表示低值/安全）
 *
 * 常用于表示与距离或时间衰减相关的量，
 * 或者在仿真中表示危险程度从高(红)到低(黄)的变化。
 */
std::map<decimal_t, ColorARGB> autumn_map{
    {1.0, ColorARGB(0.5, 1.0, 0.0, 0.0)},    // 红色(半透明)
    {0.95, ColorARGB(0.5, 1.0, 0.05, 0.0)},  // 偏红
    {0.9, ColorARGB(0.5, 1.0, 0.1, 0.0)},
    {0.85, ColorARGB(0.5, 1.0, 0.15, 0.0)},
    {0.8, ColorARGB(0.5, 1.0, 0.2, 0.0)},
    {0.75, ColorARGB(0.5, 1.0, 0.25, 0.0)},
    {0.7, ColorARGB(0.5, 1.0, 0.3, 0.0)},
    {0.65, ColorARGB(0.5, 1.0, 0.35, 0.0)},
    {0.6, ColorARGB(0.5, 1.0, 0.4, 0.0)},
    {0.55, ColorARGB(0.5, 1.0, 0.45, 0.0)},
    {0.5, ColorARGB(0.5, 1.0, 0.5, 0.0)},    // 橙色(半透明)
    {0.45, ColorARGB(0.5, 1.0, 0.55, 0.0)},
    {0.4, ColorARGB(0.5, 1.0, 0.6, 0.0)},
    {0.35, ColorARGB(0.5, 1.0, 0.65, 0.0)},
    {0.3, ColorARGB(0.5, 1.0, 0.7, 0.0)},
    {0.25, ColorARGB(0.5, 1.0, 0.75, 0.0)},
    {0.2, ColorARGB(0.5, 1.0, 0.8, 0.0)},
    {0.15, ColorARGB(0.5, 1.0, 0.85, 0.0)},
    {0.1, ColorARGB(0.5, 1.0, 0.9, 0.0)},
    {0.05, ColorARGB(0.5, 1.0, 0.95, 0.0)},
    {0.0, ColorARGB(0.5, 1.0, 1.0, 0.0)}};   // 黄色(半透明)

/*
 * GetColorByValue - 在离散颜色映射表中按值查找颜色
 *
 * 使用 std::map::upper_bound 查找第一个键值大于 val 的元素，
 * 返回该元素对应的颜色值。这意味着对于小于映射表最小键值的情况，
 * 会返回最小键对应的颜色；对于大于最大键值的情况，返回end()对应的颜色。
 *
 * @param val 查询的标量值
 * @param m 颜色映射表（键值 → 颜色）
 * @return 对应值所在区间的颜色（取upper bound处的颜色）
 */
ColorARGB GetColorByValue(const decimal_t val,
                          const std::map<decimal_t, ColorARGB>& m) {
  // upper_bound 返回第一个键值 > val 的迭代器
  auto it = m.upper_bound(val);
  return it->second;
}

/*
 * GetJetColorByValue - 在指定范围内进行jet颜色的连续插值
 *
 * 将标量值 val_in 限制在 [vmin, vmax] 范围内，
 * 然后在蓝-青-绿-黄-红渐变中进行分段线性插值：
 *
 * 颜色分段方案（归一化值 t = (val - vmin) / (vmax - vmin)）：
 *   t ∈ [0.00, 0.25): 蓝色→青色  (r=0, g增长, b=1)
 *   t ∈ [0.25, 0.50): 青色→绿色  (r=0, g=1, b递减)
 *   t ∈ [0.50, 0.75): 绿色→黄色  (r增长, g=1, b=0)
 *   t ∈ [0.75, 1.00]: 黄色→红色  (r=1, g递减, b=0)
 *
 * @param val_in 输入的标量值
 * @param vmax 颜色映射范围的上限（对应红色）
 * @param vmin 颜色映射范围的下限（对应蓝色）
 * @return 插值得到的 ARGB 颜色
 */
ColorARGB GetJetColorByValue(const decimal_t val_in, const decimal_t vmax,
                             const decimal_t vmin) {
  decimal_t val = val_in;
  if (val < vmin) val = vmin;  // 值钳位到下界
  if (val > vmax) val = vmax;  // 值钳位到上界
  double dv = vmax - vmin;    // 总范围

  ColorARGB c(1.0, 1.0, 1.0, 1.0);
  if (val < (vmin + 0.25 * dv)) {
    // 区间1: 蓝色→青色 (g从0增长到1)
    c.r = 0;
    c.g = 4.0 * (val - vmin) / dv;
  } else if (val < (vmin + 0.5 * dv)) {
    // 区间2: 青色→绿色 (b从1递减到0)
    c.r = 0;
    c.b = 1.0 + 4.0 * (vmin + 0.25 * dv - val) / dv;
  } else if (val < (vmin + 0.75 * dv)) {
    // 区间3: 绿色→黄色 (r从0增长到1)
    c.r = 4.0 * (val - vmin - 0.5 * dv) / dv;
    c.b = 0.0;
  } else {
    // 区间4: 黄色→红色 (g从1递减到0)
    c.g = 1 + 4.0 * (vmin + 0.75 * dv - val) / dv;
    c.b = 0.0;
  }
  return c;
}

}  // namespace common
