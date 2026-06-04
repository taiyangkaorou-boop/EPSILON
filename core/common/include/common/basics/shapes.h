/**
 * @file shapes.h
 * @author HKUST Aerial Robotics Group
 * @brief EPSILON 自动驾驶决策规划系统 — 几何图元定义与碰撞检测头文件
 *
 * @details
 * 本文件定义了 EPSILON 系统中所有的几何基础图元（点、线、圆、多边形、包围盒）
 * 以及基于这些图元的碰撞检测工具类 ShapeUtils。
 *
 * 几何图元体系：
 *   - 基础图元：Point（三维点）、Point2i（二维整数点）、PointWithValue<T>（带值点）
 *   - 一维图元：PolyLine（折线，有序点序列+方向）
 *   - 二维图元：Circle（圆）、Polygon（多边形）
 *   - 包围盒：
 *       OrientedBoundingBox2D（二维有向包围盒，用于车辆碰撞检测）
 *       AxisAlignedBoundingBoxND<N>（N 维轴对齐包围盒，重心+半边长表示）
 *       AxisAlignedCubeNd<T, N_DIM>（N 维轴对齐立方体，上下边界表示，用于时空碰撞检测）
 *
 * ShapeUtils 提供的碰撞检测算法：
 *   - SAT（分离轴定理）用于 OBB-OBB 碰撞检测
 *   - AABB 包含判断 / 碰撞检测 / 表面相交维度计算
 *   - 坐标类型转换（common::Point2i <-> cv::Point2i）
 *
 * @note 本文件是碰撞检测和可视化模块的核心依赖。OBB 碰撞检测使用 SAT 算法，
 *       AABB 碰撞检测使用区间重叠判断，适用于实时在线计算。
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_BASICS_SHAPES_H__
#define _COMMON_INC_COMMON_BASICS_SHAPES_H__

#include <map>
#include <opencv2/core/core.hpp>

#include "common/basics/basics.h"
#include "common/basics/colormap.h"
#include "common/math/calculations.h"

namespace common {

/**
 * @struct Point
 * @brief 三维浮点坐标点 — EPSILON 中最基础的空间坐标表示
 *
 * @details
 * 表示笛卡尔坐标系下的 (x, y, z) 点。在 2D 应用中 z 分量通常为 0 或忽略。
 * 支持默认构造、2D 构造和 3D 构造。
 */
struct Point {
  decimal_t x = 0.0;  ///< X 坐标（米）
  decimal_t y = 0.0;  ///< Y 坐标（米）
  decimal_t z = 0.0;  ///< Z 坐标（米），2D 应用下通常为 0

  /// @brief 默认构造函数（原点）
  Point();

  /**
   * @brief 二维坐标构造函数
   * @param _x X 坐标
   * @param _y Y 坐标
   */
  Point(decimal_t _x, decimal_t _y);

  /**
   * @brief 三维坐标构造函数
   * @param _x X 坐标
   * @param _y Y 坐标
   * @param _z Z 坐标
   */
  Point(decimal_t _x, decimal_t _y, decimal_t _z);

  /**
   * @brief 打印点坐标（调试用）
   */
  void print() const;
};

/**
 * @struct Point2i
 * @brief 二维整数坐标点 — 用于栅格坐标和像素坐标表示
 *
 * @details
 * 常用于栅格地图中的格数坐标 (col, row) 和图像像素坐标 (u, v)。
 * 与 Point（浮点）的区别在于使用 int 类型，表示离散化坐标。
 */
struct Point2i {
  int x = 0;  ///< X 坐标（整数，格数/像素）
  int y = 0;  ///< Y 坐标（整数，格数/像素）

  /// @brief 默认构造函数（原点）
  Point2i();

  /**
   * @brief 参数化构造函数
   * @param _x X 坐标
   * @param _y Y 坐标
   */
  Point2i(int _x, int _y);

  /**
   * @brief 打印坐标（调试用）
   */
  void print() const;
};

/**
 * @struct PointWithValue
 * @brief 带附加值的点 — 将几何坐标与任意类型的标签/值绑定
 *
 * @details
 * 模板化的点+值组合，常用于：
 *   - 车道采样点 + 车道 ID 标签
 *   - KD树查询：通过最近邻找到点的位置和对应的语义值
 *
 * @tparam T 值的类型（如 int 表示车道ID、double 表示代价等）
 */
template <typename T>
struct PointWithValue {
  Point pt;                  ///< 空间坐标
  std::vector<T> values;     ///< 附加的值列表（支持多值标签）
};

/**
 * @struct OrientedBoundingBox2D
 * @brief 二维有向包围盒 — EPSILON 碰撞检测的核心几何类型
 *
 * @details
 * OBB（Oriented Bounding Box）区别于 AABB（Axis-Aligned Bounding Box），
 * 其边与坐标轴不一定平行。OBB 的表示方式为 (x, y, angle, width, length)：
 *   - (x, y)：包围盒几何中心的全局坐标
 *   - angle：包围盒纵向轴线与 X 轴正方向的夹角（弧度/度，取决于调用上下文）
 *   - width：包围盒的宽度（沿横向，垂直于 angle 方向）
 *   - length：包围盒的长度（沿纵向，平行于 angle 方向）
 *
 * @note 在车辆表示中，length 沿车辆行驶方向，angle = 车辆航向角 yaw。
 *       碰撞检测使用 SAT（分离轴定理）算法。
 */
struct OrientedBoundingBox2D {
  decimal_t x;       ///< 包围盒中心 X 坐标
  decimal_t y;       ///< 包围盒中心 Y 坐标
  decimal_t angle;   ///< 包围盒方向角（纵向轴线与 X 轴正方向夹角）
  decimal_t width;   ///< 包围盒宽度（横向尺寸）
  decimal_t length;  ///< 包围盒长度（纵向尺寸）

  /// @brief 默认构造函数
  OrientedBoundingBox2D();

  /**
   * @brief 完整参数构造函数
   * @param x_ 中心 X 坐标
   * @param y_ 中心 Y 坐标
   * @param angle_ 方向角（纵向轴线与 X 轴正方向夹角）
   * @param width_ 宽度（横向尺寸）
   * @param length_ 长度（纵向尺寸）
   */
  OrientedBoundingBox2D(const decimal_t x_, const decimal_t y_,
                        const decimal_t angle_, const decimal_t width_,
                        const decimal_t length_);
};

/**
 * @struct AxisAlignedBoundingBoxND
 * @brief N 维轴对齐包围盒 — 通过中心坐标和半边长定义
 *
 * @details
 * 与 OrientedBoundingBox2D 不同，AABB 的边与坐标轴平行，因此仅需
 * 中心坐标和每个维度的长度即可定义。碰撞检测效率极高（仅需比较坐标区间）。
 *
 * 表示方式：
 *   - coord：中心坐标 (center_x, center_y, center_z, ...)
 *   - len：每个维度的长度 (length_x, length_y, length_z, ...)
 *
 * @tparam N 空间维度数
 */
template <int N>
struct AxisAlignedBoundingBoxND {
  std::array<decimal_t, N> coord;  ///< N 维中心坐标
  std::array<decimal_t, N> len;    ///< N 维度长度（每个维度的跨度）

  /// @brief 默认构造函数
  AxisAlignedBoundingBoxND() {}

  /**
   * @brief 参数化构造函数
   * @param coord_ N 维中心坐标
   * @param len_ N 维度长度
   */
  AxisAlignedBoundingBoxND(const std::array<decimal_t, N> coord_,
                           const std::array<decimal_t, N> len_)
      : coord(coord_), len(len_) {}
};

/**
 * @struct AxisAlignedCubeNd
 * @brief N 维轴对齐立方体 — 通过上下边界定义，用于时空碰撞检测
 *
 * @details
 * 与 AxisAlignedBoundingBoxND（中心+长度）不同，此结构使用 [lower_bound, upper_bound]
 * 的方式定义区间。这种方式更适合描述：
 *   - 车道边界（左/右边界）
 *   - 速度/加速度约束范围
 *   - 时间窗口
 *
 * 典型实例：
 *   - AxisAlignedCubeNd<int, 3>：DrivingCube 的边界 (x, y, t)
 *   - AxisAlignedCubeNd<decimal_t, 3>：时空立方体用于碰撞检测 (s, d, t)
 *
 * @tparam T 边界值的类型（int 用于栅格坐标，decimal_t 用于连续坐标）
 * @tparam N_DIM 维度数
 */
template <typename T, int N_DIM>
struct AxisAlignedCubeNd {
  std::array<T, N_DIM> upper_bound;  ///< 各维度上界（包含）
  std::array<T, N_DIM> lower_bound;  ///< 各维度下界（包含）

  /// @brief 默认构造函数
  AxisAlignedCubeNd() = default;

  /**
   * @brief 参数化构造函数
   * @param ub 各维度上界
   * @param lb 各维度下界
   */
  AxisAlignedCubeNd(const std::array<T, N_DIM> ub,
                    const std::array<T, N_DIM> lb)
      : upper_bound(ub), lower_bound(lb) {}
};

/**
 * @struct Circle
 * @brief 圆形 — 用圆心和半径定义的几何图元
 *
 * @details
 * 用于简化表示行人、路锥、圆柱等可用圆近似的对象。
 * 圆形碰撞检测仅需比较圆心距离与半径之和。
 */
struct Circle {
  Point center;        ///< 圆心坐标
  decimal_t radius;    ///< 半径

  /**
   * @brief 打印圆信息（调试用）
   */
  void print() const;
};

/**
 * @struct PolyLine
 * @brief 折线 — 具有方向的有序点序列
 *
 * @details
 * 表示由多个线段首尾连接组成的折线。dir 字段指定折线的方向。
 * 常用于表示车道中心线的离散采样形式。
 */
struct PolyLine {
  int dir;                   ///< 折线方向（1=正向, -1=反向）
  std::vector<Point> points; ///< 有序顶点序列

  /**
   * @brief 打印折线信息（调试用）
   */
  void print() const;
};

/**
 * @struct Polygon
 * @brief 多边形 — 封闭的有序点序列
 *
 * @details
 * 顶点按顺序（顺时针或逆时针）排列，首尾隐含相连形成封闭多边形。
 * 用于表示建筑物的精确轮廓或复杂障碍物边界。
 * 碰撞检测使用 SAT（分离轴定理）或射线法（Ray Casting）。
 */
struct Polygon {
  std::vector<Point> points;  ///< 有序顶点序列（首尾隐式闭合）

  /**
   * @brief 打印多边形信息（调试用）
   */
  void print() const;
};

/**
 * @class ShapeUtils
 * @brief 几何工具类 — 提供碰撞检测和坐标变换的静态函数集合
 *
 * @details
 * 这是 EPSILON 中所有碰撞检测算法的集中实现。包含：
 *   - OBB 碰撞检测（SAT 分离轴定理）
 *   - AABB 包含/碰撞/相交检测
 *   - 坐标类型转换（common <-> OpenCV）
 *
 * 所有函数均为静态方法，无需实例化。
 *
 * SAT (Separating Axis Theorem) 核心思想：
 *   如果两个凸多边形不相交，则存在一条分离轴使得两者在该轴上的投影不重叠。
 *   OBB 只有两个方向轴需要检测（矩形相邻边的法向量），
 *   分别投影到 4 条候选分离轴上判断是否分离。
 */
class ShapeUtils {
 public:
  /**
   * @brief 检测两个 OBB 是否相交（使用 SAT 算法）
   *
   * @details
   * 分离轴定理（SAT）实现：
   *   1. 获取两个 OBB 的顶点
   *   2. 提取候选分离轴（两个 OBB 各两条边的法向量）
   *   3. 将所有顶点投影到轴上
   *   4. 若任一条轴上投影不重叠，则两 OBB 不相交
   *   5. 若所有轴上投影都重叠，则两 OBB 相交
   *
   * @param obb_a OBB A
   * @param obb_b OBB B
   * @return true 相交 / false 不相交
   */
  static bool CheckIfOrientedBoundingBoxIntersect(
      const OrientedBoundingBox2D& obb_a, const OrientedBoundingBox2D& obb_b);

  /**
   * @brief 获取 OBB 的四个顶点坐标
   *
   * @details
   * 根据 OBB 的中心 (x,y)、方向角 angle、宽度 width、长度 length
   * 计算四个角点的坐标（顺序：左上、右上、右下、左下）。
   *
   * @param obb 有向包围盒
   * @param vertices [out] 顶点向量（4个 Vec2f）
   * @return ErrorType
   */
  static ErrorType GetVerticesOfOrientedBoundingBox(
      const OrientedBoundingBox2D& obb, vec_E<Vecf<2>>* vertices);

  /**
   * @brief 获取 OBB 的两条垂直轴（候选分离轴）
   *
   * @details
   * 从 OBB 的 4 个顶点中提取两条相互垂直的边方向向量（归一化后返回）。
   * 这两条轴将作为 SAT 碰撞检测的候选分离轴。
   *
   * @param vertices OBB 的 4 个顶点
   * @param axes [out] 两条垂直轴（归一化方向向量）
   * @return ErrorType
   */
  static ErrorType GetPerpendicularAxesOfOrientedBoundingBox(
      const vec_E<Vecf<2>>& vertices, vec_E<Vecf<2>>* axes);

  /**
   * @brief 计算点集在指定轴上的投影区间
   *
   * @details
   * 将所有顶点投影到轴上，返回投影的最小值和最大值。
   * 即 proj = [min(dot(v_i, axis)), max(dot(v_i, axis))]。
   *
   * @param vertices 顶点集
   * @param axis 投影轴（单位向量）
   * @param proj [out] 投影区间 [min, max]
   * @return ErrorType
   */
  static ErrorType GetProjectionOnAxis(const vec_E<Vecf<2>>& vertices,
                                       const Vecf<2>& axis, Vecf<2>* proj);

  /**
   * @brief 获取 OBB 的第 index 条边的外法向量方向（归一化）
   *
   * @param vertices OBB 顶点
   * @param index 边索引（0=第一条边, 1=第二条边）
   * @param axis [out] 边的外法向量方向
   * @return ErrorType
   */
  static ErrorType GetPerpendicularAxisOfOrientedBoundingBox(
      const vec_E<Vecf<2>>& vertices, const int index, Vecf<2>* axis);

  /**
   * @brief 计算两个一维区间 [a_min, a_max] 和 [b_min, b_max] 的重叠长度
   *
   * @param a 区间 A [min, max]
   * @param b 区间 B [min, max]
   * @param len [out] 重叠长度（若 <= 0 表示不重叠，应视为 0）
   * @return ErrorType
   */
  static ErrorType GetOverlapLength(const Vecf<2> a, const Vecf<2> b,
                                    decimal_t* len);

  /**
   * @brief 将 common::Point2i 向量转换为 OpenCV cv::Point2i 向量
   *
   * @details
   * 用于将 EPSILON 内部的点坐标转换为 OpenCV 可识别的格式，
   * 常用于可视化渲染（通过 OpenCV 绘制点/线/多边形）。
   *
   * @param pts_in 输入的 common::Point2i 向量
   * @param pts_out [out] 输出的 cv::Point2i 向量
   * @return ErrorType
   */
  static ErrorType GetCvPoint2iVecUsingCommonPoint2iVec(
      const std::vector<Point2i>& pts_in, std::vector<cv::Point2i>* pts_out);

  /**
   * @brief 将单个 common::Point2i 转换为 OpenCV cv::Point2i
   *
   * @param pt_in 输入的 common::Point2i
   * @param pt_out [out] 输出的 cv::Point2i
   * @return ErrorType
   */
  static ErrorType GetCvPoint2iUsingCommonPoint2i(const Point2i& pt_in,
                                                  cv::Point2i* pt_out);

  /**
   * @brief 检测 AABB A 是否完全包含 AABB B
   *
   * @details
   * 对于全部 N_DIM 个维度，检查 A 的下界 <= B 的下界 且 A 的上界 >= B 的上界。
   *
   * @tparam T 边界值类型
   * @tparam N_DIM 维度数
   * @param cube_a 候选外层 AABB
   * @param cube_b 候选内层 AABB
   * @return true A 包含 B / false 存在维度上不包含
   */
  template <typename T, int N_DIM>
  static bool CheckIfAxisAlignedCubeAContainsAxisAlignedCubeB(
      const common::AxisAlignedCubeNd<T, N_DIM>& cube_a,
      const common::AxisAlignedCubeNd<T, N_DIM>& cube_b) {
    for (int i = 0; i < N_DIM; ++i) {
      if (cube_a.lower_bound[i] > cube_b.lower_bound[i] ||
          cube_a.upper_bound[i] < cube_b.upper_bound[i]) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief 检测两个 AABB 是否发生碰撞（区间重叠）
   *
   * @details
   * 对于全部 N_DIM 个维度，检查区间 [A_lower, A_upper] 和 [B_lower, B_upper] 是否重叠。
   * 仅当所有维度都重叠时才判定为碰撞。
   * 注意：若一个 AABB 完全包含另一个 AABB，此函数也返回 true。
   *
   * @tparam T 边界值类型
   * @tparam N_DIM 维度数
   * @param aabb_a AABB A
   * @param aabb_b AABB B
   * @return true 碰撞（有重叠）/ false 分离
   */
  template <typename T, int N_DIM>
  static bool CheckIfAxisAlignedCubeCollide(
      const common::AxisAlignedCubeNd<T, N_DIM>& aabb_a,
      const common::AxisAlignedCubeNd<T, N_DIM>& aabb_b) {
    for (int i = 0; i < N_DIM; ++i) {
      decimal_t half_len_a = fabs(
          static_cast<double>(aabb_a.upper_bound[i] - aabb_a.lower_bound[i]) /
          2.0);
      decimal_t half_len_b = fabs(
          static_cast<double>(aabb_b.upper_bound[i] - aabb_b.lower_bound[i]) /
          2.0);

      decimal_t center_a =
          static_cast<double>(aabb_a.upper_bound[i] + aabb_a.lower_bound[i]) /
          2.0;
      decimal_t center_b =
          static_cast<double>(aabb_b.upper_bound[i] + aabb_b.lower_bound[i]) /
          2.0;

      decimal_t len_c = fabs(center_a - center_b);

      // ~ 若中心距 >= 半长之和，则在该维度上分离
      if (fabs(half_len_a + half_len_b) <= len_c) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief 检测两个 AABB 是否"真正相交"（排除包含关系）
   *
   * @details
   * 与 CheckIfAxisAlignedCubeCollide 的区别：
   *   - 排除 A 包含 B 或 B 包含 A 的情况
   *   - 仅在两 AABB 发生交叉（既非分离也非包含）时返回 true
   *   - 同时输出相交时各自参与相交的表面维度
   *
   * inter_dim_a 和 inter_dim_b 是长度为 N_DIM*2 的布尔数组：
   *   - [2*i] = upper_bound 方向参与相交
   *   - [2*i+1] = lower_bound 方向参与相交
   *
   * 这在碰撞响应（如计算排斥力方向）时非常有用。
   *
   * @tparam T 边界值类型
   * @tparam N_DIM 维度数
   * @param aabb_a AABB A
   * @param aabb_b AABB B
   * @param inter_dim_a [out] A 的参与相交的表面维度标志
   * @param inter_dim_b [out] B 的参与相交的表面维度标志
   * @return true 真正相交（交叉）/ false 分离或包含
   */
  template <typename T, int N_DIM>
  static bool CheckIfAxisAlignedCubeNdIntersect(
      const AxisAlignedCubeNd<T, N_DIM>& aabb_a,
      const AxisAlignedCubeNd<T, N_DIM>& aabb_b,
      std::array<bool, N_DIM * 2>* inter_dim_a,
      std::array<bool, N_DIM * 2>* inter_dim_b) {
    inter_dim_a->fill(false);
    inter_dim_b->fill(false);
    // ~ 排除包含关系：A 包含 B 或 B 包含 A
    if (CheckIfAxisAlignedCubeAContainsAxisAlignedCubeB(aabb_a, aabb_b) ||
        CheckIfAxisAlignedCubeAContainsAxisAlignedCubeB(aabb_b, aabb_a)) {
      return false;
    }
    // ~ 检查是否发生碰撞
    if (!CheckIfAxisAlignedCubeCollide(aabb_a, aabb_b)) {
      return false;
    }
    // ~ 计算 A 的哪些表面参与相交
    for (int i = 0; i < N_DIM; ++i) {
      if (CheckIfAxisAlignedCubeNdCollideOnOneDim(aabb_a, aabb_b, i)) {
        // A 的上界在 B 的区间内 -> A 的 upper 表面参与相交
        if (aabb_a.upper_bound[i] < aabb_b.upper_bound[i] &&
            aabb_a.upper_bound[i] > aabb_b.lower_bound[i]) {
          (*inter_dim_a)[2 * i] = true;
        }
        // A 的下界在 B 的区间内 -> A 的 lower 表面参与相交
        if (aabb_a.lower_bound[i] < aabb_b.upper_bound[i] &&
            aabb_a.lower_bound[i] > aabb_b.lower_bound[i]) {
          (*inter_dim_a)[2 * i + 1] = true;
        }
      }
    }
    // ~ 计算 B 的哪些表面参与相交
    for (int i = 0; i < N_DIM; ++i) {
      if (CheckIfAxisAlignedCubeNdCollideOnOneDim(aabb_a, aabb_b, i)) {
        if (aabb_b.upper_bound[i] > aabb_a.lower_bound[i] &&
            aabb_b.upper_bound[i] < aabb_a.upper_bound[i]) {
          (*inter_dim_b)[2 * i] = true;
        }
        if (aabb_b.lower_bound[i] > aabb_a.lower_bound[i] &&
            aabb_b.lower_bound[i] < aabb_a.upper_bound[i]) {
          (*inter_dim_b)[2 * i + 1] = true;
        }
      }
    }
    return true;
  }

  /**
   * @brief 判断两个 AABB 在单维度上是否发生"交叉"（既非分离也非包含）
   *
   * @details
   * 条件：两个区间有重叠且互不包含。
   *   分离条件：中心距 >= 半长之和
   *   包含条件：中心距 <= |半长之差|
   *   "交叉"即为既不满足分离条件也不满足包含条件。
   *
   * @tparam T 边界值类型
   * @tparam N_DIM 维度数
   * @param aabb_a AABB A
   * @param aabb_b AABB B
   * @param i 维度索引
   * @return true 在该维度上交叉 / false 分离或包含
   */
  template <typename T, int N_DIM>
  static bool CheckIfAxisAlignedCubeNdIntersectionOnOneDim(
      const AxisAlignedCubeNd<T, N_DIM>& aabb_a,
      const AxisAlignedCubeNd<T, N_DIM>& aabb_b, const int& i) {
    decimal_t half_len_a = fabs(
        static_cast<double>(aabb_a.upper_bound[i] - aabb_a.lower_bound[i]) /
        2.0);
    decimal_t half_len_b = fabs(
        static_cast<double>(aabb_b.upper_bound[i] - aabb_b.lower_bound[i]) /
        2.0);

    decimal_t center_a = aabb_a.lower_bound[i] + half_len_a;
    decimal_t center_b = aabb_b.lower_bound[i] + half_len_b;
    decimal_t len_c = fabs(center_a - center_b);

    // ~ 分离条件：中心距 >= 半长之和
    // ~ 包含条件：中心距 <= |半长之差|
    if (fabs(half_len_a + half_len_b) <= len_c ||
        fabs(half_len_a - half_len_b) >= len_c) {
      return false;
    }
    return true;
  };

  /**
   * @brief 判断两个 AABB 在单维度上是否发生碰撞（仅检查重叠）
   *
   * @details
   * 仅检查区间是否重叠，不排除包含关系。
   * 条件：中心距 < 半长之和 = 在该维度上重叠
   *
   * @tparam T 边界值类型
   * @tparam N_DIM 维度数
   * @param aabb_a AABB A
   * @param aabb_b AABB B
   * @param i 维度索引
   * @return true 在该维度上重叠 / false 分离
   */
  template <typename T, int N_DIM>
  static bool CheckIfAxisAlignedCubeNdCollideOnOneDim(
      const AxisAlignedCubeNd<T, N_DIM>& aabb_a,
      const AxisAlignedCubeNd<T, N_DIM>& aabb_b, const int& i) {
    decimal_t half_len_a = fabs(
        static_cast<double>(aabb_a.upper_bound[i] - aabb_a.lower_bound[i]) /
        2.0);
    decimal_t half_len_b = fabs(
        static_cast<double>(aabb_b.upper_bound[i] - aabb_b.lower_bound[i]) /
        2.0);

    decimal_t center_a = aabb_a.lower_bound[i] + half_len_a;
    decimal_t center_b = aabb_b.lower_bound[i] + half_len_b;
    decimal_t len_c = fabs(center_a - center_b);

    // ~ 若中心距 >= 半长之和，则在当前维度上分离
    if (fabs(half_len_a + half_len_b) <= len_c) {
      return false;
    }
    return true;
  };

};  // class ShapeUtils

}  // namespace common

#endif  //_COMMON_INC_COMMON_BASICS_SHAPES_H__
