/**
 * @file shapes.cc
 * @author HKUST Aerial Robotics Group
 * @brief 几何形状基础数据结构的实现文件
 *
 * 本文件实现了EPSILON自动驾驶规划系统中基础几何图元的数据结构及操作函数。
 * 这些几何类型是整个规划管线中碰撞检测、占用地图构建和路径规划的基础。
 *
 * 核心功能包括：
 * - Point / Point2i: 二维/三维浮点坐标点和二维整数坐标点
 * - OrientedBoundingBox2D: 二维定向包围盒（OBB），用于描述有旋转变换的矩形
 * - Circle: 圆形几何体，用于简化碰撞检测
 * - PolyLine: 多段线（折线），含方向属性
 * - Polygon: 多边形，由顶点序列定义的封闭形状
 * - ShapeUtils: 几何工具函数集合，提供SAT（分离轴定理）碰撞检测、
 *   OBB顶点计算、投影计算等核心几何算法
 *
 * 碰撞检测采用分离轴定理（Separating Axis Theorem, SAT）：
 * 对于两个凸多边形，如果存在一条轴使得它们在轴上的投影不重叠，
 * 则两多边形不相交。对于OBB，只需要检查四条边的法向量轴即可。
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */

#include "common/basics/shapes.h"

namespace common {

// Point 默认构造函数：坐标初始化为零（由Vecf默认构造）
Point::Point() {}

/*
 * Point - 二维点构造函数
 * @param _x x坐标
 * @param _y y坐标
 */
Point::Point(decimal_t _x, decimal_t _y) : x(_x), y(_y) {}

/*
 * Point - 三维点构造函数
 * @param _x x坐标
 * @param _y y坐标
 * @param _z z坐标
 */
Point::Point(decimal_t _x, decimal_t _y, decimal_t _z) : x(_x), y(_y), z(_z) {}

/*
 * print - 打印点的二维坐标（忽略z坐标）
 */
void Point::print() const { printf("(%f, %f)", x, y); }

// Point2i 默认构造函数
Point2i::Point2i() {}

/*
 * Point2i - 整数坐标点构造函数
 * @param _x 整数x坐标
 * @param _y 整数y坐标
 */
Point2i::Point2i(int _x, int _y) : x(_x), y(_y) {}

/*
 * print - 打印整数坐标点
 */
void Point2i::print() const { printf("(%d, %d)", x, y); }

// OBB 默认构造函数
OrientedBoundingBox2D::OrientedBoundingBox2D() {}

/*
 * OrientedBoundingBox2D - 二维定向包围盒构造函数
 * @param x_ 包围盒中心的x坐标
 * @param y_ 包围盒中心的y坐标
 * @param angle_ 包围盒的旋转角度（弧度）
 * @param width_ 包围盒的宽度（沿局部y轴方向）
 * @param length_ 包围盒的长度（沿局部x轴方向）
 */
OrientedBoundingBox2D::OrientedBoundingBox2D(const decimal_t x_,
                                             const decimal_t y_,
                                             const decimal_t angle_,
                                             const decimal_t width_,
                                             const decimal_t length_)
    : x(x_), y(y_), angle(angle_), width(width_), length(length_) {}

/*
 * Circle::print - 打印圆形信息
 * 输出圆心坐标和半径
 */
void Circle::print() const {
  printf("Circle:\n");
  printf(" -- center:");
  center.print();
  printf("\n -- radius: %lf\n", radius);
}

/*
 * PolyLine::print - 打印折线信息
 * 输出方向和顶点数量
 */
void PolyLine::print() const {
  printf("PolyLine:\n");
  printf(" -- dir: %d", dir);
  printf(" -- num of pts: %d", (int)points.size());
}

/*
 * Polygon::print - 打印多边形信息
 * 输出顶点数量
 */
void Polygon::print() const {
  printf("Polygon:\n");
  printf(" -- num of pts: %d", (int)points.size());
}

/*
 * ShapeUtils::CheckIfOrientedBoundingBoxIntersect - 检测两个OBB是否相交
 *
 * 算法：分离轴定理（SAT, Separating Axis Theorem）
 * 1. 计算两个OBB的四个顶点坐标
 * 2. 收集所有可能的分离轴（两个OBB各边的法向量，共4条轴）
 * 3. 对每条分离轴，将两个OBB的顶点投影到轴上
 * 4. 如果在任一条轴上投影不重叠，则两OBB不相交
 * 5. 如果所有轴上的投影都重叠，则两OBB相交
 *
 * @param obb_a 第一个定向包围盒
 * @param obb_b 第二个定向包围盒
 * @return true 两包围盒相交，false 不相交
 */
bool ShapeUtils::CheckIfOrientedBoundingBoxIntersect(
    const OrientedBoundingBox2D& obb_a, const OrientedBoundingBox2D& obb_b) {
  vec_E<Vecf<2>> vertices_a, vertices_b;
  GetVerticesOfOrientedBoundingBox(obb_a, &vertices_a);
  GetVerticesOfOrientedBoundingBox(obb_b, &vertices_b);
  vec_E<Vecf<2>> axes;
  GetPerpendicularAxesOfOrientedBoundingBox(vertices_a, &axes);
  GetPerpendicularAxesOfOrientedBoundingBox(vertices_b, &axes);
  Vecf<2> proj_a, proj_b;
  decimal_t overlap_len;
  // 遍历所有分离轴进行投影重叠检测
  for (auto& axis : axes) {
    GetProjectionOnAxis(vertices_a, axis, &proj_a);
    GetProjectionOnAxis(vertices_b, axis, &proj_b);
    GetOverlapLength(proj_a, proj_b, &overlap_len);
    if (fabs(overlap_len) < kEPS) {  // 投影不重叠（重叠长度为0），两形状不相交
      return false;
    }
  }
  return true;  // 所有轴上都重叠，两形状相交
}

/*
 * ShapeUtils::GetVerticesOfOrientedBoundingBox - 获取OBB的四个顶点坐标
 *
 * 顶点计算原理：
 * 给定OBB的中心(x,y)、旋转角度angle、宽度width和长度length，
 * 通过刚体变换计算四个顶点的世界坐标。
 *
 * 顶点编号布局（OBB局部坐标系，x轴为长度方向）：
 *    corner2(-l/2,+w/2)  corner1(+l/2,+w/2)
 *         ________
 *         |      |   ----> x (长度方向)
 *         |      |
 *         |______|
 *    corner3(-l/2,-w/2)  corner4(+l/2,-w/2)
 *
 * 变换公式:
 *   corner_x = center_x + local_x * cos(theta) + local_y * sin(theta)
 *   corner_y = center_y + local_x * sin(theta) - local_y * cos(theta)
 *
 * @param obb 输入定向包围盒
 * @param vertices 输出参数，按corner1-4的顺序存储四个顶点
 * @return kSuccess
 */
ErrorType ShapeUtils::GetVerticesOfOrientedBoundingBox(
    const OrientedBoundingBox2D& obb, vec_E<Vecf<2>>* vertices) {
  vertices->clear();
  vertices->reserve(4);
  decimal_t cos_theta = cos(obb.angle);
  decimal_t sin_theta = sin(obb.angle);
  // 通过旋转变换计算四个顶点的世界坐标
  Vecf<2> corner1(
      obb.x + 0.5 * obb.length * cos_theta + 0.5 * obb.width * sin_theta,
      obb.y + 0.5 * obb.length * sin_theta - 0.5 * obb.width * cos_theta);
  Vecf<2> corner2(
      obb.x + 0.5 * obb.length * cos_theta - 0.5 * obb.width * sin_theta,
      obb.y + 0.5 * obb.length * sin_theta + 0.5 * obb.width * cos_theta);
  Vecf<2> corner3(
      obb.x - 0.5 * obb.length * cos_theta - 0.5 * obb.width * sin_theta,
      obb.y - 0.5 * obb.length * sin_theta + 0.5 * obb.width * cos_theta);
  Vecf<2> corner4(
      obb.x - 0.5 * obb.length * cos_theta + 0.5 * obb.width * sin_theta,
      obb.y - 0.5 * obb.length * sin_theta - 0.5 * obb.width * cos_theta);
  vertices->push_back(corner1);
  vertices->push_back(corner2);
  vertices->push_back(corner3);
  vertices->push_back(corner4);
  return kSuccess;
}

/*
 * ShapeUtils::GetPerpendicularAxesOfOrientedBoundingBox - 获取OBB的所有分离轴
 *
 * 对于二维OBB，有两条分离轴就够了：每个OBB包含两条平行边，
 * 每条边有唯一的外法向量。因此每个OBB贡献两条分离轴。
 *
 * @param vertices OBB的四个顶点
 * @param axes 输出参数，存储的两条分离轴（单位法向量）
 * @return kSuccess
 */
ErrorType ShapeUtils::GetPerpendicularAxesOfOrientedBoundingBox(
    const vec_E<Vecf<2>>& vertices, vec_E<Vecf<2>>* axes) {
  Vecf<2> axis0, axis1;
  GetPerpendicularAxisOfOrientedBoundingBox(vertices, 0, &axis0);  // 边0→1的右手法向量
  GetPerpendicularAxisOfOrientedBoundingBox(vertices, 1, &axis1);  // 边1→2的右手法向量
  axes->push_back(axis0);
  axes->push_back(axis1);
  return kSuccess;
}

/*
 * ShapeUtils::GetProjectionOnAxis - 计算一组顶点在某轴上的投影区间
 *
 * 对于每个顶点v，计算其在轴上的标量投影: projection = v dot axis
 * 投影区间为 [min(projections), max(projections)]
 *
 * @param vertices 顶点数组
 * @param axis 投影轴（单位向量，不一定需要归一化）
 * @param proj 输出参数，(投影最小值, 投影最大值)
 * @return kSuccess
 */
ErrorType ShapeUtils::GetProjectionOnAxis(const vec_E<Vecf<2>>& vertices,
                                          const Vecf<2>& axis, Vecf<2>* proj) {
  decimal_t min = std::numeric_limits<decimal_t>::infinity();
  decimal_t max = -std::numeric_limits<decimal_t>::infinity();
  decimal_t projection;
  for (auto& vertex : vertices) {
    projection = vertex.dot(axis);  // 计算顶点在轴上的标量投影
    if (projection < min) {
      min = projection;
    }
    if (projection > max) {
      max = projection;
    }
  }
  *proj = Vecf<2>(min, max);
  return kSuccess;
}

/*
 * ShapeUtils::GetPerpendicularAxisOfOrientedBoundingBox - 获取OBB某条边的外法向量
 *
 * 给定OBB某条边的两个连续顶点vertices[index]和vertices[index+1]，
 * 计算该边的方向向量，然后通过右手定则旋转90度得到外法向量。
 * 具体：如果边向量为(vec_x, vec_y)，则法向量为(-vec_y, vec_x)（归一化后的右旋法向量）。
 *
 * @param vertices OBB的四个顶点
 * @param index 起始顶点索引（0-3，取vertices[index]和vertices[index+1]构成的边）
 * @param axis 输出参数，该边对应的右手法向量
 * @return kSuccess
 */
ErrorType ShapeUtils::GetPerpendicularAxisOfOrientedBoundingBox(
    const vec_E<Vecf<2>>& vertices, const int index, Vecf<2>* axis) {
  assert(index >= 0 && index < 4);
  Vecf<2> vec = vertices[index + 1] - vertices[index];  // 边向量
  decimal_t length = vec.norm();
  Vecf<2> normalized_vec = Vecf<2>::Zero();
  if (length > kEPS) normalized_vec = vec / length;  // 归一化边向量
  // 右手法向量：将边向量逆时针旋转90度
  (*axis)[0] = -normalized_vec[1];
  (*axis)[1] = normalized_vec[0];
  return kSuccess;
}

/*
 * ShapeUtils::GetOverlapLength - 计算两个一维区间的重叠长度
 *
 * 对于两个区间 [a.x(), a.y()] 和 [b.x(), b.y()]，
 * 如果它们不相交则返回0，相交则返回重叠部分的长度。
 *
 * @param a 第一个一维投影区间 (min, max)
 * @param b 第二个一维投影区间 (min, max)
 * @param len 输出参数，重叠长度（不重叠为0.0）
 * @return kSuccess
 */
ErrorType ShapeUtils::GetOverlapLength(const Vecf<2> a, const Vecf<2> b,
                                       decimal_t* len) {
  if (a.x() > b.y() || a.y() < b.x()) {  // 区间不相交
    *len = 0.0;
    return kSuccess;
  }

  // 重叠长度 = 较小最大值 - 较大最小值
  *len = std::min(a.y(), b.y()) - std::max(a.x(), b.x());
  return kSuccess;
}

/*
 * ShapeUtils::GetCvPoint2iVecUsingCommonPoint2iVec - 批量转换Point2i到OpenCV格式
 *
 * 将自定义的 Point2i 向量批量转换为 OpenCV 的 cv::Point2i 向量。
 *
 * @param pts_in 输入的 Point2i 向量
 * @param pts_out 输出参数，OpenCV格式的 cv::Point2i 向量
 * @return kSuccess
 */
ErrorType ShapeUtils::GetCvPoint2iVecUsingCommonPoint2iVec(
    const std::vector<Point2i>& pts_in, std::vector<cv::Point2i>* pts_out) {
  int num = pts_in.size();
  pts_out->resize(num);
  for (int i = 0; i < num; ++i) {
    GetCvPoint2iUsingCommonPoint2i(pts_in[i], pts_out->data() + i);
  }
  return kSuccess;
}

/*
 * ShapeUtils::GetCvPoint2iUsingCommonPoint2i - 单个Point2i到OpenCV格式的转换
 * @param pt_in 输入的 Point2i 点
 * @param pt_out 输出参数，OpenCV格式的 cv::Point2i 点
 * @return kSuccess
 */
ErrorType ShapeUtils::GetCvPoint2iUsingCommonPoint2i(const Point2i& pt_in,
                                                     cv::Point2i* pt_out) {
  pt_out->x = pt_in.x;
  pt_out->y = pt_in.y;
  return kSuccess;
}

}  // namespace common
