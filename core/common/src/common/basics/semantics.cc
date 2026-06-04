/**
 * @file semantics.cc
 * @author HKUST Aerial Robotics Group
 * @brief 语义层基础数据结构的实现文件
 *
 * 本文件实现了EPSILON自动驾驶规划系统中语义层（semantics layer）的核心数据结构，
 * 包括车辆参数、车辆状态、车辆集合、控制信号、栅格地图以及车道网络等类型。
 *
 * 核心功能包括：
 * - VehicleParam: 车辆的物理参数（尺寸、动力学约束等）
 * - Vehicle: 车辆对象的完整描述（ID、参数、状态），并提供几何计算接口
 * - VehicleControlSignal: 车辆控制信号（加速度、转向速率或开环状态）
 * - GridMapND: 通用的N维栅格地图模板类，支持坐标与世界位置的互转
 * - GridMapMetaInfo: 栅格地图的元信息
 * - LaneRaw / LaneNet / SemanticLaneSet: 道路网络拓扑结构
 * - TrafficSignal / SpeedLimit / StoppingSign / TrafficLight: 交通信号体系
 * - SemanticsUtils: 车辆几何计算的工具函数集合
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/basics/semantics.h"

namespace common {

/*
 * VehicleParam::print - 打印车辆参数信息
 * 输出车辆的所有物理参数，用于调试和日志输出
 */
void VehicleParam::print() const {
  printf("VehicleParam:\n");
  printf(" -- width:\t %lf.\n", width_);          // 车辆宽度
  printf(" -- length:\t %lf.\n", length_);         // 车辆长度
  printf(" -- wheel_base:\t %lf.\n", wheel_base_);  // 轴距
  printf(" -- front_suspension:\t %lf.\n", front_suspension_);  // 前悬长度
  printf(" -- rear_suspension:\t %lf.\n", rear_suspension_);    // 后悬长度
  printf(" -- d_cr:\t %lf.\n", d_cr_);  // 质心到后轴中心的距离
  printf(" -- max_steering_angle:\t %lf.\n", max_steering_angle_);  // 最大转向角
  printf(" -- max_longitudinal_acc:\t %lf.\n", max_longitudinal_acc_);  // 最大纵向加速度
  printf(" -- max_lateral_acc:\t %lf.\n", max_lateral_acc_);  // 最大横向加速度
}

// 默认构造函数：不初始化任何成员
Vehicle::Vehicle() {}

/*
 * Vehicle - 使用参数和状态构造车辆对象
 * @param param 车辆物理参数（尺寸、动力学限制等）
 * @param state 车辆的当前运动状态（位置、速度、朝向等）
 */
Vehicle::Vehicle(const VehicleParam &param, const State &state)
    : param_(param), state_(state) {}

/*
 * Vehicle - 使用ID、参数和状态构造车辆对象
 * @param id 车辆唯一标识符
 * @param param 车辆物理参数
 * @param state 车辆当前运动状态
 */
Vehicle::Vehicle(const int &id, const VehicleParam &param, const State &state)
    : id_(id), param_(param), state_(state) {}

/*
 * Vehicle - 使用ID、子类、参数和状态构造车辆对象
 * @param id 车辆唯一标识符
 * @param subclass 车辆子类型字符串（如"car", "truck"等）
 * @param param 车辆物理参数
 * @param state 车辆当前运动状态
 */
Vehicle::Vehicle(const int &id, const std::string &subclass,
                 const VehicleParam &param, const State &state)
    : id_(id), subclass_(subclass), param_(param), state_(state) {}

/*
 * Ret3DofState - 获取车辆的三自由度简化状态
 * 返回 (x, y, theta) 三元组，忽略速度和加速度信息
 * @return 包含位置(x,y)和朝向角(theta)的Vec3f向量
 */
Vec3f Vehicle::Ret3DofState() const {
  return Vec3f(state_.vec_position(0), state_.vec_position(1), state_.angle);
}

/*
 * Ret3DofStateAtGeometryCenter - 计算车辆几何中心处的三自由度状态
 *
 * 车辆状态通常定义在后轴中心，此函数将状态转换到车辆几何中心
 * （即车身矩形中心）的位置。
 * 转换公式: center_xy = rear_axle_xy + d_cr * (cos(theta), sin(theta))
 * 其中 d_cr 是质心到后轴中心的距离
 *
 * @param state 输出参数，几何中心处的(x, y, theta)
 * @return kSuccess 操作成功
 */
ErrorType Vehicle::Ret3DofStateAtGeometryCenter(Vec3f *state) const {
  decimal_t cos_theta = cos(state_.angle);
  decimal_t sin_theta = sin(state_.angle);
  // 后轴中心沿车辆朝向方向偏移 d_cr 到达几何中心
  decimal_t x = state_.vec_position(0) + param_.d_cr() * cos_theta;
  decimal_t y = state_.vec_position(1) + param_.d_cr() * sin_theta;
  (*state)(0) = x;
  (*state)(1) = y;
  (*state)(2) = state_.angle;
  return kSuccess;
}

/*
 * print - 打印车辆信息
 * 输出车辆ID、子类型、参数和状态信息
 */
void Vehicle::print() const {
  printf("\nVehicle:\n");
  printf(" -- ID:\t%d\n", id_);
  printf(" -- Subclass:\t%s\n", subclass_.c_str());
  param_.print();
  state_.print();
}

/*
 * RetOrientedBoundingBox - 获取车辆的定向包围盒（OBB）
 *
 * 根据车辆的当前状态和参数，计算其二维定向包围盒。
 * OBB中心位于车辆几何中心（由后轴中心偏移d_cr得到），
 * 朝向与车辆朝向一致，尺寸由车辆宽度和长度确定。
 *
 * @return 描述车辆轮廓的OrientedBoundingBox2D对象
 */
OrientedBoundingBox2D Vehicle::RetOrientedBoundingBox() const {
  OrientedBoundingBox2D obb;
  double cos_theta = cos(state_.angle);
  double sin_theta = sin(state_.angle);
  // 包围盒中心 = 后轴位置 + d_cr * 朝向向量
  obb.x = state_.vec_position(0) + param_.d_cr() * cos_theta;
  obb.y = state_.vec_position(1) + param_.d_cr() * sin_theta;
  obb.angle = state_.angle;
  obb.width = param_.width();
  obb.length = param_.length();
  return obb;
}

/*
 * RetVehicleVertices - 获取车辆四个顶点的坐标
 *
 * 计算车辆矩形轮廓的四个顶点在世界坐标系下的位置。
 * 顶点按逆时针方向，从左前顶点开始排列。
 *
 * @param vertices 输出参数，按逆时针存储四个顶点坐标的向量
 * @return kSuccess 操作成功
 */
ErrorType Vehicle::RetVehicleVertices(vec_E<Vec2f> *vertices) const {
  SemanticsUtils::GetVehicleVertices(param_, state_, vertices);
  return kSuccess;
}

/*
 * RetBumperVertices - 获取车辆前后保险杠中心点的坐标
 *
 * 计算车辆前端和后端中心（保险杠位置）的坐标，共两个点。
 * 这些点位于车辆的纵向中轴线上，分别位于几何中心的前后方。
 *
 * @param vertices 输出参数，包含两个顶点：[0]=后端中心, [1]=前端中心
 * @return kSuccess 操作成功
 */
ErrorType Vehicle::RetBumperVertices(std::array<Vec2f, 2> *vertices) const {
  decimal_t cos_theta = cos(state_.angle);
  decimal_t sin_theta = sin(state_.angle);

  // 几何中心位置
  decimal_t c_x = state_.vec_position(0) + param_.d_cr() * cos_theta;
  decimal_t c_y = state_.vec_position(1) + param_.d_cr() * sin_theta;

  // 沿朝向方向半车长的偏移量
  decimal_t d_lx = param_.length() / 2.0 * cos_theta;
  decimal_t d_ly = param_.length() / 2.0 * sin_theta;

  (*vertices)[0] = Vec2f(c_x - d_lx, c_y - d_ly);  // 后端中心
  (*vertices)[1] = Vec2f(c_x + d_lx, c_y + d_ly);  // 前端中心

  return kSuccess;
}

/*
 * VehicleSet::print - 打印车辆集合中所有车辆的信息
 * 遍历整个车辆集合，按ID顺序输出每辆车的信息
 */
void VehicleSet::print() const {
  printf("Vehicle Set Info:\n");
  for (auto iter = vehicles.begin(); iter != vehicles.end(); ++iter) {
    printf("\n -- ID. %d:\n", iter->first);
    iter->second.print();
  }
  printf("\n");
}

// 默认构造函数：加速度和转向速率为0，非开环模式
VehicleControlSignal::VehicleControlSignal() {}

/*
 * VehicleControlSignal - 闭环控制信号构造函数
 * @param acc 纵向加速度控制量
 * @param steer_rate 转向速率控制量
 * 此构造的信号用于闭环控制模式（is_openloop=false）
 */
VehicleControlSignal::VehicleControlSignal(double acc, double steer_rate)
    : acc(acc), steer_rate(steer_rate), is_openloop(false) {}

/*
 * VehicleControlSignal - 开环控制信号构造函数
 * @param state 目标状态（位置、速度、朝向等）
 * 此构造的信号用于开环控制模式（is_openloop=true），
 * 加速度和转向速率均设为0，由外部控制器直接跟踪state
 */
VehicleControlSignal::VehicleControlSignal(common::State state)
    : acc(0.0), steer_rate(0.0), is_openloop(true), state(state) {}

// 默认构造函数
GridMapMetaInfo::GridMapMetaInfo() {}

/*
 * GridMapMetaInfo - 栅格地图元信息构造函数
 * @param w 栅格地图的宽度（沿x方向的单元格数）
 * @param h 栅格地图的高度（沿y方向的单元格数）
 * @param res 每个单元格的分辨率（米/格）
 * 自动计算以米为单位的实际尺寸：w_metric = w * res, h_metric = h * res
 */
GridMapMetaInfo::GridMapMetaInfo(const int w, const int h, const double res)
    : width(w), height(h), resolution(res) {
  w_metric = w * resolution;  // 宽度方向实际距离（米）
  h_metric = h * resolution;  // 高度方向实际距离（米）
}

/*
 * print - 打印栅格地图元信息
 */
void GridMapMetaInfo::print() const {
  printf("GridMapMetaInfo:\n");
  printf(" -- width:%d\n", width);
  printf(" -- height:%d\n", height);
  printf(" -- resolution:%lf\n", resolution);
  printf(" -- w_metric:%lf\n", w_metric);
  printf(" -- h_metric:%lf\n", h_metric);
  printf("\n");
}

// GridMapND 默认构造函数
template <typename T, int N_DIM>
GridMapND<T, N_DIM>::GridMapND() {}

/*
 * GridMapND - N维栅格地图构造函数
 * @param dims_size 各维度的大小（单元格数量）数组
 * @param dims_resolution 各维度的分辨率数组（单位/格）
 * @param dims_name 各维度的名称数组
 *
 * 构造函数执行流程：
 * 1. 保存维度的尺寸和分辨率信息
 * 2. 计算多维索引到一维索引的步长（用于坐标映射）
 * 3. 根据各维尺寸计算总数据量并分配内存
 * 4. 将原点设为全零
 */
template <typename T, int N_DIM>
GridMapND<T, N_DIM>::GridMapND(
    const std::array<int, N_DIM> &dims_size,
    const std::array<decimal_t, N_DIM> &dims_resolution,
    const std::array<std::string, N_DIM> &dims_name) {
  dims_size_ = dims_size;
  dims_resolution_ = dims_resolution;
  dims_name_ = dims_name;

  SetNDimSteps(dims_size_);   // 计算多维索引→一维索引的步长
  SetDataSize(dims_size_);    // 计算总数据量
  data_ = std::vector<T>(data_size_, 0);
  origin_.fill(0);
}

/*
 * GetValueUsingCoordinate - 通过多维坐标获取栅格值
 * @param coord N维整数坐标
 * @param val 输出参数，对应位置的值
 * @return kSuccess 成功，kWrongStatus 坐标越界
 */
template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetValueUsingCoordinate(
    const std::array<int, N_DIM> &coord, T *val) const {
  if (!CheckCoordInRange(coord)) {
    return kWrongStatus;
  }
  int idx = GetMonoIdxUsingNDimIdx(coord);  // 将多维坐标映射为一维索引
  *val = data_[idx];
  return kSuccess;
}

/*
 * GetValueUsingGlobalPosition - 通过世界坐标获取栅格值
 * @param p_w N维世界坐标（连续值）
 * @param val 输出参数，对应栅格的值
 * @return kSuccess 操作完成（坐标可能在范围外）
 */
template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetValueUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w, T *val) const {
  std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
  GetValueUsingCoordinate(coord, val);
  return kSuccess;
}

/*
 * CheckIfEqualUsingGlobalPosition - 检查给定世界坐标处的值是否与指定值相等
 * @param p_w 世界坐标
 * @param val_in 待比较的值
 * @param res 输出参数，是否相等
 * @return kSuccess
 */
template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::CheckIfEqualUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w, const T &val_in, bool *res) const {
  std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
  T val;
  if (GetValueUsingCoordinate(coord, &val) != kSuccess) {
    *res = false;
  } else {
    *res = (val == val_in);
  }
  return kSuccess;
}

/*
 * CheckIfEqualUsingCoordinate - 检查给定坐标处的值是否与指定值相等
 * @param coord 整数坐标
 * @param val_in 待比较的值
 * @param res 输出参数，是否相等
 * @return kSuccess
 */
template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::CheckIfEqualUsingCoordinate(
    const std::array<int, N_DIM> &coord, const T &val_in, bool *res) const {
  T val;
  if (GetValueUsingCoordinate(coord, &val) != kSuccess) {
    *res = false;
  } else {
    *res = (val == val_in);
  }
  return kSuccess;
}

/*
 * SetValueUsingCoordinate - 通过多维坐标设置栅格值
 * @param coord N维整数坐标
 * @param val 要设置的值
 * @return kSuccess 成功，kWrongStatus 坐标越界
 */
template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetValueUsingCoordinate(
    const std::array<int, N_DIM> &coord, const T &val) {
  if (!CheckCoordInRange(coord)) {
    return kWrongStatus;
  }
  int idx = GetMonoIdxUsingNDimIdx(coord);
  data_[idx] = val;
  return kSuccess;
}

/*
 * SetValueUsingGlobalPosition - 通过世界坐标设置栅格值
 * @param p_w N维世界坐标
 * @param val 要设置的值
 * @return kSuccess
 */
template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetValueUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w, const T &val) {
  std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
  SetValueUsingCoordinate(coord, val);
  return kSuccess;
}

/*
 * GetCoordUsingGlobalPosition - 将世界坐标转换为整数栅格坐标
 *
 * 转换公式: coord[i] = round((p_w[i] - origin_[i]) / dims_resolution_[i])
 * 使用四舍五入将连续的物理坐标映射到最近的栅格单元
 *
 * @param p_w N维世界坐标（连续值）
 * @return N维整数栅格坐标
 */
template <typename T, int N_DIM>
std::array<int, N_DIM> GridMapND<T, N_DIM>::GetCoordUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w) const {
  std::array<int, N_DIM> coord = {};
  for (int i = 0; i < N_DIM; ++i) {
    coord[i] = std::round((p_w[i] - origin_[i]) / dims_resolution_[i]);
  }
  return coord;
}

/*
 * GetRoundedPosUsingGlobalPosition - 获取世界坐标所在栅格中心的世界坐标
 *
 * 此函数区别于 GetCoordUsingGlobalPosition，
 * 它返回的是离散栅格中心对应的连续世界坐标（而非整数坐标）。
 * 等同于将连续坐标"吸附"到最近栅格单元的中心。
 *
 * @param p_w 输入的N维世界坐标
 * @return 对应栅格单元中心的N维世界坐标
 */
template <typename T, int N_DIM>
std::array<decimal_t, N_DIM>
GridMapND<T, N_DIM>::GetRoundedPosUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w) const {
  std::array<int, N_DIM> coord = {};
  for (int i = 0; i < N_DIM; ++i) {
    coord[i] = std::round((p_w[i] - origin_[i]) / dims_resolution_[i]);
  }
  std::array<decimal_t, N_DIM> round_pos = {};
  for (int i = 0; i < N_DIM; ++i) {
    round_pos[i] = coord[i] * dims_resolution_[i] + origin_[i];
  }
  return round_pos;
}

/*
 * GetGlobalPositionUsingCoordinate - 将整数栅格坐标转换为世界坐标
 *
 * 转换公式: p_w[i] = coord[i] * dims_resolution_[i] + origin_[i]
 * 返回的是栅格单元中心点对应的世界坐标。
 *
 * @param coord N维整数栅格坐标
 * @param p_w 输出参数，对应的N维世界坐标
 * @return kSuccess
 */
template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetGlobalPositionUsingCoordinate(
    const std::array<int, N_DIM> &coord,
    std::array<decimal_t, N_DIM> *p_w) const {
  auto ptr = p_w->data();
  for (int i = 0; i < N_DIM; ++i) {
    *(ptr + i) = coord[i] * dims_resolution_[i] + origin_[i];
  }
  return kSuccess;
}

/*
 * GetCoordUsingGlobalMetricOnSingleDim - 单维度的世界坐标到整数坐标转换
 * @param metric 该维度的连续度量值
 * @param i 维度索引
 * @param idx 输出参数，对应的整数坐标
 * @return kSuccess
 */
template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetCoordUsingGlobalMetricOnSingleDim(
    const decimal_t &metric, const int &i, int *idx) const {
  *idx = std::round((metric - origin_[i]) / dims_resolution_[i]);
  return kSuccess;
}

/*
 * GetGlobalMetricUsingCoordOnSingleDim - 单维度的整数坐标到连续度量值转换
 * @param idx 该维度的整数坐标
 * @param i 维度索引
 * @param metric 输出参数，对应的连续度量值
 * @return kSuccess
 */
template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetGlobalMetricUsingCoordOnSingleDim(
    const int &idx, const int &i, decimal_t *metric) const {
  *metric = idx * dims_resolution_[i] + origin_[i];
  return kSuccess;
}

/*
 * CheckCoordInRange - 检查N维坐标是否在有效范围内
 * 对每个维度检查坐标是否在 [0, dims_size_[i]) 区间内
 * @param coord N维整数坐标
 * @return true 坐标有效，false 越界
 */
template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::CheckCoordInRange(
    const std::array<int, N_DIM> &coord) const {
  for (int i = 0; i < N_DIM; ++i) {
    if (coord[i] < 0 || coord[i] >= dims_size_[i]) {
      return false;
    }
  }
  return true;
}

/*
 * CheckCoordInRangeOnSingleDim - 检查单维度的坐标是否在有效范围内
 * @param idx 该维度的整数坐标
 * @param i 维度索引
 * @return true 坐标有效
 */
template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::CheckCoordInRangeOnSingleDim(const int &idx,
                                                       const int &i) const {
  return (idx >= 0) && (idx < dims_size_[i]);
}

/*
 * GetMonoIdxUsingNDimIdx - 将N维坐标映射为一维线性索引
 *
 * 采用行主序（row-major）方式将多维坐标展平：
 * mono_idx = sum_{i=0}^{N_DIM-1} dims_step_[i] * idx[i]
 * 其中 dims_step_[i] 是第i维的步长（该维连续两个索引在一维数组中的距离）
 *
 * @param idx N维整数坐标
 * @return 一维线性索引
 */
template <typename T, int N_DIM>
int GridMapND<T, N_DIM>::GetMonoIdxUsingNDimIdx(
    const std::array<int, N_DIM> &idx) const {
  int mono_idx = 0;
  for (int i = 0; i < N_DIM; ++i) {
    mono_idx += dims_step_[i] * idx[i];
  }
  return mono_idx;
}

/*
 * GetNDimIdxUsingMonoIdx - 将一维线性索引还原为N维坐标
 *
 * 这是 GetMonoIdxUsingNDimIdx 的逆操作。
 * 通过连续取模和整除从高维到低维逐步分解出一维索引。
 *
 * @param idx 一维线性索引
 * @return N维整数坐标
 */
template <typename T, int N_DIM>
std::array<int, N_DIM> GridMapND<T, N_DIM>::GetNDimIdxUsingMonoIdx(
    const int &idx) const {
  std::array<int, N_DIM> idx_nd = {};
  int tmp = idx;
  for (int i = N_DIM - 1; i >= 0; --i) {
    idx_nd[i] = tmp / dims_step_[i];  // 除法得到该维坐标
    tmp = tmp % dims_step_[i];        // 取余数继续分解
  }
  return idx_nd;
}

/*
 * SetNDimSteps - 根据各维度大小计算多维索引到一维索引的步长
 *
 * 步长计算方式: dims_step_[i] = prod_{j=0}^{i-1} dims_size[j]
 * 即 dims_step_[0]=1, step[1]=size[0], step[2]=size[0]*size[1], ...
 *
 * @param dims_size 各维度的大小
 * @return kSuccess
 */
template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetNDimSteps(
    const std::array<int, N_DIM> &dims_size) {
  int step = 1;
  for (int i = 0; i < N_DIM; ++i) {
    dims_step_[i] = step;
    step = step * dims_size[i];
  }
  return kSuccess;
}

/*
 * SetDataSize - 根据各维度大小计算总数据量
 * data_size_ = prod_{i=0}^{N_DIM-1} dims_size[i]
 * @param dims_size 各维度的大小
 * @return kSuccess
 */
template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetDataSize(
    const std::array<int, N_DIM> &dims_size) {
  int total_ele_num = 1;
  for (int i = 0; i < N_DIM; ++i) {
    total_ele_num = total_ele_num * dims_size_[i];
  }
  data_size_ = total_ele_num;
  return kSuccess;
}

/*
 * 显式模板实例化：支持常用的数据类型和维度组合
 * - uint8_t: 用于占用栅格地图（0=空闲, 255=占用）
 * - int: 用于语义栅格地图或其他整数值地图
 * - 2D: 二维平面地图
 * - 3D: 三维时空地图
 */
template class GridMapND<uint8_t, 2>;
template class GridMapND<uint8_t, 3>;
template class GridMapND<int, 2>;
template class GridMapND<int, 3>;

/*
 * LaneRaw::print - 打印原始车道信息
 * 输出车道的ID、方向、拓扑连接关系（父子车道）、
 * 几何属性（长度、起终点）、变道可用性以及行为标记
 */
void LaneRaw::print() const {
  printf("Lane %d:\n", id);
  printf(" -- dir:\t%d\n", dir);  // 车道方向

  printf(" -- child_id:\t[");
  for (const auto &id : child_id) {
    printf(" %d ", id);
  }
  printf("]\n");

  printf(" -- father_id:\t[");
  for (const auto &id : father_id) {
    printf(" %d ", id);
  }
  printf("]\n");
  printf(" -- length:\t%f\n", length);
  printf(" -- l_lane_id:\t%d\n", l_lane_id);      // 左侧相邻车道ID
  printf(" -- l_change_avbl:\t%d\n", l_change_avbl); // 是否可以向左变道
  printf(" -- r_lane_id:\t%d\n", r_lane_id);      // 右侧相邻车道ID
  printf(" -- r_change_avbl:\t%d\n", r_change_avbl); // 是否可以向右变道

  printf(" -- behavior:\t%s\n", behavior.c_str());  // 行为标记（如"保持车道"）

  printf(" -- start_point: (%f, %f)\n", start_point(0), start_point(1));
  printf(" -- final_point: (%f, %f)\n", final_point(0), final_point(1));
  printf(" -- point number:\t%d\n", static_cast<int>(lane_points.size()));
}

/*
 * LaneNet::print - 打印车道网络信息
 * 输出车道网络中包含的车道总数以及每条车道的详细数据
 */
void LaneNet::print() const {
  printf("LaneNet:\n");
  printf(" -- Number of lanes:\t%d\n", static_cast<int>(lane_set.size()));
  for (auto it = lane_set.begin(); it != lane_set.end(); ++it) {
    it->second.print();
  }
  printf("\n");
}

/*
 * SemanticLaneSet::print - 打印语义车道集合信息
 */
void SemanticLaneSet::print() const {
  printf("SemanticLaneSet:\n");
  printf(" -- Number of lanes:\t%d\n", static_cast<int>(semantic_lanes.size()));
}

// 圆形障碍物的打印函数
void CircleObstacle::print() const {
  printf("id: %d\n", id);
  circle.print();
  printf("\n");
}

// 多边形障碍物的打印函数
void PolygonObstacle::print() const {
  printf("id: %d\n", id);
  polygon.print();
  printf("\n");
}

/*
 * ObstacleSet::print - 打印障碍物集合中的所有障碍物
 * 先输出所有圆形障碍物，再输出所有多边形障碍物
 */
void ObstacleSet::print() const {
  for (const auto &obs : obs_circle) {
    obs.second.print();
  }
  for (const auto &obs : obs_polygon) {
    obs.second.print();
  }
}

/*
 * TrafficSignal - 默认构造函数
 * 初始化起点和终点为零向量，有效时间范围为[0, max)，
 * 速度范围为[0, 0]，默认横向范围为[-1.75, 1.75]（约等于车道宽度）
 */
TrafficSignal::TrafficSignal()
    : start_point_(Vec2f::Zero()),
      end_point_(Vec2f::Zero()),
      valid_time_(Vec2f(0.0, std::numeric_limits<decimal_t>::max())),
      vel_range_(Vec2f::Zero()),
      lateral_range_(Vec2f(-1.75, 1.75)) {}

/*
 * TrafficSignal - 参数化的交通信号构造函数
 * @param start_point 信号生效的起始位置（Frenet坐标系中的s值）
 * @param end_point 信号生效的结束位置
 * @param valid_time 信号有效的时间区间（秒）
 * @param vel_range 信号规定的速度范围（最小速度, 最大速度）
 */
TrafficSignal::TrafficSignal(const Vec2f &start_point, const Vec2f &end_point,
                             const Vec2f &valid_time, const Vec2f &vel_range)
    : start_point_(start_point),
      end_point_(end_point),
      valid_time_(valid_time),
      vel_range_(vel_range),
      lateral_range_(Vec2f(-1.75, 1.75)) {}

// 交通信号属性设置器（setters）和获取器（getters）
void TrafficSignal::set_start_point(const Vec2f &start_point) {
  start_point_ = start_point;
}

void TrafficSignal::set_end_point(const Vec2f &end_point) {
  end_point_ = end_point;
}

void TrafficSignal::set_valid_time_til(const decimal_t max_valid_time) {
  valid_time_(1) = max_valid_time;
}

void TrafficSignal::set_valid_time_begin(const decimal_t min_valid_time) {
  valid_time_(0) = min_valid_time;
}

void TrafficSignal::set_valid_time(const Vec2f &valid_time) {
  valid_time_ = valid_time;
}

void TrafficSignal::set_vel_range(const Vec2f &vel_range) {
  vel_range_ = vel_range;
}

void TrafficSignal::set_lateral_range(const Vec2f &lateral_range) {
  lateral_range_ = lateral_range;
}

void TrafficSignal::set_max_velocity(const decimal_t max_velocity) {
  vel_range_(1) = max_velocity;
}

Vec2f TrafficSignal::start_point() const { return start_point_; }
Vec2f TrafficSignal::end_point() const { return end_point_; }
Vec2f TrafficSignal::valid_time() const { return valid_time_; }
Vec2f TrafficSignal::vel_range() const { return vel_range_; }
Vec2f TrafficSignal::lateral_range() const { return lateral_range_; }
decimal_t TrafficSignal::max_velocity() const { return vel_range_(1); }

/*
 * SpeedLimit - 限速信号构造函数
 * 继承自 TrafficSignal，将有效时间设为[0, max)范围的默认值
 * @param start_point 限速起始位置
 * @param end_point 限速结束位置
 * @param vel_range 限速范围（最小速度, 最大速度）
 */
SpeedLimit::SpeedLimit(const Vec2f &start_point, const Vec2f &end_point,
                       const Vec2f &vel_range)
    : TrafficSignal(start_point, end_point,
                    Vec2f(0.0, std::numeric_limits<decimal_t>::max()),
                    vel_range) {}

/*
 * StoppingSign - 停车标志构造函数
 * 继承自 TrafficSignal，将速度范围设为[0, 0]（强制停车）
 * @param start_point 停车标志起始位置
 * @param end_point 停车标志结束位置
 */
StoppingSign::StoppingSign(const Vec2f &start_point, const Vec2f &end_point)
    : TrafficSignal(start_point, end_point,
                    Vec2f(0.0, std::numeric_limits<decimal_t>::max()),
                    Vec2f::Zero()) {}

// 交通灯类型设置器
void TrafficLight::set_type(const Type &type) { type_ = type; }

// 交通灯类型获取器
TrafficLight::Type TrafficLight::type() const { return type_; }

/*
 * SemanticsUtils::GetOrientedBoundingBoxForVehicleUsingState - 根据车辆状态计算OBB
 *
 * 与 Vehicle::RetOrientedBoundingBox 功能相同，但以静态工具函数形式提供。
 * 定向包围盒中心为后轴位置沿车辆朝向偏移d_cr到达的几何中心，
 * 尺寸由车辆长度和宽度确定，朝向与车辆朝向一致。
 *
 * @param param 车辆物理参数（宽度、长度、d_cr等）
 * @param s 车辆的运动状态（位置、朝向）
 * @param obb 输出参数，计算得到的定向包围盒
 * @return kSuccess
 */
ErrorType SemanticsUtils::GetOrientedBoundingBoxForVehicleUsingState(
    const VehicleParam &param, const State &s, OrientedBoundingBox2D *obb) {
  double cos_theta = cos(s.angle);
  double sin_theta = sin(s.angle);
  obb->x = s.vec_position[0] + param.d_cr() * cos_theta;
  obb->y = s.vec_position[1] + param.d_cr() * sin_theta;
  obb->angle = s.angle;
  obb->length = param.length();
  obb->width = param.width();
  return kSuccess;
}

/*
 * SemanticsUtils::GetVehicleVertices - 计算车辆四个顶点的世界坐标
 *
 * 算法流程：
 * 1. 计算车辆几何中心的世界坐标 (c_x, c_y) = 后轴位置 + d_cr * 朝向向量
 * 2. 计算宽度方向（垂直于朝向）和长度方向（平行于朝向）的半尺寸偏移
 * 3. 按逆时针方向计算四个顶点（从左前顶点开始）
 *
 * 顶点布局（车辆坐标系）：
 *    corner2（左后）  corner1（左前）
 *         ________
 *         |      |
 *         |      |
 *         |______|
 *    corner3（右后）  corner4（右前）
 *
 * @param param 车辆物理参数
 * @param state 车辆的运动状态
 * @param vertices 输出参数，按逆时针存储四个顶点坐标
 * @return kSuccess
 */
ErrorType SemanticsUtils::GetVehicleVertices(const VehicleParam &param,
                                             const State &state,
                                             vec_E<Vec2f> *vertices) {
  decimal_t angle = state.angle;

  decimal_t cos_theta = cos(angle);
  decimal_t sin_theta = sin(angle);

  // 几何中心位置
  decimal_t c_x = state.vec_position(0) + param.d_cr() * cos_theta;
  decimal_t c_y = state.vec_position(1) + param.d_cr() * sin_theta;

  // 宽度和长度方向在全局坐标系中的投影偏移
  decimal_t d_wx = param.width() / 2 * sin_theta;   // 半宽在x方向的投影
  decimal_t d_wy = param.width() / 2 * cos_theta;   // 半宽在y方向的投影
  decimal_t d_lx = param.length() / 2 * cos_theta;   // 半长在x方向的投影
  decimal_t d_ly = param.length() / 2 * sin_theta;   // 半长在y方向的投影

  // 按逆时针方向，从左前顶点开始计算
  vertices->push_back(Vec2f(c_x - d_wx + d_lx, c_y + d_wy + d_ly));  // 左前
  vertices->push_back(Vec2f(c_x - d_wx - d_lx, c_y - d_ly + d_wy));  // 左后
  vertices->push_back(Vec2f(c_x + d_wx - d_lx, c_y - d_wy - d_ly));  // 右后
  vertices->push_back(Vec2f(c_x + d_wx + d_lx, c_y + d_ly - d_wy));  // 右前

  return kSuccess;
}

/*
 * SemanticsUtils::InflateVehicleBySize - 按给定增量膨胀车辆尺寸
 *
 * 创建一个新的车辆对象，其宽度和长度在原基础上分别增加 delta_w 和 delta_l。
 * 用于碰撞检测中创建带安全边距的扩大轮廓。
 *
 * @param vehicle_in 输入车辆
 * @param delta_w 宽度增量
 * @param delta_l 长度增量
 * @param vehicle_out 输出参数，膨胀后的车辆对象
 * @return kSuccess
 */
ErrorType SemanticsUtils::InflateVehicleBySize(const Vehicle &vehicle_in,
                                               const decimal_t delta_w,
                                               const decimal_t delta_l,
                                               Vehicle *vehicle_out) {
  common::Vehicle inflated_vehicle = vehicle_in;
  common::VehicleParam vehicle_param = vehicle_in.param();
  vehicle_param.set_width(vehicle_param.width() + delta_w);
  vehicle_param.set_length(vehicle_param.length() + delta_l);
  inflated_vehicle.set_param(vehicle_param);
  *vehicle_out = inflated_vehicle;
  return kSuccess;
}

}  // namespace common
