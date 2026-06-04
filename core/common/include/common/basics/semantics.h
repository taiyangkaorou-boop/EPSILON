/**
 * @file semantics.h
 * @author HKUST Aerial Robotics Group
 * @brief EPSILON 自动驾驶决策规划系统 — 语义层核心类型定义头文件
 *
 * @details
 * 本文件是 EPSILON 系统中最重要的头文件之一，定义了整个自动驾驶语义层的核心数据结构。
 * 语义层位于"原始感知数据"与"行为决策/运动规划"之间，负责将低层几何信息
 * 抽象为具有物理和逻辑含义的高层语义对象。
 *
 * 核心概念体系：
 *   - 车辆模型：VehicleParam（车辆物理参数）、Vehicle（车辆实例）
 *   - 行为语义：LateralBehavior（横向行为：车道保持/左换道/右换道）、
 *     LongitudinalBehavior（纵向行为：保持/加速/减速/停车）、
 *     SemanticBehavior（复合语义行为）、ProbDistOfLatBehaviors（概率分布）
 *   - 交通环境：LaneNet（车道网络）、ObstacleSet（障碍物集合）、
 *     TrafficSignal（交通信号灯/限速/停车标志）
 *   - 空间表示：GridMapND（N维栅格地图）、SpatioTemporalSemanticCubeNd（时空语义立方体）、
 *     DrivingCube / DrivingCorridor（可行驶空间走廊）
 *   - 控制信号：VehicleControlSignal（车辆控制信号，支持开/闭环两种模式）
 *   - KD树辅助：PointVecForKdTree（用于 nanoflann 空间查询的数据适配器）
 *
 * @note 本文件是整个 EPSILON 系统类型体系的集大成者，几乎被所有上层模块所依赖。
 *       修改本文件中的数据结构时将影响整个系统的编译和运行。
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_COMMON_INC_BASICS_SEMANTICS_H_
#define _CORE_COMMON_INC_BASICS_SEMANTICS_H_

#include <assert.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/shapes.h"
#include "common/basics/tool_func.h"
#include "common/lane/lane.h"
#include "common/state/free_state.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"

namespace common {

/**
 * @class VehicleParam
 * @brief 车辆物理参数模型
 *
 * @details
 * 存储车辆的运动学/动力学约束参数。所有使用自行车模型（Bicycle Model）
 * 的运动规划和仿真均依赖此参数结构。默认值对应标准乘用车尺寸。
 *
 * 关键参数说明：
 *   - wheel_base_：轴距，决定最小转弯半径
 *   - front_suspension_ / rear_suspension_：前后悬长度，用于计算车辆外形
 *   - d_cr_：几何中心到后轴中心的距离（Geometry center to rear axle）
 *   - max_steering_angle_：最大转向角（度），用于曲率约束
 *   - max_longitudinal_acc_ / max_lateral_acc_：纵向/横向最大加速度，用于动态可行性约束
 */
class VehicleParam {
 public:
  // ========== Accessors（访问器） ==========

  /// @brief 获取车辆宽度（米）
  inline double width() const { return width_; }
  /// @brief 获取车辆长度（米）
  inline double length() const { return length_; }
  /// @brief 获取轴距（米）
  inline double wheel_base() const { return wheel_base_; }
  /// @brief 获取前悬长度（米），即前轴到车头的距离
  inline double front_suspension() const { return front_suspension_; }
  /// @brief 获取后悬长度（米），即后轴到车尾的距离
  inline double rear_suspension() const { return rear_suspension_; }
  /// @brief 获取最大转向角（度）
  inline double max_steering_angle() const { return max_steering_angle_; }
  /// @brief 获取最大纵向加速度（m/s²），正值表示加速能力上限
  inline double max_longitudinal_acc() const { return max_longitudinal_acc_; }
  /// @brief 获取最大横向加速度（m/s²），用于变道/转弯时的向心加速度约束
  inline double max_lateral_acc() const { return max_lateral_acc_; }
  /// @brief 获取几何中心到后轴中心的距离（米）
  inline double d_cr() const { return d_cr_; }

  // ========== Mutators（修改器） ==========

  inline void set_width(const double val) { width_ = val; }
  inline void set_length(const double val) { length_ = val; }
  inline void set_wheel_base(const double val) { wheel_base_ = val; }
  inline void set_front_suspension(const double val) {
    front_suspension_ = val;
  }
  inline void set_rear_suspension(const double val) { rear_suspension_ = val; }
  inline void set_max_steering_angle(const double val) {
    max_steering_angle_ = val;
  }
  inline void set_max_longitudinal_acc(const double val) {
    max_longitudinal_acc_ = val;
  }
  inline void set_max_lateral_acc(const double val) { max_lateral_acc_ = val; }
  inline void set_d_cr(const double val) { d_cr_ = val; }

  /**
   * @brief 打印车辆参数信息（调试用）
   */
  void print() const;

 private:
  double width_ = 1.90;              ///< 车辆宽度（m），默认 1.90m
  double length_ = 4.88;             ///< 车辆长度（m），默认 4.88m
  double wheel_base_ = 2.85;         ///< 轴距（m），默认 2.85m
  double front_suspension_ = 0.93;   ///< 前悬长度（m）
  double rear_suspension_ = 1.10;    ///< 后悬长度（m）
  double max_steering_angle_ = 45.0; ///< 最大转向角（度），默认 45°

  double max_longitudinal_acc_ = 2.0; ///< 最大纵向加速度（m/s²）
  double max_lateral_acc_ = 2.0;      ///< 最大横向加速度（m/s²）

  double d_cr_ = 1.34;  ///< 几何中心到后轴中心的距离（m），d_cr = length/2 - rear_suspension
};

/**
 * @class Vehicle
 * @brief 车辆实例 — 将车辆物理参数与实时状态绑定
 *
 * @details
 * Vehicle 是 EPSILON 中表示道路参与者的基本单元。它组合了以下要素：
 *   - id_：全局唯一的车辆标识
 *   - subclass_ / type_：车辆分类（轿车/卡车/公交等）
 *   - param_：车辆物理参数（尺寸、动力学约束）
 *   - state_：当前运动状态（位置、速度、加速度等）
 *
 * 提供的方法主要包括：
 *   - Ret3DofState()：返回后轴中心的三自由度状态 (x, y, yaw)
 *   - RetOrientedBoundingBox()：返回 OBB 包围盒，用于碰撞检测
 *   - RetVehicleVertices()：返回车辆四个角点，用于可视化/几何计算
 *   - RetBumperVertices()：返回前后保险杠中点，用于跟车距离计算
 *
 * @note Vehicle 的 State 使用的是笛卡尔坐标系下的自由状态（FreeState），
 *       而非 Frenet 状态。Frenet 状态由 FsVehicle 结构单独管理。
 */
class Vehicle {
 public:
  /// @brief 默认构造函数
  Vehicle();
  /// @brief 通过参数和状态构造车辆
  Vehicle(const VehicleParam &param, const State &state);
  /// @brief 通过 ID、参数和状态构造车辆
  Vehicle(const int &id, const VehicleParam &param, const State &state);
  /// @brief 通过 ID、子类型、参数和状态构造车辆
  Vehicle(const int &id, const std::string &subclass, const VehicleParam &param,
          const State &state);

  // ========== Accessors ==========
  inline int id() const { return id_; }
  inline std::string subclass() const { return subclass_; }
  inline VehicleParam param() const { return param_; }
  inline State state() const { return state_; }
  inline std::string type() const { return type_; }

  // ========== Mutators ==========
  inline void set_id(const int &id) { id_ = id; }
  inline void set_subclass(const std::string &subclass) {
    subclass_ = subclass;
  }
  inline void set_type(const std::string &type) { type_ = type; }
  inline void set_param(const VehicleParam &in) { param_ = in; }
  inline void set_state(const State &in) { state_ = in; }

  /**
   * @brief 获取后轴中心的三自由度状态
   *
   * @details
   * 提取车辆后轴中心点的 (x, y, yaw)，这是自行车模型的标准参考点。
   * 运动规划和仿真均以后轴中心作为控制点。
   *
   * @return Vec3f 三自由度状态向量 [x, y, yaw]ᵀ
   */
  Vec3f Ret3DofState() const;

  /**
   * @brief 获取车辆的二维有向包围盒
   *
   * @details
   * OBB 是碰撞检测的核心几何表示。中心位于车辆几何中心，
   * 方向与车辆朝向一致，宽度和长度对应车辆外形尺寸。
   *
   * @return OrientedBoundingBox2D 车辆 OBB 包围盒
   */
  OrientedBoundingBox2D RetOrientedBoundingBox() const;

  /**
   * @brief 获取车辆四个角点坐标
   *
   * @details
   * 按顺序返回左上、右上、右下、左下四个角点（相对于车辆朝向），
   * 常用于可视化渲染和点-多边形包含判断。
   *
   * @param vertices [out] 角点容器指针
   * @return ErrorType 错误码
   */
  ErrorType RetVehicleVertices(vec_E<Vec2f> *vertices) const;

  /**
   * @brief 获取车辆前后保险杠中点坐标
   *
   * @details
   * 返回纵向轴上的前后端点，常用于计算跟车距离、安全间距等。
   *
   * @param vertices [out] 包含2个点的数组指针，[0]为前保险杠，[1]为后保险杠
   * @return ErrorType 错误码
   */
  ErrorType RetBumperVertices(std::array<Vec2f, 2> *vertices) const;

  /**
   * @brief 获取车辆几何中心的三自由度状态
   *
   * @details
   * 与 Ret3DofState() 的区别在于参考点为几何中心（而非后轴中心）。
   * 通过 d_cr_ 参数进行坐标偏移转换。
   *
   * @param state [out] 几何中心状态 [x, y, yaw]ᵀ
   * @return ErrorType 错误码
   */
  ErrorType Ret3DofStateAtGeometryCenter(Vec3f *state) const;

  /**
   * @brief 打印车辆信息（调试用）
   */
  void print() const;

 private:
  int id_{kInvalidAgentId};  ///< 车辆唯一标识，默认值为无效ID
  std::string subclass_;     ///< 车辆子类型（如 "car", "truck", "bus"）
  std::string type_;         ///< 车辆大类（如 "vehicle", "pedestrian", "cyclist"）
  VehicleParam param_;       ///< 车辆物理参数（尺寸/动力学约束）
  State state_;              ///< 车辆当前运动状态（自由坐标系）
};

/**
 * @enum LongitudinalBehavior
 * @brief 纵向行为枚举 — 描述车辆在 Frenet 坐标系 s 方向上的意图
 *
 * @details
 * 纵向行为用于描述车辆的加减速意图，是行为预测（Behavior Prediction）
 * 和自车决策的基本语义单元。
 *
 * - kMaintain：保持当前速度（匀速巡航）
 * - kAccelerate：加速（如驶离路口、超车意图）
 * - kDecelerate：减速（如来车让行、接近前车）
 * - kStopping：停车（如到达停止线、红灯停车）
 */
enum class LongitudinalBehavior {
  kMaintain = 0,   ///< 保持当前速度
  kAccelerate,     ///< 加速
  kDecelerate,     ///< 减速
  kStopping        ///< 停车
};

/**
 * @enum LateralBehavior
 * @brief 横向行为枚举 — 描述车辆在 Frenet 坐标系 d 方向上的意图
 *
 * @details
 * 横向行为是 EPSILON 行为决策系统的核心概念，决定了车辆的空间移动策略：
 *
 * - kUndefined：未定义（初始状态/感知丢失时使用）
 * - kLaneKeeping：车道保持（沿当前车道中心线行驶）
 * - kLaneChangeLeft：向左换道
 * - kLaneChangeRight：向右换道
 *
 * @note 横向行为与纵向行为正交，两者组合成 SemanticBehavior 来描述
 *       一个完整的驾驶意图（如"向左换道+加速"）。
 */
enum class LateralBehavior {
  kUndefined = 0,     ///< 未定义（默认初始值）
  kLaneKeeping,       ///< 车道保持
  kLaneChangeLeft,    ///< 向左换道
  kLaneChangeRight,   ///< 向右换道
};

/**
 * @struct EnumClassHash
 * @brief 为 enum class 提供哈希函数，使其可用作 unordered_map 的键
 *
 * @details
 * C++ 标准库默认不支持 enum class 的哈希，需要自定义哈希函数。
 * 此结构将枚举值 static_cast 为 size_t，实现简单的哈希映射。
 * 用于 ProbDistOfLatBehaviors 中的 unordered_map<LateralBehavior, decimal_t>。
 */
struct EnumClassHash {
  template <typename T>
  std::size_t operator()(T t) const {
    return static_cast<std::size_t>(t);
  }
};

/**
 * @struct ProbDistOfLatBehaviors
 * @brief 横向行为概率分布 — 行为预测模块的输出
 *
 * @details
 * 存储某车辆执行三种横向行为（左换道/右换道/车道保持）的预测概率。
 * 是行为预测模块的核心输出数据结构，供决策模块进行风险评估使用。
 *
 * 主要功能：
 *   - SetEntry()：设置特定行为的概率值
 *   - CheckIfNormalized()：验证概率分布是否归一化（和是否为 1.0）
 *   - GetMaxProbBehavior()：返回最大概率对应的行为
 *
 * @note is_valid 标志用于指示该概率分布是否有效（行为预测模块是否成功运行）
 */
struct ProbDistOfLatBehaviors {
  bool is_valid = false;  ///< 概率分布是否有效的标志位
  /// @brief 行为概率映射表，key=横向行为枚举，value=概率值
  std::unordered_map<LateralBehavior, decimal_t, EnumClassHash> probs{
      {common::LateralBehavior::kLaneChangeLeft, 0.0},
      {common::LateralBehavior::kLaneChangeRight, 0.0},
      {common::LateralBehavior::kLaneKeeping, 0.0}};

  /**
   * @brief 设置指定行为的概率值
   * @param beh 行为类型
   * @param val 概率值
   */
  void SetEntry(const LateralBehavior &beh, const decimal_t &val) {
    probs[beh] = val;
  }

  /**
   * @brief 检查概率分布是否已归一化（所有概率之和是否接近 1.0）
   * @return true 已归一化 / false 未归一化
   */
  bool CheckIfNormalized() const {
    decimal_t sum = 0.0;
    for (const auto &entry : probs) {
      sum += entry.second;
    }
    if (fabs(sum - 1.0) < kEPS) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * @brief 返回最大概率对应的行为（ArgMax）
   * @param beh [out] 最大概率行为
   * @return true 成功获取 / false 分布无效
   */
  bool GetMaxProbBehavior(LateralBehavior *beh) const {
    if (!is_valid) return false;

    decimal_t max_prob = -1.0;
    LateralBehavior max_beh;
    for (const auto &entry : probs) {
      if (entry.second > max_prob) {
        max_prob = entry.second;
        max_beh = entry.first;
      }
    }
    *beh = max_beh;
    return true;
  }
};

/**
 * @struct SemanticBehavior
 * @brief 语义行为 — 横向与纵向行为的复合体，用于描述完整的驾驶意图
 *
 * @details
 * 语义行为 = 横向行为 + 纵向行为 + 参考车道 + 期望速度，描述了一个完整
 * 的驾驶意图（例如："沿给定车道向左变道，并加速至目标速度"）。
 *
 * 此外还包含：
 *   - forward_trajs：未来多个时间步的前向轨迹（用于预测）
 *   - forward_behaviors：未来时间步对应的横向行为序列
 *   - surround_trajs：周围多智能体的前向轨迹
 *   - state：当前车辆状态
 *
 * @note ref_lane 可能不是物理世界中实际存在的车道，而是由物理车道和离散行为
 *       重构出的"逻辑车道"（例如变道过程中的虚拟引导线）。
 */
struct SemanticBehavior {
  LateralBehavior lat_behavior;       ///< 横向行为（车道保持/左变道/右变道）
  LongitudinalBehavior lon_behavior;  ///< 纵向行为（保持/加速/减速/停车）
  Lane ref_lane;                      ///< 参考车道（可能为重构的逻辑车道）
  decimal_t actual_desired_velocity{0.0}; ///< 实际期望速度（m/s）

  vec_E<vec_E<Vehicle>> forward_trajs;                    ///< 多模态前向轨迹（每模态为一条时序序列）
  std::vector<LateralBehavior> forward_behaviors;         ///< 前向行为序列（每个时间步对应一个行为）
  vec_E<std::unordered_map<int, vec_E<Vehicle>>> surround_trajs; ///< 周围车辆的前向轨迹（key=车辆ID）

  State state;  ///< 当前车辆状态

  /// @brief 默认构造函数：默认为"车道保持+匀速"
  SemanticBehavior() {
    lat_behavior = LateralBehavior::kLaneKeeping;
    lon_behavior = LongitudinalBehavior::kMaintain;
  }
  /// @brief 通过横向行为构造（纵向行为使用默认值）
  SemanticBehavior(const LateralBehavior &beh) : lat_behavior(beh) {}
};

/**
 * @struct SemanticVehicle
 * @brief 语义车辆 — 在 Vehicle 基础上附加车道关联和预测信息
 *
 * @details
 * SemanticVehicle 将"纯几何/状态"的 Vehicle 提升到语义层面：
 *   - nearest_lane_id：最近车道 ID，用于建立车辆-车道拓扑关联
 *   - dist_to_lane：到最近车道的垂直距离
 *   - arc_len_onlane：在最近车道上的弧长投影位置
 *   - probs_lat_behaviors：行为预测概率分布（由预测模块填充）
 *   - lat_behavior：取 argmax 后的最可能行为
 *   - lane：关联的 Lane 对象
 *
 * @note 此结构是语义地图构建和态势理解的核心输入。
 */
struct SemanticVehicle {
  Vehicle vehicle;  ///< 基础车辆对象（几何+状态）

  // ========== 最近车道信息 ==========
  int nearest_lane_id{kInvalidLaneId};  ///< 最近车道 ID，默认无效 ID
  decimal_t dist_to_lane{-1.0};         ///< 到最近车道的垂直距离（m），-1.0 表示未计算
  decimal_t arc_len_onlane{-1.0};       ///< 在最近车道上的弧长位置（m），-1.0 表示未计算

  // ========== 行为预测 ==========
  ProbDistOfLatBehaviors probs_lat_behaviors; ///< 横向行为概率分布（预测模块输出）

  // ========== ArgMax 行为 ==========
  LateralBehavior lat_behavior{LateralBehavior::kUndefined}; ///< 最可能的横向行为
  Lane lane; ///< 关联的车道对象
};

/**
 * @struct SemanticVehicleSet
 * @brief 语义车辆集合 — 以 unordered_map 管理所有语义车辆
 *
 * @details
 * key=车辆 ID，value=对应 SemanticVehicle。提供 O(1) 查找和高效的迭代遍历。
 */
struct SemanticVehicleSet {
  std::unordered_map<int, SemanticVehicle> semantic_vehicles;
};

/**
 * @struct VehicleSet
 * @brief 基础车辆集合 — 管理非语义化（纯几何+状态）的车辆对象
 */
struct VehicleSet {
  std::unordered_map<int, Vehicle> vehicles;  ///< key=车辆ID, value=Vehicle

  /**
   * @brief 打印集合信息（调试用）
   */
  void print() const;
};

/**
 * @struct FsVehicle
 * @brief Frenet 框架下的车辆表示
 *
 * @details
 * 将车辆状态转换到 Frenet 坐标系 [s, s_dot, s_ddot, d, d_dot, d_ddot]，
 * 并保存其角点坐标。在车道级决策和轨迹优化中广泛使用。
 *
 * @note frenet_state 表示车辆在 Frenet 坐标系下的完整运动状态，
 *       vertices 保存笛卡尔坐标系下的角点坐标（用于可视化/碰撞检测）。
 */
struct FsVehicle {
  FrenetState frenet_state;  ///< Frenet 坐标系下的车辆状态
  vec_E<Vec2f> vertices;     ///< 笛卡尔坐标系下的车辆角点
};

/**
 * @struct VehicleControlSignal
 * @brief 车辆控制信号 — 支持开环和闭环两种控制模式
 *
 * @details
 * 闭环模式（is_openloop = false）：
 *   使用 acc（纵向加速度）和 steer_rate（转向速率）作为控制输入。
 *   由运动规划输出的轨迹经过控制器转化得到。
 *
 * 开环模式（is_openloop = true）：
 *   直接使用 state（目标状态）作为控制指令。适用于仿真中的理想执行场景，
 *   假设车辆可以瞬间达到目标状态。
 */
struct VehicleControlSignal {
  double acc = 0.0;           ///< 纵向加速度（m/s²），闭环模式使用
  double steer_rate = 0.0;    ///< 转向速率（rad/s），闭环模式使用
  bool is_openloop = false;   ///< 开环模式标志：false=闭环, true=开环
  common::State state;        ///< 目标状态，开环模式使用

  /// @brief 默认构造函数
  VehicleControlSignal();

  /**
   * @brief 闭环模式构造函数
   * @param acc 纵向加速度（m/s²）
   * @param steer_rate 转向速率（rad/s）
   */
  VehicleControlSignal(double acc, double steer_rate);

  /**
   * @brief 开环模式构造函数
   * @param state 目标期望状态
   */
  VehicleControlSignal(common::State state);
};

/**
 * @struct VehicleControlSignalSet
 * @brief 车辆控制信号集合 — 多车控制的批量管理
 *
 * @details
 * key=车辆 ID，value=对应控制信号。支持同时向多个车辆下发控制指令。
 */
struct VehicleControlSignalSet {
  std::unordered_map<int, VehicleControlSignal> signal_set;  ///< 控制信号映射表
};

/**
 * @struct GridMapMetaInfo
 * @brief 栅格地图元信息 — 描述栅格地图的物理和逻辑尺寸
 *
 * @details
 * 包含栅格地图的基本参数：
 *   - width/height：栅格宽度/高度（像素/格数）
 *   - resolution：每个栅格的物理尺寸（m/pixel）
 *   - w_metric/h_metric：地图的物理宽/高（米），w_metric = width * resolution
 */
struct GridMapMetaInfo {
  int width = 0;         ///< 栅格宽度（格数）
  int height = 0;        ///< 栅格高度（格数）
  double resolution = 0; ///< 每个栅格的物理分辨率（m/pixel）
  double w_metric = 0;   ///< 地图物理宽度（米），= width * resolution
  double h_metric = 0;   ///< 地图物理高度（米），= height * resolution

  /// @brief 默认构造函数
  GridMapMetaInfo();

  /**
   * @brief 参数化构造函数
   * @param w 栅格宽度
   * @param h 栅格高度
   * @param res 分辨率（m/pixel）
   */
  GridMapMetaInfo(const int w, const int h, const double res);

  /**
   * @brief 打印元信息（调试用）
   */
  void print() const;
};

/**
 * @class GridMapND
 * @brief N维栅格地图 — EPSILON 中所有离散化空间表示的基础模板类
 *
 * @details
 * 这是一个泛型 N 维栅格地图模板类，支持任意维度、任意数据类型的栅格存储和查询。
 * 在 EPSILON 中的典型实例包括：
 *   - GridMapND<uint8_t, 2>：二维占用栅格地图（OCCUPIED/FREE/UNKNOWN）
 *   - GridMapND<double, 2>：二维代价地图（用于路径规划的成本函数）
 *   - GridMapND<double, 3>：三维时空栅格（x, y, t），用于时空碰撞检测
 *
 * 核心功能：
 *   - 全局坐标 <-> 栅格坐标 的双向转换
 *   - 坐标范围检查
 *   - N维索引 <-> 一维索引（展平索引）的转换
 *   - 基于坐标或全局位置的读写操作
 *
 * 栅格值类型枚举 ValType：
 *   - OCCUPIED = 70：已占用
 *   - FREE = 0：空闲可通行
 *   - UNKNOWN = 0：未知区域
 *   - SCANNED_OCCUPIED = 128：通过传感器扫描确认为占用
 *
 * @tparam T 栅格存储的数据类型（uint8_t, double, int 等）
 * @tparam N_DIM 栅格地图的维度数
 */
template <typename T, int N_DIM>
class GridMapND {
 public:
  /**
   * @enum ValType
   * @brief 栅格值类型枚举（适用于 uint8_t 类型的地图）
   */
  enum ValType {
    OCCUPIED = 70,           ///< 已占用（障碍物）
    // FREE = 102,            // ~ 废弃的自由值
    FREE = 0,                ///< 空闲可通行
    SCANNED_OCCUPIED = 128,  ///< 传感器扫描确认占用
    UNKNOWN = 0              ///< 未知区域（值与 FREE 相同，语义不同）
  };

  /// @brief 默认构造函数
  GridMapND();

  /**
   * @brief 参数化构造函数
   * @param dims_size 各维度尺寸（格数）
   * @param dims_resolution 各维度分辨率（物理单位/格）
   * @param dims_name 各维度名称（如 {"x", "y", "t"}）
   */
  GridMapND(const std::array<int, N_DIM> &dims_size,
            const std::array<decimal_t, N_DIM> &dims_resolution,
            const std::array<std::string, N_DIM> &dims_name);

  // ========== Accessors ==========
  /// @brief 获取各维度尺寸
  inline std::array<int, N_DIM> dims_size() const { return dims_size_; }
  /// @brief 获取指定维度的尺寸
  inline int dims_size(const int &dim) const { return dims_size_.at(dim); }
  /// @brief 获取各维度的步长（用于 N 维 -> 1 维索引转换）
  inline std::array<int, N_DIM> dims_step() const { return dims_step_; }
  /// @brief 获取指定维度的步长
  inline int dims_step(const int &dim) const { return dims_step_.at(dim); }
  /// @brief 获取各维度分辨率
  inline std::array<decimal_t, N_DIM> dims_resolution() const {
    return dims_resolution_;
  }
  /// @brief 获取指定维度的分辨率
  inline decimal_t dims_resolution(const int &dim) const {
    return dims_resolution_.at(dim);
  }
  /// @brief 获取各维度名称
  inline std::array<std::string, N_DIM> dims_name() const { return dims_name_; }
  /// @brief 获取指定维度的名称
  inline std::string dims_name(const int &dim) const {
    return dims_name_.at(dim);
  }
  /// @brief 获取地图原点在全局坐标系下的位置
  inline std::array<decimal_t, N_DIM> origin() const { return origin_; }
  /// @brief 获取展平后的总数据量 = product(dims_size_[i])
  inline int data_size() const { return data_size_; }
  /// @brief 获取原始数据指针（只读）
  inline const std::vector<T> *data() const { return &data_; }
  /// @brief 获取第 i 个栅格的值
  inline T data(const int &i) const { return data_[i]; };
  /// @brief 获取原始数据指针（可写）
  inline T *get_data_ptr() { return data_.data(); }
  /// @brief 获取原始数据常量指针
  inline const T *data_ptr() const { return data_.data(); }

  // ========== Mutators ==========

  /// @brief 设置地图原点
  inline void set_origin(const std::array<decimal_t, N_DIM> &origin) {
    origin_ = origin;
  }
  /// @brief 设置各维度尺寸（会同步计算步长和数据量）
  inline void set_dims_size(const std::array<int, N_DIM> &dims_size) {
    dims_size_ = dims_size;
    SetNDimSteps(dims_size);
    SetDataSize(dims_size);
  }
  /// @brief 设置各维度分辨率
  inline void set_dims_resolution(
      const std::array<decimal_t, N_DIM> &dims_resolution) {
    dims_resolution_ = dims_resolution;
  }
  /// @brief 设置各维度名称
  inline void set_dims_name(const std::array<std::string, N_DIM> &dims_name) {
    dims_name_ = dims_name;
  }
  /// @brief 设置地图数据
  inline void set_data(const std::vector<T> &in) { data_ = in; }

  /**
   * @brief 将所有栅格数据清零
   */
  inline void clear_data() { data_ = std::vector<T>(data_size_, 0); }

  /**
   * @brief 将所有栅格填充为指定值
   * @param val 填充值
   */
  inline void fill_data(const T &val) {
    data_ = std::vector<T>(data_size_, val);
  }

  /**
   * @brief 通过栅格坐标（N 维索引）获取栅格值
   * @param coord 栅格坐标
   * @param val [out] 输出值
   * @return ErrorType
   */
  ErrorType GetValueUsingCoordinate(const std::array<int, N_DIM> &coord,
                                    T *val) const;

  /**
   * @brief 通过全局物理坐标获取栅格值
   * @param p_w 全局物理坐标
   * @param val [out] 输出值
   * @return ErrorType
   */
  ErrorType GetValueUsingGlobalPosition(const std::array<decimal_t, N_DIM> &p_w,
                                        T *val) const;

  /**
   * @brief 检查指定全局位置处的栅格值是否等于给定值
   * @param p_w 全局物理坐标
   * @param val_in 待比较值
   * @param res [out] 比较结果
   * @return ErrorType
   */
  ErrorType CheckIfEqualUsingGlobalPosition(
      const std::array<decimal_t, N_DIM> &p_w, const T &val_in,
      bool *res) const;

  /**
   * @brief 检查指定栅格坐标处的值是否等于给定值
   * @param coord 栅格坐标
   * @param val_in 待比较值
   * @param res [out] 比较结果
   * @return ErrorType
   */
  ErrorType CheckIfEqualUsingCoordinate(const std::array<int, N_DIM> &coord,
                                        const T &val_in, bool *res) const;

  /**
   * @brief 通过栅格坐标设置栅格值
   * @param coord 栅格坐标
   * @param val 待写入值
   * @return ErrorType
   */
  ErrorType SetValueUsingCoordinate(const std::array<int, N_DIM> &coord,
                                    const T &val);

  /**
   * @brief 通过全局物理坐标设置栅格值
   * @param p_w 全局物理坐标
   * @param val 待写入值
   * @return ErrorType
   */
  ErrorType SetValueUsingGlobalPosition(const std::array<decimal_t, N_DIM> &p_w,
                                        const T &val);

  /**
   * @brief 全局物理坐标 -> 栅格坐标转换
   * @param p_w 全局物理坐标
   * @return std::array<int, N_DIM> 栅格坐标（向下取整）
   */
  std::array<int, N_DIM> GetCoordUsingGlobalPosition(
      const std::array<decimal_t, N_DIM> &p_w) const;

  /**
   * @brief 获取全局坐标对应栅格中心点的物理坐标（即四舍五入后的坐标）
   * @param p_w 全局物理坐标
   * @return std::array<decimal_t, N_DIM> 栅格中心物理坐标
   */
  std::array<decimal_t, N_DIM> GetRoundedPosUsingGlobalPosition(
      const std::array<decimal_t, N_DIM> &p_w) const;

  /**
   * @brief 栅格坐标 -> 全局物理坐标转换
   * @param coord 栅格坐标
   * @param p_w [out] 对应的全局物理坐标（栅格中心）
   * @return ErrorType
   */
  ErrorType GetGlobalPositionUsingCoordinate(
      const std::array<int, N_DIM> &coord,
      std::array<decimal_t, N_DIM> *p_w) const;

  /**
   * @brief 在单个维度上，将全局度量值转换为栅格坐标
   * @param metric 全局物理坐标在某维度的分量
   * @param i 维度索引
   * @param idx [out] 该维度的栅格坐标
   * @return ErrorType
   */
  ErrorType GetCoordUsingGlobalMetricOnSingleDim(const decimal_t &metric,
                                                 const int &i, int *idx) const;

  /**
   * @brief 在单个维度上，将栅格坐标转换为全局度量值（即该栅格中心点的物理坐标）
   * @param idx 栅格坐标
   * @param i 维度索引
   * @param metric [out] 全局度量值
   * @return ErrorType
   */
  ErrorType GetGlobalMetricUsingCoordOnSingleDim(const int &idx, const int &i,
                                                 decimal_t *metric) const;

  /**
   * @brief 检查给定的 N 维栅格坐标是否在地图范围内
   * @param coord 栅格坐标
   * @return true 在范围内 / false 越界
   */
  bool CheckCoordInRange(const std::array<int, N_DIM> &coord) const;

  /**
   * @brief 检查指定维度上的栅格坐标是否在范围内
   * @param idx 单维栅格坐标
   * @param i 维度索引
   * @return true 在范围内 / false 越界
   */
  bool CheckCoordInRangeOnSingleDim(const int &idx, const int &i) const;

  /**
   * @brief N维索引 -> 一维展平索引转换
   *
   * @details
   * 采用行优先（Row-Major）布局进行索引展平。例如 2D 情况下：
   *   mono_idx = idx[1] * dims_step_[1] + idx[0] = idx_y * width + idx_x
   *
   * @param idx N 维栅格索引
   * @return int 一维展平索引
   */
  int GetMonoIdxUsingNDimIdx(const std::array<int, N_DIM> &idx) const;

  /**
   * @brief 一维展平索引 -> N维索引转换
   * @param idx 一维展平索引
   * @return std::array<int, N_DIM> N 维栅格索引
   */
  std::array<int, N_DIM> GetNDimIdxUsingMonoIdx(const int &idx) const;

 private:
  /**
   * @brief 根据各维度尺寸计算步长数组
   *
   * @details
   * 步长用于展平索引的计算。例如 (x, y, z) 三个维度对应的步长为：
   *   dims_step_ = {1, x_dim, x_dim * y_dim}
   * 展平索引 = coord[0]*step[0] + coord[1]*step[1] + coord[2]*step[2]
   *
   * @param dims_size 各维度尺寸
   * @return ErrorType
   */
  ErrorType SetNDimSteps(const std::array<int, N_DIM> &dims_size);

  /**
   * @brief 计算并设置数据总量
   * @param dims_size 各维度尺寸
   * @return ErrorType
   */
  ErrorType SetDataSize(const std::array<int, N_DIM> &dims_size);

  std::array<int, N_DIM> dims_size_;             ///< 各维度尺寸（格数）
  std::array<int, N_DIM> dims_step_;             ///< 各维度步长（用于N维->1维索引转换）
  std::array<decimal_t, N_DIM> dims_resolution_; ///< 各维度分辨率（物理单位/格）
  std::array<std::string, N_DIM> dims_name_;      ///< 各维度名称
  std::array<decimal_t, N_DIM> origin_;           ///< 地图原点在全局坐标系下的位置

  int data_size_{0};         ///< 展平后的总数据量
  std::vector<T> data_;      ///< 栅格数据存储（一维展平存储）
};

/**
 * @struct LaneRaw
 * @brief 原始车道信息 — 从地图文件解析出的车道拓扑和几何数据
 *
 * @details
 * 表示一条完整的车道段（Lane Segment），包含：
 *   - 拓扑信息：父子车道 ID、左右相邻车道 ID、变道可行性
 *   - 几何信息：起点、终点、车道线采样点
 *   - 语义信息：车道行为类型（直行、左转、右转等）、车道长度
 *
 * LaneRaw 与 Lane（common/lane/lane.h 中定义）的关系：
 *   - LaneRaw 是地图输入的"原始"表示（离散采样点 + 整数拓扑）
 *   - Lane 是 LaneRaw 经过样条拟合后的"连续"表示（多项式参数化曲线）
 *
 * @note l_change_avbl / r_change_avbl 表示是否允许向左/右变道，
 *       通常被实线或物理隔离带限制为 false。
 */
struct LaneRaw {
  int id;                     ///< 车道唯一 ID
  int dir;                    ///< 车道方向（顺向/逆向）

  std::vector<int> child_id;  ///< 后继车道 ID 列表（下游）
  std::vector<int> father_id; ///< 前驱车道 ID 列表（上游）

  int l_lane_id;              ///< 左侧相邻车道 ID（kInvalidLaneId 表示无左侧车道）
  bool l_change_avbl;         ///< 是否允许向左变道
  int r_lane_id;              ///< 右侧相邻车道 ID（kInvalidLaneId 表示无右侧车道）
  bool r_change_avbl;         ///< 是否允许向右变道

  std::string behavior;       ///< 车道行为类型（如 "straight", "left", "right"）
  decimal_t length;           ///< 车道长度（米）

  Vec2f start_point;          ///< 车道起点坐标（笛卡尔坐标系）
  Vec2f final_point;          ///< 车道终点坐标（笛卡尔坐标系）
  vec_E<Vec2f> lane_points;   ///< 车道中心线采样点序列

  /**
   * @brief 打印车道信息（调试用）
   */
  void print() const;
};

/**
 * @struct LaneNet
 * @brief 车道网络 — 由 LaneRaw 组成的完整地图拓扑图
 *
 * @details
 * 以 unordered_map 管理所有车道段，key=车道ID，value=LaneRaw。
 * 是整个路径规划、行为决策模块的空间参考基础。
 *
 * 典型操作：
 *   - 通过车道 ID 查找车道几何和拓扑信息
 *   - 遍历所有车道生成全局路径
 *   - 查询变道可行性
 */
struct LaneNet {
  std::unordered_map<int, LaneRaw> lane_set;  ///< 车道ID -> LaneRaw 映射表

  /// @brief 清空车道网络
  inline void clear() { lane_set.clear(); }

  /**
   * @brief 打印车道网络信息（调试用）
   */
  void print() const;
};

/**
 * @struct SemanticLane
 * @brief 语义车道 — LaneRaw 的"连续化+语义化"版本
 *
 * @details
 * 与 LaneRaw 具有相同的拓扑信息，但通过 Lane 对象实现了连续的几何表示。
 * Lane 对象提供了在车道线上进行弧长参数化查询的能力（位置、方向、曲率等）。
 *
 * 使用场景：
 *   - 在 LaneRaw 上构建 Lane 后存入 SemanticLane
 *   - SemanticLaneSet 是行为决策模块的直接输入
 */
struct SemanticLane {
  int id;                     ///< 车道唯一 ID
  int dir;                    ///< 车道方向

  std::vector<int> child_id;  ///< 后继车道 ID 列表
  std::vector<int> father_id; ///< 前驱车道 ID 列表

  int l_lane_id;              ///< 左侧相邻车道 ID
  bool l_change_avbl;         ///< 是否允许向左变道
  int r_lane_id;              ///< 右侧相邻车道 ID
  bool r_change_avbl;         ///< 是否允许向右变道

  std::string behavior;       ///< 车道行为类型
  decimal_t length;           ///< 车道长度（米）

  Lane lane;                  ///< 连续化的车道几何对象（样条曲线表示）
};

/**
 * @struct SemanticLaneSet
 * @brief 语义车道集合 — 管理所有 SemanticLane 的容器
 */
struct SemanticLaneSet {
  std::unordered_map<int, SemanticLane> semantic_lanes;  ///< key=车道ID

  /// @brief 获取集合大小
  inline int size() const { return semantic_lanes.size(); }

  /// @brief 清空所有语义车道
  void clear() { semantic_lanes.clear(); }

  /**
   * @brief 打印集合信息（调试用）
   */
  void print() const;
};

/**
 * @struct CircleObstacle
 * @brief 圆形障碍物 — 用圆心和半径建模的简化障碍物
 *
 * @details
 * 适用于行人、路锥、圆柱等可用圆近似描述的障碍物。
 * 圆形碰撞检测计算简单高效。
 */
struct CircleObstacle {
  int id;             ///< 障碍物唯一 ID
  int type = 0;       ///< 障碍物类型（预留，如 0=默认, 1=行人, 2=路锥等）
  Circle circle;      ///< 圆形几何描述（圆心+半径）

  /**
   * @brief 打印障碍物信息（调试用）
   */
  void print() const;
};

/**
 * @struct PolygonObstacle
 * @brief 多边形障碍物 — 用多边形顶点序列建模的精确障碍物
 *
 * @details
 * 适用于形状不规则的障碍物（建筑物、护栏、大型车辆等）。
 * 使用 SAT（分离轴定理）进行碰撞检测。
 */
struct PolygonObstacle {
  int id;             ///< 障碍物唯一 ID
  int type = 0;       ///< 障碍物类型
  Polygon polygon;     ///< 多边形几何描述（有序顶点序列）

  /**
   * @brief 打印障碍物信息（调试用）
   */
  void print() const;
};

/**
 * @struct ObstacleSet
 * @brief 障碍物集合 — 统一管理圆形和多边形障碍物
 *
 * @details
 * 将障碍物分为圆形和多边形两类管理，碰撞检测时分别处理。
 * size() 返回两类障碍物总数。
 */
struct ObstacleSet {
  std::unordered_map<int, CircleObstacle> obs_circle;   ///< 圆形障碍物映射表
  std::unordered_map<int, PolygonObstacle> obs_polygon; ///< 多边形障碍物映射表

  /// @brief 获取障碍物总数（圆形+多边形）
  inline int size() const { return obs_circle.size() + obs_polygon.size(); }

  /**
   * @brief 打印障碍物集合信息（调试用）
   */
  void print() const;
};

/**
 * @struct PointVecForKdTree
 * @brief nanoflann KD树适配器 — 将 PointWithValue<int> 容器适配为 KD树查询接口
 *
 * @details
 * nanoflann 库要求数据源实现三个方法：
 *   - kdtree_get_point_count()：返回点数
 *   - kdtree_get_pt(idx, dim)：返回第 idx 个点的第 dim 维坐标
 *   - kdtree_get_bbox()：返回包围盒（可省略）
 *
 * 此结构将 std::vector<PointWithValue<int>> 封装并实现上述接口，
 * 用于驾驶走廊/语义立方体的最近邻查询（如查找车道上最近的点）。
 *
 * @note 此适配器仅输出 x（dim=0）和 y（dim=1），即仅用于 2D 空间查询。
 */
struct PointVecForKdTree {
  std::vector<PointWithValue<int>> pts;  ///< 带值的点集，用于 KD树构建

  /// @brief nanoflann 接口：返回点总数
  inline size_t kdtree_get_point_count() const { return pts.size(); }

  /**
   * @brief nanoflann 接口：获取第 idx 个点的第 dim 维坐标
   * @param idx 点索引
   * @param dim 维度（0=x, 1=y）
   * @return decimal_t 坐标值
   */
  inline decimal_t kdtree_get_pt(const size_t idx, const size_t dim) const {
    return dim == 0 ? pts[idx].pt.x : pts[idx].pt.y;
  }

  /**
   * @brief nanoflann 接口：返回包围盒（未实现，总返回 false）
   * @tparam BBOX 包围盒类型
   * @return false 表示未提供包围盒
   */
  template <class BBOX>
  bool kdtree_get_bbox(BBOX & /* bb */) const {
    return false;
  }
};

/**
 * @struct SpatioTemporalSemanticCubeNd
 * @brief N维时空语义立方体 — 描述一个高维状态空间的轴对齐边界
 *
 * @details
 * 这是一个泛型时空边界描述结构，用于在多维状态空间中进行约束定义。
 * 包含以下边界：
 *   - 时间边界 [t_lb, t_ub]：有效时间窗口
 *   - 位置边界 [p_lb, p_ub]：N 维空间位置范围
 *   - 速度边界 [v_lb, v_ub]：N 维速度范围
 *   - 加速度边界 [a_lb, a_ub]：N 维加速度范围
 *
 * 典型应用场景：
 *   - 轨迹优化中的时空约束定义
 *   - 碰撞避免中的可达集（Reachable Set）近似
 *   - 驾驶走廊的语义化表示
 *
 * @note FillDefaultBounds() 使用松弛边界（而非无穷大），以避免
 *       数值不稳定问题（无穷大边界可能导致优化求解器发散）。
 *
 * @tparam N_DIM 空间维度数
 */
template <int N_DIM>
struct SpatioTemporalSemanticCubeNd {
  decimal_t t_lb, t_ub;                       ///< 时间下界/上界（秒）
  std::array<decimal_t, N_DIM> p_lb, p_ub;    ///< 位置下界/上界（米）
  std::array<decimal_t, N_DIM> v_lb, v_ub;    ///< 速度下界/上界（m/s）
  std::array<decimal_t, N_DIM> a_lb, a_ub;    ///< 加速度下界/上界（m/s²）

  /// @brief 默认构造函数，填充松弛默认边界
  SpatioTemporalSemanticCubeNd() { FillDefaultBounds(); }

  /**
   * @brief 填充松弛的默认边界值
   *
   * @details
   * 为避免数值不稳定，使用有限但足够大的边界值：
   *   - 时间：[0, 1] 秒
   *   - 位置：[-1, 1] 米
   *   - 速度：[-50, 50] m/s
   *   - 加速度：[-20, 20] m/s²
   */
  void FillDefaultBounds() {
    t_lb = 0.0;
    t_ub = 1.0;

    const decimal_t default_pos_lb = -1;
    const decimal_t default_pos_ub = 1;
    const decimal_t default_vel_lb = -50.0;
    const decimal_t default_vel_ub = 50.0;
    const decimal_t default_acc_lb = -20.0;
    const decimal_t default_acc_ub = 20.0;

    p_lb.fill(default_pos_lb);
    v_lb.fill(default_vel_lb);
    a_lb.fill(default_acc_lb);

    p_ub.fill(default_pos_ub);
    v_ub.fill(default_vel_ub);
    a_ub.fill(default_acc_ub);
  }
};

/**
 * @struct DrivingCube
 * @brief 可行驶立方体 — 三维时空网格中有种子的可通行区域单元
 *
 * @details
 * 将时空空间离散化为三维网格（x, y, t），每个 DrivingCube 表示一个
 * 可通行的时空体素，包含：
 *   - seeds：该体素内的种子点列表（栅格坐标）
 *   - cube：该体素的轴对齐边界
 *
 * 多个 DrivingCube 级联组成 DrivingCorridor，为运动规划提供时空走廊约束。
 */
struct DrivingCube {
  vec_E<Vec3i> seeds;                        ///< 种子点列表（x, y, t 栅格坐标）
  AxisAlignedCubeNd<int, 3> cube;            ///< 轴对齐立方体边界（格数坐标）
};

/**
 * @struct DrivingCorridor
 * @brief 驾驶走廊 — 时空连续的可行驶区域序列
 *
 * @details
 * 由多个 DrivingCube 按时间顺序组成的走廊，定义了车辆在整个规划时域内的
 * 可行空间范围。是时空轨迹优化（Spatio-Temporal Trajectory Optimization）
 * 的核心约束输入。
 *
 * - id：走廊唯一标识
 * - is_valid：走廊是否有效（碰撞检测或可达性分析失败时置为 false）
 * - cubes：按时间排序的 DrivingCube 序列
 */
struct DrivingCorridor {
  int id;                       ///< 走廊唯一 ID
  bool is_valid;                ///< 走廊有效性标志
  vec_E<DrivingCube> cubes;     ///< 按时间排序的可行驶立方体序列
};

/**
 * @class TrafficSignal
 * @brief 交通信号基类 — 描述各种交通控制设施的共同属性
 *
 * @details
 * 作为 SpeedLimit（限速牌）、StoppingSign（停止标志）、TrafficLight（红绿灯）的基类。
 * 定义了交通信号的通用属性：
 *   - start_point_ / end_point_：信号作用区域的起终点（二维坐标）
 *   - valid_time_：信号有效期 [起始时间, 结束时间]
 *   - vel_range_：速度限制范围 [下界, 上界]
 *   - lateral_range_：横向作用范围 [左边界, 右边界]
 *
 * 子类在构造函数中根据信号类型自动设置默认值。
 */
class TrafficSignal {
 public:
  /// @brief 默认构造函数
  TrafficSignal();
  /**
   * @brief 完整参数构造函数
   * @param start_point 作用区域起点
   * @param end_point 作用区域终点
   * @param valid_time 有效期 [开始时间, 结束时间]
   * @param vel_range 速度范围 [下限, 上限]
   */
  TrafficSignal(const Vec2f &start_point, const Vec2f &end_point,
                const Vec2f &valid_time, const Vec2f &vel_range);

  // ========== Mutators ==========
  void set_start_point(const Vec2f &start_point);
  void set_end_point(const Vec2f &end_point);
  /// @brief 设置有效期终止时间（开始时间保持不变的便捷方法）
  void set_valid_time_til(const decimal_t max_valid_time);
  /// @brief 设置有效期起始时间（终止时间保持不变的便捷方法）
  void set_valid_time_begin(const decimal_t min_valid_time);
  void set_valid_time(const Vec2f &valid_time);
  void set_vel_range(const Vec2f &vel_range);
  void set_lateral_range(const Vec2f &lateral_range);
  void set_max_velocity(const decimal_t max_velocity);

  void set_start_angle(const decimal_t angle) { start_angle_ = angle; }
  void set_end_angle(const decimal_t angle) { end_angle_ = angle; }

  // ========== Accessors ==========
  Vec2f start_point() const;
  Vec2f end_point() const;
  Vec2f valid_time() const;
  Vec2f vel_range() const;
  Vec2f lateral_range() const;
  decimal_t max_velocity() const;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  ///< Eigen 内存对齐宏（因为成员中包含 Eigen 类型）

  decimal_t start_angle() const { return start_angle_; }
  decimal_t end_angle() const { return end_angle_; }

 protected:
  Vec2f start_point_;          ///< 信号起点（x-y 平面二维坐标）
  decimal_t start_angle_ = 0;  ///< 起点方向角（临时，用于可视化）
  Vec2f end_point_;            ///< 信号终点（x-y 平面二维坐标）
  decimal_t end_angle_ = 0;    ///< 终点方向角（临时，用于可视化）
  Vec2f valid_time_;           ///< 有效期 [stamp_begin, stamp_end]
  Vec2f vel_range_;            ///< 速度范围 [lower_bound, upper_bound]（m/s）
  Vec2f lateral_range_;        ///< 横向作用范围 [左边界, 右边界]（m）
};

/**
 * @class SpeedLimit
 * @brief 限速标志 — 继承 TrafficSignal，表示道路限速区域
 *
 * @details
 * 构造函数中自动将 valid_time 设置为 (0, kInf)，表示无限期有效。
 * vel_range 为用户指定的速度限制范围。
 */
class SpeedLimit : public TrafficSignal {
 public:
  /**
   * @brief 限速标志构造函数
   * @param start_point 限速区域起点
   * @param end_point 限速区域终点
   * @param vel_range 速度范围 [下界, 上界]（m/s）
   */
  SpeedLimit(const Vec2f &start_point, const Vec2f &end_point,
             const Vec2f &vel_range);
};

/**
 * @class StoppingSign
 * @brief 停止标志 — 继承 TrafficSignal，表示停车让行标志
 *
 * @details
 * 在生成时将速度上限设为 0，表示车辆必须在停止线前完全停车。
 */
class StoppingSign : public TrafficSignal {
 public:
  /**
   * @brief 停止标志构造函数
   * @param start_point 停止线起点
   * @param end_point 停止线终点
   */
  StoppingSign(const Vec2f &start_point, const Vec2f &end_point);
};

/**
 * @class TrafficLight
 * @brief 红绿灯 — 继承 TrafficSignal，额外管理信号灯状态
 *
 * @details
 * 在 TrafficSignal 的基础上增加 Type 枚举表示灯色：
 *   - Green：绿灯，可通行
 *   - Red：红灯，必须停车
 *   - Yellow：黄灯，建议减速准备停车
 *   - RedYellow：红黄灯（欧洲常见信号阶段）
 *
 * 红绿灯的有效期会根据信号时序动态更新（通过 valid_time 的 setter 方法）。
 */
class TrafficLight : public TrafficSignal {
 public:
  /**
   * @enum Type
   * @brief 红绿灯灯色枚举
   */
  enum Type { Green = 0, Red, Yellow, RedYellow };

  /// @brief 设置灯色
  void set_type(const Type &type);
  /// @brief 获取当前灯色
  Type type() const;

 private:
  Type type_;  ///< 当前灯色
};

/**
 * @class SemanticsUtils
 * @brief 语义工具类 — 提供语义相关的静态工具函数
 *
 * @details
 * 包含一系列静态辅助函数，用于：
 *   - 行为枚举 -> 字符串名称的转换（用于日志/可视化）
 *   - 车辆状态 -> 包围盒的几何计算
 *   - 车辆尺寸缩放（安全裕度膨胀）
 */
class SemanticsUtils {
 public:
  /**
   * @brief 获取纵向行为名称（单字符缩写）
   *
   * 映射关系：kMaintain->"M", kAccelerate->"A", kDecelerate->"D", kStopping->"S"
   *
   * @param b 纵向行为枚举值
   * @return std::string 行为名称缩写
   */
  static std::string RetLonBehaviorName(const LongitudinalBehavior b) {
    std::string b_str;
    switch (b) {
      case LongitudinalBehavior::kMaintain: {
        b_str = std::string("M");
        break;
      }
      case LongitudinalBehavior::kAccelerate: {
        b_str = std::string("A");
        break;
      }
      case LongitudinalBehavior::kDecelerate: {
        b_str = std::string("D");
        break;
      }
      case LongitudinalBehavior::kStopping: {
        b_str = std::string("S");
        break;
      }
      default: {
        b_str = std::string("Null");
        break;
      }
    }
    return b_str;
  }

  /**
   * @brief 获取横向行为名称（单字符缩写）
   *
   * 映射关系：
   *   kUndefined->"U", kLaneKeeping->"K",
   *   kLaneChangeLeft->"L", kLaneChangeRight->"R"
   *
   * @param b 横向行为枚举值
   * @return std::string 行为名称缩写
   */
  static std::string RetLatBehaviorName(const LateralBehavior b) {
    std::string b_str;
    switch (b) {
      case LateralBehavior::kUndefined: {
        b_str = std::string("U");
        break;
      }
      case LateralBehavior::kLaneKeeping: {
        b_str = std::string("K");
        break;
      }
      case LateralBehavior::kLaneChangeLeft: {
        b_str = std::string("L");
        break;
      }
      case LateralBehavior::kLaneChangeRight: {
        b_str = std::string("R");
        break;
      }
      default: {
        b_str = std::string("Null");
        break;
      }
    }
    return b_str;
  }

  /**
   * @brief 根据车辆参数和状态计算 OBB 包围盒
   *
   * @details
   * 将车辆参数（长宽）和状态（位置+朝向）转换为二维有向包围盒。
   * OBB 中心位于车辆几何中心，方向角 = 车辆 yaw。
   *
   * @param param 车辆物理参数
   * @param s 车辆状态
   * @param obb [out] 输出的 OBB 包围盒
   * @return ErrorType
   */
  static ErrorType GetOrientedBoundingBoxForVehicleUsingState(
      const VehicleParam &param, const State &s, OrientedBoundingBox2D *obb);

  /**
   * @brief 获取车辆四个角点的坐标
   *
   * @details
   * 基于车辆尺寸和状态计算四个角点（左上、右上、右下、左下），
   * 常用于可视化和碰撞检测。
   *
   * @param param 车辆物理参数
   * @param state 车辆状态
   * @param vertices [out] 输出的角点向量
   * @return ErrorType
   */
  static ErrorType GetVehicleVertices(const VehicleParam &param,
                                      const State &state,
                                      vec_E<Vec2f> *vertices);

  /**
   * @brief 对车辆尺寸进行膨胀（安全裕度扩展）
   *
   * @details
   * 在原有车辆尺寸基础上增加安全裕度（delta_w 宽度增量, delta_l 长度增量）。
   * 通常用于：
   *   - 保守碰撞检测（前方预留更多安全距离）
   *   - 考虑感知噪声的包络扩展
   *
   * @param vehicle_in 输入车辆
   * @param delta_w 宽度增量（m）
   * @param delta_l 长度增量（m）
   * @param vehicle_out [out] 膨胀后的车辆
   * @return ErrorType
   */
  static ErrorType InflateVehicleBySize(const Vehicle &vehicle_in,
                                        const decimal_t delta_w,
                                        const decimal_t delta_l,
                                        Vehicle *vehicle_out);

};  // SemanticsUtils

}  // namespace common

#endif  // _CORE_COMMON_INC_BASICS_SEMANTICS_H_
