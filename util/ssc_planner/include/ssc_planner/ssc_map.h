/**
 * @file ssc_map.h
 * @author HKUST Aerial Robotics Group
 * @brief SSC（时空语义走廊）三维时空占据栅格地图
 *
 * [概述]
 * SscMap 维护一个 (s, d, t) 三维时空占据栅格地图，用于在 Frenet 坐标系中
 * 表达障碍物信息。它是 SSC 规划器构建时空走廊（Spatio-Temporal Corridor）
 * 的核心数据结构。
 *
 * [坐标轴含义]
 *   - X 轴 (轴0): s = 沿参考车道的纵向距离 (单位: 米)
 *   - Y 轴 (轴1): d = 相对车道中心线的横向偏移 (单位: 米)
 *   - Z 轴 (轴2): t = 时间 (单位: 秒)
 *
 * [核心功能]
 *   1. ConstructSscMap: 构建时空占据地图，填充静态和动态障碍物
 *   2. ConstructCorridorUsingInitialTrajectory: 沿前向轨迹膨胀无碰撞立方体，生成走廊
 *   3. InflateCubeIn3dGrid: 在六方向（s+/s-/d+/d-/t+/t-）上膨胀立方体
 *   4. GetFinalGlobalMetricCubesList: 从栅格坐标转换为物理单位 (米, 秒)
 *
 * [膨胀策略]
 *   使用种子点（seed）法：沿参考轨迹采样点作为种子，对每个种子膨胀为无碰撞的
 *   轴对齐立方体（AxisAlignedCube），膨胀受运动学约束（最大速度/加速度）限制。
 *
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_MAP_H_
#define _UTIL_SSC_PLANNER_INC_SSC_MAP_H_

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "common/basics/semantics.h"
#include "common/state/state.h"

namespace planning {

using ObstacleMapType = uint8_t;   ///< 障碍物地图单元格数据类型
using SscMapDataType = uint8_t;    ///< SSC 地图单元格数据类型

/// @brief 风险占据图数据类型 —— 浮点概率值 [0.0, 1.0]
/// @note 与 SscMapDataType (uint8_t 二值占据) 互补，用于表达碰撞风险的概率强度
using RiskMapDataType = float;
/// @brief 三维风险占据栅格图 —— 一维平铺存储，索引 = t_idx * w * h + d_idx * w + s_idx
/// @note MVP-0 使用 std::vector<float> 而非 GridMapND<float,3>，
///       避免对 common 库的额外模板实例化
using RiskGridMap3D = std::vector<RiskMapDataType>;

/// @brief 风险占据栅格统计结构 —— 用于 MVP-1A 验证风险图是否被正确填充和重置
/// @note 由 ComputeRiskGridStats() 填充，PrintRiskGridStatsIfNeeded() 输出日志。
///       该结构不参与任何规划决策，仅用于调试和验证
struct RiskGridStats {
  size_t total_cells = 0;                       ///< 风险图总栅格数
  size_t nonzero_cells = 0;                      ///< 风险值 > 0 的栅格数
  RiskMapDataType max_risk = 0.0f;               ///< 最大风险值
  double sum_risk = 0.0;                         ///< 风险值总和 (double 累加避免 float 截断)
  size_t active_time_layers = 0;                 ///< 至少有一个非零栅格的时间层数
  std::vector<size_t> nonzero_cells_per_layer;   ///< 每时间层的非零栅格数 [layer_0, ... , layer_t-1]
};

/// @class SscMap
/// @brief 三维时空占据栅格地图 —— SSC 规划器的环境表示层
///
/// 该地图在 (s, d, t) 三维空间中对障碍物进行建模和膨胀，其核心目的
/// 是为后续的 QP 轨迹优化提供安全的空间约束边界。
///
/// 地图维护两套栅格：
///   - p_3d_grid_: 原始障碍物占据栅格
///   - p_3d_inflated_grid_: 膨胀后的障碍物占据栅格
///
/// 走廊构建流程：
///   1. 将所有障碍物（静态栅格 + 动态车辆轨迹）写入 p_3d_grid_
///   2. 沿前向轨迹均匀采样作为种子点（seed）
///   3. 从每对相邻种子点生成初始轴对齐立方体
///   4. 在六方向上膨胀立方体直到接触障碍物或达到运动学约束边界
///   5. 将膨胀后的立方体序列按时间轴裁剪并输出为物理坐标的走廊
class SscMap {
 public:
  using GridMap3D = common::GridMapND<ObstacleMapType, 3>;  ///< 三维栅格地图类型

  /// @struct Config
  /// @brief SscMap 的全部配置参数
  ///
  /// 配置通过 protobuf 文件加载，关键参数分类：
  ///   - 地图参数: map_size (s,d,t轴尺寸), map_resolution (分辨率)
  ///   - 运动学约束: kMaxLongitudinalVel, kMaxLateralAcc 等 (限制膨胀范围)
  ///   - 膨胀步长: inflate_steps (六方向每次膨胀的栅格数)
  struct Config {
    std::array<int, 3> map_size = {{1000, 100, 81}};               ///< 地图尺寸 [s方向, d方向, t方向]
    std::array<decimal_t, 3> map_resolution = {{0.25, 0.2, 0.1}};  ///< 地图分辨率 [s方向, d方向, t方向] (米, 米, 秒)
    std::array<std::string, 3> axis_name = {{"s", "d", "t"}};      ///< 轴名称
    decimal_t s_back_len = 0.0;                                     ///< s 方向向后预留长度

    // --- 运动学约束 (用于限制走廊膨胀的物理范围) ---
    decimal_t kMaxLongitudinalVel = 50.0;       ///< 最大纵向速度 (m/s)
    decimal_t kMinLongitudinalVel = 0.0;         ///< 最小纵向速度 (m/s)
    decimal_t kMaxLongitudinalAcc = 3.0;         ///< 最大纵向加速度 (m/s^2)
    decimal_t kMaxLongitudinalDecel = -8.0;      ///< 最大纵向减速度 (m/s^2, 驾驶员平均水平)
    decimal_t kMaxLateralVel = 3.0;              ///< 最大横向速度 (m/s)
    decimal_t kMaxLateralAcc = 2.5;              ///< 最大横向加速度 (m/s^2)

    int kMaxNumOfGridAlongTime = 2;              ///< 时间轴最大膨胀栅格数

    std::array<int, 6> inflate_steps = {{20, 5, 10, 10, 1, 1}};  ///< 六方向膨胀步长 [x+, x-, y+, y-, z+, z-]

    /// @brief 打印所有配置参数到控制台
    void Print() {
      printf("\nSscMap Config:\n");
      printf(" -- map_size: [%d, %d, %d]\n", map_size[0], map_size[1],
             map_size[2]);
      printf(" -- map_resolution: [%lf, %lf, %lf]\n", map_resolution[0],
             map_resolution[1], map_resolution[2]);
      printf(" -- axis_name: [%s, %s, %s]\n", axis_name[0].c_str(),
             axis_name[1].c_str(), axis_name[2].c_str());
      printf(" -- s_back_len: %lf\n", s_back_len);
      printf(" -- kMaxLongitudinalVel: %lf\n", kMaxLongitudinalVel);
      printf(" -- kMinLongitudinalVel: %lf\n", kMinLongitudinalVel);
      printf(" -- kMaxLongitudinalAcc: %lf\n", kMaxLongitudinalAcc);
      printf(" -- kMaxLongitudinalDecel: %lf\n", kMaxLongitudinalDecel);
      printf(" -- kMaxLateralVel: %lf\n", kMaxLateralVel);
      printf(" -- kMaxLateralAcc: %lf\n", kMaxLateralAcc);
      printf(" -- kMaxNumOfGridAlongTime: %d\n", kMaxNumOfGridAlongTime);
      printf(" -- inflate_steps: [%d, %d, %d, %d, %d, %d]\n", inflate_steps[0],
             inflate_steps[1], inflate_steps[2], inflate_steps[3],
             inflate_steps[4], inflate_steps[5]);
    }
  };

  SscMap() {}
  /// @brief 用配置参数构造 SSC 地图并分配三维栅格内存
  SscMap(const Config &config);
  ~SscMap() {}

  /// @brief 获取原始障碍物占据栅格（3D）
  GridMap3D *p_3d_grid() const { return p_3d_grid_; }
  /// @brief 获取膨胀后的障碍物占据栅格（3D）
  GridMap3D *p_3d_inflated_grid() const { return p_3d_inflated_grid_; }

  /// @brief 获取风险占据栅格图的只读引用（调试/验证用）
  /// @return 三维风险占据图的 const 引用，值为风险概率 [0.0, 1.0]
  /// @note MVP-0 中风险图不参与 corridor/QP/行为选择，仅供外部调试读取
  const RiskGridMap3D& risk_grid() const { return p_3d_risk_grid_; }

  /// @brief 计算风险占据栅格的统计信息（只读，不修改任何地图数据）
  /// @return RiskGridStats 结构体，包含 total/ nonzero/ max/ sum/ active_layers/ per_layer
  /// @note MVP-1A: 用于验证 risk grid 是否被正确 reset 和填充，不参与规划决策
  RiskGridStats ComputeRiskGridStats() const;

  /// @brief 获取配置
  Config config() const { return config_; }

  /// @brief 获取走廊向量（栅格坐标下的 DrivingCorridor 列表）
  vec_E<common::DrivingCorridor> driving_corridor_vec() const {
    return driving_corridor_vec_;
  }
  /// @brief 获取最终物理坐标下的时空走廊
  vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> final_corridor_vec()
      const {
    return final_corridor_vec_;
  };

  /// @brief 获取各走廊有效性标志 (1=有效, 0=无效)
  std::vector<int> if_corridor_valid() const { return if_corridor_valid_; }

  /// @brief 设置规划起始时间戳
  void set_start_time(const decimal_t &t) { start_time_ = t; }
  /// @brief 设置初始 Frenet 状态
  void set_initial_fs(const common::FrenetState &fs) { initial_fs_ = fs; }

  /// @brief 更新地图原点（以当前 Frenet 状态为参考）
  /// 纵向上从当前 s 位置减去 s_back_len 作为地图原点
  /// 横向上居中，时间轴以当前时间戳为原点
  void UpdateMapOrigin(const common::FrenetState &ori_fs);

  /// @brief 构建 SSC 时空占据地图
  ///
  /// 分两步：
  ///   1. FillStaticPart:  将静态障碍物栅格填充到所有时间层
  ///   2. FillDynamicPart: 将动态车辆预测轨迹逐帧填充（使用多边形填充）
  ///
  /// @param sur_vehicle_trajs_fs 周围车辆在 Frenet 坐标下的预测轨迹
  /// @param obstacle_grids       静态障碍物栅格的 Frenet 坐标列表
  /// @param traj_probs           周车轨迹存在概率表，key=车辆ID
  /// @return 错误码
  ErrorType ConstructSscMap(
      const std::unordered_map<int, vec_E<common::FsVehicle>>
          &sur_vehicle_trajs_fs,
      const vec_E<Vec2f> &obstacle_grids,
      const std::unordered_map<int, decimal_t> &traj_probs);

  /// @brief 对障碍物栅格进行车辆尺寸膨胀
  /// 根据车辆参数（长、宽、后轴到车尾距离）计算膨胀量并填充 p_3d_inflated_grid_
  ErrorType InflateObstacleGrid(const common::VehicleParam &param);

  /// @brief 沿初始参考轨迹构建时空走廊
  ///
  /// 核心算法（两阶段）：
  ///   Stage I  - 种子点采样:   沿前向轨迹的状态点采样种子
  ///   Stage II - 立方体膨胀:   对每对种子生成的初始立方体进行六方向膨胀
  ///
  /// @param p_grid 3D 栅格地图
  /// @param trajs  Frenet 坐标下的前向车辆轨迹
  /// @return 错误码，种子点少于 2 时失败
  ErrorType ConstructCorridorUsingInitialTrajectory(
      GridMap3D *p_grid, const vec_E<common::FsVehicle> &trajs);

  /// @brief 清空三维栅格地图数据
  ErrorType ClearGridMap();

  /// @brief 清空走廊数据
  ErrorType ClearDrivingCorridor();

  /// @brief 将栅格坐标下的走廊转换为物理坐标 (SpationTemporalSemanticCube)
  /// 填充 velocity 和 acceleration 边界（源自 config 中的运动学约束）
  ErrorType GetFinalGlobalMetricCubesList();

  /// @brief 重置 SSC 地图：清空走廊、清空栅格、更新原点
  /// @param ini_frenet_state 当前的初始 Frenet 状态
  ErrorType ResetSscMap(const common::FrenetState &ini_frenet_state);

  /// @brief 重置风险占据图 —— 将所有栅格值清零为 0.0f
  /// @return kSuccess
  /// @note 仅清空风险图，不影响原始 binary 占据图
  ErrorType ResetRiskMap();

 private:
  /// @brief 按需输出风险占据栅格统计日志
  /// @param stats 由 ComputeRiskGridStats() 计算得到的统计结构
  /// @note MVP-1A: 使用 LOG(WARNING) 输出，前缀 [Ssc][RiskGridStats]。
  ///       输出 summary 行（total/nonzero/max/sum/active_layers）和 per-layer 行。
  ///       该函数不触发任何规划逻辑，不影响 corridor/QP/control
  void PrintRiskGridStatsIfNeeded(const RiskGridStats &stats) const;

  /// @brief 将风险占据栅格统计结果追加写入 CSV 文件
  /// @param stats 由 ComputeRiskGridStats() 计算得到的统计结构
  /// @note MVP-1B: 仅用于论文实验数据记录。该函数只读取统计结果并写入
  ///       /tmp/epsilon_risk_grid_stats.csv，不修改地图、不参与 corridor/QP/control。
  void AppendRiskGridStatsToCsv(const RiskGridStats &stats) const;

  /// @brief 检查立方体在 3D 栅格中是否完全无障碍
  /// 遍历立方体内所有栅格单元格，确认均为 0（空闲）
  bool CheckIfCubeIsFree(GridMap3D *p_grid,
                         const common::AxisAlignedCubeNd<int, 3> &cube) const;

  /// @brief 检查 X 轴（s方向）上的平面（固定 x 坐标）是否完全空闲
  /// 遍历固定 x 坐标下 (d, t) 平面上的所有单元格
  bool CheckIfPlaneIsFreeOnXAxis(GridMap3D *p_grid,
                                 const common::AxisAlignedCubeNd<int, 3> &cube,
                                 const int &x) const;

  /// @brief 检查 Y 轴（d方向）上的平面（固定 y 坐标）是否完全空闲
  bool CheckIfPlaneIsFreeOnYAxis(GridMap3D *p_grid,
                                 const common::AxisAlignedCubeNd<int, 3> &cube,
                                 const int &y) const;

  /// @brief 检查 Z 轴（t方向）上的平面（固定 z 坐标）是否完全空闲
  bool CheckIfPlaneIsFreeOnZAxis(GridMap3D *p_grid,
                                 const common::AxisAlignedCubeNd<int, 3> &cube,
                                 const int &z) const;

  /// @brief 检查种子点是否在已膨胀的立方体内部
  /// 在三个维度上分别验证 lower_bound <= seed <= upper_bound
  bool CheckIfCubeContainsSeed(const common::AxisAlignedCubeNd<int, 3> &cube_a,
                               const Vec3i &seed) const;

  /// @brief 根据两个种子点生成初始轴对齐立方体
  /// 初始立方体的边界由两个种子在各维度上的 min/max 决定
  ErrorType GetInitialCubeUsingSeed(
      const Vec3i &seed_0, const Vec3i &seed_1,
      common::AxisAlignedCubeNd<int, 3> *cube) const;

  /// @brief 获取沿时间轴方向（向前/向后）可达的立方体索引
  /// @param p_corridor 走廊对象
  /// @param start_id   起始立方体索引
  /// @param dir        方向: 1=向前(时间增大), 0=向后(时间减小)
  /// @param t_trans    目标时间跨度
  /// @param idx_list   输出: 可达的立方体索引列表
  ErrorType GetTimeCoveredCubeIndices(const common::DrivingCorridor *p_corridor,
                                      const int &start_id, const int &dir,
                                      const int &t_trans,
                                      std::vector<int> *idx_list) const;

  /// @brief 走廊松弛：在连续立方体连接处扩大 s 和 d 方向的连接空间
  /// 当相邻立方体在 s 或 d 方向的间隙小于阈值时，尝试双向膨胀以平滑连接
  ErrorType CorridorRelaxation(GridMap3D *p_grid,
                               common::DrivingCorridor *p_corridor);

  /// @brief 三维栅格中的立方体六方向膨胀主循环
  ///
  /// 算法：
  ///   - 在 s+, s-, d+, d- 四方向上交替尝试膨胀
  ///   - 膨胀受运动学边界限制（基于最大加速度的外推 s 上/下界）
  ///   - t+ 方向单独膨胀，受 kMaxNumOfGridAlongTime 限制
  ///   - 不向 t- 方向膨胀（禁止时间倒流）
  ///
  /// @param p_grid       3D 栅格
  /// @param dir_disabled  六方向禁用标记
  /// @param dir_step     六方向膨胀步长
  /// @param cube         待膨胀的立方体 (in/out)
  ErrorType InflateCubeIn3dGrid(GridMap3D *p_grid,
                                const std::array<bool, 6> &dir_disabled,
                                const std::array<int, 6> &dir_step,
                                common::AxisAlignedCubeNd<int, 3> *cube);

  /// @brief 获取膨胀方向配置：除第一个立方体禁止 t- 膨胀外，其他方向均允许
  ErrorType GetInflationDirections(const bool &if_first_cube,
                                   std::array<bool, 6> *dirs_disabled);

  // --- 六方向单步膨胀函数 ---
  /// @brief 向 X 轴正方向（s增大的方向）膨胀 n_step 个栅格
  bool InflateCubeOnXPosAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  /// @brief 向 X 轴负方向（s减小的方向）膨胀 n_step 个栅格
  bool InflateCubeOnXNegAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  /// @brief 向 Y 轴正方向（d增大的方向）膨胀 n_step 个栅格
  bool InflateCubeOnYPosAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  /// @brief 向 Y 轴负方向（d减小的方向）膨胀 n_step 个栅格
  bool InflateCubeOnYNegAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  /// @brief 向 Z 轴正方向（时间增大的方向）膨胀 n_step 个栅格
  bool InflateCubeOnZPosAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  /// @brief 向 Z 轴负方向（时间减小的方向）膨胀 n_step 个栅格
  bool InflateCubeOnZNegAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);

  /// @brief 填充静态障碍物：将静态栅格点在该 s,d 坐标下所有时间层全部标记为占用
  /// @param obs_grid_fs 障碍物栅格的 Frenet 坐标 (s, d)
  ErrorType FillStaticPart(const vec_E<Vec2f> &obs_grid_fs);

  /// @brief 填充动态障碍物：将所有周围车辆的预测轨迹逐帧填充到 3D 栅格
  ErrorType FillDynamicPart(
      const std::unordered_map<int, vec_E<common::FsVehicle>>
          &sur_vehicle_trajs_fs);

  /// @brief 概率化填充动态障碍物 —— 遍历所有周围车辆并将其预测轨迹写入风险占据图
  /// @param sur_vehicle_trajs_fs 周围车辆在 Frenet 坐标系下的预测轨迹
  ///                              (key=车辆ID, value=Frenet车辆状态序列)
  /// @param traj_probs           周车轨迹存在概率表，缺失时回退到 1.0
  /// @return kSuccess
  /// @note 与 FillDynamicPart 的区别：写入 p_3d_risk_grid_（float概率）而非
  ///       p_3d_grid_（uint8_t二值占据）。MVP-2 只影响 risk grid。
  ErrorType FillDynamicPartProbabilistic(
      const std::unordered_map<int, vec_E<common::FsVehicle>>
          &sur_vehicle_trajs_fs,
      const std::unordered_map<int, decimal_t> &traj_probs);

  /// @brief 将单条 Frenet 车辆轨迹填充到 3D 栅格
  ///
  /// 使用策略：对轨迹中每帧的车辆轮廓顶点，在对应的 s-d 平面对应的 t 层上，
  /// 使用 OpenCV 的 fillPoly 填充多边形区域（实现精确的车辆形状占据）
  ErrorType FillMapWithFsVehicleTraj(const vec_E<common::FsVehicle> traj);

  /// @brief 概率化填充单条车辆 Frenet 轨迹到风险占据图
  /// @param traj 单辆周围车辆在 Frenet 坐标系下的完整预测轨迹
  /// @param existence_prob 该条确定性轨迹写入 risk grid 的存在概率
  /// @return kSuccess 填充成功 / kWrongStatus 轨迹为空
  /// @note 复用原始 FillMapWithFsVehicleTraj 的几何流程（坐标转换、范围检查、
  ///       OpenCV fillPoly），但写入目标为 p_3d_risk_grid_（CV_32FC1），
  ///       填充值为 existence_prob（MVP-2 来自行车 argmax 行为概率）
  ErrorType FillMapWithFsVehicleTrajProbabilistic(
      const vec_E<common::FsVehicle> traj, const float existence_prob);

  // =========================================================================
  // 成员变量
  // =========================================================================

  /// 原始 3D 占据栅格地图
  common::GridMapND<SscMapDataType, 3> *p_3d_grid_;
  /// 膨胀后的 3D 占据栅格地图
  common::GridMapND<SscMapDataType, 3> *p_3d_inflated_grid_;

  /// @brief 三维风险占据栅格图 —— 浮点概率值 [0.0, 1.0]
  /// @note MVP-0 定位: side-channel 调试数据，不参与 corridor 构建和 QP 优化。
  ///       后续 MVP-1 将接入真实概率预测来源（MOBIL 概率/EUDM 不确定性），
  ///       MVP-2 将作为 risk cost 或 chance constraint 的输入
  RiskGridMap3D p_3d_risk_grid_;

  /// @brief 是否启用 RiskGridStats CSV 导出
  /// @note MVP-1B 默认启用，便于直接获得论文实验统计数据；关闭该开关时
  ///       AppendRiskGridStatsToCsv() 会立即返回，不影响规划流程。
  bool risk_stats_csv_enabled_ = true;

  /// @brief RiskGridStats CSV 默认输出路径
  /// @note 使用 /tmp 目录避免要求用户提前配置输出目录；后续 MVP 可再接入配置文件。
  std::string risk_stats_csv_path_ = "/tmp/epsilon_risk_grid_stats.csv";

  /// @brief CSV 表头是否已经在本进程中处理过
  /// @note mutable 允许 const 统计输出函数记录 IO 状态，不改变地图或规划语义。
  mutable bool risk_stats_csv_header_written_ = false;

  /// @brief CSV 导出的规划周期计数器
  /// @note 每次 ConstructSscMap() 完成风险统计后递增，用于关联日志和 CSV 行。
  mutable size_t risk_stats_cycle_count_ = 0;

  /// 立方体膨胀方向的禁用记录（暂未激活使用）
  std::unordered_map<int, std::array<bool, 6>> inters_for_cube_;

  /// 配置参数
  Config config_;

  /// 规划起始时间戳
  decimal_t start_time_;

  /// 初始 Frenet 状态（用于计算地图原点和膨胀边界）
  common::FrenetState initial_fs_;

  /// 地图有效性标志
  bool map_valid_ = false;

  /// 走廊向量（栅格坐标表示）
  vec_E<common::DrivingCorridor> driving_corridor_vec_;

  /// 各走廊有效性标志 (1=有效, 0=无效)
  std::vector<int> if_corridor_valid_;
  /// 最终物理坐标下的时空走廊列表 [behavior_idx][cube_idx]
  vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> final_corridor_vec_;
};

}  // namespace planning
#endif  // _UTIL_SSC_PLANNER_INC_SSC_MAP_H_
