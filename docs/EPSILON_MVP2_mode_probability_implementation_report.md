# MVP-2 实施报告

## 1. 目标

将周围车辆已有横向行为概率接入 SSC risk grid，使风险填图不再固定使用 `existence_prob = 1.0f`。

本 MVP 只对当前单条确定性周车预测轨迹进行概率加权，不生成多模态轨迹，不影响 corridor/QP/control。

## 2. 修改范围

- `src/EPSILON/util/ssc_planner/include/ssc_planner/map_interface.h`
- `src/EPSILON/util/ssc_planner/include/ssc_planner/map_adapter.h`
- `src/EPSILON/util/ssc_planner/src/ssc_planner/map_adapter.cc`
- `src/EPSILON/util/ssc_planner/include/ssc_planner/ssc_planner.h`
- `src/EPSILON/util/ssc_planner/src/ssc_planner/ssc_planner.cc`
- `src/EPSILON/util/ssc_planner/include/ssc_planner/ssc_map.h`
- `src/EPSILON/util/ssc_planner/src/ssc_planner/ssc_map.cc`

## 3. 新增结构体 / 函数

未新增结构体。

新增接口：

- `SscPlannerMapItf::GetSurroundingTrajectoryExistenceProbabilities()`
- `SscPlannerAdapter::GetSurroundingTrajectoryExistenceProbabilities()`

新增成员：

- `SscPlanner::surround_traj_existence_probs_`

修改函数签名：

- `SscMap::ConstructSscMap(..., const std::unordered_map<int, decimal_t> &traj_probs)`
- `SscMap::FillDynamicPartProbabilistic(..., const std::unordered_map<int, decimal_t> &traj_probs)`
- `SscMap::FillMapWithFsVehicleTrajProbabilistic(..., const float existence_prob)`

## 4. 调用链

概率来源：

```text
SemanticMapManager::semantic_surrounding_vehicles()
  -> SemanticVehicle.probs_lat_behaviors
  -> SemanticVehicle.lat_behavior
  -> probability = probs_lat_behaviors.probs[lat_behavior]
```

SSC 传递链：

```text
SscPlanner::RunOnce()
  -> map_itf_->GetSurroundingTrajectoryExistenceProbabilities()
  -> surround_traj_existence_probs_
  -> SscMap::ConstructSscMap(..., surround_traj_existence_probs_)
  -> FillDynamicPartProbabilistic()
  -> FillMapWithFsVehicleTrajProbabilistic(traj, existence_prob)
  -> cv::fillPoly(..., cv::Scalar(existence_prob))
```

缺省策略：

```text
probability missing / invalid / undefined behavior
  -> fallback existence_prob = 1.0
```

## 5. 不改动项

- 不修改 `SemanticBehavior`
- 不修改 `common::FsVehicle`
- 不修改 EUDM / MPDM / behavior planner
- 不修改多模态轨迹生成
- 不修改 binary SSC map `p_3d_grid_`
- 不修改 corridor 构建
- 不修改 QP / control / trajectory selection
- 不修改 risk grid CSV / RViz 结构

## 6. 编译结果

已执行：

```bash
cd /home/ros/work/graduateworkcc
colcon build --packages-up-to ssc_planner --symlink-install
```

结果：

- `common` 编译通过；
- `ssc_planner` 编译通过；
- 总计 7 个 package finished；
- 编译无 error；
- 仍有既有 warning：`ssc_planner.cc` 中 signed/unsigned 比较和未用变量、`ssc_server_ros.cc` 中 `wheel_base` 未使用、第三方 `tk_spline` 未用函数 warning，本 MVP 未修改这些无关项。

## 7. 运行 / 日志 / CSV / RViz 结果

本 MVP 没有启动仿真或 RViz 实机截图验证。

预期运行态变化：

- `/tmp/epsilon_risk_grid_stats.csv` 仍按 MVP-1B 字段输出；
- `sum_risk` 和 `max_risk` 会随 `existence_prob` 变化；
- MVP-1C 的 `/vis/agent_{node_id}/ssc/risk_grid_vis` 会显示概率加权后的 risk grid；
- `p_3d_grid_` 仍由原始确定性动态障碍物填充，不受概率影响。

## 8. Git commit 信息

推荐 commit message：

```text
feat(ssc): weight risk grid by behavior probability
```

## 9. tag 信息

构建确认后，建议 tag：

```text
v0.2.0-mvp2-mode-probability
```

## 10. 当前风险

- 当前 `SemanticMapManager::UpdateSemanticVehicles()` 默认使用 Naive 规则预测，输出常为 0/1 硬概率；MVP-2 接通了概率链路，但上游概率是否足够连续取决于后续是否启用 MOBIL 或其他预测器。
- 当前周车仍只有一条确定性预测轨迹，不是 LK/LCL/LCR 多模态轨迹；本 MVP 只能给这条轨迹加权。
- 多车风险重叠区域仍由 `cv::fillPoly` 后写覆盖，不做 `max`、概率并集或累加。
- 若语义车辆概率缺失，系统保守回退 `1.0`，风险不会被低估，但 CSV 可能接近旧版本结果。

## 11. 下一步计划

1. 将 MVP-2 合并回 `dev/risk-aware-ssc` 并打 `v0.2.0-mvp2-mode-probability`。
2. MVP-3 开始多模态周车轨迹生成，避免继续只给单条轨迹加权。
3. MVP-4 再实现真正的 `Risk(s,d,t)=sum p_i * Occupancy_i(s,d,t)` 概率叠加。
