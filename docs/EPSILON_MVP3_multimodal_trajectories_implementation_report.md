# MVP-3 实施报告

## 1. 目标

为周围车辆生成 LK / LCL / LCR 多模态预测轨迹，并将其作为 SSC risk grid 的概率化输入。

本 MVP 只让 risk grid 消费多模态轨迹，不改变原始 binary SSC map、corridor、QP、control 或轨迹选择。

## 2. 修改范围

- `src/EPSILON/util/ssc_planner/include/ssc_planner/map_interface.h`
- `src/EPSILON/util/ssc_planner/include/ssc_planner/map_adapter.h`
- `src/EPSILON/util/ssc_planner/src/ssc_planner/map_adapter.cc`
- `src/EPSILON/util/ssc_planner/include/ssc_planner/ssc_planner.h`
- `src/EPSILON/util/ssc_planner/src/ssc_planner/ssc_planner.cc`
- `src/EPSILON/util/ssc_planner/include/ssc_planner/ssc_map.h`
- `src/EPSILON/util/ssc_planner/src/ssc_planner/ssc_map.cc`

## 3. 新增结构体 / 函数

新增结构体 / 类型：

- `SurroundingVehicleTrajectoryMode`
- `MultiModalSurroundingTrajectories`
- `SurroundingVehicleFsTrajectoryMode`
- `MultiModalSurroundingFsTrajectories`

新增接口：

- `SscPlannerMapItf::GetMultiModalSurroundingTrajectories()`
- `SscPlannerAdapter::GetMultiModalSurroundingTrajectories()`

新增成员：

- `SscPlanner::multimodal_surround_trajs_`
- `SscPlanner::multimodal_surround_trajs_fs_`

新增函数重载：

- `SscMap::FillDynamicPartProbabilistic(const MultiModalSurroundingFsTrajectories &multimodal_trajs_fs)`

## 4. 调用链

多模态生成：

```text
SscPlannerAdapter::GetMultiModalSurroundingTrajectories()
  -> SemanticMapManager::semantic_surrounding_vehicles()
  -> SemanticVehicle.probs_lat_behaviors
  -> for each behavior in {LK, LCL, LCR}
  -> GetRefLaneForStateByBehavior()
  -> TrajectoryPredictionForVehicle()
  -> SurroundingVehicleTrajectoryMode
```

Frenet 转换：

```text
SscPlanner::RunOnce()
  -> GetMultiModalSurroundingTrajectories()
  -> StateTransformForInputData()
  -> multimodal_surround_trajs_fs_
```

risk grid 填图：

```text
SscMap::ConstructSscMap()
  -> FillStaticPart()
  -> FillDynamicPart()                  // binary map: 仍使用确定性周车轨迹
  -> FillDynamicPartProbabilistic()      // risk grid: 优先使用多模态轨迹
  -> FillMapWithFsVehicleTrajProbabilistic(traj, probability)
```

## 5. 不改动项

- 不修改 `core/common`
- 不修改 `SemanticBehavior`
- 不修改 `common::FsVehicle`
- 不修改 EUDM / MPDM / behavior planner 主循环
- 不修改 binary SSC map 的动态障碍物填充
- 不修改 corridor 构建
- 不修改 QP / control / trajectory selection
- 不修改 CSV / RViz topic 结构

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

本 MVP 未启动仿真或 RViz 截图验证。

预期运行态变化：

- `/tmp/epsilon_risk_grid_stats.csv` 会记录多模态风险图统计结果；
- `/vis/agent_{node_id}/ssc/risk_grid_vis` 会显示多模态风险体素；
- binary SSC map 仍与 baseline 一致；
- corridor/QP/control 轨迹输出不因本 MVP 改变。

## 8. Git commit 信息

推荐 commit message：

```text
feat(ssc): generate multimodal surrounding risk trajectories
```

## 9. tag 信息

构建确认后，建议 tag：

```text
v0.3.0-mvp3-multimodal-trajectories
```

## 10. 当前风险

- 当前 `SemanticMapManager` 上游概率多为 Naive 规则硬概率，多模态数量可能退化为单模态。
- LCL/LCR 参考车道不可达时会跳过该模态，不会强行 fallback。
- `TrajectoryPredictionForVehicle()` 使用开环自由流预测，不包含周车之间交互约束。
- risk grid 重叠区域仍由 `cv::fillPoly` 覆盖写入，尚未实现概率累加或 `max` 融合；这留给 MVP-4。

## 11. 下一步计划

1. 合并 MVP-3 并打 `v0.3.0-mvp3-multimodal-trajectories`。
2. MVP-4 实现真正的概率风险图完整填充与融合逻辑。
3. 在仿真/RViz 中验证多模态风险体素是否随周车概率分布变化。
