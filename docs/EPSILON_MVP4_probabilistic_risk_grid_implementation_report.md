# MVP-4 实施报告

## 1. 目标

MVP-4 的目标是将风险占据图从“后写覆盖”升级为概率叠加语义：

```text
Risk(s,d,t) = min(1.0, sum_i p_i * Occupancy_i(s,d,t))
```

其中 `p_i` 是周车确定性轨迹或多模态轨迹的存在概率，`Occupancy_i` 是该轨迹在某个时空栅格内的 0/1 占据。

## 2. 修改范围

本次代码修改只涉及：

```text
util/ssc_planner/include/ssc_planner/ssc_map.h
util/ssc_planner/src/ssc_planner/ssc_map.cc
```

未修改 EUDM、MPDM、SemanticBehavior、corridor、QP、control、RViz topic 和 CSV 字段。

## 3. 新增结构体 / 函数

本 MVP 未新增结构体，未新增公开函数。

核心变化是更新已有函数：

```text
SscMap::FillMapWithFsVehicleTrajProbabilistic()
```

原逻辑：

```text
cv::fillPoly(layer_mat, polygon, existence_prob)
```

会直接覆盖旧风险值。

新逻辑：

```text
1. 对单条轨迹在每个时间层内生成 0/1 occupancy mask；
2. 将 mask 转成 delta risk = mask * existence_prob；
3. 将 delta risk 累加到 p_3d_risk_grid_；
4. 使用 THRESH_TRUNC 将风险值截断到 1.0。
```

## 4. 调用链

```text
SscPlanner::RunOnce()
  -> SscMap::ConstructSscMap()
    -> FillDynamicPart()
       写入原始 binary p_3d_grid_
    -> FillDynamicPartProbabilistic()
       写入 p_3d_risk_grid_
    -> FillMapWithFsVehicleTrajProbabilistic()
       执行概率叠加与 1.0 截断
    -> ComputeRiskGridStats()
    -> PrintRiskGridStatsIfNeeded()
    -> AppendRiskGridStatsToCsv()
```

## 5. 不改动项

```text
1. 不改变原始 p_3d_grid_ 二值占据图；
2. 不改变 corridor 构建输入；
3. 不改变 QP 代价或约束；
4. 不改变控制输出；
5. 不改变 MVP-1B CSV 字段；
6. 不改变 MVP-1C RViz topic 和颜色映射；
7. 不新增参数开关，保持最小 MVP 范围。
```

## 6. 编译结果

执行命令：

```bash
cd /home/ros/work/graduateworkcc
colcon build --packages-up-to ssc_planner --symlink-install
```

结果：

```text
Summary: 7 packages finished
```

编译通过。仍存在既有 warning，例如 `DrivingCorridor::id may be used uninitialized`，本 MVP 未修改该路径。

## 7. 运行 / 日志 / CSV / RViz 结果

本次完成编译验证，未启动完整 ROS 场景运行。

预期观察点：

```text
1. max_risk 不应超过 1.0；
2. sum_risk 在多车或多模态重叠时应高于覆盖写版本；
3. nonzero_cells 未必增加，因为同一批栅格内的风险叠加不会改变非零栅格数量；
4. active_time_layers 通常保持一致；
5. CSV /tmp/epsilon_risk_grid_stats.csv 会自然记录新的统计值；
6. RViz risk_grid_vis 会自然显示更高风险的颜色和透明度。
```

## 8. Git commit 信息

推荐 commit message：

```text
feat(ssc): accumulate probabilistic risk grid occupancy
```

## 9. tag 信息

建议合并回 `dev/risk-aware-ssc` 后打 tag：

```text
v0.4.0-mvp4-probabilistic-risk-grid
```

## 10. 当前风险

```text
1. 当前静态障碍物仍只写入 binary map，不写入 risk grid；
2. 多模态概率归一性依赖上游输入，本 MVP 仅做单 cell 截断；
3. 每条轨迹会为涉及的时间层构造临时 mask，运行时开销略高于覆盖写；
4. risk grid 仍不参与 corridor/QP/control，这是当前 MVP 的刻意边界。
```

## 11. 下一步计划

下一阶段 MVP-5 可在保持 `lambda_risk = 0` 或高阈值退化为 baseline 的前提下，将：

```text
risk(s,d,t) > threshold
```

转换为 high-risk occupied 区域，使 corridor 避开高风险区域。
