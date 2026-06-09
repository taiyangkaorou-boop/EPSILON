# MVP-1B 实施报告

## 1. 目标

将 `RiskGridStats` 追加导出到 CSV 文件，便于论文实验阶段离线统计和画图。

## 2. 修改范围

- `src/EPSILON/util/ssc_planner/include/ssc_planner/ssc_map.h`
- `src/EPSILON/util/ssc_planner/src/ssc_planner/ssc_map.cc`

## 3. 新增结构体 / 函数

未新增结构体。

新增函数：

- `SscMap::AppendRiskGridStatsToCsv(const RiskGridStats &stats)`

新增成员：

- `risk_stats_csv_enabled_`
- `risk_stats_csv_path_`
- `risk_stats_csv_header_written_`
- `risk_stats_cycle_count_`

## 4. 调用链

`SscMap::ConstructSscMap()`

1. `FillStaticPart()`
2. `FillDynamicPart()`
3. `FillDynamicPartProbabilistic()`
4. `ComputeRiskGridStats()`
5. `PrintRiskGridStatsIfNeeded()`
6. `AppendRiskGridStatsToCsv()`

CSV 默认路径：

```text
/tmp/epsilon_risk_grid_stats.csv
```

CSV 字段：

```csv
cycle,total_cells,nonzero_cells,max_risk,sum_risk,active_time_layers,total_time_layers
```

## 5. 不改动项

- 不修改 `ssc_planner.cc`
- 不修改 corridor 构建逻辑
- 不修改 QP / control
- 不修改 EUDM / MPDM / SemanticBehavior
- 不修改 CMakeLists.txt / package.xml
- 不接入真实行为概率
- 不做 RViz 可视化

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
- 首次完整构建中 `ssc_planner` 输出既有 warning，但无 error；
- 最终增量复验构建再次通过，7 个 package finished，未再输出 warning。

## 7. 运行 / 日志 / CSV / RViz 结果

CSV 导出为规划调试旁路：

- 文件不可写时只输出 `WARNING`，不阻断规划主流程；
- header 只在目标文件为空或首次生成时写入；
- 每次 `ConstructSscMap()` 追加一行统计数据；
- 本 MVP 不包含 RViz 可视化。

运行态 CSV 生成验证：

- 已通过 `/tmp/ssc_risk_csv_smoke` 临时程序触发一次 `SscMap::ConstructSscMap()`；
- 已生成 `/tmp/epsilon_risk_grid_stats.csv`；
- CSV 共 2 行：header + 1 行统计数据；
- 验证得到首行数据：

```csv
cycle,total_cells,nonzero_cells,max_risk,sum_risk,active_time_layers,total_time_layers
0,24,0,0,0,0,2
```

## 8. Git commit 信息

推荐 commit message：

```text
feat(ssc): export risk grid statistics to csv
```

## 9. tag 信息

构建和运行确认后，建议 tag：

```text
v0.1.2-mvp1b-risk-stats-csv
```

不要在未完成编译和运行确认前打 tag。

## 10. 当前风险

- 当前 CSV 周期计数是 `SscMap` 实例内计数，进程重启后从 0 重新开始。
- 默认路径位于 `/tmp`，系统重启或清理临时目录后文件会消失。
- 当前 risk grid 仍使用 MVP-0 的固定 `existence_prob = 1.0f`，CSV 记录的是确定性风险镜像统计，不是真实多模态概率风险。

## 11. 下一步计划

1. 在真实 planning launch 中确认 CSV 行随 planning cycle 连续追加。
2. MVP-1C 再做 RViz 可视化，不在本次提交混入。
3. MVP-2/MVP-3 再接入真实行为概率和多模态轨迹，不在本次提交混入。
