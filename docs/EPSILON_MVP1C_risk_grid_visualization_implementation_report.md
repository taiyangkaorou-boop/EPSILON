# MVP-1C 实施报告

## 1. 目标

将 `SscMap::risk_grid()` 以 RViz `MarkerArray` 形式可视化，便于论文实验截图和风险图调试。

## 2. 修改范围

- `src/EPSILON/util/ssc_planner/include/ssc_planner/ssc_visualizer.h`
- `src/EPSILON/util/ssc_planner/src/ssc_planner/ssc_visualizer.cc`

## 3. 新增结构体 / 函数

未新增结构体。

新增函数：

- `SscVisualizer::VisualizeRiskGridInSscSpace(const rclcpp::Time &stamp, const SscMap *p_ssc_map)`

新增成员：

- `risk_grid_pub_`
- `last_risk_grid_mk_cnt`

新增 RViz topic：

```text
/vis/agent_{node_id}/ssc/risk_grid_vis
```

## 4. 调用链

`SscPlannerServer::PublishData()`

1. `SscVisualizer::VisualizeDataWithStamp()`
2. `VisualizeSscMap()`
3. `VisualizeRiskGridInSscSpace()`
4. 其他既有可视化函数

风险栅格索引解释：

```text
idx = t_idx * s_dim * d_dim + d_idx * s_dim + s_idx
```

RViz 坐标解释：

```text
X = s
Y = d
Z = t - start_time_
```

## 5. 不改动项

- 不修改 `SscMap` 填图逻辑
- 不修改 corridor 构建逻辑
- 不修改 QP / control
- 不修改 EUDM / MPDM / SemanticBehavior
- 不修改 `ssc_server_ros.cc`
- 不新增 ROS message 或 package 依赖
- 不接入真实行为概率

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
- 仍有既有 warning：`ssc_server_ros.cc` 中 `wheel_base` 未使用，以及第三方 `tk_spline` 未用函数 warning，本 MVP 未修改这些无关项。

## 7. 运行 / 日志 / CSV / RViz 结果

本 MVP 新增 RViz topic：

```text
/vis/agent_{node_id}/ssc/risk_grid_vis
```

可视化策略：

- 使用单个 `CUBE_LIST` marker 表示风险体素；
- 仅显示 `risk > 1.0e-6` 的 cell；
- 默认最多显示前 20 个时间层；
- 默认最多发布 5000 个风险体素；
- 风险越高越偏红且越不透明；
- 越远时间层透明度越低，便于观察近时域风险；
- 空地图时发布空 marker 覆盖上一帧，避免 RViz 残留。

验证边界：

- 已完成编译验证；
- 未在本次报告中启动 RViz 截图验证；
- 实车或仿真运行时还需要在 RViz2 中订阅该 topic 进行视觉确认。

## 8. Git commit 信息

推荐 commit message：

```text
feat(ssc): visualize risk grid in rviz
```

## 9. tag 信息

构建确认后，建议 tag：

```text
v0.1.3-mvp1c-risk-grid-visualization
```

## 10. 当前风险

- 当前 risk grid 仍来自 MVP-0 的固定 `existence_prob = 1.0f`，可视化显示的是确定性风险镜像，不是真实多模态概率风险。
- 默认仅显示前 20 个时间层，长时域风险需要后续增加参数化配置。
- 默认最多显示 5000 个 cell，高密度场景下会截断显示，以保护 RViz/DDS 性能。
- Z 轴表示相对时间，不是世界坐标高度，只用于时空调试图。

## 11. 下一步计划

1. 将 MVP-1C 合并回 `dev/risk-aware-ssc` 并打 `v0.1.3-mvp1c-risk-grid-visualization`。
2. 在真实 launch/RViz 中检查 topic 和颜色显示。
3. MVP-2 接入真实横向行为概率，不在 MVP-1C 中提前修改预测或行为模块。
