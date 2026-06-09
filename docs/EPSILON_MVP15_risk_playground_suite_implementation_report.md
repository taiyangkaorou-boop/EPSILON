# MVP-15 实施报告：Risk Playground Suite

## 1. 目标

MVP-15 的目标是补充论文实验可用的高风险初始交通场景库，让 MVP-13/MVP-14 的闭环实验工具能够在更密集、更有相对速度差的场景中生成风险图、summary 和 matrix。

本阶段新增三个 playground：

```text
risk_dense_following_v1.0
risk_merge_pressure_v1.0
risk_lane_change_conflict_v1.0
```

## 2. 修改范围

允许修改：

- `core/playgrounds/risk_dense_following_v1.0/*`
- `core/playgrounds/risk_merge_pressure_v1.0/*`
- `core/playgrounds/risk_lane_change_conflict_v1.0/*`
- `core/playgrounds/CMakeLists.txt`
- `util/ssc_planner/scripts/risk_experiment_scenario_suite.py`
- `util/ssc_planner/scripts/README_risk_experiment_suite.md`
- `docs/EPSILON_MVP15_risk_playground_suite_implementation_report.md`

禁止修改：

- SSC / EUDM / MPDM / QP / fallback 规划逻辑；
- `phy_simulator` 运动学模型；
- `highway_lite` 原始场景；
- `highway_v1.0` / `ring_*` 原始场景；
- risk grid / corridor / exposure 计算逻辑。

## 3. 场景设计

三个新场景均复用 `highway_lite` 的：

```text
agent_config.json
lane_net_norm.json
obstacles_norm.json
```

只修改：

```text
vehicle_set.json
```

这样可以避免新建路网带来的坐标和拓扑风险，同时让场景仍能被现有 `phy_simulator`、`planning_integrated`、`ssc_planner` 直接加载。

## 4. 场景语义

### risk_dense_following_v1.0

目标：

```text
高密度跟车
短车距
前慢后快
```

用途：

```text
测试 risk grid 在纵向密集车流中的覆盖范围
测试 risk-aware corridor 是否更保守
测试 risk exposure 指标是否随车距变短而上升
```

### risk_merge_pressure_v1.0

目标：

```text
近距离汇入压力
相邻车道车辆速度差
自车周围多车同时进入预测范围
```

用途：

```text
测试多周车风险叠加
测试 fallback / adaptive risk weight 的触发倾向
```

### risk_lane_change_conflict_v1.0

目标：

```text
相邻车道并行冲突
前后车速度差
横向候选行为存在风险差异
```

用途：

```text
测试行为条件化 risk grid 的差异
测试 risk-aware corridor 对高风险横向区域的避让效果
```

## 5. 重要边界

当前闭环实验入口只启动：

```text
phy_simulator_planning_node
planning_integrated
```

未启动周车 AI agent，因此非自车车辆没有主动 cut-in 控制器。

当车辆初始速度为非零时，未收到控制信号的周车会沿初始朝向以零加速度继续运动。因此 MVP-15 场景应表述为：

```text
高密度 / 相对速度风险初始交通态
```

不应表述为：

```text
主动 cut-in 行为场景
```

主动 cut-in / merge 行为控制应作为后续单独 MVP 实现。

## 6. 接入项

`core/playgrounds/CMakeLists.txt` 已安装三个新场景。

`risk_experiment_scenario_suite.py` 默认场景列表已增加：

```text
risk_dense_following_v1.0
risk_merge_pressure_v1.0
risk_lane_change_conflict_v1.0
```

因此 MVP-14 的多场景套件会自动包含 MVP-15 场景。

## 7. 验证结果

已执行：

```text
python3 -m json.tool 新增场景 JSON -> 通过
risk_experiment_scenario_suite.py dry-run -> 新场景进入 scenario_run_plan.sh / scenario_suite_manifest.json
ros2 launch planning_integrated risk_experiment_closed_loop_launch.py playground:=risk_dense_following_v1.0 --show-args -> 通过
colcon build --packages-up-to playgrounds planning_integrated ssc_planner --symlink-install -> 通过
timeout 3s ros2 launch ... playground:=risk_dense_following_v1.0 -> 通过并生成 risk grid CSV
timeout 3s ros2 launch ... playground:=risk_merge_pressure_v1.0 -> 通过并生成 risk grid CSV
timeout 3s ros2 launch ... playground:=risk_lane_change_conflict_v1.0 -> 通过并生成 risk grid CSV
```

短时闭环 smoke 中的 risk grid 末行示例：

```text
risk_dense_following_v1.0: nonzero_cells=11104, risk_source_vehicles=7,  risk_source_modes=7
risk_merge_pressure_v1.0: nonzero_cells=12363, risk_source_vehicles=9,  risk_source_modes=9
risk_lane_change_conflict_v1.0: nonzero_cells=12436, risk_source_vehicles=10, risk_source_modes=10
```

这些结果说明新增场景能够被闭环 launch 加载，并且高密度初始交通态已经进入 risk grid 统计链路。

## 8. 不改动项

MVP-15 不改变规划算法，只提供新的实验输入场景。

所有风险策略是否生效仍由既有 SSC 配置控制：

```text
ssc_config.pb.txt
ssc_config_risk_observe.pb.txt
ssc_config_risk_corridor.pb.txt
ssc_config_risk_full.pb.txt
```

## 9. 当前风险

场景使用现有 highway_lite 路网，因此路网加载风险较低。

主要风险是车辆初始相对位置较密，极端情况下可能导致仿真初始碰撞或风险图过密。该风险适合作为论文 stress test，但在最终实验章节中需要明确标注为高风险场景。

## 10. Git 信息

推荐 commit：

```bash
feat(ssc): add risk playground suite
```

推荐 tag：

```bash
v0.15.0-mvp15-risk-playground-suite
```

## 11. 下一步计划

下一阶段建议进入 MVP-16：

```text
主动周车行为场景控制
```

重点是让周车 AI agent 或开环控制信号参与实验，从而形成真正的 cut-in / merge / lane-change 动态交互场景。
