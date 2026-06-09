# EPSILON 风险感知 SSC 相对原始项目的工作与创新总结报告

## 1. 报告基准

本报告回答：当前 `dev/risk-aware-ssc` 分支相对原始 EPSILON/SSC 确定性规划框架，已经做了哪些创新和工程工作。

证据范围：

- 仓库路径：`/home/ros/work/graduateworkcc/src/EPSILON`
- 当前分支：`dev/risk-aware-ssc`
- 当前已推送里程碑 HEAD：`1528c21 docs(project): add risk aware ssc milestone reports`
- 当前状态：工作树包含 MVP-10 ego candidate 条件化风险图通道、可靠性补强和文档更新，尚未提交。
- 已存在里程碑 tag：`v0.1.1` 到 `v0.9.1-risk-ssc-reports`

可靠性说明：

- 本地 `origin` 指向 `git@github.com:taiyangkaorou-boop/EPSILON.git`。
- 本地 `upstream` 当前指向 `https://github.com/ZhouTao415/Autonomous-Motorsports-Motion-Planning-for-the-IAC.git`，不是计划书中写的 `HKUST-Aerial-Robotics/EPSILON`。
- 因此本文不声称已经完成对某个官方上游仓库的逐行 diff 审计；本文的“原始项目”指当前代码和历史报告中可确认的原 SSC baseline 机制：二值时空占据图、确定性周车轨迹、QP/corridor 不使用概率风险。

## 2. 一句话结论

当前分支已经把原始 EPSILON/SSC 从“确定性二值时空走廊规划”扩展成了一个较完整的“概率风险图驱动的风险感知 SSC 原型系统”：行为概率和多模态周车轨迹可以进入并行 risk grid，risk grid 可以统计、导出、可视化、按阈值影响 corridor，并可用于候选轨迹风险暴露评价、自适应风险权重、保守候选兜底和离线实验汇总。

需要准确降调的是：当前 MVP-6 不是严格把风险项写进 QP Hessian/目标函数，而是在 QP 生成候选之后做风险暴露评分和候选重选；MVP-10 已建立 ego candidate 条件化风险图数据通道，但仍不是完整的让行、阻塞、加速、减速博弈式交互预测。

## 3. 原始 SSC 基线能力边界

原始 SSC 的核心机制可以概括为：

```text
周车确定性预测轨迹
→ SscMap::FillDynamicPart()
→ SscMap::FillMapWithFsVehicleTraj()
→ p_3d_grid_ 二值占据图
→ corridor 构建
→ QP / primitive 轨迹生成
```

原始机制的主要限制：

1. 周车进入 SSC 的是单条确定性轨迹，不保留行为概率分布。
2. `p_3d_grid_` 只表达 free / occupied，不表达概率风险强度。
3. 多个可能行为不会以概率叠加形式进入规划地图。
4. corridor 默认只避开 hard occupied cell，不能区分低概率风险和高概率风险。
5. QP 候选选择没有轨迹风险暴露统计、CSV、实验分析链路。
6. 没有针对风险图的 RViz 可视化和论文实验复现工具。

## 4. 已完成工作总览

| 阶段 | 已完成内容 | 主要证据文件/函数 | 是否默认影响原始规划 |
|---|---|---|---|
| MVP-0 | 新增并行概率风险栅格 `p_3d_risk_grid_` | `ssc_map.h` 的 `RiskGridMap3D`、`risk_grid()`；`ssc_map.cc` 的 `ResetRiskMap()` | 否 |
| MVP-1A | 新增风险图统计结构和日志 | `RiskGridStats`、`ComputeRiskGridStats()`、`PrintRiskGridStatsIfNeeded()` | 否 |
| MVP-1B | RiskGridStats CSV 导出 | `AppendRiskGridStatsToCsv()`；默认 `/tmp/epsilon_risk_grid_stats.csv` | 否，但默认会写 CSV |
| MVP-1C | RViz 风险图可视化 | `SscVisualizer::VisualizeRiskGridInSscSpace()`；话题 `/vis/agent_{id}/ssc/risk_grid_vis` | 否 |
| MVP-2 | 接入周车行为概率 | `GetSurroundingTrajectoryExistenceProbabilities()`；`surround_traj_existence_probs_` | 否，仅加权 risk grid |
| MVP-3 | 生成周车 LK/LCL/LCR 多模态预测轨迹 | `SurroundingVehicleTrajectoryMode`、`GetMultiModalSurroundingTrajectories()`、`multimodal_surround_trajs_fs_` | 否，仅供 risk grid |
| MVP-4 | 概率风险图完整填充和概率叠加 | `FillDynamicPartProbabilistic()`、`FillMapWithFsVehicleTrajProbabilistic()` | 否，除非后续开关启用 |
| MVP-5 | risk threshold 影响 corridor | `ApplyRiskThresholdToBinaryOccupancy()`；配置 `enable_risk_aware_corridor` | 默认否，打开后是 |
| MVP-6 | 候选轨迹 risk exposure 评价和重选 | `RiskExposureMetrics`、`ComputeRiskExposureForCandidate()`、`SelectRiskAwareTrajectoryIndex()` | 默认否，打开后可能改变最终候选 |
| MVP-7 | 场景自适应风险权重 | `AdaptiveRiskWeightContext`、`BuildBaseAdaptiveRiskWeightContext()`、`UpdateAdaptiveRiskWeightContextByMetrics()` | 默认否 |
| MVP-8 | 风险兜底候选过滤 | `SafetyFallbackResult`、`ApplySafetyFallbackIfNeeded()` | 默认否 |
| MVP-9 | 离线实验汇总工具 | `util/ssc_planner/scripts/risk_experiment_report.py` | 否 |
| MVP-10 | Ego candidate 条件化多模态风险图通道 | `BehaviorConditionedMultiModalSurroundingTrajectories`、`GetBehaviorConditionedMultiModalSurroundingTrajectories()` | 否，仅改变 risk grid 旁路输入 |

## 5. 主要创新和工程工作

### 5.1 概率风险占据图

这是当前最核心、最扎实的新增能力。

原始 SSC 只有二值 `p_3d_grid_`。当前新增了并行的 `p_3d_risk_grid_`，用 `float` 风险值表达 `(s,d,t)` cell 的概率占据强度。它不替代原始二值图，而是与原图并行维护。

关键证据：

- `util/ssc_planner/include/ssc_planner/ssc_map.h`
  - `using RiskMapDataType = float`
  - `using RiskGridMap3D = std::vector<RiskMapDataType>`
  - `const RiskGridMap3D& risk_grid() const`
  - `RiskGridStats`
- `util/ssc_planner/src/ssc_planner/ssc_map.cc`
  - 构造和 reset 时初始化 / 清空 `p_3d_risk_grid_`
  - `ComputeRiskGridStats()`
  - `AppendRiskGridStatsToCsv()`

可声称的创新：

- 在原 SSC 二值时空地图之外，增加了一套概率风险表达层。
- 风险图可被统计、导出、查询、可视化，并可作为后续 corridor 和轨迹评价的统一数据源。

不能过度声称：

- 当前 risk grid 不是贝叶斯滤波器，也不是完整占据概率后验估计。
- 风险值目前主要来自行为概率和轨迹占据叠加，而不是端到端学习得到的概率场。

### 5.2 行为概率到多模态周车轨迹的传播

原始系统中已有 `SemanticVehicle.probs_lat_behaviors` 这类横向行为概率，但 SSC 主要消费确定性周车轨迹。当前分支把这些概率接入 SSC risk grid。

关键证据：

- `util/ssc_planner/include/ssc_planner/map_interface.h`
  - `SurroundingVehicleTrajectoryMode`
  - `MultiModalSurroundingTrajectories`
  - `BehaviorConditionedMultiModalSurroundingTrajectories`
  - `GetSurroundingTrajectoryExistenceProbabilities()`
  - `GetMultiModalSurroundingTrajectories()`
  - `GetBehaviorConditionedMultiModalSurroundingTrajectories()`
- `util/ssc_planner/src/ssc_planner/map_adapter.cc`
  - `GetSurroundingTrajectoryExistenceProbabilities()` 读取 argmax 行为概率
  - `GetMultiModalSurroundingTrajectories()` 对 LK / LCL / LCR 生成多模态轨迹
  - `GetBehaviorConditionedMultiModalSurroundingTrajectories()` 按 ego candidate 覆盖 deterministic/argmax 模态入口
- `util/ssc_planner/src/ssc_planner/ssc_planner.cc`
  - `Prepare` 阶段读取概率和多模态轨迹
  - `StateTransformForInputData()` 将全局多模态和 ego candidate 条件化多模态轨迹转换到 Frenet 空间

可声称的创新：

- 把行为预测层的概率信息从语义层传播到 SSC risk grid。
- 每辆周车可以以 LK / LCL / LCR 多个横向行为模态进入风险图，而不是只保留一条 argmax 轨迹。
- 每个 ego candidate 可以使用独立的周车 deterministic/argmax 风险入口，避免所有候选共用同一份全局 risk field。

不能过度声称：

- 当前多模态预测只覆盖横向行为 LK / LCL / LCR，尚未覆盖 Yield / Block / Merge / Cut-in 等复杂交互模态。
- 当前条件化主要覆盖 deterministic/argmax 模态，非 argmax 补充模态仍来自规则式开环预测；这还不是完整博弈式交互预测。
- 多模态预测 horizon 已配置化并可从 SSC map 时间域自动推导，仍需真实场景验证远时域风险是否按预期填充。

### 5.3 概率风险图填充与叠加

当前概率填图实现的核心公式是：

```text
Risk(s,d,t) = min(1.0, Σ p_i · Occupancy_i(s,d,t))
```

关键证据：

- `SscMap::FillDynamicPartProbabilistic()`
- `SscMap::FillMapWithFsVehicleTrajProbabilistic()`
- `cv::fillPoly()` 先生成单条轨迹在每个时间层的占据 mask
- `cv::add()` 累加概率贡献
- `cv::threshold(..., 1.0, 1.0, cv::THRESH_TRUNC)` 截断到 `[0,1]`

可声称的创新：

- 同一 cell 被多个周车或多个行为模态覆盖时，可以体现风险叠加强度。
- 单条轨迹同一时间层重复覆盖不会重复计数，降低了同一轨迹内部重复采样造成的风险膨胀。
- 当前工作树已经补强 deterministic fallback：如果某些车辆没有生成多模态轨迹，会继续用 deterministic 轨迹和 MVP-2 概率补齐 risk grid。

不能过度声称：

- 该叠加是工程上的风险强度近似，不是严格独立事件概率合成公式。
- s/d 方向部分越界已做边界裁剪，时间层越界仍会跳过该帧；若要更精确，可后续替换为真正的多边形裁剪。

### 5.4 Risk-aware corridor

MVP-5 让 risk grid 第一次进入规划约束链路。

关键证据：

- `util/ssc_planner/include/ssc_planner/ssc_map.h`
  - `Config::enable_risk_aware_corridor`
  - `Config::risk_occupied_threshold`
- `util/ssc_planner/src/ssc_planner/ssc_map.cc`
  - `ApplyRiskThresholdToBinaryOccupancy()`
  - `ConstructSscMap()` 中按开关调用
- `util/ssc_planner/proto/ssc_config.proto`
  - `optional bool enable_risk_aware_corridor`
  - `optional double risk_occupied_threshold`
- `util/ssc_planner/config/ssc_config.pb.txt`
  - 默认 `enable_risk_aware_corridor: false`
  - 默认 `risk_occupied_threshold: 1.0`

可声称的创新：

- 将概率风险图按阈值投影到原始二值占据图，使 corridor 可以避开高风险区域。
- 没有重写 corridor 膨胀算法，而是复用原 SSC 的 occupied/free 语义，接入点明确、可回退。

不能过度声称：

- 这是风险阈值约束，不是连续风险约束优化。
- 只要默认开关关闭，risk grid 仍不会影响 corridor。

### 5.5 候选轨迹风险暴露评价

MVP-6 之后，系统可以沿候选轨迹采样 risk grid，得到 `exposure_sum`、`exposure_mean`、`exposure_max`、`high_risk_hits` 和 `risk_score`。

关键证据：

- `util/ssc_planner/include/ssc_planner/ssc_planner.h`
  - `RiskExposureMetrics`
  - `behavior_risk_grid_snapshots_`
  - `candidate_risk_grid_snapshots_`
- `util/ssc_planner/src/ssc_planner/ssc_planner.cc`
  - `ComputeRiskExposureForCandidate()`
  - `SelectRiskAwareTrajectoryIndex()`
  - `AppendRiskExposureMetricsToCsv()`
  - `EnsureRiskExposureMetrics()`

可声称的创新：

- 建立了“risk grid → ego candidate risk exposure → 轨迹选择”的闭环。
- 每个 ego behavior 构图后保存 risk grid snapshot，避免所有候选误用最后一次构图残留的风险图。
- 当前工作树已修复低速模式下 primitive / Bezier 评价错配：高速采样 `FrenetBezierTrajectory`，低速采样 `FrenetPrimitiveTrajectory`。

必须降调：

- 文件名和 tag 中有 `risk-cost-qp`，但当前实现不是把风险项直接写进 QP 优化器内部目标函数。
- 更准确表述是：QP 后候选轨迹风险暴露评分与重选，或 risk-aware candidate selection。

### 5.6 场景自适应风险权重

MVP-7 在候选风险评价链路上增加了场景自适应参数。

关键证据：

- `AdaptiveRiskWeightContext`
- `BuildBaseAdaptiveRiskWeightContext()`
- `UpdateAdaptiveRiskWeightContextByMetrics()`
- 配置项：
  - `enable_adaptive_risk_weight`
  - `adaptive_high_speed_threshold`
  - `adaptive_high_speed_weight_scale`
  - `adaptive_lane_change_weight_scale`
  - `adaptive_high_risk_threshold`
  - `adaptive_high_risk_weight_scale`
  - `adaptive_max_risk_weight`
  - `adaptive_switch_margin_scale`

可声称的创新：

- 在高速、变道、高交互风险场景下动态调整风险权重和候选切换裕度。
- 为论文中的场景自适应消融实验提供了工程入口。

不能过度声称：

- 当前是规则驱动的启发式自适应，不是学习型策略，也不是完整场景理解系统。

### 5.7 风险兜底安全过滤器

MVP-8 增加了保守候选过滤逻辑。

关键证据：

- `SafetyFallbackResult`
- `ShouldComputeRiskExposureMetrics()`
- `ApplySafetyFallbackIfNeeded()`
- 配置项：
  - `enable_safety_fallback`
  - `safety_fallback_max_risk_threshold`
  - `safety_fallback_exposure_sum_threshold`
  - `safety_fallback_high_risk_hits_threshold`
  - `safety_fallback_min_risk_reduction`
  - `safety_fallback_prefer_lane_keeping`

可声称的创新：

- 当当前候选风险过高时，可以在已有 QP 成功候选中选择更低风险或更保守候选。
- CSV 会记录 fallback 是否触发、是否切换、原候选和兜底候选风险。

必须降调：

- 当前不是 emergency braking。
- 当前不是 CBF 或形式化安全过滤器。
- 当前只在已有候选集合内切换，不会生成新的紧急制动轨迹。

### 5.8 可视化、CSV 和离线实验工具链

这是论文实验复现最有用的工程工作。

关键证据：

- `SscMap::AppendRiskGridStatsToCsv()`
  - 输出 `/tmp/epsilon_risk_grid_stats.csv`
  - 字段：`map_build,stamp,behavior_index,behavior,total_cells,nonzero_cells,max_risk,sum_risk,active_time_layers,total_time_layers`
- `SscPlanner::AppendRiskExposureMetricsToCsv()`
  - 输出 `/tmp/epsilon_mvp6_risk_exposure.csv`
  - 包含 candidate、behavior、baseline/selected、risk exposure、adaptive、fallback 等字段
- `SscVisualizer::VisualizeRiskGridInSscSpace()`
  - 发布 risk grid MarkerArray
  - 按风险值调颜色和透明度
  - 限制时间层和 cell 数，避免 RViz 卡顿
- `util/ssc_planner/scripts/risk_experiment_report.py`
  - 读取 risk grid CSV 和 risk exposure CSV
  - 输出 `summary.csv`、`summary.json`
  - 可选输出趋势图

可声称的工作：

- 风险图、轨迹风险暴露、自适应和兜底结果都可以被 CSV 追踪。
- 离线脚本能把实验数据整理成论文可用的 summary。
- 里程碑报告和 tag 使实验数据可回溯到 commit/tag。

不能过度声称：

- MVP-9 是离线汇总工具，不是自动场景生成器，也不是完整实验调度平台。
- `risk_exposure` CSV 仍是候选轨迹逐行记录；脚本已经新增 `*_cycles` 与
  `cycle_selected.*` / `cycle_baseline.*` 周期级指标，论文统计触发次数时应优先使用周期级字段。

## 6. 相比最初项目的主要工程改动范围

主要改动集中在 `util/ssc_planner`：

1. `util/ssc_planner/include/ssc_planner/ssc_map.h`
   - 新增 risk grid 类型、统计结构、查询接口、概率填图接口、risk-aware corridor 接口。

2. `util/ssc_planner/src/ssc_planner/ssc_map.cc`
   - 新增 risk grid 初始化、reset、统计、CSV、概率填图、风险阈值投影。

3. `util/ssc_planner/include/ssc_planner/map_interface.h`
   - 新增周车概率和多模态轨迹接口。

4. `util/ssc_planner/src/ssc_planner/map_adapter.cc`
   - 从 semantic map 读取周车行为概率，并生成 LK / LCL / LCR 多模态预测轨迹。

5. `util/ssc_planner/include/ssc_planner/ssc_planner.h`
   - 新增 risk exposure、adaptive risk weight、safety fallback 的数据结构和接口。

6. `util/ssc_planner/src/ssc_planner/ssc_planner.cc`
   - 接入概率/多模态输入、保存风险图快照、计算候选风险暴露、风险重选、自适应权重和兜底过滤。

7. `util/ssc_planner/include/ssc_planner/ssc_visualizer.h`
   - 新增 risk grid 可视化接口和 publisher。

8. `util/ssc_planner/src/ssc_planner/ssc_visualizer.cc`
   - 新增 risk grid RViz MarkerArray 发布逻辑。

9. `util/ssc_planner/proto/ssc_config.proto`
   - 新增 risk-aware corridor、risk exposure、自适应风险权重、安全兜底相关配置。

10. `util/ssc_planner/config/ssc_config.pb.txt`
    - 增加默认配置，并保持影响规划的开关默认关闭。

11. `util/ssc_planner/scripts/risk_experiment_report.py`
    - 新增离线实验汇总脚本。

12. `docs/`
    - 新增 MVP-0 到 MVP-9 实施报告、构建阻塞计划、白盒审计报告等论文工程过程文档。

## 7. 创新强度和实现难度评估

| 模块 | 创新强度 | 实现难度 | 评价 |
|---|---:|---:|---|
| 概率风险占据图 | 高 | 中高 | 最核心贡献，把 SSC 环境表达从 binary 扩展到 probabilistic risk |
| 多模态行为概率传播 | 中高 | 中高 | 已有 ego candidate 条件化数据通道，但完整交互响应还需后续模型增强 |
| 概率风险叠加填图 | 高 | 中高 | 与 SSC map 几何填图深度结合，工程含金量较足 |
| risk-aware corridor | 高 | 中 | 接入点简洁，实验效果预计明显，适合硕士论文主贡献 |
| risk exposure 候选重选 | 中 | 中高 | 有实用价值，但需明确不是 QP 内部 risk cost |
| 自适应风险权重 | 中 | 中 | 规则启发式，适合作为小创新点和消融项 |
| safety fallback | 中 | 中 | 可作为安全兜底机制，但不应写成 emergency control |
| CSV/RViz/离线脚本 | 中 | 中 | 理论创新弱，但论文实验和工程完整性价值高 |

总体判断：

- 作为硕士论文工程系统，当前实现难度和工作量是足够的。
- 作为论文创新，最稳的主线应放在：
  1. 概率风险 SSC 占据图；
  2. 行为概率/多模态轨迹到风险图的传播；
  3. risk-aware corridor；
  4. 候选轨迹风险暴露评价与实验工具链。
- 不建议把当前版本包装成完整概率最优控制或完整交互博弈预测系统。

## 8. 当前可靠性验证

已有白盒审计报告记录的验证结果：

```bash
cd /home/ros/work/graduateworkcc
colcon build --packages-up-to ssc_planner --symlink-install
```

结果：`Summary: 7 packages finished [8.80s]`，结论为通过；仅有第三方 `tk_spline` unused-function warning。

```bash
cd /home/ros/work/graduateworkcc
colcon test --packages-select ssc_planner --event-handlers console_direct+
```

结果：`Summary: 1 package finished [0.50s]`，结论为通过。

```bash
python3 -m py_compile util/ssc_planner/scripts/risk_experiment_report.py
```

结果：通过。

MVP-9/MVP-10 离线脚本还通过了合成 CSV 单元级测试：

```text
synthetic_risk_experiment_report_test=PASS
risk_grid_behavior_groups=2
risk_grid_behavior_multi_behavior_groups=2
risk_grid_behavior_variant_groups=1
risk_grid_behavior_variant_ratio=0.5
```

验证边界：

- 上述构建和包测试能证明当前包级编译与测试入口不失败。
- 还不能证明真实 ROS 场景中风险策略一定优于 baseline。
- 还缺少系统化场景消融实验、TTC、碰撞率、最小距离、舒适性和耗时统计。
- 离线脚本合成 CSV 测试能证明 summary 逻辑可运行，但不能替代真实场景日志数据。

## 9. 本轮补强与仍需谨慎的风险

### 9.1 本轮已补强项

1. 多模态预测时域已从硬编码 `5.0s` 改为配置项；配置为 `0.0` 时会自动使用 SSC map 时间域和时间分辨率。
2. RiskGridStats CSV 已增加 `map_build`、`stamp`、`behavior_index`、`behavior`，降低把构图次数误读成 planning cycle 的风险。
3. 离线脚本已新增按 `(cycle, stamp)` 聚合的 `*_cycles` 和 `cycle_selected.*` / `cycle_baseline.*` 指标。
4. `SscPlannerAdapter::set_map()` 已拒绝空地图快照，`GetTimeStamp()` 会检查有效性。
5. `SscPlanner::ReadConfig()` 已从直接 assert 改为 open/parse/initialized 错误返回。
6. RViz corridor marker 已改用传入的同帧 `stamp`，避免同一帧 marker 时间戳不一致。
7. risk grid 概率填图在车辆轮廓部分越过 s/d 边界时已改为保守裁剪到地图边界内，不再直接丢弃整帧风险。
8. 默认 RViz 配置已加入 `/vis/agent_0/ssc/risk_grid_vis` 的 `RiskGrid` MarkerArray 显示项。
9. MVP-10 已新增 ego candidate 条件化多模态风险图通道，构图循环会按 `behavior_index` 使用对应的周车多模态风险场。
10. 静态障碍物填图已使用 SSC map 绝对时间原点，避免 ROS 时间戳非零时静态障碍落到负时间层。
11. 多模态预测部分成功时，risk grid 会用 deterministic 轨迹补齐剩余概率质量，避免高概率失败模态漏风险。
12. RiskGridStats CSV 已增加 `risk_source_vehicles` 和 `risk_source_modes`，
    便于追踪每个 ego behavior 构图实际使用的周车和模态数量。
13. RViz `risk_grid_vis` 已改为优先显示最终 selected candidate 的 risk grid
    snapshot；无候选快照时才回退显示 live `SscMap::risk_grid()`。
14. 空多模态轨迹不再消耗 deterministic fallback 概率质量，避免“空 mode 扣概率但不填图”导致风险低估。
15. `RunOnce()` 开头会清空候选 risk grid 快照和 selected/baseline 索引，避免失败帧暴露上一帧风险图。
16. 离线脚本新增 `risk_grid_behavior_variant_*` 与 `risk_grid_behavior_*_range_*` 指标，可量化同一 planning cycle 内不同 ego candidate 的风险图差异。

### 9.2 仍需谨慎的技术风险

1. 多模态风险图已新增 SSC 内部 ego candidate 条件化数据通道；
   其中周车 deterministic/argmax 模态会使用 `surround_trajs[ego_behavior_index]`
   覆盖，从而使不同 ego candidate 可以拥有不同 risk grid。仍需注意：
   非 argmax 的 LK/LCL/LCR 补充模态仍来自规则式开环预测，不是完整博弈式交互预测。
2. RViz risk grid 仍需实际运行截图验收；周车轨迹可视化只显示第一组 behavior，不能替代完整风险场证明。
3. 当前影响规划的风险功能默认关闭，需要实验配置显式打开后再评估效果。

### 9.3 下一阶段结构性增强建议

严格 ego candidate 条件化多模态风险场已经开始以 SSC 内部旁路形式落地。当前完成的是
MVP-10 的工程通道第一步：

```text
MultiModalSurroundingTrajectories
  从 [vehicle_id][mode]
  扩展为 [ego_behavior_index][vehicle_id][mode]

SscPlanner::RunOnce()
  每个 ego forward behavior 构图时使用对应的周车多模态风险场

实验验收
  LK / LCL / LCR 自车候选下，周车风险图快照不同
```

后续若要把论文表述升级到“完整交互条件化预测”，还需要在预测器或行为层显式引入
ego candidate 对周车让行、加速、减速、阻塞等纵向交互响应的建模。

这样才能真正支撑“交互条件化风险场”的论文表述。

### 9.4 论文表述风险

建议避免这些说法：

- “已实现完整 QP risk cost”
- “已实现完整交互条件化多模态预测”
- “已实现 emergency braking”
- “已完成自动化实验平台”
- “风险图是真实后验概率地图”

建议使用这些说法：

- “构建了基于行为概率传播的概率风险占据图”
- “提出了风险阈值约束的 SSC corridor 构建方法”
- “设计了基于 risk exposure 的候选轨迹评价与重选机制”
- “实现了场景自适应风险权重和风险兜底候选过滤”
- “搭建了 CSV、RViz 和离线汇总组成的实验复现工具链”

## 10. 论文贡献建议写法

推荐论文贡献点可以写成：

1. 提出一种面向 SSC 的概率风险占据图表达方法，在原有确定性二值时空地图旁路维护风险强度，并支持统计、查询和可视化。
2. 设计行为概率到多模态周车轨迹再到风险图的传播链路，使横向行为不确定性能够进入运动规划层。
3. 提出风险阈值约束的 risk-aware SSC corridor 方法，在保持 baseline 可回退的前提下，使高风险时空栅格能够约束走廊构建。
4. 构建候选轨迹 risk exposure 评价、风险自适应权重和保守候选兜底机制，用于降低最终轨迹穿越高风险区域的概率。
5. 实现面向论文实验的 CSV / RViz / 离线汇总工具链，使风险图和轨迹风险暴露可以被复现实验数据支撑。

## 11. 最终结论

当前分支相对原始 SSC 已经不只是“加了一个风险代价项”，而是完成了一条比较完整的风险感知规划工程链：

```text
原始确定性 SSC
+ 行为概率读取
+ 周车多模态预测
+ Ego candidate 条件化风险图通道
+ 概率风险占据图
+ 风险统计/CSV/RViz
+ 风险阈值 corridor
+ 候选轨迹 risk exposure
+ 自适应风险权重
+ 安全兜底候选过滤
+ 离线实验汇总工具
```

从硕士论文角度看，工程量和实现难度已经足够；最强创新应聚焦在“概率风险占据图 + risk-aware corridor + ego candidate 条件化风险图通道 + 风险暴露评价工具链”。后续最需要补强的是真实场景消融实验、完整交互响应预测、真正 QP 内部风险代价，以及 RViz/CSV 在真实场景中的截图和数据闭环。
