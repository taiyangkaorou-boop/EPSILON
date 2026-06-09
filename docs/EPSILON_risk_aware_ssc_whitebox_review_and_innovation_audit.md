# EPSILON 风险感知 SSC 白盒审计、测试与论文创新性评估报告

## 1. 审计目标

本报告面向当前 `dev/risk-aware-ssc` 分支，对 MVP-0 到 MVP-10 的风险感知 SSC 工程实现进行白盒审计，重点回答四个问题：

1. 当前代码是否存在潜在 bug 或实验结论误判风险；
2. 论文创新点和工程工作量是否足够扎实、充分；
3. 新增代码及关键原有代码是否有充足、详细的中文注释；
4. 当前构建、包测试和离线工具单元级测试是否通过。

审计时间：2026-06-09 17:15:06 CST  
修复复核：2026-06-09，本轮已修复 H1、H2、M1，并完成 H3 的 SSC 内部 ego candidate 条件化风险图通道；同时补强空多模态轨迹概率质量守恒、失败帧快照清理和候选数组尺寸防御。  
审计分支：`dev/risk-aware-ssc`  
审计 HEAD：`1528c21 docs(project): add risk aware ssc milestone reports`

## 2. 当前结论摘要

当前工程已经形成完整的风险感知 SSC 原型链路：

```text
行为概率
→ 多模态周车轨迹
→ 概率风险占据图
→ risk-aware corridor 阈值投影
→ 候选轨迹风险暴露评价
→ 场景自适应风险权重
→ 风险兜底候选过滤
→ CSV / RViz / 离线实验汇总
```

但从论文表述上必须降调：

- 当前实现适合写成“风险场辅助的 SSC 候选选择与走廊约束框架”；
- 不宜直接写成“完整概率最优控制器”；
- MVP-6 不是 QP 内部风险代价，而是 QP 成功候选后的风险暴露评分与重选；
- MVP-8 不是 emergency braking，只是在已有候选轨迹集合内进行保守候选替换；
- MVP-9 是离线汇总工具，不是自动实验调度平台。

整体判断：

| 维度 | 评价 | 说明 |
|---|---:|---|
| 工程完整性 | 较强 | map、planner、adapter、config、CSV、RViz、脚本、报告都有落地 |
| 论文创新性 | 中等偏强 | 风险图与 corridor 接入较扎实，ego candidate 条件化通道已落地，完整博弈式交互预测和 QP 内部风险代价仍偏弱 |
| 实验支撑 | 当前不足 | 需要真实场景消融、TTC、碰撞、舒适性、耗时数据支撑 |
| 代码注释 | 较充分 | 风险相关新增代码中文注释覆盖高，已持续修正随 MVP 演进过期的注释 |
| 当前代码风险 | 中等 | 主要高优先级 bug 已修复，剩余风险集中在真实场景验证和完整交互预测语义边界 |

## 3. 白盒审计发现

### 3.1 高优先级问题

#### H1. 低速模式执行轨迹与风险评估轨迹可能不一致

状态：已修复。

位置：

- `util/ssc_planner/src/ssc_planner/ssc_planner.cc:637-663`
- `util/ssc_planner/src/ssc_planner/ssc_planner.cc:766-800`
- `util/ssc_planner/include/ssc_planner/ssc_planner.h:159-167`

现象：

低速模式下最终执行可能使用 `FrenetPrimitiveTrajectory`，但 MVP-6/7/8 的风险暴露评价采样的是 `qp_trajs_` 中的 Bezier 轨迹。并且低速时即使 Bezier 生成失败，该候选也不会像高速模式一样被跳过。

风险：

开启 `enable_risk_exposure_eval`、`enable_risk_exposure_reselect` 或 `enable_safety_fallback` 后，系统可能基于一条非最终执行轨迹或无效 Bezier 的风险值做重选和兜底，导致风险结论和实际控制输出不一致。

建议：

- 低速模式下对 `primitive_trajs_[i]` 做风险采样；
- 或在 Bezier 失败时不生成对应 risk metric；
- 新增测试：低速、Bezier 失败但 primitive 成功、开启 fallback，检查风险指标是否对应最终输出轨迹。

本轮修复：

- `ComputeRiskExposureForCandidate()` 已改为通过统一的 `common::FrenetTrajectory` 接口采样候选实际执行轨迹；
- 高速模式采样 `FrenetBezierTrajectory`；
- 低速模式采样 `FrenetPrimitiveTrajectory`；
- 因此 risk exposure、risk reselect 和 safety fallback 的评价对象与最终输出轨迹保持一致。

#### H2. 原多模态预测时域固定为 5 秒，短于 SSC map 时间域

状态：已修复。

位置：

- `util/ssc_planner/src/ssc_planner/map_adapter.cc:152-154`
- `util/ssc_planner/src/ssc_planner/ssc_map.cc:156-160`
- `util/ssc_planner/config/ssc_config.pb.txt:45-51`

现象：

原问题中多模态周车预测固定使用：

```cpp
kPredictionTime = 5.0;
kPredictionStep = 0.2;
```

但当前 SSC map 配置为：

```text
map_size_z = 41
map_resl_z = 0.2
time horizon ≈ 8.2s
```

且只要 `multimodal_trajs_fs` 非空，risk grid 就完全使用多模态分支，不再回退 deterministic risk 填图。

风险：

5 秒之后的风险图可能系统性变成 0，导致候选轨迹后半段 risk exposure 被低估；开启 risk-aware corridor 时，远时域高风险也不会被提升为 hard occupied。

建议：

- 多模态预测时域应从 SSC map 时间域或规划 horizon 推导；
- 或对多模态缺失时间层回退 deterministic trajectory；
- 新增测试：构造 6 到 8 秒的周车占用，验证风险统计和 exposure 后半段不为 0。

本轮修复：

- `ConstructSscMap()` 已改为调用新的多模态填图重载；
- 多模态成功的车辆仍按多模态概率叠加；
- 未生成多模态的车辆会使用 deterministic 周车轨迹和 MVP-2 概率补齐 risk grid；
- 新增 `multimodal_prediction_time` / `multimodal_prediction_step` 配置；
- 配置为 `0.0` 时，`SscPlanner` 会从 SSC map 时间域和时间分辨率推导预测时长与步长；
- `SscPlannerAdapter` 已使用配置后的时长/步长生成 LK/LCL/LCR 多模态轨迹；
- 空多模态轨迹不会再消耗 deterministic fallback 概率质量，避免“空 mode 扣概率但不填图”的风险低估。

#### H3. 多模态风险场不是 ego candidate 条件化的

状态：已完成 SSC 内部条件化通道；仍不是完整博弈式交互预测。

原问题位置：

- `util/ssc_planner/src/ssc_planner/ssc_planner.cc:354-370`
- `util/ssc_planner/src/ssc_planner/map_adapter.cc:141-209`

现象：

每个 ego candidate 都保存风险图快照，但传入 `ConstructSscMap()` 的 `multimodal_surround_trajs_fs_` 是同一份全局多模态周车轨迹，不随 `forward_behaviors_[i]` 改变。

风险：

当前 risk field 不是严格意义上的“交互条件化风险场”。risk-aware reselect 只能比较不同 ego 轨迹穿过同一风险场的风险暴露，不能表达“自车 LK/LCL/LCR 导致周车预测不同”的交互响应。

建议：

- 将多模态输入扩展为 `[ego_behavior_idx][vehicle_id][mode]`；
- 原方案若不修复，则需在论文中明确是 ego-behavior-independent risk field；
- 后续实验不要把该实现夸大为完整交互条件化预测。

本轮修复：

- 新增 `BehaviorConditionedMultiModalSurroundingTrajectories`；
- `SscPlannerAdapter::GetBehaviorConditionedMultiModalSurroundingTrajectories()` 会按 `ego_behavior().surround_trajs[ego_behavior_index]` 为每个自车候选生成独立周车风险输入；
- `SscPlanner::RunOnce()` 在每个 behavior 构图时优先使用对应的条件化多模态风险场；
- `risk_experiment_report.py` 新增 `risk_grid_behavior_variant_*` 指标，用于量化同一 planning cycle 内不同 behavior 的 risk grid 是否产生差异；
- 语义边界：当前是 SSC 内部条件化风险图通道，不是完整的让行/阻塞/加速/减速博弈预测器。

#### H4. 失败帧可能暴露上一帧 selected risk grid snapshot

状态：已修复。

位置：

- `util/ssc_planner/src/ssc_planner/ssc_planner.cc:selected_risk_grid_snapshot()`
- `util/ssc_planner/src/ssc_planner/ssc_planner.cc:RunOnce()`

现象：

正常成功路径中 `candidate_risk_grid_snapshots_` 与 `qp_trajs_` / `valid_behaviors_`
按同一顺序写入；但如果某一帧在输入读取、坐标变换、构图或走廊阶段提前失败，
外部 accessor 可能仍看到上一成功帧的 selected snapshot。

本轮修复：

- `RunOnce()` 开头统一清空 `behavior_risk_grid_snapshots_`、`candidate_risk_grid_snapshots_`、`baseline_candidate_index_` 和 `selected_candidate_index_`；
- `selected_risk_grid_snapshot()` / `baseline_risk_grid_snapshot()` 额外交叉检查 `qp_trajs_` 与 `valid_behaviors_` 尺寸；
- `RunQpOptimization()` 增加走廊、行为、有效标志、全局/Frenet 前向轨迹的同维检查，并跳过空前向轨迹候选。

### 3.2 中优先级问题

#### M1. safety fallback 的 high-risk hits 阈值为 0 时会恒触发

状态：已修复。

位置：

- `util/ssc_planner/src/ssc_planner/ssc_planner.cc:1037-1047`

现象：

代码将 `safety_fallback_high_risk_hits_threshold` 截断到最小 0，然后判断：

```cpp
high_risk_hits >= threshold
```

如果用户把阈值设为 0，以为关闭该条件，结果会导致所有候选都满足该触发条件。

建议：

改成：

```cpp
threshold > 0 && high_risk_hits >= threshold
```

或者在配置和注释中明确 `0` 表示“总是触发”。

本轮修复：

- `trigger_by_hits` 已增加 `high_risk_hits_threshold > 0` 判断；
- 现在 `safety_fallback_high_risk_hits_threshold == 0` 表示关闭 high-risk hits 触发条件；
- 避免开启 safety fallback 后因为 `high_risk_hits >= 0` 恒成立而每帧误触发。

#### M2. RiskGridStats CSV 默认开启且 cycle 语义容易误读

状态：已修复主要语义问题；CSV 默认开启仍保留。

位置：

- `util/ssc_planner/include/ssc_planner/ssc_map.h:459-466`
- `util/ssc_planner/src/ssc_planner/ssc_map.cc:174-176`
- `util/ssc_planner/src/ssc_planner/ssc_map.cc:395-443`

现象：

RiskGridStats CSV 默认写入 `/tmp/epsilon_risk_grid_stats.csv`。同时 `ConstructSscMap()` 是按 ego candidate 循环调用，旧字段 `cycle` 容易被误读为完整 planning cycle。

风险：

默认 baseline 仍会产生 IO 和大量 WARNING 日志；但 CSV 已改为 `map_build,stamp,behavior_index,behavior,...`，显式表达这是构图记录而不是 planning cycle。

建议：

- 将 CSV 开关纳入 proto，并默认关闭；
- 后续如需更安静的 baseline，可继续将 CSV 开关纳入 proto 并默认关闭；
- 论文实验应使用 `map_build`/`behavior_index` 理解 RiskGridStats 粒度。

#### M3. 风险填图遇到车辆轮廓部分越界时整帧跳过

状态：已修复 risk grid 侧的保守裁剪；binary map baseline 逻辑未改。

位置：

- `util/ssc_planner/src/ssc_planner/ssc_map.cc:1431-1441`

现象：

概率风险填图要求车辆轮廓所有顶点都在 map 范围内，否则整帧跳过。

风险：

原实现中车辆一部分进入 SSC map 边界时，实际可见占用可能被完全丢弃，可能漏报边界风险。

建议：

- 当前已在 `FillMapWithFsVehicleTrajProbabilistic()` 中对 s/d 部分越界顶点做边界夹取；
- 时间层越界仍跳过整帧，避免写到不存在的风险层；
- 后续如要更精确，可替换为真正的多边形裁剪算法。

### 3.3 低优先级问题

#### L1. 默认 RViz 配置已补齐 risk_grid_vis 显示项，仍需截图验收

位置：

- `util/ssc_planner/src/ssc_planner/ssc_visualizer.cc:44-65`
- `util/ssc_planner/rviz/ssc_config.rviz`

现象：

代码发布 `/vis/agent_0/ssc/risk_grid_vis`，默认 RViz 配置已加入 `RiskGrid` MarkerArray 显示项。

风险：

当前代码与配置侧风险已降低；剩余风险是尚未通过实际 RViz 截图确认颜色、透明度和显示数量是否满足论文截图需求。

建议：

保留默认 `risk_grid_vis` MarkerArray 配置，并在下一次仿真运行时做截图验收。

#### L2. 少数中文注释随 MVP 演进后语义过期

位置：

- `util/ssc_planner/src/ssc_planner/ssc_map.cc:172-176`
- `util/ssc_planner/src/ssc_planner/ssc_visualizer.cc:178-181`

现象：

部分注释仍强调 risk grid “不影响规划决策”。在 MVP-5 开启 `enable_risk_aware_corridor` 后，风险图会通过 `ApplyRiskThresholdToBinaryOccupancy()` 修改 `p_3d_grid_`，从而影响 corridor。

建议：

将注释改为：

```text
统计函数 / 可视化函数本身只读；但当 enable_risk_aware_corridor 打开时，
risk grid 已会通过阈值投影间接影响 corridor。
```

## 4. 创新点与工作量评估

### 4.1 大创新点 1：交互概率驱动的多模态周车轨迹生成

当前完成度：中等。

已完成：

- 读取 `SemanticVehicle.probs_lat_behaviors`；
- 支持 LK / LCL / LCR 多模态周车轨迹；
- 将模态概率传入 risk grid；
- MVP-10 已新增 `[ego_behavior_index][vehicle_id][mode]` 条件化风险输入通道。

不足：

- 当前概率来源主要是已有横向行为概率，不是新设计的学习式预测模型；
- 非 argmax 补充模态仍以规则式开环预测为主，不是完整博弈式交互响应模型；
- 真实场景下仍需用 `risk_grid_behavior_variant_*` 指标证明不同 ego candidate 的风险图确实出现差异。

论文建议表述：

```text
本文在 EPSILON 语义行为概率基础上，实现了面向 SSC 风险图的多模态轨迹传播接口，并建立了 ego candidate 条件化的风险图数据通道。
```

不建议表述为：

```text
提出完整交互式多智能体概率预测模型。
```

### 4.2 大创新点 2：概率风险 SSC 占据图与风险时空走廊

当前完成度：较强。

已完成：

- `p_3d_risk_grid_`；
- risk reset / stats / CSV / RViz / query；
- 多车、多模态概率叠加并截断；
- risk threshold 可投影到 binary map 影响 corridor。

这是目前最扎实、最适合写成论文核心贡献的部分。

不足：

- 静态障碍物仍主要在 binary map 中表达，不进入概率 risk field；
- risk-aware corridor 默认关闭，需要实验配置明确开启；
- 仍需真实场景日志验证风险阈值对 corridor 的安全收益和规划耗时影响。

### 4.3 大创新点 3：风险约束 / 风险代价下的轨迹优化

当前完成度：中等偏弱。

已完成：

- QP 候选轨迹 risk exposure 查询；
- 候选 risk score；
- 可选风险重选；
- CSV 可记录 selected / baseline / adaptive / fallback 指标。

不足：

- 没有修改 QP Hessian、linear term 或约束；
- 不是真正的 QP 内部风险代价；
- 当前是 QP 后候选风险暴露评价与重选，论文表述需要避免写成“QP risk cost”。

论文建议表述：

```text
在 QP 候选轨迹生成后引入风险暴露评价与候选重排序机制。
```

不建议表述为：

```text
提出风险代价 QP 优化器。
```

### 4.4 小创新点 4：风险图可视化、CSV 导出与实验评价工具链

当前完成度：较强。

已完成：

- RiskGridStats；
- risk grid CSV；
- risk exposure CSV；
- RViz MarkerArray；
- 离线 summary CSV / JSON / 可选图表脚本；
- MVP 报告已入库。

不足：

- MVP-9 是汇总工具，不是自动实验运行平台；
- 缺少真实场景数据和完整消融表。

### 4.5 小创新点 5：场景自适应风险权重与安全兜底机制

当前完成度：中等。

已完成：

- 高速、变道、高交互风险启发式权重调节；
- fallback 触发与候选切换；
- 默认关闭，避免破坏 baseline。

不足：

- 场景识别规则较简单；
- fallback 不是 emergency braking；
- 缺少 TTC / 最小距离等安全指标联动。

## 5. 中文注释覆盖评估

对关键文件进行注释行数抽样统计：

| 文件 | 总行数 | 注释行 | 中文注释行 | risk/MVP 相关行 |
|---|---:|---:|---:|---:|
| `ssc_map.cc` | 1477 | 511 | 388 | 89 |
| `ssc_planner.cc` | 1624 | 462 | 345 | 259 |
| `map_adapter.cc` | 279 | 102 | 46 | 4 |
| `ssc_visualizer.cc` | 516 | 135 | 102 | 46 |
| `ssc_map.h` | 501 | 309 | 232 | 57 |
| `ssc_planner.h` | 484 | 296 | 234 | 64 |
| `map_interface.h` | 159 | 115 | 67 | 4 |
| `ssc_config.proto` | 100 | 24 | 24 | 46 |
| `ssc_config.pb.txt` | 80 | 14 | 14 | 36 |
| `risk_experiment_report.py` | 255 | 16 | 13 | 41 |

结论：

- 新增风险相关代码的中文注释总体充足，且覆盖了物理意义、默认开关、风险边界和论文用途；
- 原有 SSC 主流程也补充了大量中文解释；
- 需要修正少量“risk grid 不影响规划”的过期注释；
- 建议后续修 bug 时继续保持“函数级中文说明 + 关键分支中文注释 + 默认行为说明”的风格。

## 6. 白盒与单元级测试结果

### 6.1 构建验证

命令：

```bash
cd /home/ros/work/graduateworkcc
colcon build --packages-up-to ssc_planner --symlink-install
```

结果：

```text
Summary: 7 packages finished [1.51s]
```

结论：通过。

### 6.2 包测试

命令：

```bash
cd /home/ros/work/graduateworkcc
colcon test --packages-select ssc_planner --event-handlers console_direct+
```

结果：

```text
Summary: 1 package finished [0.36s]
```

结论：通过。

限制：

当前 `ssc_planner` 包内未发现专门覆盖 risk grid / risk exposure / fallback 的 C++ 单元测试，因此该命令只能证明包测试入口不失败，不能证明风险逻辑完全正确。

### 6.3 Python 工具语法检查

命令：

```bash
python3 -m py_compile util/ssc_planner/scripts/risk_experiment_report.py
```

结果：通过。

### 6.4 MVP-9 离线脚本合成 CSV 单元级测试

测试内容：

- 构造 2 行 risk grid CSV；
- 构造 2 行 risk exposure CSV；
- 覆盖 selected / baseline / adaptive / high_interaction / safety_fallback 字段；
- 断言 summary JSON 中关键统计项正确。

结果：

```text
synthetic_risk_experiment_report_test=PASS
summary_csv=/tmp/epsilon_risk_unit_oNBUww/out/summary.csv
summary_json=/tmp/epsilon_risk_unit_oNBUww/out/summary.json
```

结论：通过。

## 7. 推荐修复与补强优先级

### P0：必须优先修复

1. 低速 primitive 与 Bezier 风险评估错配：已修复；
2. 多模态风险图缺少 deterministic fallback 或按车补齐逻辑：已修复；
3. 多模态预测 horizon 与 SSC map horizon 不一致：已通过配置化和 map horizon 自动推导修复；
4. ego candidate 条件化多模态风险场：已完成 SSC 内部数据通道；完整博弈式交互响应仍属于后续增强。

### P1：建议下一轮修复

1. `safety_fallback_high_risk_hits_threshold == 0` 恒触发问题：已修复；
2. RiskGridStats CSV 增加行为索引和时间戳：已修复；
3. 修正过期中文注释：已部分修复，后续继续随代码演进维护；
4. RViz 默认配置加入 `risk_grid_vis`：配置文件已补齐，仍待实际运行截图验收。

### P2：论文强化项

1. 做真实场景消融实验；
2. 输出 baseline / risk corridor / risk exposure / adaptive / fallback 对比；
3. 统计碰撞、最小距离、TTC、risk exposure、规划耗时、jerk、acc；
4. 若时间允许，再考虑真正接入 QP 内部 risk cost。

## 8. 论文写法建议

推荐论文贡献写法：

```text
1. 构建了并行概率风险占据图，将周车行为概率传播到 SSC 时空地图；
2. 提出基于风险阈值的风险感知时空走廊构建方法；
3. 设计了候选轨迹风险暴露评价、风险重选和安全兜底机制；
4. 构建了 CSV/RViz/离线汇总一体化实验工具链。
```

避免夸大写法：

```text
1. 不要声称已经实现完整交互式概率预测模型；
2. 不要声称已经把风险代价写入 QP 目标函数；
3. 不要声称 safety fallback 等价于紧急制动或形式化安全控制；
4. 不要把 MVP-9 称为自动实验平台。
```

如果后续补完剩余 P0 项和真实消融实验，该工程对硕士论文已经足够扎实；如果不补真实实验，则工程量够，但论文说服力不足。

## 9. 当前状态

当前仓库存在 MVP-10 及可靠性补强相关未提交改动，建议与本轮实现作为同一阶段提交：

```bash
git add util/ssc_planner docs/EPSILON_MVP10_ego_conditioned_multimodal_risk_grid_implementation_report.md \
  docs/EPSILON_risk_aware_ssc_whitebox_review_and_innovation_audit.md \
  docs/EPSILON_risk_aware_ssc_work_and_innovation_summary.md
git commit -m "feat(ssc): condition multimodal risk grid on ego candidates"
```
