# MVP-10 实施报告：Ego Candidate 条件化多模态风险图

## 1. 目标

将 SSC risk grid 的多模态周车输入从单份全局结构：

```text
[vehicle_id][mode]
```

升级为按自车候选行为分组的结构：

```text
[ego_behavior_index][vehicle_id][mode]
```

从而避免 LK / LCL / LCR 自车候选共用同一份周车多模态风险场。

## 2. 修改范围

本阶段只修改 `util/ssc_planner` 内部接口和调用链：

- `util/ssc_planner/include/ssc_planner/map_interface.h`
- `util/ssc_planner/include/ssc_planner/map_adapter.h`
- `util/ssc_planner/src/ssc_planner/map_adapter.cc`
- `util/ssc_planner/include/ssc_planner/ssc_planner.h`
- `util/ssc_planner/src/ssc_planner/ssc_planner.cc`
- `util/ssc_planner/src/ssc_planner/ssc_map.cc`
- `util/ssc_planner/scripts/risk_experiment_report.py`
- `util/ssc_planner/scripts/README_risk_experiment_suite.md`
- `docs/EPSILON_risk_aware_ssc_work_and_innovation_summary.md`

## 3. 新增结构体 / 函数

新增类型：

```cpp
BehaviorConditionedMultiModalSurroundingTrajectories
```

新增接口：

```cpp
GetBehaviorConditionedMultiModalSurroundingTrajectories()
```

新增 Planner 成员：

```cpp
behavior_conditioned_multimodal_surround_trajs_
behavior_conditioned_multimodal_surround_trajs_fs_
```

## 4. 调用链

```text
SscPlanner::RunOnce()
  -> map_itf_->GetBehaviorConditionedMultiModalSurroundingTrajectories()
  -> SscPlanner::StateTransformForInputData()
  -> behavior_conditioned_multimodal_surround_trajs_fs_
  -> SscMap::ConstructSscMap(..., multimodal_trajs_for_behavior, ...)
  -> SscMap::FillDynamicPartProbabilistic()
  -> p_3d_risk_grid_
```

## 5. 关键实现说明

`SscPlannerAdapter` 会先生成原有 LK/LCL/LCR 多模态周车轨迹，再用
`ego_behavior().surround_trajs[ego_behavior_index]` 覆盖对应车辆的 argmax 模态。

这样做的含义是：

- 每个 ego candidate 都有独立的周车 deterministic 轨迹入口；
- 非 argmax 的补充模态仍复用规则式开环预测；
- 当前实现是 SSC 内部条件化风险图通道，不是完整博弈式交互预测器。

## 6. 同步修复的技术风险

本阶段同时修复了若干 SSC 内部风险链路问题：

1. `FillStaticPart()` 使用 SSC map 绝对时间原点填充静态障碍物，避免 ROS 时间戳非零时静态障碍落到负时间层。
2. 多模态预测部分成功时，`FillDynamicPartProbabilistic()` 会用 deterministic 轨迹补齐剩余概率质量，避免高概率失败模态被漏掉。
3. `RiskGridStats` CSV 增加 `risk_source_vehicles` 和 `risk_source_modes`，
   用于追踪每次构图实际使用了多少周车和模态。
4. RViz `risk_grid_vis` 优先显示最终 selected candidate 对应的 risk grid
   snapshot，避免误看 `SscMap` 中最后一个 behavior 构图残留的风险图。
5. 离线实验脚本新增同一 planning cycle 内不同 behavior 的风险图差异度统计，
   用于量化验证 ego candidate 条件化风险图是否产生了可观测差异。

## 7. 不改动项

本阶段不修改：

- `core/semantic_map_manager`
- `core/motion_predictor`
- `core/behavior_planner`
- `util/eudm_planner`
- QP Hessian / objective
- 控制器输出接口

## 8. 验收标准

建议验收：

```bash
cd /home/ros/work/graduateworkcc
colcon build --packages-up-to ssc_planner --symlink-install
colcon test --packages-select ssc_planner --event-handlers console_direct+
```

并检查：

```bash
rg -n "BehaviorConditionedMultiModal|GetBehaviorConditioned|behavior_conditioned_multimodal" \
  src/EPSILON/util/ssc_planner
```

运行实验时，应确认同一 planning cycle 内不同 `behavior_index` 的 risk grid stats /
risk exposure 结果可以出现差异。

`/tmp/epsilon_risk_grid_stats.csv` 中可检查：

```text
behavior_index
risk_source_vehicles
risk_source_modes
nonzero_cells
sum_risk
```

离线汇总时，可使用：

```bash
python3 util/ssc_planner/scripts/risk_experiment_report.py \
  --risk-grid-csv /tmp/epsilon_risk_grid_stats.csv \
  --risk-exposure-csv /tmp/epsilon_mvp6_risk_exposure.csv \
  --output-dir /tmp/epsilon_risk_experiment_report \
  --experiment-name mvp10 \
  --scenario-name ego_conditioned_risk \
  --git-ref local
```

重点检查：

```text
risk_grid_behavior_multi_behavior_groups
risk_grid_behavior_variant_groups
risk_grid_behavior_variant_ratio
risk_grid_behavior_unique_signature_groups
risk_grid_behavior_sum_risk_range_max
risk_grid_behavior_max_risk_range_max
```

其中 `risk_grid_behavior_variant_groups > 0` 表示至少存在一个 planning cycle
中，不同 ego candidate 的 risk grid 摘要数值不同；若该值长期为 0，应回查
场景中周车预测是否真的随 `ego_behavior().surround_trajs` 变化。

RViz 验收时，`/vis/agent_0/ssc/risk_grid_vis` 默认应对应最终选中候选风险图；
若当前帧没有候选快照，则会回退显示 live `SscMap::risk_grid()`。

## 9. 论文表述边界

可以表述为：

```text
建立了 ego candidate 条件化的多模态风险图数据通道。
```

不建议表述为：

```text
已实现完整博弈交互预测。
```

## 10. 推荐 Commit

```bash
git add util/ssc_planner docs/EPSILON_MVP10_ego_conditioned_multimodal_risk_grid_implementation_report.md \
  docs/EPSILON_risk_aware_ssc_whitebox_review_and_innovation_audit.md \
  docs/EPSILON_risk_aware_ssc_work_and_innovation_summary.md
git commit -m "feat(ssc): condition risk grid on ego candidates"
```
