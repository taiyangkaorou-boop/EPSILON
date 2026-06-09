# EPSILON Risk-Aware SSC 实验工具链

## 1. 目标

该目录提供 MVP-9 离线实验汇总脚本，用于把规划运行时导出的 CSV 转换为论文实验表格和趋势图。

## 2. 输入文件

默认读取：

```text
/tmp/epsilon_risk_grid_stats.csv
/tmp/epsilon_mvp6_risk_exposure.csv
```

对应来源：

```text
MVP-1B: RiskGridStats CSV
MVP-6: 候选轨迹 risk exposure CSV
MVP-7: adaptive risk weight 上下文字段
MVP-8: safety fallback 上下文字段
```

## 3. 运行方式

```bash
cd /home/ros/work/graduateworkcc/src/EPSILON
python3 util/ssc_planner/scripts/risk_experiment_report.py \
  --risk-grid-csv /tmp/epsilon_risk_grid_stats.csv \
  --risk-exposure-csv /tmp/epsilon_mvp6_risk_exposure.csv \
  --output-dir /tmp/epsilon_risk_experiment_report \
  --experiment-name safety_fallback \
  --scenario-name cut_in \
  --git-ref v0.9.0-mvp9-experiment-suite
```

## 4. 输出文件

必定输出：

```text
summary.csv
summary.json
```

如果本机安装了 `matplotlib`，额外输出：

```text
risk_grid_sum.png
risk_grid_nonzero.png
trajectory_exposure_max.png
trajectory_risk_score.png
```

## 5. 推荐消融实验分组

```text
Baseline SSC
Risk grid only
Risk grid + behavior probability
Risk grid + multimodal trajectories
Risk-aware corridor
Risk exposure selection
Adaptive risk weight
Safety fallback
```

## 6. 论文指标对应

```text
risk_grid.sum_risk              -> 风险图总体强度
risk_grid.nonzero_cells         -> 风险图空间覆盖范围
risk_grid.active_time_layers    -> 风险在预测时间轴上的持续范围
risk_grid.risk_source_vehicles  -> 本次构图参与风险填充的周车数量
risk_grid.risk_source_modes     -> 本次构图参与风险填充的周车模态数量
risk_grid_behavior_variant_groups -> 同一 planning cycle 内不同 behavior 风险图数值不同的周期数
risk_grid_behavior_variant_ratio -> 存在多 behavior 风险图的周期中，风险图数值不同的比例
risk_grid_behavior_unique_signature_groups -> 同一 planning cycle 内风险图统计签名不同的周期数
risk_grid_behavior_sum_risk_range.* -> 同一 planning cycle 内不同 behavior 的 sum_risk 差异范围统计
risk_grid_behavior_max_risk_range.* -> 同一 planning cycle 内不同 behavior 的 max_risk 差异范围统计
risk_grid_behavior_nonzero_cells_range.* -> 同一 planning cycle 内不同 behavior 的 nonzero_cells 差异范围统计
risk_exposure.exposure_max      -> 轨迹最大单点风险
risk_exposure.exposure_sum      -> 轨迹风险暴露总量
risk_exposure.high_risk_hits    -> 高风险采样点数量
selected.*                      -> 最终选中轨迹的风险统计
baseline.*                      -> 原始 SSC baseline 候选的风险统计
adaptive_enabled_rows           -> 自适应风险权重参与评价的记录数
high_interaction_risk_rows      -> 高交互风险场景记录数
safety_fallback_switched_rows   -> 安全兜底触发并切换的候选行数
cycle_selected.*                -> 按 planning cycle 聚合后的最终选中轨迹风险统计
cycle_baseline.*                -> 按 planning cycle 聚合后的 baseline 候选风险统计
safety_fallback_triggered_cycles -> 安全兜底触发的 planning cycle 数
safety_fallback_switched_cycles -> 安全兜底触发并切换的 planning cycle 数
```

## 7. 注意事项

脚本不参与 ROS 编译，不改变规划行为。它只读取 CSV 并写入离线报告文件，适合在每次实验运行后单独执行。

`risk_exposure` CSV 是每个候选轨迹一行，因此 `*_rows` 指候选行数，不等于
planning cycle 次数。论文中统计 fallback/adaptive 触发次数时，优先使用
`*_cycles` 和 `cycle_selected.*` / `cycle_baseline.*` 指标。

建议每组消融实验都显式填写 `--experiment-name`、`--scenario-name` 和 `--git-ref`，
这样 `summary.csv` / `summary.json` 能直接追溯到实验配置和代码版本。
