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
  --output-dir /tmp/epsilon_risk_experiment_report
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
risk_exposure.exposure_max      -> 轨迹最大单点风险
risk_exposure.exposure_sum      -> 轨迹风险暴露总量
risk_exposure.high_risk_hits    -> 高风险采样点数量
safety_fallback_switched_rows   -> 安全兜底触发并切换次数
```

## 7. 注意事项

脚本不参与 ROS 编译，不改变规划行为。它只读取 CSV 并写入离线报告文件，适合在每次实验运行后单独执行。
