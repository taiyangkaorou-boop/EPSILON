# EPSILON Risk-Aware SSC 实验工具链

## 1. 目标

该目录提供 MVP-9/MVP-11/MVP-12 实验工具，用于把规划运行时导出的 CSV
转换为论文实验表格、趋势图和多实验消融矩阵，并支持批量编排消融实验。

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

## 3. 推荐实验配置

MVP-11 新增三份可选 SSC 配置。默认 `ssc_config.pb.txt` 仍保持 baseline 行为，
下面配置只在 launch 时显式传入，便于回滚和做消融对比。

```text
ssc_config.pb.txt                 -> Baseline SSC，风险功能默认关闭
ssc_config_risk_observe.pb.txt    -> 只记录 risk grid / exposure，不改变最终候选
ssc_config_risk_corridor.pb.txt   -> 开启 risk-aware corridor，不开启候选重选
ssc_config_risk_full.pb.txt       -> 开启 corridor + risk exposure reselect + adaptive + fallback
```

规划节点可通过 `ssc_config_path` 覆盖配置，例如：

```bash
ros2 launch planning_integrated test_ssc_with_eudm_ros_launch.py \
  playground:=highway_v1.0 \
  ssc_config_path:=/home/ros/work/graduateworkcc/install/ssc_planner/share/ssc_planner/config/ssc_config_risk_observe.pb.txt
```

每次实验开始前建议清理旧 CSV，避免跨实验混写：

```bash
rm -f /tmp/epsilon_risk_grid_stats.csv /tmp/epsilon_mvp6_risk_exposure.csv
```

推荐每一组实验都按以下顺序执行：

```text
清理旧 CSV
-> 运行一组指定配置
-> 立即用 risk_experiment_report.py 导出该组 summary
-> 再清理 CSV 并运行下一组
```

这样可以避免 observe / corridor / full 等不同实验组写入同一个 `/tmp` CSV 后互相污染。

## 4. 批量实验编排

MVP-12 新增 `risk_experiment_batch.py`，用于固化如下流程：

```text
清理 /tmp CSV
-> 运行一组实验命令
-> 归档 raw_risk_grid_stats.csv / raw_risk_exposure.csv
-> 调用 risk_experiment_report.py 生成 summary
-> 所有成功组调用 risk_experiment_matrix.py 生成 matrix
```

默认只生成 dry-run 计划，不启动 ROS：

```bash
python3 util/ssc_planner/scripts/risk_experiment_batch.py \
  --output-root /tmp/epsilon_batch_highway_v1 \
  --scenario-name highway_v1.0 \
  --duration-sec 60
```

输出：

```text
manifest.json
run_plan.sh
baseline/
observe/
corridor/
full/
```

确认 `run_plan.sh` 无误后，可以显式执行：

```bash
python3 util/ssc_planner/scripts/risk_experiment_batch.py \
  --execute \
  --output-root /tmp/epsilon_batch_highway_v1 \
  --scenario-name highway_v1.0 \
  --duration-sec 60
```

如果需要接入外部仿真启动脚本，可使用命令模板：

```bash
python3 util/ssc_planner/scripts/risk_experiment_batch.py \
  --execute \
  --output-root /tmp/epsilon_batch_custom \
  --scenario-name cut_in \
  --run-command-template 'source {setup_bash} && ./run_one_case.sh {experiment_name} {config_path}'
```

常用占位符：

```text
{experiment_name}
{scenario_name}
{config_path}
{playground}
{planner_backend}
{launch_file}
{duration_sec}
{risk_grid_csv}
{risk_exposure_csv}
{output_dir}
{setup_bash}
```

`manifest.json` 会记录每组实验命令、配置路径、返回码、CSV 是否存在、summary 路径和 matrix 命令。
如果实验命令返回成功但没有生成 risk grid CSV，该组会标记为 `data_missing`，不会静默进入最终矩阵。

## 5. 单组实验汇总

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

## 6. 多组实验矩阵

完成多组实验后，可用 `risk_experiment_matrix.py` 汇总多个 `summary.csv`：

```bash
python3 util/ssc_planner/scripts/risk_experiment_matrix.py \
  --summary baseline=/tmp/epsilon_reports/baseline/summary.csv \
  --summary observe=/tmp/epsilon_reports/risk_observe/summary.csv \
  --summary corridor=/tmp/epsilon_reports/risk_corridor/summary.csv \
  --summary full=/tmp/epsilon_reports/risk_full/summary.csv \
  --output-dir /tmp/epsilon_reports/matrix
```

输出：

```text
matrix.csv
matrix.json
```

`matrix.csv` 每行对应一个实验组，列为常用论文指标，适合直接导入表格或绘图脚本。

## 7. 输出文件

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

## 8. 推荐消融实验分组

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

## 9. 论文指标对应

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

## 10. 注意事项

脚本不参与 ROS 编译，不改变规划行为。它只读取 CSV 并写入离线报告文件，适合在每次实验运行后单独执行。

`risk_exposure` CSV 是每个候选轨迹一行，因此 `*_rows` 指候选行数，不等于
planning cycle 次数。论文中统计 fallback/adaptive 触发次数时，优先使用
`*_cycles` 和 `cycle_selected.*` / `cycle_baseline.*` 指标。

建议每组消融实验都显式填写 `--experiment-name`、`--scenario-name` 和 `--git-ref`，
这样 `summary.csv` / `summary.json` 能直接追溯到实验配置和代码版本。
