# MVP-9 实施报告

## 1. 目标

MVP-9 构建离线实验平台与消融实验工具链，将前序 MVP 导出的风险图统计 CSV 和候选轨迹风险暴露 CSV 汇总为论文可用的表格和趋势图。

## 2. 修改范围

本次只新增以下 2 个仓库内文件：

```text
util/ssc_planner/scripts/risk_experiment_report.py
util/ssc_planner/scripts/README_risk_experiment_suite.md
```

另新增本地实施报告：

```text
docs/EPSILON_MVP9_experiment_suite_implementation_report.md
```

## 3. 新增脚本

```text
risk_experiment_report.py
```

核心能力：

```text
读取 /tmp/epsilon_risk_grid_stats.csv
读取 /tmp/epsilon_mvp6_risk_exposure.csv
输出 summary.csv
输出 summary.json
可选输出 matplotlib PNG 趋势图
记录 experiment_name / scenario_name / git_ref 追溯信息
区分 selected 与 baseline 候选风险统计
```

## 4. 调用链

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

## 5. 不改动项

本次不改动任何规划运行时代码，不影响：

```text
SSC map
QP
corridor
control
prediction
EUDM / MPDM
```

## 6. 验证结果

验证命令：

```bash
python3 util/ssc_planner/scripts/risk_experiment_report.py --help
python3 util/ssc_planner/scripts/risk_experiment_report.py \
  --risk-grid-csv /tmp/nonexistent_risk_grid.csv \
  --risk-exposure-csv /tmp/nonexistent_risk_exposure.csv \
  --output-dir /tmp/epsilon_risk_experiment_report_smoke \
  --experiment-name smoke \
  --scenario-name empty_input \
  --git-ref local
```

预期：

```text
即使输入 CSV 暂不存在，也能生成空 summary.csv / summary.json
```

额外语法验证：

```bash
python3 -m py_compile util/ssc_planner/scripts/risk_experiment_report.py
```

该脚本不参与 C++ 编译，但仍执行：

```bash
cd /home/ros/work/graduateworkcc
colcon build --packages-up-to ssc_planner --symlink-install
```

用于确认新增离线脚本没有影响 `ssc_planner` 包构建。

## 7. 推荐消融组

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

## 8. Git commit 信息

已完成提交：

```text
feat(ssc): add offline risk experiment suite
feat(ssc): enrich risk experiment summaries
```

其中 `feat(ssc): enrich risk experiment summaries` 补充了实验名、场景名、git ref
追溯字段，并增加 selected / baseline / adaptive / safety fallback 分组汇总指标。

## 9. tag 信息

已打 tag：

```text
v0.9.0-mvp9-experiment-suite
```

注意：`v0.9.0-mvp9-experiment-suite` 对应 MVP-9 初始离线实验工具链；随后
`dev/risk-aware-ssc` 上追加了 summary 字段增强提交。为了不移动已发布 tag，
增强提交建议使用补充 tag：

```text
v0.9.1-mvp9-experiment-summary
```

## 10. 当前风险

当前 MVP-9 是离线工具链，不提供在线实验调度器，也不自动启动仿真场景。后续论文实验仍需要手动或脚本化运行不同配置组，再用本工具汇总 CSV。

## 11. 下一步计划

MVP-0 到 MVP-9 已形成完整工程链路。下一步建议进入论文实验阶段：固定场景、固定随机种子、记录 commit/tag、运行消融组并生成图表。
