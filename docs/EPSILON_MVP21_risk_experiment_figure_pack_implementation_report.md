# MVP-21 实施报告：风险实验论文图表包生成器

## 1. 目标

MVP-21 目标是在 MVP-20 输出完整性校验之后，新增论文图表包生成器。
该工具读取 `risk_experiment_matrix.py` 生成的 `matrix.csv`，整理常用论文指标，
输出可复核的长表 CSV、manifest，并在环境具备 `matplotlib` 时生成 PNG 图表。

## 2. 修改范围

允许修改：

```text
util/ssc_planner/scripts/risk_experiment_figure_pack.py
util/ssc_planner/CMakeLists.txt
util/ssc_planner/scripts/README_risk_experiment_suite.md
docs/EPSILON_MVP21_risk_experiment_figure_pack_implementation_report.md
```

禁止修改：

```text
SSC / EUDM / MPDM / QP / phy_simulator 核心算法
risk grid / risk exposure / scripted telemetry CSV 字段语义
batch / suite 的运行命令语义
```

## 3. 新增脚本

```text
util/ssc_planner/scripts/risk_experiment_figure_pack.py
```

支持输入：

```text
1. --matrix 指定单个或多个 matrix.csv；
2. --input-root 自动发现 batch 输出目录中的 matrix/matrix.csv；
3. --input-root 自动发现 scenario suite 输出目录中的 matrix_by_scenario/*.csv；
4. --metric 追加自定义论文指标。
```

输出：

```text
plot_values.csv
figure_manifest.json
figures/*.png
```

说明：

```text
plot_values.csv 始终输出；
figure_manifest.json 始终输出；
figures/*.png 仅在 matplotlib 可用时输出。
```

## 4. 默认绘图指标

```text
sum_risk_mean
max_risk_max
nonzero_cells_mean
cycle_selected_exposure_sum_mean
cycle_selected_exposure_max_max
cycle_baseline_exposure_sum_mean
safety_fallback_triggered_cycles
safety_fallback_switched_cycles
adaptive_enabled_cycles
scripted_actor_gap_min
scripted_actor_idm_acceleration_min
scripted_actor_command_acceleration_min
```

## 5. 推荐调用链

```text
risk_experiment_batch.py / risk_experiment_scenario_suite.py
  -> risk_experiment_verify_outputs.py
  -> risk_experiment_figure_pack.py
  -> 论文表格和图
```

## 6. 不改动项

```text
1. 不启动 ROS；
2. 不运行规划器；
3. 不修改实验原始 CSV；
4. 不修改 summary/matrix 语义；
5. 不改变任何规划行为。
```

## 7. 编译与验证结果

```bash
cd /home/ros/work/graduateworkcc/src/EPSILON
python3 -m py_compile util/ssc_planner/scripts/risk_experiment_figure_pack.py

python3 util/ssc_planner/scripts/risk_experiment_figure_pack.py \
  --input-root /tmp/epsilon_mvp20_batch_smoke \
  --output-dir /tmp/epsilon_mvp21_figure_pack_batch

python3 util/ssc_planner/scripts/risk_experiment_figure_pack.py \
  --matrix baseline=/tmp/epsilon_mvp20_batch_smoke/matrix/matrix.csv \
  --metric scripted_actor_command_acceleration_max \
  --output-dir /tmp/epsilon_mvp21_figure_pack_matrix

cd /home/ros/work/graduateworkcc
colcon build --packages-up-to ssc_planner --symlink-install
```

实际结果：

```text
1. python3 -m py_compile util/ssc_planner/scripts/risk_experiment_figure_pack.py 通过；
2. git diff --check 通过；
3. --input-root /tmp/epsilon_mvp20_batch_smoke 生成 figure_manifest.json / plot_values.csv / 6 张 PNG；
4. --matrix baseline=... 并追加 scripted_actor_command_acceleration_max 生成 figure_manifest.json / plot_values.csv / 7 张 PNG；
5. 抽查 manifest 中 image_paths 均存在；
6. colcon build --packages-up-to ssc_planner --symlink-install 通过，仅保留既有 OOQP include warning。
```

当前 smoke 数据说明：

```text
/tmp/epsilon_mvp20_batch_smoke 只有 baseline 一组，且 risk exposure 字段为空，
因此 plot_values.csv 只包含当前 matrix 中有数值的指标行。
```

## 8. 论文价值

MVP-21 把实验工具链从“数据完整”推进到“图表可直接复现”：

```text
1. 每个图表都能追溯到 matrix.csv；
2. plot_values.csv 是论文绘图的稳定中间表；
3. figure_manifest.json 记录输入 matrix、指标列表和生成图片；
4. 可以批量生成不同场景/消融组的风险指标图。
```

## 9. 当前风险

```text
1. PNG 输出依赖 matplotlib；缺失时仍保留 CSV/JSON；
2. 默认图表是横向柱状图，复杂论文版式可能需要后续 LaTeX/pgfplots 或 notebook 二次加工；
3. 当前指标来自已有 matrix 字段，若后续新增指标需要同步扩展默认列表。
```

## 10. 推荐提交信息

```text
feat(ssc): generate risk experiment figure packs
```

## 11. 推荐 tag

```text
v0.21.0-mvp21-risk-experiment-figure-pack
```
