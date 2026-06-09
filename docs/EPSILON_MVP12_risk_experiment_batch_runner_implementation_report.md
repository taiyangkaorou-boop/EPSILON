# MVP-12 实施报告：Risk Experiment Batch Runner

## 1. 目标

MVP-12 的目标是把 MVP-11 的手工实验流程固化为可复现的批量入口，用于论文第 7 章消融实验。

本阶段新增：

```text
批量实验 dry-run 计划
显式 --execute 执行模式
每组 raw CSV 归档
每组 summary 自动生成
多组 matrix 自动汇总
manifest.json 追溯记录
```

## 2. 修改范围

允许修改：

- `util/ssc_planner/scripts/risk_experiment_batch.py`
- `util/ssc_planner/scripts/README_risk_experiment_suite.md`
- `util/ssc_planner/CMakeLists.txt`
- `docs/EPSILON_MVP12_risk_experiment_batch_runner_implementation_report.md`

禁止修改：

- SSC 核心规划逻辑；
- EUDM / MPDM 行为决策逻辑；
- risk grid 填图逻辑；
- corridor / QP / safety fallback 逻辑；
- 默认 baseline 配置。

## 3. 新增脚本

新增：

```text
util/ssc_planner/scripts/risk_experiment_batch.py
```

默认实验组：

```text
baseline -> ssc_config.pb.txt
observe  -> ssc_config_risk_observe.pb.txt
corridor -> ssc_config_risk_corridor.pb.txt
full     -> ssc_config_risk_full.pb.txt
```

默认行为是 dry-run，只生成：

```text
run_plan.sh
manifest.json
```

不会启动 ROS。只有显式传入：

```text
--execute
```

才会运行实验命令。

## 4. 调用链

```text
risk_experiment_batch.py
  -> 清理 /tmp/epsilon_risk_grid_stats.csv
  -> 清理 /tmp/epsilon_mvp6_risk_exposure.csv
  -> 执行一组实验命令
  -> 归档 raw_risk_grid_stats.csv / raw_risk_exposure.csv
  -> 调用 risk_experiment_report.py
  -> 所有成功组调用 risk_experiment_matrix.py
  -> 写 manifest.json
```

## 5. 数据完整性策略

为避免“实验命令成功但没有产出 CSV”被误当作有效实验，MVP-12 增加 manifest 状态：

```text
status = dry_run       只生成计划
status = ok            命令成功且 risk grid CSV 存在
status = data_missing  命令成功但 risk grid CSV 缺失
status = run_failed    实验命令失败
status = report_failed 单组 summary 生成失败
```

同时记录：

```text
risk_grid_present
risk_exposure_present
```

其中 baseline 组可能没有 exposure CSV，但必须有 risk grid CSV 才能进入 matrix。

## 6. 使用示例

生成计划：

```bash
python3 util/ssc_planner/scripts/risk_experiment_batch.py \
  --output-root /tmp/epsilon_batch_highway_v1 \
  --scenario-name highway_v1.0 \
  --duration-sec 60
```

执行默认四组实验：

```bash
python3 util/ssc_planner/scripts/risk_experiment_batch.py \
  --execute \
  --output-root /tmp/epsilon_batch_highway_v1 \
  --scenario-name highway_v1.0 \
  --duration-sec 60
```

使用外部场景脚本：

```bash
python3 util/ssc_planner/scripts/risk_experiment_batch.py \
  --execute \
  --output-root /tmp/epsilon_batch_cut_in \
  --scenario-name cut_in \
  --run-command-template 'source {setup_bash} && ./run_one_case.sh {experiment_name} {config_path}'
```

## 7. 验证结果

已执行：

```text
python3 -m py_compile risk_experiment_batch.py risk_experiment_report.py risk_experiment_matrix.py -> 通过
risk_experiment_batch.py --help -> 通过
risk_experiment_batch.py dry-run -> 生成 run_plan.sh / manifest.json
合成 CSV execute smoke -> 生成 raw CSV / summary / matrix / manifest
缺失 CSV execute smoke -> 返回非 0 且仍写出 manifest.json
colcon build --packages-up-to planning_integrated --symlink-install -> 通过
```

构建仅保留既有 warning：

```text
OOQP include directory does not exist
semantic_map_manager::RosAdapter 成员初始化顺序 warning
thirdparty tk_spline unused-function warning
```

上述 warning 不由 MVP-12 引入。

## 8. 不改动项

MVP-12 不改变任何规划行为。它只把已有实验配置和离线汇总脚本串成可复现流程。

## 9. Git 信息

推荐 commit：

```bash
feat(ssc): add risk experiment batch runner
```

推荐 tag：

```bash
v0.12.0-mvp12-risk-experiment-batch-runner
```
