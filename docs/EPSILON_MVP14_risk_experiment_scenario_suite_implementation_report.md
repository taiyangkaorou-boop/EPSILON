# MVP-14 实施报告：Risk Experiment Scenario Suite

## 1. 目标

MVP-14 的目标是在 MVP-13 闭环实验入口之上，增加“多场景批量复现实验”能力。

本阶段新增：

```text
多 playground 实验编排脚本
场景资源完整性检查
跨场景 dry-run 计划
跨场景 execute 调用
场景级 manifest
场景级 matrix 归档
```

## 2. 修改范围

允许修改：

- `util/ssc_planner/scripts/risk_experiment_scenario_suite.py`
- `util/ssc_planner/scripts/README_risk_experiment_suite.md`
- `util/ssc_planner/CMakeLists.txt`
- `docs/EPSILON_MVP14_risk_experiment_scenario_suite_implementation_report.md`

禁止修改：

- playground 原始地图和车辆配置；
- SSC / EUDM / MPDM / QP / fallback 规划逻辑；
- MVP-13 闭环 launch 行为；
- 默认 baseline 配置。

## 3. 新增脚本

新增：

```text
util/ssc_planner/scripts/risk_experiment_scenario_suite.py
```

默认场景：

```text
highway_v1.0
highway_lite
ring_small_v1.0
ring_tiny_v1.0
```

脚本会校验每个场景是否包含：

```text
vehicle_set.json
obstacles_norm.json
lane_net_norm.json
agent_config.json
```

缺失任一文件时立即报错，避免实验运行到一半才失败。

## 4. 调用链

```text
risk_experiment_scenario_suite.py
  -> 遍历多个 playground
  -> 对每个 playground 调用 risk_experiment_batch.py
      -> 调用 risk_experiment_closed_loop_launch.py
      -> 生成单场景 raw CSV / summary / matrix / manifest
  -> 复制每个场景的 matrix.csv 到 matrix_by_scenario/<scenario>.csv
  -> 写 scenario_suite_manifest.json
```

默认只生成：

```text
scenario_run_plan.sh
scenario_suite_manifest.json
```

只有显式传入：

```text
--execute
```

才会启动闭环 ROS 实验。

## 5. 使用示例

生成默认四场景计划：

```bash
python3 util/ssc_planner/scripts/risk_experiment_scenario_suite.py \
  --output-root /tmp/epsilon_scenario_suite \
  --duration-sec 60
```

执行默认四场景实验：

```bash
python3 util/ssc_planner/scripts/risk_experiment_scenario_suite.py \
  --execute \
  --output-root /tmp/epsilon_scenario_suite \
  --duration-sec 60
```

只跑高速场景：

```bash
python3 util/ssc_planner/scripts/risk_experiment_scenario_suite.py \
  --scenario highway_v1.0 \
  --scenario highway_lite \
  --output-root /tmp/epsilon_highway_suite
```

## 6. 验证结果

已执行：

```text
python3 -m py_compile risk_experiment_scenario_suite.py -> 通过
risk_experiment_scenario_suite.py dry-run -> 生成 scenario_run_plan.sh / scenario_suite_manifest.json
missing_scene 负例校验 -> 正确报 FileNotFoundError
```

dry-run 计划会为每个场景生成一条调用 `risk_experiment_batch.py` 的命令，并正确传入：

```text
--scenario-name
--playground
--planner-backend
--duration-sec
--repo-root
--setup-bash
```

## 7. 不改动项

MVP-14 不新增或修改仿真地图，也不改变任何规划行为。

本阶段只增强论文实验工具链，保证同一套风险配置可以在多个已存在 playground 上批量复现。

## 8. 当前风险

默认四个场景的交通交互强度有限，不能完全覆盖 cut-in / merge / dense following 等论文目标场景。

因此 MVP-14 解决的是“跨场景复现实验编排”，不是“新增高交互场景库”。后续可以在 MVP-15 单独新增论文专用 playground。

## 9. Git 信息

推荐 commit：

```bash
feat(ssc): add risk experiment scenario suite
```

推荐 tag：

```bash
v0.14.0-mvp14-risk-experiment-scenario-suite
```

## 10. 下一步计划

下一阶段建议进入 MVP-15：

```text
论文高交互场景库
```

重点补充 cut-in、merge、dense following、lane-change conflict 等可复现实验场景。
