# MVP-20 实施报告：风险实验输出完整性校验器

## 1. 目标

MVP-20 目标是新增一个离线校验工具，在论文实验数据入表前检查 batch/suite 输出是否完整。
该工具用于避免以下问题：

```text
1. manifest 存在但 raw CSV 丢失；
2. summary.csv 或 matrix.csv 没有生成；
3. 脚本化周车场景缺少 raw_scripted_actor_telemetry.csv；
4. 多场景 suite 中某个场景 matrix 没有复制到 matrix_by_scenario；
5. 半截实验结果被误用于论文消融表。
```

## 2. 修改范围

允许修改：

```text
util/ssc_planner/scripts/risk_experiment_verify_outputs.py
util/ssc_planner/scripts/README_risk_experiment_suite.md
docs/EPSILON_MVP20_risk_experiment_output_verifier_implementation_report.md
```

禁止修改：

```text
SSC / EUDM / MPDM / QP / phy_simulator 核心算法
risk grid 填充逻辑
闭环 launch 运行行为
现有实验 CSV 字段语义
```

## 3. 新增脚本

```text
util/ssc_planner/scripts/risk_experiment_verify_outputs.py
```

支持输入：

```text
1. 单个 batch 输出目录；
2. 单个 manifest.json；
3. 多场景 scenario suite 输出目录；
4. 包含多个子目录 manifest.json 的普通输出根目录。
```

输出：

```text
verification/verification_report.json
verification/verification_report.csv
```

## 4. 核心校验规则

```text
1. output_root 必须存在；
2. 必须发现至少一个 batch manifest；
3. batch manifest 必须包含 experiments 列表；
4. status=ok 的实验必须有非空 risk_grid_archive；
5. status=ok 的实验必须有非空 summary_csv；
6. 若 risk_exposure_present=true，则 risk_exposure_archive 必须非空；
7. 若 scripted_risk_actors_enabled=true，则 scripted_actor_telemetry_archive 必须非空；
8. 若存在 status=ok 的实验，batch matrix/matrix.csv 必须非空；
9. scenario suite 中 status=ok 的场景必须有场景 matrix；
10. scenario suite 中 status=ok 的场景必须有 matrix_by_scenario/<scenario>.csv。
```

校验失败时脚本返回非零值，可用于论文实验流水线。

## 5. 调用示例

校验单场景 batch：

```bash
python3 util/ssc_planner/scripts/risk_experiment_verify_outputs.py \
  /tmp/epsilon_batch_highway_v1
```

校验多场景 suite：

```bash
python3 util/ssc_planner/scripts/risk_experiment_verify_outputs.py \
  /tmp/epsilon_scenario_suite
```

指定报告目录：

```bash
python3 util/ssc_planner/scripts/risk_experiment_verify_outputs.py \
  /tmp/epsilon_scenario_suite \
  --output-dir /tmp/epsilon_scenario_suite_verify
```

## 6. 不改动项

```text
1. 不启动 ROS；
2. 不运行规划器；
3. 不修改任何实验原始 CSV；
4. 不改变 batch/suite 已有输出格式；
5. 不改变规划算法与仿真行为。
```

## 7. 编译与验证结果

```bash
cd /home/ros/work/graduateworkcc/src/EPSILON
python3 -m py_compile util/ssc_planner/scripts/risk_experiment_verify_outputs.py

python3 util/ssc_planner/scripts/risk_experiment_batch.py \
  --output-root /tmp/epsilon_mvp20_batch_dry_run \
  --scenario-name risk_idm_cut_in_v1.0 \
  --duration-sec 4 \
  --experiment baseline=ssc_config.pb.txt

python3 util/ssc_planner/scripts/risk_experiment_verify_outputs.py \
  /tmp/epsilon_mvp20_batch_dry_run

python3 util/ssc_planner/scripts/risk_experiment_batch.py \
  --execute \
  --scenario-name risk_idm_cut_in_v1.0 \
  --duration-sec 4 \
  --experiment baseline=ssc_config.pb.txt \
  --output-root /tmp/epsilon_mvp20_batch_smoke

python3 util/ssc_planner/scripts/risk_experiment_verify_outputs.py \
  /tmp/epsilon_mvp20_batch_smoke
```

实际结果：

```text
1. python3 -m py_compile util/ssc_planner/scripts/risk_experiment_verify_outputs.py 通过；
2. git diff --check 通过；
3. dry-run batch 输出校验通过，verification_status=ok；
4. execute batch 输出校验通过，verification_status=ok；
5. scenario suite dry-run 输出校验通过，verification_status=ok；
6. 负例测试通过：清空 raw_scripted_actor_telemetry.csv 后，校验器返回 1，并报告 scripted_actor_telemetry_archive failed；
7. colcon build --packages-up-to ssc_planner --symlink-install 通过，仅保留既有 OOQP include warning。
```

验证过程中修复的边界：

```text
1. dry-run 输出只校验 manifest/run_plan，不要求 raw CSV 和 matrix；
2. risk_exposure_present=false 时允许 raw_risk_exposure.csv 为空占位；
3. 对复制/搬迁后的实验目录，manifest 内旧 output_root 下的绝对路径会重定位到当前校验目录。
```

## 8. 论文价值

MVP-20 补强实验可复现性和数据治理：

```text
1. 实验结果是否完整可以机器校验；
2. CSV、summary、matrix 与脚本化 actor telemetry 形成闭环证据；
3. 后续大规模多场景消融实验可以先校验再入表；
4. 论文中可以说明所有表格数据均通过输出完整性检查。
```

## 9. 当前风险

```text
1. 校验器只证明文件完整性，不证明规划算法效果优劣；
2. CSV 行数检查不能替代指标合理性分析；
3. 若未来 manifest 字段改名，需要同步更新校验规则。
```

## 10. 推荐提交信息

```text
feat(ssc): verify risk experiment outputs
```

## 11. 推荐 tag

```text
v0.20.0-mvp20-risk-experiment-output-verifier
```
