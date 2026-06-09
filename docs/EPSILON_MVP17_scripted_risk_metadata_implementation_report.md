# MVP-17 实施报告：Scripted Risk Actor 实验元数据追踪

## 1. 目标

MVP-17 的目标是让 MVP-16 新增的脚本化周车场景具备论文实验可追溯性。

具体做法：

```text
risk_actor_script.json
-> risk_experiment_batch.py
-> risk_experiment_report.py
-> summary.csv / summary.json
-> risk_experiment_matrix.py
-> matrix.csv / matrix.json
```

这样每一组实验结果都能明确记录：

```text
scripted_risk_actors_enabled
scripted_risk_actor_count
risk_actor_script_path
```

## 2. 修改范围

允许修改并已修改：

```text
util/ssc_planner/scripts/risk_experiment_batch.py
util/ssc_planner/scripts/risk_experiment_report.py
util/ssc_planner/scripts/risk_experiment_matrix.py
util/ssc_planner/scripts/README_risk_experiment_suite.md
docs/EPSILON_MVP17_scripted_risk_metadata_implementation_report.md
```

禁止修改并保持未改：

```text
SSC / EUDM / MPDM / QP
phy_simulator
planning_integrated launch
playground 场景定义
```

## 3. 新增结构体 / 函数

新增 dataclass：

```text
ScriptedRiskActorMetadata
```

新增函数：

```text
count_scripted_risk_actors()
scripted_risk_actor_metadata()
```

## 4. 调用链

```text
risk_experiment_batch.py
  -> scripted_risk_actor_metadata()
      -> 自动读取 playground/risk_actor_script.json
      -> 统计 actors 数量
  -> manifest.json 写入脚本 actor 元数据
  -> 调用 risk_experiment_report.py 时透传元数据
      -> summary.csv / summary.json 写入元数据
  -> risk_experiment_matrix.py 默认输出元数据列
```

## 5. 不改动项

本 MVP 不改变：

```text
1. 任何规划算法；
2. 任何风险图计算逻辑；
3. 任何闭环 launch 行为；
4. 任何场景车辆初始状态；
5. 任何 ROS topic。
```

## 6. 编译 / 运行结果

验证重点：

```text
1. 无脚本场景 summary 中 scripted_risk_actors_enabled = 0；
2. scripted 场景 summary 中 scripted_risk_actors_enabled = 1；
3. scripted 场景 summary 中 scripted_risk_actor_count = 2；
4. matrix.csv 默认包含 scripted_risk_actors_enabled / scripted_risk_actor_count；
5. Python 脚本语法检查通过。
```

已执行验证：

```bash
python3 -m py_compile \
  util/ssc_planner/scripts/risk_experiment_batch.py \
  util/ssc_planner/scripts/risk_experiment_report.py \
  util/ssc_planner/scripts/risk_experiment_matrix.py
```

dry-run manifest 验证：

```bash
python3 util/ssc_planner/scripts/risk_experiment_batch.py \
  --output-root /tmp/epsilon_mvp17_dry_scripted \
  --scenario-name risk_scripted_cut_in_v1.0 \
  --duration-sec 1

python3 util/ssc_planner/scripts/risk_experiment_batch.py \
  --output-root /tmp/epsilon_mvp17_dry_plain \
  --scenario-name highway_v1.0 \
  --duration-sec 1
```

结果：

```text
risk_scripted_cut_in_v1.0:
  scripted_risk_actors_enabled = true
  scripted_risk_actor_count = 2

highway_v1.0:
  scripted_risk_actors_enabled = false
  scripted_risk_actor_count = 0
```

summary / matrix 验证：

```bash
python3 util/ssc_planner/scripts/risk_experiment_report.py \
  --risk-grid-csv /tmp/epsilon_mvp16_batch_execute/baseline/raw_risk_grid_stats.csv \
  --risk-exposure-csv /tmp/epsilon_mvp16_batch_execute/baseline/raw_risk_exposure.csv \
  --output-dir /tmp/epsilon_mvp17_report_scripted \
  --experiment-name baseline \
  --scenario-name risk_scripted_cut_in_v1.0 \
  --git-ref test \
  --scripted-risk-actors-enabled \
  --scripted-risk-actor-count 2 \
  --risk-actor-script-path /home/ros/work/graduateworkcc/src/EPSILON/core/playgrounds/risk_scripted_cut_in_v1.0/risk_actor_script.json

python3 util/ssc_planner/scripts/risk_experiment_matrix.py \
  --summary baseline=/tmp/epsilon_mvp17_report_scripted/summary.csv \
  --output-dir /tmp/epsilon_mvp17_matrix_scripted
```

结果：

```text
summary.csv:
  risk_actor_script_path = /home/ros/work/graduateworkcc/src/EPSILON/core/playgrounds/risk_scripted_cut_in_v1.0/risk_actor_script.json
  scripted_risk_actor_count = 2
  scripted_risk_actors_enabled = 1

matrix.csv:
  默认列包含 scripted_risk_actors_enabled / scripted_risk_actor_count
  baseline,risk_scripted_cut_in_v1.0,test,...,1,2,...
```

## 7. 当前风险

```text
1. actor_count 来自 risk_actor_script.json 的 actors 数量，不验证每个 actor 是否在运行期实际被 ROS 订阅；
2. 元数据是离线实验追踪字段，不参与规划逻辑；
3. 若用户传入自定义 run-command-template 绕过默认 launch，summary 仍只记录脚本配置元数据，不证明外部命令实际启动了 actor 节点。
```

## 8. Git commit 信息

推荐 commit message：

```text
feat(ssc): track scripted risk actor metadata
```

## 9. tag 信息

建议 tag：

```text
v0.17.0-mvp17-scripted-risk-metadata
```
