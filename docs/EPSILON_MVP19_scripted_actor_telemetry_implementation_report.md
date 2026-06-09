# MVP-19 实施报告：脚本化风险周车 Telemetry CSV

## 1. 目标

MVP-19 目标是在 MVP-16/MVP-18 脚本化风险周车基础上，导出运行时 telemetry CSV。
该 CSV 用于证明实验中的主动交互周车确实按脚本和 IDM 闭环逻辑运行，并为论文第 7 章提供可复现实验证据。

默认输出路径：

```text
/tmp/epsilon_scripted_risk_actor_telemetry.csv
```

## 2. 修改范围

允许修改：

```text
app/planning_integrated/scripts/scripted_risk_actor_node.py
app/planning_integrated/launch/risk_experiment_closed_loop_launch.py
util/ssc_planner/scripts/risk_experiment_batch.py
util/ssc_planner/scripts/risk_experiment_report.py
util/ssc_planner/scripts/risk_experiment_matrix.py
util/ssc_planner/scripts/README_risk_experiment_suite.md
docs/EPSILON_MVP19_scripted_actor_telemetry_implementation_report.md
```

禁止修改：

```text
SSC corridor / QP / EUDM / MPDM / phy_simulator 核心算法
risk grid 填充逻辑
车辆预测与行为概率数据结构
```

## 3. 新增结构体 / 函数

`scripted_risk_actor_node.py` 新增：

```text
IdmTelemetry
TELEMETRY_FIELDS
parse_bool_param()
csv_float()
open_telemetry_csv_if_needed()
close_telemetry_csv()
compute_idm_telemetry()
telemetry_row()
append_telemetry()
```

`risk_experiment_report.py` 新增：

```text
summarize_scripted_actor_telemetry()
```

## 4. 调用链

```text
risk_experiment_batch.py
  -> ros2 launch planning_integrated risk_experiment_closed_loop_launch.py
  -> scripted_risk_actor_node.py
  -> /ctrl/agent_{id}
  -> /tmp/epsilon_scripted_risk_actor_telemetry.csv
  -> raw_scripted_actor_telemetry.csv
  -> risk_experiment_report.py summary
  -> risk_experiment_matrix.py matrix
```

## 5. CSV 字段

```csv
stamp,elapsed,actor_id,mode,target_vehicle_id,current_x,current_y,current_angle,current_velocity,current_acceleration,script_lateral,script_heading_delta,gap,relative_velocity,desired_gap,idm_acceleration,command_x,command_y,command_angle,command_velocity,command_acceleration
```

说明：

```text
open_loop actor 的 target/gap/idm 字段允许为空；
idm_follow actor 会记录 gap、relative_velocity、desired_gap 和 idm_acceleration。
```

## 6. 不改动项

```text
1. 不改变原始 SSC 二值占据图；
2. 不改变 p_3d_risk_grid_ 计算逻辑；
3. 不改变 corridor / QP / control 的规划行为；
4. 不改变 EUDM / MPDM / SemanticBehavior；
5. 不改变 phy_simulator 的车辆更新逻辑。
```

## 7. 编译与验证结果

```bash
cd /home/ros/work/graduateworkcc/src/EPSILON
python3 -m py_compile \
  app/planning_integrated/scripts/scripted_risk_actor_node.py \
  util/ssc_planner/scripts/risk_experiment_batch.py \
  util/ssc_planner/scripts/risk_experiment_report.py \
  util/ssc_planner/scripts/risk_experiment_matrix.py

cd /home/ros/work/graduateworkcc
colcon build --packages-up-to playgrounds planning_integrated ssc_planner --symlink-install

source install/setup.bash
timeout 4s ros2 launch planning_integrated risk_experiment_closed_loop_launch.py \
  playground:=risk_idm_cut_in_v1.0 \
  enable_scripted_risk_actors:=true \
  scripted_actor_telemetry_csv:=/tmp/epsilon_scripted_risk_actor_telemetry.csv

python3 src/EPSILON/util/ssc_planner/scripts/risk_experiment_batch.py \
  --execute \
  --scenario-name risk_idm_cut_in_v1.0 \
  --duration-sec 4 \
  --experiment baseline=ssc_config.pb.txt \
  --output-root /tmp/epsilon_mvp19_batch_smoke
```

实际结果：

```text
1. Python py_compile 通过；
2. git diff --check 通过；
3. ros2 launch --show-args 可见 scripted_actor_telemetry_enabled / scripted_actor_telemetry_csv；
4. colcon build --packages-up-to playgrounds planning_integrated ssc_planner --symlink-install 通过；
5. 4 秒闭环烟测生成 /tmp/epsilon_scripted_risk_actor_telemetry.csv；
6. telemetry 烟测结果：302 行数据，actor_id = 1/4，mode = open_loop/idm_follow，IDM 非空行 150；
7. batch 烟测结果：baseline status=ok，return_code=124，timed_out=true，risk_grid_present=true，scripted_actor_telemetry_present=true；
8. matrix.csv 已包含 scripted_actor_gap_min、scripted_actor_idm_acceleration_min、scripted_actor_command_acceleration_min/max 等字段。
```

说明：

```text
timeout 返回 124 是短时闭环实验的预期停止方式；
risk_idm_cut_in_v1.0 中部分周期出现 ego lane 查找失败，但同次运行仍有 Ssc planner succeed 周期，
本 MVP 验证重点是 telemetry 链路，不把该场景规划表现作为算法效果结论。
```

## 8. 论文价值

MVP-19 将“脚本化风险周车是否真实运行”从 launch 日志提升为可量化 CSV 证据。
它补强小创新点 4 的实验工具链，也为大创新点 1/2/3 的后续实验提供场景交互证据：

```text
1. 可复现 actor 的运行轨迹；
2. 可解释 IDM 跟车中的 gap 与相对速度变化；
3. 可在 summary/matrix 中直接比较脚本化交互场景；
4. 可辅助绘制论文中的交互过程曲线。
```

## 9. 当前风险

```text
1. telemetry 每周期 flush，短实验更安全，但长期高频实验会增加少量 IO 开销；
2. 当前 telemetry 记录的是脚本 actor 发布命令和最近 arena 状态，不是规划器预测结果；
3. IDM gap 按 actor 当前 heading 投影，复杂交叉路口需要后续扩展到 Frenet/车道坐标。
```

## 10. 推荐提交信息

```text
feat(ssc): export scripted actor telemetry
```

## 11. 推荐 tag

```text
v0.19.0-mvp19-scripted-actor-telemetry
```
