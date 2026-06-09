# MVP-16 实施报告：Scripted Risk Actors 主动交互场景控制

## 1. 目标

MVP-16 的目标是在不修改 SSC / EUDM / MPDM / QP 的前提下，为论文实验平台补充可复现的主动交互周车场景。

MVP-15 已提供“高密度 / 相对速度”的初始交通态，但周车运动仍主要来自初始速度和零控制信号。MVP-16 进一步新增脚本化周车控制器，通过 `/ctrl/agent_{id}` 发布 `vehicle_msgs::msg::ControlSignal`，让指定周车在闭环仿真中主动执行切入、制动等动作。

## 2. 修改范围

允许修改并已修改：

```text
app/planning_integrated/scripts/scripted_risk_actor_node.py
app/planning_integrated/launch/risk_experiment_closed_loop_launch.py
app/planning_integrated/CMakeLists.txt
core/playgrounds/CMakeLists.txt
core/playgrounds/risk_scripted_cut_in_v1.0/
util/ssc_planner/scripts/risk_experiment_batch.py
util/ssc_planner/scripts/risk_experiment_scenario_suite.py
util/ssc_planner/scripts/README_risk_experiment_suite.md
docs/EPSILON_MVP16_scripted_risk_actor_implementation_report.md
```

禁止修改并保持未改：

```text
util/ssc_planner/src/ssc_planner/ssc_map.cc
util/ssc_planner/src/ssc_planner/ssc_planner.cc
util/eudm_planner/
util/mpdm_planner/
core/semantic_map_manager/
core/phy_simulator/src/phy_simulator/phy_simulator.cc
QP / corridor 核心求解逻辑
```

## 3. 新增结构体 / 函数

Python dataclass：

```text
ActorInitialState
ActorSegment
ScriptedActor
```

新增函数 / 方法：

```text
smoothstep()
load_vehicle_initial_states()
parse_segments()
ScriptedRiskActorNode.load_script()
ScriptedRiskActorNode.active_segment()
ScriptedRiskActorNode.build_signal()
ScriptedRiskActorNode.on_timer()
```

## 4. 调用链

闭环实验调用链：

```text
risk_experiment_batch.py
  -> ros2 launch planning_integrated risk_experiment_closed_loop_launch.py
      -> phy_simulator_planning_node
      -> test_ssc_with_eudm / test_ssc_with_mpdm
      -> scripted_risk_actor_node.py
          -> publish /ctrl/agent_{id}
          -> phy_simulator CtrlSignalCallback()
          -> PhySimulation::UpdateVehicleStates()
          -> /arena_info_dynamic
          -> planning_integrated replanning
```

`scripted_risk_actor_node.py` 使用开环 `ControlSignal`：

```text
is_openloop = true
state = scripted actor state at elapsed time
```

因此可以直接复用 phy_simulator 已有开环入口，不需要改车辆动力学更新逻辑。

## 5. 新增实验场景

新增 playground：

```text
risk_scripted_cut_in_v1.0
```

该场景复用 `risk_lane_change_conflict_v1.0` 的：

```text
agent_config.json
lane_net_norm.json
obstacles_norm.json
vehicle_set.json
```

并新增：

```text
risk_actor_script.json
```

脚本动作：

```text
agent_4:
  0.5s ~ 4.0s 平滑横向偏移 -2.8m，并轻微加速，形成主动切入压力

agent_1:
  1.0s ~ 3.5s 纵向加速度 -1.5m/s^2，形成前车制动压力
```

## 6. 不改动项

本 MVP 不改变：

```text
1. risk grid 填图公式；
2. risk-aware corridor 阈值逻辑；
3. candidate trajectory risk exposure 计算；
4. safety fallback 触发逻辑；
5. EUDM / MPDM 行为生成；
6. SSC QP 目标函数；
7. phy_simulator 的车辆状态更新函数。
```

## 7. 运行 / 日志 / CSV / RViz 结果

验收重点：

```text
1. launch 参数中默认 enable_scripted_risk_actors=false，不影响 baseline；
2. playground 带 risk_actor_script.json 时，batch runner 自动启用脚本 actor；
3. scripted actor 节点向 /ctrl/agent_4 和 /ctrl/agent_1 发布开环控制；
4. phy_simulator 能继续发布 /arena_info_dynamic；
5. risk grid CSV 能继续生成；
6. planning_integrated 能在主动周车压力下继续闭环运行。
```

已执行验证：

```bash
python3 -m py_compile \
  app/planning_integrated/scripts/scripted_risk_actor_node.py \
  util/ssc_planner/scripts/risk_experiment_batch.py \
  util/ssc_planner/scripts/risk_experiment_scenario_suite.py

python3 -m json.tool \
  core/playgrounds/risk_scripted_cut_in_v1.0/risk_actor_script.json

colcon build --packages-up-to playgrounds planning_integrated ssc_planner --symlink-install
```

结果：

```text
Python 语法检查通过
JSON 解析检查通过
colcon build 通过
```

单节点验证：

```bash
timeout 1s ros2 run planning_integrated scripted_risk_actor_node.py \
  --ros-args \
  -p script_path:=/home/ros/work/graduateworkcc/src/EPSILON/core/playgrounds/risk_scripted_cut_in_v1.0/risk_actor_script.json \
  -p vehicle_info_path:=/home/ros/work/graduateworkcc/src/EPSILON/core/playgrounds/risk_scripted_cut_in_v1.0/vehicle_set.json
```

结果：

```text
return code = 124
loaded 2 scripted risk actors
```

短时闭环验证：

```bash
timeout 4s ros2 launch planning_integrated risk_experiment_closed_loop_launch.py \
  playground:=risk_scripted_cut_in_v1.0 \
  enable_scripted_risk_actors:=true \
  ssc_config_path:=/home/ros/work/graduateworkcc/src/EPSILON/util/ssc_planner/config/ssc_config.pb.txt
```

结果：

```text
return code = 124
scripted_risk_actor_node 启动成功
loaded 2 scripted risk actors
Ssc planner succeed 多次出现
/tmp/epsilon_risk_grid_stats.csv 生成成功
```

batch runner 验证：

```bash
python3 util/ssc_planner/scripts/risk_experiment_batch.py \
  --execute \
  --experiment baseline=ssc_config.pb.txt \
  --output-root /tmp/epsilon_mvp16_batch_execute \
  --scenario-name risk_scripted_cut_in_v1.0 \
  --duration-sec 4 \
  --continue-on-error
```

结果：

```text
status = ok
return_code = 124
timed_out = true
risk_grid_present = true
matrix.csv 生成成功
```

本次短时 batch 的 `matrix.csv` baseline 行：

```text
scenario_name = risk_scripted_cut_in_v1.0
risk_grid_rows = 33
max_risk_max = 1.0
nonzero_cells_mean = 13075.030303030304
```

已知非本 MVP 引入警告：

```text
planning_integrated 构建时仍有 OOQP include directory 不存在的历史 warning。
```

## 8. Git commit 信息

推荐 commit message：

```text
feat(ssc): add scripted risk actors for experiments
```

## 9. tag 信息

建议 tag：

```text
v0.16.0-mvp16-scripted-risk-actors
```

## 10. 当前风险

当前实现仍属于实验工具链增强，不是完整交互行为预测模型：

```text
1. actor 脚本使用开环状态覆盖，适合复现实验，不代表真实周车闭环驾驶策略；
2. 横向切入轨迹由局部坐标平滑插值生成，未做车道中心线投影；
3. 若脚本参数设置过激，可能制造不符合车辆动力学约束的状态；
4. 默认自动启用规则依赖 playground 是否存在 risk_actor_script.json；
5. 当前只验证主动场景入口，不改变概率预测链路。
```

## 11. 下一步计划

下一阶段可继续推进：

```text
MVP-17：实验指标增强，记录 scripted actor 是否启用、脚本场景名、主动交互时间窗；
MVP-18：更真实的周车闭环控制器，例如 IDM 跟车 + lane-change reference tracking；
MVP-19：主动交互场景与 risk-aware corridor / fallback 的消融实验矩阵。
```
