# MVP-18 实施报告：IDM Scripted Risk Actor 闭环周车控制

## 1. 目标

MVP-18 在 MVP-16 脚本化周车基础上增加 `idm_follow` 模式，让实验周车不只是开环轨迹回放，而是能够根据 `/arena_info_dynamic` 中的前车状态计算 IDM 纵向加速度。

该阶段用于构造更真实的主动交互场景：

```text
前车制动
↓
切入车根据前车距离 / 相对速度执行 IDM 纵向闭环
↓
同时叠加脚本横向切入偏移
↓
规划器观察到动态交互压力
```

## 2. 修改范围

允许修改并已修改：

```text
app/planning_integrated/scripts/scripted_risk_actor_node.py
core/playgrounds/CMakeLists.txt
core/playgrounds/risk_idm_cut_in_v1.0/
util/ssc_planner/scripts/risk_experiment_scenario_suite.py
util/ssc_planner/scripts/README_risk_experiment_suite.md
docs/EPSILON_MVP18_idm_scripted_risk_actor_implementation_report.md
```

禁止修改并保持未改：

```text
SSC / EUDM / MPDM / QP
phy_simulator 车辆更新逻辑
risk grid 填图逻辑
risk-aware corridor / fallback
```

## 3. 新增结构体 / 函数

新增 dataclass：

```text
IdmFollowConfig
```

新增或增强函数：

```text
parse_idm_follow()
ScriptedRiskActorNode.on_arena_info_dynamic()
ScriptedRiskActorNode.idm_acceleration()
ScriptedRiskActorNode.build_open_loop_signal()
ScriptedRiskActorNode.build_idm_follow_signal()
```

## 4. 调用链

```text
risk_experiment_closed_loop_launch.py
  -> scripted_risk_actor_node.py
      -> subscribe /arena_info_dynamic
      -> cache latest vehicle states
      -> compute IDM acceleration for actor mode=idm_follow
      -> publish open-loop ControlSignal to /ctrl/agent_{id}
  -> phy_simulator_planning_node
      -> apply ControlSignal
      -> publish updated /arena_info_dynamic
```

## 5. 新增场景

新增 playground：

```text
risk_idm_cut_in_v1.0
```

场景复用：

```text
risk_scripted_cut_in_v1.0/agent_config.json
risk_scripted_cut_in_v1.0/lane_net_norm.json
risk_scripted_cut_in_v1.0/obstacles_norm.json
risk_scripted_cut_in_v1.0/vehicle_set.json
```

新增脚本：

```text
risk_idm_cut_in_v1.0/risk_actor_script.json
```

脚本 actor：

```text
agent_4:
  mode = idm_follow
  target_vehicle_id = 1
  lateral_offset = -2.8m

agent_1:
  mode = open_loop
  lon_acc = -1.5m/s^2
```

## 6. 不改动项

本 MVP 不改变任何规划器行为。IDM actor 是实验场景控制器，不是 SSC 预测或优化模块的一部分。

## 7. 验证计划

需要验证：

```text
1. Python 语法检查通过；
2. risk_idm_cut_in_v1.0 JSON 可解析；
3. colcon build 通过；
4. 单节点能加载 idm_follow 脚本；
5. 短时闭环 launch 能启动 scripted_risk_actor_node 并生成 risk grid CSV；
6. batch runner 能自动识别 actor_count = 2。
```

已执行验证：

```bash
python3 -m py_compile \
  app/planning_integrated/scripts/scripted_risk_actor_node.py \
  util/ssc_planner/scripts/risk_experiment_scenario_suite.py

python3 -m json.tool core/playgrounds/risk_idm_cut_in_v1.0/risk_actor_script.json

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
  -p script_path:=/home/ros/work/graduateworkcc/src/EPSILON/core/playgrounds/risk_idm_cut_in_v1.0/risk_actor_script.json \
  -p vehicle_info_path:=/home/ros/work/graduateworkcc/src/EPSILON/core/playgrounds/risk_idm_cut_in_v1.0/vehicle_set.json
```

结果：

```text
return code = 124
loaded 2 scripted risk actors
```

短时闭环验证：

```bash
timeout 4s ros2 launch planning_integrated risk_experiment_closed_loop_launch.py \
  playground:=risk_idm_cut_in_v1.0 \
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
  --output-root /tmp/epsilon_mvp18_batch_execute \
  --scenario-name risk_idm_cut_in_v1.0 \
  --duration-sec 4 \
  --continue-on-error
```

结果：

```text
status = ok
return_code = 124
timed_out = true
risk_grid_present = true
scripted_risk_actors_enabled = true
scripted_risk_actor_count = 2
matrix.csv 生成成功
```

本次短时 batch 的 `matrix.csv` baseline 行：

```text
scenario_name = risk_idm_cut_in_v1.0
scripted_risk_actors_enabled = 1
scripted_risk_actor_count = 2
risk_grid_rows = 27
max_risk_max = 1.0
nonzero_cells_mean = 13643.814814814816
```

已知非本 MVP 引入警告：

```text
planning_integrated 构建时仍有 OOQP include directory 不存在的历史 warning。
```

## 8. 当前风险

```text
1. IDM 使用局部前向投影近似前车距离，没有沿车道中心线求精确 Frenet gap；
2. 横向切入仍由脚本偏移控制，未做完整换道控制器；
3. 该能力用于复现实验场景，不代表真实交通参与者策略学习；
4. 如果 target_vehicle_id 配置不合理，IDM actor 会退化为仅脚本横向/纵向动作。
```

## 9. Git commit 信息

推荐 commit message：

```text
feat(ssc): add idm scripted risk actors
```

## 10. tag 信息

建议 tag：

```text
v0.18.0-mvp18-idm-scripted-risk-actors
```
