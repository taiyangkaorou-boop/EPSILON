# MVP-13 实施报告：Closed-loop Risk Experiment Launch

## 1. 目标

MVP-13 的目标是把 MVP-12 的批量实验从“只启动 planning 节点”升级为“物理仿真器 + 规划器闭环”。

本阶段新增：

```text
closed-loop risk experiment launch
headless phy_simulator 启动方式
EUDM / MPDM 后端参数选择
batch runner 默认闭环命令
timeout 124 正常实验停止语义
```

## 2. 修改范围

允许修改：

- `app/planning_integrated/launch/risk_experiment_closed_loop_launch.py`
- `util/ssc_planner/scripts/risk_experiment_batch.py`
- `util/ssc_planner/scripts/README_risk_experiment_suite.md`
- `docs/EPSILON_MVP13_closed_loop_risk_experiment_launch_implementation_report.md`

禁止修改：

- SSC 核心规划逻辑；
- EUDM / MPDM 行为决策逻辑；
- risk grid 填图逻辑；
- corridor / QP / fallback 逻辑；
- baseline SSC 配置。

## 3. 新增 launch

新增：

```text
app/planning_integrated/launch/risk_experiment_closed_loop_launch.py
```

该 launch 启动：

```text
phy_simulator_planning_node
test_ssc_with_eudm 或 test_ssc_with_mpdm
```

关键参数：

```text
planner_backend=eudm|mpdm
playground=highway_v1.0
ssc_config_path=<ssc config pb.txt>
arena_info_static_topic=/arena_info_static
arena_info_dynamic_topic=/arena_info_dynamic
ctrl_topic=/ctrl/agent_0
```

## 4. Headless 设计

原 `phy_simulator_planning_launch.py` 会 include `joy_ctrl_launch.py`，批量实验机器如果没有 joystick 或 `joy` 环境异常，可能在实验启动阶段引入无关失败。

MVP-13 的闭环 launch 直接启动 `phy_simulator_planning_node`：

```text
vehicle_set.json
obstacles_norm.json
lane_net_norm.json
```

因此不依赖 `/dev/input/js0`，更适合作为论文批量实验入口。

## 5. Batch runner 更新

`risk_experiment_batch.py` 默认命令从：

```text
ros2 launch planning_integrated test_ssc_with_eudm_ros_launch.py ...
```

更新为：

```text
ros2 launch planning_integrated risk_experiment_closed_loop_launch.py \
  planner_backend:=eudm \
  playground:=highway_v1.0 \
  ssc_config_path:=...
```

同时新增 manifest 字段：

```text
timed_out
```

`timeout` 到达实验时长时返回 124。MVP-13 将其解释为正常实验停止，只要 risk grid CSV 已生成，该组实验仍标记为：

```text
status = ok
timed_out = true
```

## 6. 调用链

```text
risk_experiment_batch.py --execute
  -> 清理 /tmp/epsilon_risk_grid_stats.csv
  -> ros2 launch planning_integrated risk_experiment_closed_loop_launch.py
      -> phy_simulator_planning_node 发布 arena_info
      -> planning_integrated 订阅 arena_info 并发布 ctrl
      -> SSC 运行并写出 risk grid CSV
  -> timeout 到时停止
  -> 归档 raw_risk_grid_stats.csv
  -> risk_experiment_report.py 生成 summary
  -> risk_experiment_matrix.py 生成 matrix
  -> manifest.json 记录 timed_out / status
```

## 7. 验证结果

已执行：

```text
python3 -m py_compile risk_experiment_batch.py risk_experiment_closed_loop_launch.py -> 通过
ros2 launch planning_integrated risk_experiment_closed_loop_launch.py --show-args -> 通过
ros2 launch planning_integrated risk_experiment_closed_loop_launch.py planner_backend:=mpdm --show-args -> 通过
colcon build --packages-up-to phy_simulator planning_integrated --symlink-install -> 通过
timeout 4s ros2 launch planning_integrated risk_experiment_closed_loop_launch.py ... -> 闭环启动成功
risk_experiment_batch.py --execute --experiment baseline=ssc_config.pb.txt --duration-sec 4 -> 通过
```

单组 execute smoke 生成：

```text
/tmp/epsilon_mvp13_execute_smoke/baseline/raw_risk_grid_stats.csv
/tmp/epsilon_mvp13_execute_smoke/baseline/report/summary.csv
/tmp/epsilon_mvp13_execute_smoke/matrix/matrix.csv
/tmp/epsilon_mvp13_execute_smoke/manifest.json
```

manifest 关键结果：

```text
return_code = 124
timed_out = true
risk_grid_present = true
status = ok
```

## 8. 不改动项

MVP-13 不改变规划算法，只改变实验启动和批量执行入口。

风险图、corridor、QP、fallback 的行为均由既有配置开关控制。

## 9. 当前风险

短时闭环启动中仍可看到既有 EUDM glog 提示：

```text
Could not create log file
```

该问题来自 EUDM 内部固定日志目录设置，不影响 ROS launch 启动、SSC 规划、risk grid CSV、summary 或 matrix 输出。为避免跨 MVP 修改，MVP-13 不改 EUDM 源码。

## 10. Git 信息

推荐 commit：

```bash
feat(ssc): add closed-loop risk experiment launch
```

推荐 tag：

```bash
v0.13.0-mvp13-closed-loop-risk-experiment-launch
```

## 11. 下一步计划

下一阶段建议进入 MVP-14：

```text
实验场景库与论文级批量复现实验
```

重点是增加 cut-in / following / lane-change / merge 等场景配置，并用 MVP-13 的闭环入口生成可复现实验数据。
