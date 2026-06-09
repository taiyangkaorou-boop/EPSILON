# MVP-5 实施报告

## 1. 目标

MVP-5 的目标是让概率风险图参与 SSC corridor 构建：

```text
risk(s,d,t) > threshold
↓
high-risk occupied
↓
corridor 避开高风险区域
```

本阶段采用稳妥版实现：不修改 QP 目标函数、不修改控制输出，而是在 corridor 构建前把高风险栅格提升为二值占据图中的 hard occupied。

## 2. 修改范围

本次修改文件：

```text
util/ssc_planner/proto/ssc_config.proto
util/ssc_planner/config/ssc_config.pb.txt
util/ssc_planner/include/ssc_planner/ssc_map.h
util/ssc_planner/src/ssc_planner/ssc_map.cc
util/ssc_planner/src/ssc_planner/ssc_planner.cc
```

## 3. 新增结构体 / 函数

新增配置字段：

```text
enable_risk_aware_corridor
risk_occupied_threshold
```

新增 `SscMap::Config` 成员：

```cpp
bool enable_risk_aware_corridor = false;
RiskMapDataType risk_occupied_threshold = 1.0f;
```

新增函数：

```cpp
size_t SscMap::ApplyRiskThresholdToBinaryOccupancy();
```

该函数遍历 `p_3d_risk_grid_`，将满足：

```text
risk > risk_occupied_threshold
```

的 cell 写入 `p_3d_grid_ = 100`。

## 4. 调用链

```text
SscPlanner::Init()
  -> 读取 ssc_config.pb.txt
  -> 映射 enable_risk_aware_corridor / risk_occupied_threshold 到 SscMap::Config

SscPlanner::RunOnce()
  -> SscMap::ConstructSscMap()
    -> FillStaticPart()
    -> FillDynamicPart()
    -> FillDynamicPartProbabilistic()
    -> ApplyRiskThresholdToBinaryOccupancy()  [仅 enable=true 时执行]
  -> ConstructCorridorUsingInitialTrajectory(p_3d_grid_, ...)
  -> RunQpOptimization()
```

## 5. 不改动项

```text
1. 不修改 EUDM / MPDM；
2. 不修改 SemanticBehavior；
3. 不修改 QP cost；
4. 不修改 control 输出；
5. 不修改 RViz 可视化接口；
6. 不修改 risk grid CSV 字段；
7. 不改变默认 baseline 行为。
```

默认配置：

```text
enable_risk_aware_corridor: false
risk_occupied_threshold: 1.0
```

因此默认情况下 risk grid 仍只用于统计、CSV 和 RViz，不影响 corridor。

## 6. 编译结果

执行命令：

```bash
cd /home/ros/work/graduateworkcc
colcon build --packages-up-to ssc_planner --symlink-install
```

结果：

```text
Summary: 7 packages finished
```

编译通过。

仍存在既有 warning：

```text
ssc_planner.cc signed/unsigned comparison
unused variable beh
DrivingCorridor::id may be used uninitialized
ssc_server_ros.cc unused variable wheel_base
tk_spline unused functions
```

这些 warning 不是本 MVP 新增问题，本次未越界修改。

## 7. 运行 / 日志 / CSV / RViz 结果

本次完成编译验证，未启动完整 ROS 场景运行。

运行观察建议：

```bash
cd /home/ros/work/graduateworkcc
source install/setup.bash
rm -f /tmp/epsilon_risk_grid_stats.csv
ros2 launch planning_integrated test_ssc_with_eudm_ros_launch.py
```

日志观察：

```text
[Ssc][RiskAwareCorridor] enabled=true threshold=... high_risk_cells=...
```

阈值实验预期：

```text
enable=false:
  完全退化为 baseline

threshold=1.0 且使用 risk > threshold:
  risk grid 已截断到 [0,1]，通常不会新增 high-risk occupied

threshold 越低:
  high_risk_cells 越多
  corridor 越保守
```

## 8. Git commit 信息

推荐 commit message：

```text
feat(ssc): construct risk-aware corridor from risk grid
```

## 9. tag 信息

建议合并回 `dev/risk-aware-ssc` 后打 tag：

```text
v0.5.0-mvp5-risk-aware-corridor
```

## 10. 当前风险

```text
1. 风险阈值过低可能导致 initial cube invalid；
2. 当前 high-risk cell 是 hard constraint，不是连续 risk cost；
3. 多模态概率归一性依赖上游；
4. risk-aware corridor 默认关闭，需要实验时显式开启；
5. 本 MVP 通过 corridor 可行域间接影响 QP，不代表已实现 QP risk cost。
```

## 11. 下一步计划

下一阶段 MVP-6 进入 QP risk cost：

```text
J_risk = sum Risk(s_k,d_k,t_k)
```

需要重点保证：

```text
lambda_risk = 0 时退化为 baseline
lambda_risk 增大时轨迹更避险
QP 可行性和实时性可接受
```
