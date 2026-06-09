# MVP-6 实施报告

## 1. 目标

MVP-6 的目标是在已生成的概率风险占据图基础上，为 SSC 轨迹优化链路加入风险代价入口，使系统能够计算候选轨迹的风险暴露：

```text
J_risk = Σ Risk(s_k, d_k, t_k)
```

本次实现采用最小可退化版本：不修改底层 QP 求解器和 Bezier Hessian 构造，而是在 QP 成功生成候选轨迹后，对每条候选 Bezier 轨迹沿时间采样查询 risk grid，计算风险暴露软代价，并可选在 QP 成功候选之间做风险重选。

## 2. 修改范围

本次只修改以下 6 个 MVP-6 相关文件：

```text
util/ssc_planner/proto/ssc_config.proto
util/ssc_planner/config/ssc_config.pb.txt
util/ssc_planner/include/ssc_planner/ssc_map.h
util/ssc_planner/src/ssc_planner/ssc_map.cc
util/ssc_planner/include/ssc_planner/ssc_planner.h
util/ssc_planner/src/ssc_planner/ssc_planner.cc
```

## 3. 新增结构体 / 函数

新增配置字段：

```text
enable_risk_exposure_eval
enable_risk_exposure_reselect
risk_exposure_weight
risk_exposure_switch_margin
risk_exposure_sample_dt
risk_exposure_high_threshold
risk_exposure_csv_path
```

新增结构体：

```cpp
SscPlanner::RiskExposureMetrics
```

新增函数：

```cpp
SscMap::QueryRiskByMetricPosition()
SscMap::QueryRiskByMetricPositionInGrid()
SscPlanner::FindBaselineTrajectoryIndex()
SscPlanner::ComputeRiskExposureForCandidate()
SscPlanner::SelectRiskAwareTrajectoryIndex()
SscPlanner::AppendRiskExposureMetricsToCsv()
SscPlanner::IsRiskExposureEvaluationEnabled()
```

新增成员：

```cpp
behavior_risk_grid_snapshots_
candidate_risk_grid_snapshots_
```

其中 `behavior_risk_grid_snapshots_` 在每个行为调用 `ConstructSscMap()` 后立即保存该行为对应的 risk grid，`candidate_risk_grid_snapshots_` 与 `qp_trajs_ / valid_behaviors_` 对齐，避免候选轨迹风险评价误用最后一次构图残留的 risk grid。

## 4. 调用链

```text
SscPlanner::RunOnce()
  -> 每个行为 ConstructSscMap() 后保存 behavior_risk_grid_snapshots_
  -> RunQpOptimization()
     -> 生成 QP 成功候选 qp_trajs_
     -> 保存与候选对齐的 candidate_risk_grid_snapshots_
  -> UpdateTrajectoryWithCurrentBehavior()
     -> FindBaselineTrajectoryIndex()
     -> ComputeRiskExposureForCandidate()
        -> SscMap::QueryRiskByMetricPositionInGrid()
     -> SelectRiskAwareTrajectoryIndex()
     -> AppendRiskExposureMetricsToCsv()
```

## 5. 不改动项

本次没有修改：

```text
EUDM / MPDM
SemanticBehavior
motion_predictor
forward_simulator
common/spline/spline_generator
ooqp_interface
QP hard constraints
corridor construction
control output interface
```

底层 QP objective 内部仍保持原始：

```text
jerk + proximity
```

MVP-6 当前版本将风险代价实现为候选轨迹层的 risk exposure soft cost，而不是直接把离散 risk grid 写入 QP Hessian。

## 6. 编译结果

编译命令：

```bash
cd /home/ros/work/graduateworkcc
colcon build --packages-up-to ssc_planner --symlink-install
```

结果：

```text
Summary: 7 packages finished
```

`ssc_planner` 编译通过。实测摘要为：

```text
Summary: 7 packages finished [31.0s]
```

编译输出中仍有若干既有 warning，例如 signed/unsigned comparison、未使用变量、第三方 tk_spline 未使用函数、DrivingCorridor 未初始化提示等；本次未扩大范围修复这些历史 warning。

## 7. 运行 / 日志 / CSV 结果

默认配置：

```text
enable_risk_exposure_eval: false
enable_risk_exposure_reselect: false
risk_exposure_weight: 0.0
```

因此默认不计算风险暴露、不写 MVP-6 CSV、不改变原始 SSC 轨迹选择。

开启 `enable_risk_exposure_eval=true` 后，会输出：

```text
[Ssc][MVP6RiskExposure]
```

并写入：

```text
/tmp/epsilon_mvp6_risk_exposure.csv
```

CSV 字段：

```csv
cycle,stamp,candidate_index,behavior,is_baseline,is_selected,sample_count,exposure_sum,exposure_mean,exposure_max,high_risk_hits,risk_score
```

开启 `enable_risk_exposure_reselect=true` 且 `risk_exposure_weight > 0` 后，系统只会在 QP 已成功候选之间比较风险软代价，并要求风险优势超过 `risk_exposure_switch_margin` 才允许替换 baseline 候选。

## 8. Git commit 信息

推荐 commit message：

```text
feat(ssc): add risk exposure cost for qp candidates
```

## 9. tag 信息

建议合并到 `dev/risk-aware-ssc` 后打 tag：

```text
v0.6.0-mvp6-risk-cost-qp
```

## 10. 当前风险

当前 MVP-6 是工程稳妥版：

```text
risk grid -> QP 成功候选轨迹 -> risk exposure soft cost -> 可选候选重选
```

不是严格意义上的：

```text
risk grid -> 修改 QP Hessian/linear term -> 求解器内部风险代价
```

原因是离散 risk grid 查表对优化变量是非光滑/非凸的，直接塞入 QP objective 会破坏二次规划结构。后续如果要做加强版，需要设计局部线性化、插值和数值归一化策略。

## 11. 下一步计划

下一阶段进入 MVP-7：

```text
场景自适应风险权重
```

建议基于当前 MVP-6 的配置项动态调整：

```text
risk_exposure_weight
risk_exposure_switch_margin
risk_exposure_high_threshold
```

同时保持默认配置可退化，避免破坏 baseline。
