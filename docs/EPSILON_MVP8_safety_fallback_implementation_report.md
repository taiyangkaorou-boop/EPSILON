# MVP-8 实施报告

## 1. 目标

MVP-8 实现风险兜底安全过滤器，在不修改 control / QP / EUDM / MPDM 的前提下，基于候选轨迹 risk exposure 指标对最终候选做最后一道保守检查。

## 2. 修改范围

本次只修改以下 4 个文件：

```text
util/ssc_planner/proto/ssc_config.proto
util/ssc_planner/config/ssc_config.pb.txt
util/ssc_planner/include/ssc_planner/ssc_planner.h
util/ssc_planner/src/ssc_planner/ssc_planner.cc
```

## 3. 新增结构体 / 函数

新增配置字段：

```text
enable_safety_fallback
safety_fallback_max_risk_threshold
safety_fallback_exposure_sum_threshold
safety_fallback_high_risk_hits_threshold
safety_fallback_min_risk_reduction
safety_fallback_prefer_lane_keeping
```

新增结构体：

```cpp
SscPlanner::SafetyFallbackResult
```

新增函数：

```cpp
SscPlanner::ShouldComputeRiskExposureMetrics()
SscPlanner::EnsureRiskExposureMetrics()
SscPlanner::ApplySafetyFallbackIfNeeded()
```

## 4. 调用链

```text
SscPlanner::UpdateTrajectoryWithCurrentBehavior()
  -> FindBaselineTrajectoryIndex()
  -> EnsureRiskExposureMetrics()
  -> SelectRiskAwareTrajectoryIndex()
  -> ApplySafetyFallbackIfNeeded()
  -> trajectory_ / low_spd_alternative_traj_ / final_corridor_ / final_ref_states_
```

兜底过滤器位于风险重选之后、最终轨迹封装之前，因此所有输出结构会一致指向 fallback 后的候选。

## 5. 不改动项

本次没有修改：

```text
control
QP / SplineGenerator / OOQP
EUDM / MPDM
SemanticBehavior
SscMap risk fill
风险走廊构建
```

## 6. 编译结果

编译命令：

```bash
cd /home/ros/work/graduateworkcc
colcon build --packages-up-to ssc_planner --symlink-install
```

结果：

```text
Summary: 7 packages finished [26.7s]
```

`ssc_planner` 编译通过。编译输出仍有历史 warning，本次未扩大范围修复。

## 7. 运行 / 日志 / CSV 结果

默认配置：

```text
enable_safety_fallback: false
```

因此默认保持 MVP-7 行为，不额外计算兜底、不替换候选。

开启后触发条件：

```text
selected.exposure_max > safety_fallback_max_risk_threshold
OR selected.exposure_sum > safety_fallback_exposure_sum_threshold
OR selected.high_risk_hits >= safety_fallback_high_risk_hits_threshold
```

触发后优先尝试低风险 LaneKeeping 候选，否则选择全候选中相对最低风险者；只有最大风险降低超过 `safety_fallback_min_risk_reduction` 才替换。

日志前缀：

```text
[Ssc][MVP8SafetyFallback]
```

CSV 在 MVP-6 risk exposure 文件中追加字段：

```csv
safety_fallback_triggered,safety_fallback_switched,safety_original_index,safety_fallback_index,safety_original_max_risk,safety_fallback_max_risk
```

## 8. Git commit 信息

推荐 commit message：

```text
feat(ssc): add risk-aware safety fallback
```

## 9. tag 信息

建议合并到 `dev/risk-aware-ssc` 后打 tag：

```text
v0.8.0-mvp8-safety-fallback
```

## 10. 当前风险

当前 MVP-8 是最小安全过滤器：

```text
已有 QP 成功候选 -> 风险阈值检查 -> 可选切换到更低风险候选
```

它不直接发 emergency deceleration，也不让 planner 返回失败；如果所有候选都高风险，会保留当前候选并输出 warning。

## 11. 下一步计划

下一阶段进入 MVP-9：

```text
实验平台与消融实验
```

建议围绕 MVP-1B/MVP-6 CSV 和 RViz risk grid，补充离线绘图脚本和实验配置记录。
