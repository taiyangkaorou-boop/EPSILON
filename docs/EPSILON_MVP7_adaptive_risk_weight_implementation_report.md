# MVP-7 实施报告

## 1. 目标

MVP-7 实现场景自适应风险权重，在不修改 EUDM / MPDM / SemanticBehavior / QP / control 的前提下，基于 SSC 已有输入动态调整候选轨迹风险软代价参数。

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
enable_adaptive_risk_weight
adaptive_high_speed_threshold
adaptive_high_speed_weight_scale
adaptive_lane_change_weight_scale
adaptive_high_risk_threshold
adaptive_high_risk_weight_scale
adaptive_max_risk_weight
adaptive_switch_margin_scale
```

新增结构体：

```cpp
SscPlanner::AdaptiveRiskWeightContext
```

新增函数：

```cpp
SscPlanner::BuildBaseAdaptiveRiskWeightContext()
SscPlanner::UpdateAdaptiveRiskWeightContextByMetrics()
```

## 4. 调用链

```text
SscPlanner::UpdateTrajectoryWithCurrentBehavior()
  -> BuildBaseAdaptiveRiskWeightContext()
     -> 根据 initial_state_.velocity / ego_behavior_ 识别高速与变道场景
  -> ComputeRiskExposureForCandidate()
     -> 只测量 exposure_sum / exposure_max / high_risk_hits
  -> UpdateAdaptiveRiskWeightContextByMetrics()
     -> 根据 max(exposure_max) 识别高交互风险
     -> 高交互风险触发后放大风险权重，并按配置倍率降低候选切换裕度
  -> risk_score = adaptive_risk_weight * exposure_sum
  -> SelectRiskAwareTrajectoryIndex()
     -> 使用 adaptive_switch_margin 决定是否替换 baseline
```

## 5. 不改动项

本次没有修改：

```text
EUDM / MPDM
SemanticBehavior
motion_predictor
forward_simulator
SscMap / risk grid fill
common/spline/spline_generator
ooqp_interface
QP hard constraints
control output interface
```

## 6. 编译结果

编译命令：

```bash
cd /home/ros/work/graduateworkcc
colcon build --packages-up-to ssc_planner --symlink-install
```

结果：

```text
Summary: 7 packages finished [9.48s]
```

`ssc_planner` 编译通过。编译输出仍有历史 warning，本次未扩大范围修复。

## 7. 运行 / 日志 / CSV 结果

默认配置：

```text
enable_adaptive_risk_weight: false
```

因此默认保持 MVP-6 静态参数，不额外改变风险权重或切换裕度。

开启 `enable_adaptive_risk_weight=true` 后，自适应权重会在 MVP-6 risk exposure 评价链路中生效；也就是说，需要同时打开 `enable_risk_exposure_eval` 或 `enable_risk_exposure_reselect` 才会计算候选轨迹 risk exposure 并写日志/CSV。若 `risk_exposure_weight=0.0`，仍不会发生风险重选。

开启后会在已有 MVP-6 risk exposure CSV 中附加自适应上下文字段：

```csv
adaptive_enabled,high_speed,lane_change,high_interaction_risk,adaptive_risk_weight,adaptive_switch_margin,adaptive_high_risk_threshold,adaptive_max_candidate_risk,adaptive_applied_scale
```

高风险场景触发时会输出：

```text
[Ssc][MVP7AdaptiveRiskWeight]
```

## 8. Git commit 信息

推荐 commit message：

```text
feat(ssc): adapt risk weight by driving scenario
```

## 9. tag 信息

建议合并到 `dev/risk-aware-ssc` 后打 tag：

```text
v0.7.0-mvp7-adaptive-risk-weight
```

## 10. 当前风险

当前 MVP-7 只基于 SSC 内部已有变量做轻量场景判断：

```text
高速: initial_state_.velocity > adaptive_high_speed_threshold
变道: ego_behavior_ 为 LaneChangeLeft / LaneChangeRight
高交互风险: max(candidate.exposure_max) > adaptive_high_risk_threshold
```

它不是完整场景识别器，也不做 TTC / emergency fallback；这些留给 MVP-8。

## 11. 下一步计划

下一阶段进入 MVP-8：

```text
安全兜底机制
```

建议复用 MVP-6/7 已有 risk exposure 指标，先实现保守模式日志与候选拒绝策略，再评估是否接入 emergency deceleration。
