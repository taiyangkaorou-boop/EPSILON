# EPSILON SSC MVP-1A Risk Grid Stats 实施报告

> **实施日期**：2026-06-04
> **范围**：为 MVP-0 已有 `p_3d_risk_grid_` 增加统计结构、统计函数和日志输出，不改变任何规划行为

---

## 1. 改动概要

| 维度 | 详情 |
|------|------|
| 修改文件 | `ssc_map.h`、`ssc_map.cc`（MVP-0 + MVP-1A 累计代码 diff 仍仅限这两个文件） |
| 新增结构体 | `RiskGridStats` |
| 新增 public 方法 | `ComputeRiskGridStats()` |
| 新增 private 方法 | `PrintRiskGridStatsIfNeeded()` |
| 修改原有函数 | `ConstructSscMap()`（在 `FillDynamicPartProbabilistic()` 之后追加统计日志调用） |
| 不改动 | 所有 MVP-0 不改动项 + `SscMapDataType`, `p_3d_grid_`, corridor, QP, EUDM, MPDM |

---

## 2. 调用链

```text
ConstructSscMap()
  -> ResetRiskMap()
  -> FillStaticPart()
  -> FillDynamicPart()
  -> FillDynamicPartProbabilistic()
  -> ComputeRiskGridStats()        ← 新增: 只读 p_3d_risk_grid_，返回 RiskGridStats
  -> PrintRiskGridStatsIfNeeded()   ← 新增: 输出 LOG(WARNING) 统计日志
```

统计函数不写任何地图数据，不影响 corridor / QP / control output。

---

## 3. 新增内容详情

### 3.1 `RiskGridStats` 结构体 (`ssc_map.h`)

```cpp
struct RiskGridStats {
  size_t total_cells = 0;                       // 风险图总栅格数
  size_t nonzero_cells = 0;                      // 风险值 > 0 的栅格数
  RiskMapDataType max_risk = 0.0f;               // 最大风险值
  double sum_risk = 0.0;                         // 风险值总和 (double 累加)
  size_t active_time_layers = 0;                 // 至少有一个非零栅格的时间层数
  std::vector<size_t> nonzero_cells_per_layer;   // 每时间层的非零栅格数
};
```

### 3.2 `ComputeRiskGridStats()` (`ssc_map.cc`)

**算法**：单次遍历 `p_3d_risk_grid_`，O(N) 时间复杂度。

- `total_cells` = `p_3d_risk_grid_.size()`
- `w` / `h` / `t` 从 `p_3d_grid_->dims_size()` 获取
- `nonzero_cells_per_layer` 初始化为 `t` 个 `0`
- `sum_risk` 使用 `double` 累加，避免 `float` 截断
- `max_risk` 初始为 `0.0f`
- `layer_idx = idx / (w * h)`
- 若 `w <= 0`、`h <= 0`、`t <= 0` 或 risk grid 为空，直接返回空统计，避免调试统计函数触发除零或越界
- 遍历结束后统计 `active_time_layers` = `count(nonzero_cells_per_layer[layer] > 0)`

### 3.3 `PrintRiskGridStatsIfNeeded()` (`ssc_map.cc`)

**日志格式**（使用 `LOG(WARNING)`）：

```
[Ssc][RiskGridStats] total_cells=8100000 nonzero_cells=1523 max_risk=1 sum_risk=1523 active_time_layers=55/81
[Ssc][RiskGridStats] layer[0] nonzero_cells=0
[Ssc][RiskGridStats] layer[1] nonzero_cells=15
...
[Ssc][RiskGridStats] layer[80] nonzero_cells=0
```

- 前缀固定 `[Ssc][RiskGridStats]`
- 第一行 summary：total_cells, nonzero_cells, max_risk, sum_risk, active_time_layers/total_layers
- 后续行：每层 `nonzero_cells`

### 3.4 `ConstructSscMap()` 中的追加调用

```cpp
FillDynamicPartProbabilistic(sur_vehicle_trajs_fs);

// 第4步（新增 MVP-1A）: 计算并输出 risk grid 统计日志
const auto risk_stats = ComputeRiskGridStats();
PrintRiskGridStatsIfNeeded(risk_stats);
```

---

## 4. 验收状态

| 验收项 | 状态 |
|--------|------|
| `git diff --name-only` 仅 2 文件 | ✅ PASS |
| 不允许出现 `ssc_planner.cc` | ✅ PASS |
| 不允许出现 EUDM / MPDM / common / semantic 相关文件 | ✅ PASS |
| `SscMapDataType` 仍为 `uint8_t` | ✅ PASS |
| `p_3d_grid_` 类型和用途无变化 | ✅ PASS |
| `FillStaticPart()` / `FillDynamicPart()` / `FillMapWithFsVehicleTraj()` 无 diff | ✅ PASS |
| `ConstructCorridorUsingInitialTrajectory()` 无 diff | ✅ PASS |
| `RunQpOptimization()` 无 diff | ✅ PASS |
| `ComputeRiskGridStats()` 只读 `p_3d_risk_grid_` | ✅ PASS |
| `PrintRiskGridStatsIfNeeded()` 只输出日志 | ✅ PASS |
| 统计调用在 `FillDynamicPartProbabilistic()` 之后 | ✅ PASS |
| 正常配置下 `nonzero_cells_per_layer.size() == map_size[2]`；异常维度返回空统计 | ✅ PASS |
| `sum_risk` 使用 `double` 累加 | ✅ PASS |
| `max_risk` 初始为 `0.0f` | ✅ PASS |
| 日志含 summary 行 + per-layer 行 | ✅ PASS |
| 日志前缀 `[Ssc][RiskGridStats]` | ✅ PASS |
| `ssc_map.cc` 窄范围语法检查 | ✅ PASS — `g++ -std=c++17 -fsyntax-only ... ssc_map.cc` |
| 完整 `colcon build --packages-up-to ssc_planner` | ⚠️ BLOCKED — `common` 依赖包预构建错误（与本次改动无关，见下方阻塞详情） |

---

## 5. 完整构建阻塞详情

本次审查已重新执行：

```bash
colcon build --packages-up-to ssc_planner --symlink-install
```

构建仍在 `common` 包阶段失败，`ssc_planner` 尚未进入编译。当前阻塞点为：

- `core/common/src/common/idm/intelligent_driver_model.cc:66-72`：注释内容疑似破坏 C++ 注释边界，导致 `@param` 被编译器当作源码解析。
- `core/common/src/common/spline/spline_generator.cc:147`：`d` 未声明。

上述两个错误均不在 MVP-1A 允许修改范围内，不能在本阶段顺手修复。MVP-1A 当前只能声明 `ssc_map.cc` 窄范围语法检查通过，完整 workspace 构建仍受既有 `common` 问题阻塞。

---

## 6. MVP-0 + MVP-1A 累计 diff 范围

当前代码层面的 tracked diff 仍只落在：

- `util/ssc_planner/include/ssc_planner/ssc_map.h`
- `util/ssc_planner/src/ssc_planner/ssc_map.cc`

不建议在本报告中固化精确插入行数，因为后续审查补丁会改变统计数字。验收以 `git diff --name-only` 是否只出现上述两个允许文件为准。

---

## 7. Codex 审查补充

本次审查确认 MVP-1A 的架构方向合理：risk stats 仍是 `SscMap` 内部 side-channel，只读 `p_3d_risk_grid_`，不进入 corridor / QP / behavior selection。

本次审查修复了一个健壮性问题：`ComputeRiskGridStats()` 原先按 `idx / (w * h)` 计算时间层索引，若出现异常地图维度或空 risk grid，调试统计函数存在除零风险。当前已加入维度与空数组保护，正常配置下统计结果不变。

---

## 8. 后续里程碑

| 阶段 | 目标 |
|------|------|
| MVP-1B | risk grid 可视化（RViz marker/heatmap）或 CSV/JSON 离线导出 |
| MVP-1C | `existence_prob` 从固定 1.0f 替换为真实概率来源 |
| MVP-2 | risk grid 参与 QP 优化（risk cost / chance constraint） |
