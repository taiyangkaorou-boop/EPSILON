# EPSILON SSC MVP-0 风险占据图实施报告

> **实施日期**：2026-06-04
> **范围**：`SscMap` 内新增并行 float 风险占据栅格 `p_3d_risk_grid_`，不改变任何原始规划行为

---

## 1. 改动概要

| 维度 | 详情 |
|------|------|
| 修改文件 | `ssc_map.h`、`ssc_map.cc` |
| 新增类型 | `RiskMapDataType` (float)、`RiskGridMap3D` (std::vector\<float\>) |
| 新增成员 | `p_3d_risk_grid_` |
| 新增 public 方法 | `risk_grid()`, `ResetRiskMap()` |
| 新增 private 方法 | `FillDynamicPartProbabilistic()`, `FillMapWithFsVehicleTrajProbabilistic()` |
| 修改原有函数 | 构造函数（初始化 risk grid）、`ResetSscMap()`（追加 `ResetRiskMap()`）、`ConstructSscMap()`（每次构图前重置 risk grid，并追加概率化填充） |
| 不改动 | `SscMapDataType`, `p_3d_grid_`, `p_3d_inflated_grid_`, `FillStaticPart()`, `FillDynamicPart()`, `FillMapWithFsVehicleTraj()`, `ConstructCorridorUsingInitialTrajectory()`, `RunQpOptimization()`, EUDM, MPDM, SemanticBehavior, SscPlannerAdapter |

---

## 2. 架构

```
                    p_3d_grid_                  p_3d_risk_grid_
                    (uint8_t, binary)           (float, [0.0, 1.0])
                         │                            │
FillStaticPart() ────────┤                            │
FillDynamicPart() ───────┤                            │
                         │                            │
ConstructCorridor() ─────┤       (不读取 risk grid)    │
RunQpOptimization() ─────┤                            │
                         │                            │
                         │         FillDynamicPartProbabilistic()
                         │              → FillMapWithFsVehicleTrajProbabilistic()
                         │                   → cv::fillPoly(layer_mat, ..., 1.0f)
                         │
                    [原始规划链路完整保留]        [风险表达层 side-channel]
```

---

## 3. 逐点改动详情

### 3.1 ssc_map.h — 6 处新增

**点 A**（`using SscMapDataType` 之后）：新增风险图类型别名

```cpp
using RiskMapDataType = float;
using RiskGridMap3D = std::vector<RiskMapDataType>;
```

**点 B**（`p_3d_inflated_grid()` getter 之后）：新增只读访问器

```cpp
const RiskGridMap3D& risk_grid() const { return p_3d_risk_grid_; }
```

**点 C**（`ResetSscMap()` 声明之后）：新增风险图重置声明

```cpp
ErrorType ResetRiskMap();
```

**点 D**（`FillDynamicPart()` 声明之后）：新增概率化填充方法声明

```cpp
ErrorType FillDynamicPartProbabilistic(
    const std::unordered_map<int, vec_E<common::FsVehicle>>& sur_vehicle_trajs_fs);
ErrorType FillMapWithFsVehicleTrajProbabilistic(
    const vec_E<common::FsVehicle> traj);
```

**点 E**（`p_3d_inflated_grid_` 成员之后）：新增风险图成员变量

```cpp
RiskGridMap3D p_3d_risk_grid_;
```

### 3.2 ssc_map.cc — 5 处修改/新增

**点 1** — 构造函数末尾：初始化 risk grid

```cpp
int total_cells = config_.map_size[0] * config_.map_size[1] * config_.map_size[2];
p_3d_risk_grid_.resize(total_cells, 0.0f);
```

**点 2** — `ResetSscMap()` 中 `ClearGridMap()` 之后：追加

```cpp
ResetRiskMap();
```

**点 3** — `ConstructSscMap()` 中 binary map 清空后：追加 risk grid 清零，避免多 behavior 构图时风险图串行为

```cpp
ResetRiskMap();
```

**点 4** — `ConstructSscMap()` 中 `FillDynamicPart()` 之后：追加

```cpp
FillDynamicPartProbabilistic(sur_vehicle_trajs_fs);
```

**点 5** — 新增 `ResetRiskMap()` 实现（位于 `ClearDrivingCorridor()` 之后）：

```cpp
ErrorType SscMap::ResetRiskMap() {
  std::fill(p_3d_risk_grid_.begin(), p_3d_risk_grid_.end(), 0.0f);
  return kSuccess;
}
```

**点 6** — 新增 `FillDynamicPartProbabilistic()` 和 `FillMapWithFsVehicleTrajProbabilistic()` 实现（位于原始 `FillMapWithFsVehicleTraj()` 之后）：

- `FillDynamicPartProbabilistic()`：遍历 `sur_vehicle_trajs_fs`，逐车调用 `FillMapWithFsVehicleTrajProbabilistic()`
- `FillMapWithFsVehicleTrajProbabilistic()`：与原始 `FillMapWithFsVehicleTraj()` 几何流程完全一致（顶点验证 → Frenet→栅格坐标转换 → 范围检查 → OpenCV fillPoly），区别为写入目标 `p_3d_risk_grid_`（CV_32FC1 浮点图层），填充值固定为 `existence_prob = 1.0f`

---

## 4. MVP-0 关键设计决策

| 决策 | 理由 |
|------|------|
| 用 `std::vector<float>` 而非 `GridMapND<float,3>` | 避免修改 `common` 库（GridMapND 模板未显式实例化 `float,3`） |
| 复用 `p_3d_grid_->dims_size()` 计算 w/h/t_idx | 确保 risk grid 与 binary map 空间布局完全一致 |
| `existence_prob = 1.0f` | MVP-0 仅做确定性轨迹镜像，后续 MVP-1 接入真实概率 |
| 不做概率累加/max/decay | 重叠区域被最后写入的 fillPoly 覆盖，保持实现简单 |
| 不修改 corridor/QP/EUDM/MPDM | baseline 行为 100% 不变 |
| 全部新增代码带中文 Doxygen 注释 | 与 EPSILON 代码库风格一致 |

---

## 5. 验收状态

| 验收项 | 状态 |
|--------|------|
| `git diff --name-only` 仅 2 文件 | ✅ PASS |
| `SscMapDataType` 仍为 `uint8_t` | ✅ PASS |
| `FillStaticPart()` 无 diff | ✅ PASS |
| `FillDynamicPart()` 无 diff | ✅ PASS |
| `FillMapWithFsVehicleTraj()` 无 diff | ✅ PASS |
| `ConstructCorridorUsingInitialTrajectory()` 无 diff | ✅ PASS |
| `RunQpOptimization()` 无 diff | ✅ PASS |
| EUDM / MPDM / SemanticBehavior / adapter 无 diff | ✅ PASS |
| risk grid 构造链路：构造函数 resize(0.0f) | ✅ PASS |
| risk grid 重置链路：ResetSscMap() → ResetRiskMap() | ✅ PASS |
| risk grid 构图重置：ConstructSscMap() 每次构图前调用 ResetRiskMap() | ✅ PASS |
| risk grid 填充链路：ConstructSscMap() → FillDynamicPartProbabilistic() → FillMapWithFsVehicleTrajProbabilistic() | ✅ PASS |
| `ssc_map.cc` 窄语法检查 | ✅ PASS — `g++ -std=c++17 -fsyntax-only ... ssc_map.cc` 通过 |
| `colcon build --packages-select ssc_planner` | ⚠️ BLOCKED — workspace install 中缺少 `common`、`vehicle_model`、`vehicle_msgs`、`forward_simulator`、`motion_predictor`、`semantic_map_manager` |
| `colcon build --packages-up-to ssc_planner` | ⚠️ BLOCKED — `common` 依赖包存在既有编译错误，阻塞 `ssc_planner` 完整链路 |

---

## 6. 编译阻塞详情

`common` 包预构建错误（非本次改动引入）：

```
core/common/src/common/idm/intelligent_driver_model.cc:69:
  error: stray '@' in program
  （Doxygen @param 注释中的中文全角字符导致编译错误）

core/common/src/common/spline/spline_generator.cc:147:
  error: 'd' was not declared in this scope
  （tk_spline 模板实例化问题）
```

该阻塞与本次新增代码无关。为隔离验证本次改动，已对 `ssc_map.cc` 执行窄语法检查：

```bash
g++ -std=c++17 -fsyntax-only src/EPSILON/util/ssc_planner/src/ssc_planner/ssc_map.cc \
  -I src/EPSILON/util/ssc_planner/include \
  -I src/EPSILON/core/common/include \
  -I src/EPSILON/core/common/thirdparty \
  -I src/EPSILON/core/common/thirdparty/nanoflann/include \
  -I /usr/include/eigen3 \
  $(pkg-config --cflags opencv4)
```

该检查通过，说明 `ssc_map.h` / `ssc_map.cc` 新增类型、函数声明、函数实现与 OpenCV 写法在语法层面成立。完整包级编译仍需先修复 `common` 包的既有错误。

---

## 7. 后续里程碑

| 阶段 | 目标 | 涉及文件 |
|------|------|----------|
| MVP-1 | `existence_prob` 从固定 1.0f 替换为真实概率来源（MOBIL/EUDM） | `ssc_map.cc`（仅替换概率值） + 可能 `eudm_manager.cc` |
| MVP-2 | risk grid 参与 QP 优化（risk cost / chance constraint） | `ssc_planner.cc`（`RunQpOptimization`） |
| MVP-3 | Covariance propagation 和概率走廊（probabilistic corridor） | `ssc_map.cc` + `ssc_planner.cc` |
