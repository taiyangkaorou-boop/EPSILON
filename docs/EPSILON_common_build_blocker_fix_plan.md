# EPSILON common 构建阻塞修复计划

建议保存路径与文件名：`/home/ros/work/graduateworkcc/docs/EPSILON_common_build_blocker_fix_plan.md`

## 1. 需要检查的文件

本阶段只允许检查并最小修改以下两个文件：

- `src/EPSILON/core/common/src/common/idm/intelligent_driver_model.cc`
- `src/EPSILON/core/common/src/common/spline/spline_generator.cc`

需要重新执行的构建命令：

```bash
cd /home/ros/work/graduateworkcc
colcon build --packages-up-to ssc_planner --symlink-install
```

辅助检查命令：

```bash
cd /home/ros/work/graduateworkcc
sed -n '55,80p' src/EPSILON/core/common/src/common/idm/intelligent_driver_model.cc
sed -n '120,155p' src/EPSILON/core/common/src/common/spline/spline_generator.cc
```

## 2. 每个错误的可能原因

### 2.1 `intelligent_driver_model.cc:66-72`

当前错误：

```text
stray '@' in program
extended character ... is not valid in an identifier
```

可能原因：

- `GetIdmDesiredAcceleration()` 前的块注释中存在文本 `s*/s_alpha`。
- 在 C/C++ 块注释 `/* ... */` 中，连续字符 `*/` 会提前结束注释。
- 因此后续 `@param`、中文说明等注释内容被编译器当作 C++ 源码解析，触发 `stray '@'` 和中文字符非法标识符错误。

最小修复方向：

- 只修改注释文本，避免注释内部出现连续 `*/`。
- 可将注释中的 `s*/s_alpha` 改成不破坏注释边界的表达，例如 `s_star / s_alpha`、`s^* / s_alpha` 或 `s_star_over_s_alpha`。
- 不修改 IDM 公式代码，不修改函数签名，不修改任何参数或算法。

### 2.2 `spline_generator.cc:147`

当前错误：

```text
error: 'd' was not declared in this scope
```

可能原因：

- `GetCubicSplineBySampleInterpolation()` 中存在嵌套循环：
  - 外层 `for (int i = 0; i < N_DIM; i++)` 表示当前样条维度。
  - 内层 `for (int d = 0; d <= N_DEG; d++)` 表示多项式系数阶次。
- `d` 的作用域只在内层循环中有效。
- 内层循环结束后调用 `cubic_spline(n, d).set_coeff(coeff);`，此时 `d` 已离开作用域，因此编译失败。
- 从上下文看，`set_coeff(coeff)` 应写入当前维度的样条段，最可能的正确维度索引是外层变量 `i`。

最小修复方向：

- 先确认 `cubic_spline(segment_idx, dim_idx)` 的第二个参数语义确实是维度索引。
- 若确认无误，只将 `cubic_spline(n, d).set_coeff(coeff);` 恢复为当前维度索引版本，即使用外层维度变量。
- 不改样条插值算法，不改循环结构，不改模板参数，不改函数签名。

## 3. 给执行 agent 的最小修复任务包

任务目标：

只修复 `common` 包当前阻塞 `ssc_planner` 构建的两个编译错误，不改变算法逻辑，不触碰 MVP-1A risk grid stats 功能。

执行目录：

```bash
cd /home/ros/work/graduateworkcc
```

允许修改文件：

```text
src/EPSILON/core/common/src/common/idm/intelligent_driver_model.cc
src/EPSILON/core/common/src/common/spline/spline_generator.cc
```

禁止修改文件：

```text
src/EPSILON/util/ssc_planner/include/ssc_planner/ssc_map.h
src/EPSILON/util/ssc_planner/src/ssc_planner/ssc_map.cc
src/EPSILON/util/ssc_planner/src/ssc_planner/ssc_planner.cc
src/EPSILON/util/eudm_planner/**
src/EPSILON/app/planning_integrated/**
任意 CMakeLists.txt
任意 package.xml
```

实现要求：

1. 在 `intelligent_driver_model.cc` 中，只修复破坏块注释的 `*/` 文本。
2. 不修改 `GetIdmDesiredAcceleration()` 的任何可执行语句。
3. 在 `spline_generator.cc` 中，定位 `GetCubicSplineBySampleInterpolation()` 的 `cubic_spline(n, d).set_coeff(coeff);`。
4. 根据上下文恢复正确的维度索引，预期为外层维度变量 `i`。
5. 不新增算法、不重构、不调整模板、不修改函数签名。
6. 不修改 MVP-1A 的 `ssc_map.h` / `ssc_map.cc`。
7. 不自动 commit。

修复后必须执行：

```bash
cd /home/ros/work/graduateworkcc
colcon build --packages-up-to ssc_planner --symlink-install
```

如果构建继续失败：

- 若错误仍在上述两个允许文件内，继续按“只修编译错误、不改算法”的原则定位。
- 若错误出现在其他文件，立即停止，记录完整错误文件、行号和原因，不得扩大修改范围。

## 4. 验收标准

### 4.1 diff 验收

PASS 条件：

- 本次新增修改只涉及以下两个 `common` 文件：
  - `src/EPSILON/core/common/src/common/idm/intelligent_driver_model.cc`
  - `src/EPSILON/core/common/src/common/spline/spline_generator.cc`
- MVP-1A 文件未被继续改动：
  - `src/EPSILON/util/ssc_planner/include/ssc_planner/ssc_map.h`
  - `src/EPSILON/util/ssc_planner/src/ssc_planner/ssc_map.cc`
- 没有修改 `ssc_planner.cc`、EUDM、MPDM、CMake 或 package metadata。

建议检查：

```bash
cd /home/ros/work/graduateworkcc/src/EPSILON
git diff --name-only
git diff --check
```

注意：当前工作区可能已有 MVP-1A 的 `ssc_map.h` / `ssc_map.cc` diff。执行 agent 需要确认本次任务没有继续改动这两个文件。

### 4.2 编译验收

PASS 条件：

```bash
cd /home/ros/work/graduateworkcc
colcon build --packages-up-to ssc_planner --symlink-install
```

必须通过，至少应满足：

- `common` 包通过编译；
- `ssc_planner` 进入并完成编译；
- 不再出现：
  - `intelligent_driver_model.cc` 的 `stray '@' in program`；
  - `spline_generator.cc:147` 的 `'d' was not declared in this scope`。

### 4.3 行为验收

PASS 条件：

- IDM 可执行代码无变化；
- 三次样条插值流程无算法变化；
- 只恢复正确变量或修复注释语法；
- MVP-1A risk grid stats 不受影响。

## 5. 禁止事项

严格禁止：

1. 不改变算法逻辑；
2. 不改变函数签名；
3. 不重构 `common`；
4. 不修改 CMake / package.xml；
5. 不修改 `ssc_map.h` / `ssc_map.cc` 的 MVP-1A 功能；
6. 不修改 `ssc_planner.cc`；
7. 不修改 EUDM / MPDM / planning integrated launch；
8. 不新增测试框架或调试工具；
9. 不引入新的依赖；
10. 不自动 commit；
11. 不用 `git reset --hard` 或其他破坏性回滚命令。

若执行 agent 认为必须修改两个允许文件以外的文件，必须先停止并向用户说明新的阻塞点，不得直接扩大范围。
