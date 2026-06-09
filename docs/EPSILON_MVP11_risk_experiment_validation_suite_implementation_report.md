# MVP-11 实施报告：Risk Experiment Validation Suite

## 1. 目标

MVP-11 的目标是补齐风险感知 SSC 的实验验收闭环，让前面 MVP-0 到 MVP-10
产生的风险图、候选 risk exposure、自适应权重和 safety fallback 能够被稳定复现、
汇总和对比。

本阶段不继续修改核心规划算法，而是新增：

```text
实验配置覆盖
launch 配置路径覆盖
单组 summary 汇总
多组 matrix 汇总
论文消融实验操作说明
```

## 2. 修改范围

允许修改：

- `app/planning_integrated/launch/test_ssc_with_eudm_ros_launch.py`
- `app/planning_integrated/launch/test_ssc_with_mpdm_ros_launch.py`
- `app/planning_integrated/src/test_ssc_with_eudm.cc`
- `app/planning_integrated/src/test_ssc_with_mpdm.cc`
- `util/ssc_planner/CMakeLists.txt`
- `util/ssc_planner/config/ssc_config_risk_observe.pb.txt`
- `util/ssc_planner/config/ssc_config_risk_corridor.pb.txt`
- `util/ssc_planner/config/ssc_config_risk_full.pb.txt`
- `util/ssc_planner/scripts/risk_experiment_matrix.py`
- `util/ssc_planner/scripts/README_risk_experiment_suite.md`
- `core/behavior_planner/src/behavior_planner/behavior_planner.cc`
  - 仅修复注释中的 `s*/s` 被 C++ 解析为块注释结束符的问题；
  - 不改变 MPDM 算法、函数签名或运行逻辑。

禁止修改：

- `core/semantic_map_manager`
- `core/motion_predictor`
- `core/behavior_planner` 算法逻辑
- `util/eudm_planner`
- `util/ssc_planner/src`
- `util/ssc_planner/include`
- QP、corridor、risk grid 核心逻辑

## 3. 新增配置

新增三组 SSC 实验配置：

```text
ssc_config_risk_observe.pb.txt
  只开启 risk exposure eval 和 CSV 记录，不改变最终轨迹选择。

ssc_config_risk_corridor.pb.txt
  开启 risk-aware corridor，并记录 risk exposure；不做候选风险重选。

ssc_config_risk_full.pb.txt
  开启 risk-aware corridor、risk exposure reselect、adaptive risk weight 和 safety fallback。
```

默认 `ssc_config.pb.txt` 不改，仍保持 baseline 行为。

## 4. Launch 覆盖入口

EUDM / MPDM 两个 planning launch 新增参数：

```text
ssc_config_path
```

默认值仍指向：

```text
ssc_planner/config/ssc_config.pb.txt
```

实验时可以显式覆盖：

```bash
ros2 launch planning_integrated test_ssc_with_eudm_ros_launch.py \
  playground:=highway_v1.0 \
  ssc_config_path:=/home/ros/work/graduateworkcc/install/ssc_planner/share/ssc_planner/config/ssc_config_risk_full.pb.txt
```

同时，两个 planning 入口补充声明 `use_sim_state` 参数，保持 launch 参数与
`SscPlannerServer::Init()` 的读取契约一致，避免 ROS2 严格参数模式下出现未声明参数异常。

## 5. 新增脚本

新增：

```text
util/ssc_planner/scripts/risk_experiment_matrix.py
```

作用：

```text
多个 risk_experiment_report.py 输出的 summary.csv
↓
matrix.csv / matrix.json
```

示例：

```bash
python3 util/ssc_planner/scripts/risk_experiment_matrix.py \
  --summary baseline=/tmp/epsilon_reports/baseline/summary.csv \
  --summary observe=/tmp/epsilon_reports/risk_observe/summary.csv \
  --summary corridor=/tmp/epsilon_reports/risk_corridor/summary.csv \
  --summary full=/tmp/epsilon_reports/risk_full/summary.csv \
  --output-dir /tmp/epsilon_reports/matrix
```

## 6. 调用链

```text
ros2 launch planning_integrated ...
  -> ssc_config_path 覆盖 SSC 配置
  -> SscPlanner 按配置记录 /tmp CSV
  -> risk_experiment_report.py 生成单组 summary
  -> risk_experiment_matrix.py 生成多组消融矩阵
```

## 7. 不改动项

本阶段不改变：

- baseline 默认配置；
- risk grid 填图逻辑；
- risk-aware corridor 逻辑；
- QP 轨迹优化逻辑；
- candidate selection / fallback 逻辑。

因此 MVP-11 只增强实验复现能力，不改变默认规划行为。

## 8. 验收命令

```bash
cd /home/ros/work/graduateworkcc/src/EPSILON

python3 -m py_compile \
  util/ssc_planner/scripts/risk_experiment_report.py \
  util/ssc_planner/scripts/risk_experiment_matrix.py

python3 util/ssc_planner/scripts/risk_experiment_matrix.py --help

cd /home/ros/work/graduateworkcc
  colcon build --packages-up-to ssc_planner planning_integrated --symlink-install
```

本次实际执行：

```text
python3 -m py_compile ...                                   -> 通过
python3 util/ssc_planner/scripts/risk_experiment_matrix.py --help -> 通过
合成 summary.csv 生成 matrix.csv / matrix.json              -> 通过
colcon build --packages-up-to planning_integrated --symlink-install -> 通过
colcon build --packages-select playgrounds --symlink-install -> 通过
```

说明：`planning_integrated` 构建仅有已有 warning：

```text
OOQP include directory does not exist
部分成员初始化顺序 warning
thirdparty tk_spline unused-function warning
```

上述 warning 不由 MVP-11 引入。

## 9. 论文价值

MVP-11 对应论文第 7 章实验章节，价值在于：

1. 每组实验配置可追溯；
2. 每组实验 CSV 可独立清理和导出；
3. 单组结果可生成 summary；
4. 多组结果可自动汇成 matrix；
5. 消融实验表格可以从脚本复现。

## 10. 推荐 Commit

```bash
git add app/planning_integrated/launch \
  app/planning_integrated/src/test_ssc_with_eudm.cc \
  app/planning_integrated/src/test_ssc_with_mpdm.cc \
  core/behavior_planner/src/behavior_planner/behavior_planner.cc \
  util/ssc_planner/CMakeLists.txt \
  util/ssc_planner/config/ssc_config_risk_*.pb.txt \
  util/ssc_planner/scripts docs/EPSILON_MVP11_risk_experiment_validation_suite_implementation_report.md
git commit -m "feat(ssc): add risk experiment validation suite"
```
