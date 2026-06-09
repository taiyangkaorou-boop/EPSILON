#!/usr/bin/env python3
"""EPSILON 风险感知 SSC 离线实验汇总工具。

该脚本读取 MVP-1B 的 risk grid 统计 CSV 和 MVP-6/7/8 的候选轨迹风险
暴露 CSV，生成论文实验可复现的 summary 表格。若本机安装 matplotlib，
会额外输出趋势图；没有安装时仍可生成 CSV/JSON 汇总，不影响 ROS 编译。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean
from typing import Dict, Iterable, List, Optional, Tuple, Union


Number = Optional[float]
SummaryValue = Union[str, int, float, None]
RANGE_EPSILON = 1e-9  # 风险图行为差异判定阈值，用于过滤浮点格式误差。


def parse_float(value: str) -> Number:
    """将 CSV 字段安全转换为 float，空值或非法值返回 None。"""
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    try:
        parsed = float(text)
    except ValueError:
        return None
    if math.isnan(parsed) or math.isinf(parsed):
        return None
    return parsed


def parse_int(value: str) -> Optional[int]:
    """将 CSV 字段安全转换为 int，兼容 '1.0' 这类日志输出格式。"""
    parsed = parse_float(value)
    if parsed is None:
        return None
    return int(parsed)


def read_csv(path: Path) -> List[Dict[str, str]]:
    """读取 CSV 文件；文件不存在时返回空列表，方便脚本用于部分实验数据。"""
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8") as csv_file:
        return list(csv.DictReader(csv_file))


def values(rows: Iterable[Dict[str, str]], field: str) -> List[float]:
    """提取某个数值字段，过滤空值和非法值。"""
    result: List[float] = []
    for row in rows:
        parsed = parse_float(row.get(field, ""))
        if parsed is not None:
            result.append(parsed)
    return result


def summarize_numeric(rows: List[Dict[str, str]], field: str) -> Dict[str, SummaryValue]:
    """计算单个数值字段的 count/mean/max/sum。"""
    field_values = values(rows, field)
    return summarize_value_list(field, field_values)


def summarize_value_list(metric_name: str, field_values: List[float]) -> Dict[str, SummaryValue]:
    """计算一组数值的 count/mean/max/sum，metric_name 直接作为输出字段前缀。"""
    if not field_values:
        return {
            f"{metric_name}_count": 0,
            f"{metric_name}_mean": None,
            f"{metric_name}_max": None,
            f"{metric_name}_sum": None,
        }
    return {
        f"{metric_name}_count": len(field_values),
        f"{metric_name}_mean": mean(field_values),
        f"{metric_name}_max": max(field_values),
        f"{metric_name}_sum": sum(field_values),
    }


def group_rows_by_cycle(rows: List[Dict[str, str]]) -> Dict[Tuple[str, str], List[Dict[str, str]]]:
    """按 (cycle, stamp) 聚合候选轨迹行，避免把候选行数误当规划周期数。"""
    grouped: Dict[Tuple[str, str], List[Dict[str, str]]] = {}
    for row in rows:
        cycle = row.get("cycle", "").strip()
        stamp = row.get("stamp", "").strip()
        if not cycle and not stamp:
            continue
        grouped.setdefault((cycle, stamp), []).append(row)
    return grouped


def risk_grid_planning_key(row: Dict[str, str]) -> Optional[Tuple[str, str]]:
    """提取 risk grid 行的 planning cycle key，优先使用同帧共享的 stamp。"""
    stamp = row.get("stamp", "").strip()
    if stamp:
        return ("stamp", stamp)
    cycle = row.get("cycle", "").strip()
    if cycle:
        return ("cycle", cycle)
    return None


def group_risk_grid_rows_by_planning_key(
    rows: List[Dict[str, str]]
) -> Dict[Tuple[str, str], List[Dict[str, str]]]:
    """按 planning cycle 聚合 risk grid 行，用于比较同一帧不同 behavior 的风险图。"""
    grouped: Dict[Tuple[str, str], List[Dict[str, str]]] = {}
    for row in rows:
        key = risk_grid_planning_key(row)
        if key is None:
            continue
        grouped.setdefault(key, []).append(row)
    return grouped


def risk_grid_behavior_id(row: Dict[str, str], row_index: int) -> str:
    """提取 behavior 标识；旧 CSV 缺字段时退化为行号，避免误合并候选。"""
    behavior_index = row.get("behavior_index", "").strip()
    behavior_name = row.get("behavior", "").strip()
    if behavior_index or behavior_name:
        return f"{behavior_index}:{behavior_name}"
    return f"row:{row_index}"


def risk_grid_signature(row: Dict[str, str]) -> Tuple[SummaryValue, ...]:
    """构造风险图统计签名，忽略 behavior 名称，只比较风险分布摘要是否不同。"""
    signature_fields = [
        "nonzero_cells",
        "max_risk",
        "sum_risk",
        "active_time_layers",
        "risk_source_vehicles",
        "risk_source_modes",
    ]
    signature_values: List[SummaryValue] = []
    for field in signature_fields:
        parsed = parse_float(row.get(field, ""))
        if parsed is None:
            signature_values.append(row.get(field, "").strip())
        else:
            # 四舍五入只用于离线签名，避免 CSV 浮点格式尾差造成“伪差异”。
            signature_values.append(round(parsed, 9))
    return tuple(signature_values)


def numeric_range(rows: List[Dict[str, str]], field: str) -> Optional[float]:
    """计算某字段在同一 planning cycle 内的最大最小差。"""
    field_values = values(rows, field)
    if len(field_values) <= 1:
        return None
    return max(field_values) - min(field_values)


def summarize_risk_grid_behavior_variation(
    rows: List[Dict[str, str]]
) -> Dict[str, SummaryValue]:
    """统计同一 planning cycle 内不同 behavior 的 risk grid 差异度。"""
    grouped_rows = group_risk_grid_rows_by_planning_key(rows)
    summary: Dict[str, SummaryValue] = {
        "risk_grid_behavior_groups": len(grouped_rows),
        "risk_grid_behavior_multi_behavior_groups": 0,
        "risk_grid_behavior_variant_groups": 0,
        "risk_grid_behavior_unique_signature_groups": 0,
    }
    if not grouped_rows:
        return summary

    range_fields = [
        "nonzero_cells",
        "max_risk",
        "sum_risk",
        "active_time_layers",
        "risk_source_vehicles",
        "risk_source_modes",
    ]
    ranges_by_field: Dict[str, List[float]] = {field: [] for field in range_fields}

    for rows_in_cycle in grouped_rows.values():
        behavior_ids = {
            risk_grid_behavior_id(row, row_index)
            for row_index, row in enumerate(rows_in_cycle)
        }
        if len(behavior_ids) <= 1:
            continue

        summary["risk_grid_behavior_multi_behavior_groups"] = (
            int(summary["risk_grid_behavior_multi_behavior_groups"]) + 1
        )
        signatures = {risk_grid_signature(row) for row in rows_in_cycle}
        if len(signatures) > 1:
            summary["risk_grid_behavior_unique_signature_groups"] = (
                int(summary["risk_grid_behavior_unique_signature_groups"]) + 1
            )

        has_numeric_variation = False
        for field in range_fields:
            field_range = numeric_range(rows_in_cycle, field)
            if field_range is None:
                continue
            ranges_by_field[field].append(field_range)
            if field_range > RANGE_EPSILON:
                has_numeric_variation = True
        if has_numeric_variation:
            summary["risk_grid_behavior_variant_groups"] = (
                int(summary["risk_grid_behavior_variant_groups"]) + 1
            )

    multi_behavior_groups = int(summary["risk_grid_behavior_multi_behavior_groups"])
    if multi_behavior_groups > 0:
        summary["risk_grid_behavior_variant_ratio"] = (
            int(summary["risk_grid_behavior_variant_groups"]) / multi_behavior_groups
        )
        summary["risk_grid_behavior_unique_signature_ratio"] = (
            int(summary["risk_grid_behavior_unique_signature_groups"])
            / multi_behavior_groups
        )
    else:
        summary["risk_grid_behavior_variant_ratio"] = None
        summary["risk_grid_behavior_unique_signature_ratio"] = None

    for field, field_ranges in ranges_by_field.items():
        summary.update(
            summarize_value_list(f"risk_grid_behavior_{field}_range", field_ranges)
        )
    return summary


def summarize_exposure_cycles(rows: List[Dict[str, str]]) -> Dict[str, SummaryValue]:
    """按规划周期汇总 risk exposure，专门用于 fallback/adaptive 触发次数统计。"""
    grouped_rows = group_rows_by_cycle(rows)
    cycle_rows = list(grouped_rows.values())
    summary: Dict[str, SummaryValue] = {"risk_exposure_cycles": len(cycle_rows)}
    if not cycle_rows:
        return summary

    selected_rows: List[Dict[str, str]] = []
    baseline_rows: List[Dict[str, str]] = []
    fallback_triggered_cycles = 0
    fallback_switched_cycles = 0
    adaptive_enabled_cycles = 0
    high_interaction_risk_cycles = 0

    for rows_in_cycle in cycle_rows:
        selected_row = next(
            (row for row in rows_in_cycle if parse_int(row.get("is_selected", "")) == 1),
            None,
        )
        baseline_row = next(
            (row for row in rows_in_cycle if parse_int(row.get("is_baseline", "")) == 1),
            None,
        )
        if selected_row is not None:
            selected_rows.append(selected_row)
        if baseline_row is not None:
            baseline_rows.append(baseline_row)
        if any(parse_int(row.get("safety_fallback_triggered", "")) == 1 for row in rows_in_cycle):
            fallback_triggered_cycles += 1
        if any(parse_int(row.get("safety_fallback_switched", "")) == 1 for row in rows_in_cycle):
            fallback_switched_cycles += 1
        if any(parse_int(row.get("adaptive_enabled", "")) == 1 for row in rows_in_cycle):
            adaptive_enabled_cycles += 1
        if any(parse_int(row.get("high_interaction_risk", "")) == 1 for row in rows_in_cycle):
            high_interaction_risk_cycles += 1

    summary["cycle_selected_rows"] = len(selected_rows)
    summary["cycle_baseline_rows"] = len(baseline_rows)
    summary["safety_fallback_triggered_cycles"] = fallback_triggered_cycles
    summary["safety_fallback_switched_cycles"] = fallback_switched_cycles
    summary["adaptive_enabled_cycles"] = adaptive_enabled_cycles
    summary["high_interaction_risk_cycles"] = high_interaction_risk_cycles

    # 周期级 selected/baseline 统计更适合论文表格，避免多候选行造成样本数膨胀。
    for prefix, subset_rows in [
        ("cycle_selected", selected_rows),
        ("cycle_baseline", baseline_rows),
    ]:
        for field in [
            "exposure_sum",
            "exposure_mean",
            "exposure_max",
            "high_risk_hits",
            "risk_score",
        ]:
            summary.update(
                {
                    f"{prefix}_{key}": value
                    for key, value in summarize_numeric(subset_rows, field).items()
                }
            )
    return summary


def summarize_risk_grid(rows: List[Dict[str, str]]) -> Dict[str, SummaryValue]:
    """汇总 risk grid 统计 CSV。"""
    summary: Dict[str, SummaryValue] = {
        "risk_grid_rows": len(rows),
        "risk_grid_map_builds": len(rows),
    }
    for field in [
        "nonzero_cells",
        "max_risk",
        "sum_risk",
        "active_time_layers",
        "total_time_layers",
        "risk_source_vehicles",
        "risk_source_modes",
    ]:
        summary.update(summarize_numeric(rows, field))
    summary.update(summarize_risk_grid_behavior_variation(rows))
    return summary


def summarize_exposure(rows: List[Dict[str, str]]) -> Dict[str, SummaryValue]:
    """汇总候选轨迹 risk exposure CSV。"""
    summary: Dict[str, SummaryValue] = {"risk_exposure_rows": len(rows)}
    for field in [
        "exposure_sum",
        "exposure_mean",
        "exposure_max",
        "high_risk_hits",
        "risk_score",
    ]:
        summary.update(summarize_numeric(rows, field))

    selected_rows = [row for row in rows if parse_int(row.get("is_selected", "")) == 1]
    baseline_rows = [row for row in rows if parse_int(row.get("is_baseline", "")) == 1]
    summary["selected_rows"] = len(selected_rows)
    summary["baseline_rows"] = len(baseline_rows)
    # selected/baseline 分开统计，便于论文中直接比较“最终轨迹”和“原始 SSC 候选”。
    for prefix, subset_rows in [
        ("selected", selected_rows),
        ("baseline", baseline_rows),
    ]:
        for field in [
            "exposure_sum",
            "exposure_mean",
            "exposure_max",
            "high_risk_hits",
            "risk_score",
        ]:
            summary.update(
                {
                    f"{prefix}_{key}": value
                    for key, value in summarize_numeric(subset_rows, field).items()
                }
            )

    adaptive_rows = [
        row for row in rows if parse_int(row.get("adaptive_enabled", "")) == 1
    ]
    high_interaction_rows = [
        row for row in rows if parse_int(row.get("high_interaction_risk", "")) == 1
    ]
    summary["adaptive_enabled_rows"] = len(adaptive_rows)
    summary["high_interaction_risk_rows"] = len(high_interaction_rows)

    fallback_rows = [
        row for row in rows if parse_int(row.get("safety_fallback_triggered", "")) == 1
    ]
    switched_rows = [
        row for row in rows if parse_int(row.get("safety_fallback_switched", "")) == 1
    ]
    summary["safety_fallback_triggered_rows"] = len(fallback_rows)
    summary["safety_fallback_switched_rows"] = len(switched_rows)
    summary.update(summarize_exposure_cycles(rows))
    return summary


def summarize_scripted_actor_telemetry(
    rows: List[Dict[str, str]]
) -> Dict[str, SummaryValue]:
    """汇总脚本化周车 telemetry，证明主动交互 actor 的运行过程可追溯。"""
    actor_ids = sorted({row.get("actor_id", "").strip() for row in rows if row.get("actor_id")})
    modes = sorted({row.get("mode", "").strip() for row in rows if row.get("mode")})
    idm_rows = [
        row
        for row in rows
        if row.get("mode", "").strip() == "idm_follow"
        or parse_float(row.get("idm_acceleration", "")) is not None
    ]
    summary: Dict[str, SummaryValue] = {
        "scripted_actor_telemetry_rows": len(rows),
        "scripted_actor_telemetry_actor_count": len(actor_ids),
        "scripted_actor_telemetry_actor_ids": ",".join(actor_ids),
        "scripted_actor_telemetry_modes": ",".join(modes),
        "scripted_actor_telemetry_idm_rows": len(idm_rows),
    }

    for metric_name, field in [
        ("scripted_actor_gap", "gap"),
        ("scripted_actor_relative_velocity", "relative_velocity"),
        ("scripted_actor_desired_gap", "desired_gap"),
        ("scripted_actor_idm_acceleration", "idm_acceleration"),
        ("scripted_actor_command_velocity", "command_velocity"),
        ("scripted_actor_command_acceleration", "command_acceleration"),
    ]:
        field_values = values(rows, field)
        summary.update(summarize_value_list(metric_name, field_values))
        summary[f"{metric_name}_min"] = min(field_values) if field_values else None
    return summary


def write_summary_csv(summary: Dict[str, SummaryValue], output_path: Path) -> None:
    """将 summary 写成两列表格，便于论文表格或电子表格读取。"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["metric", "value"])
        for key in sorted(summary):
            writer.writerow([key, summary[key]])


def plot_series(rows: List[Dict[str, str]], field: str, output_path: Path) -> None:
    """若安装 matplotlib，则输出单字段折线图。"""
    field_values = values(rows, field)
    if not field_values:
        return
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        return

    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(8, 4))
    plt.plot(range(len(field_values)), field_values, linewidth=1.5)
    plt.xlabel("sample")
    plt.ylabel(field)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()


def build_argument_parser() -> argparse.ArgumentParser:
    """构造命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        description="生成 EPSILON risk-aware SSC 离线实验汇总报告。"
    )
    parser.add_argument(
        "--risk-grid-csv",
        default="/tmp/epsilon_risk_grid_stats.csv",
        help="MVP-1B risk grid 统计 CSV 路径。",
    )
    parser.add_argument(
        "--risk-exposure-csv",
        default="/tmp/epsilon_mvp6_risk_exposure.csv",
        help="MVP-6/7/8 候选轨迹 risk exposure CSV 路径。",
    )
    parser.add_argument(
        "--scripted-actor-telemetry-csv",
        default="/tmp/epsilon_scripted_risk_actor_telemetry.csv",
        help="MVP-19 脚本化周车 telemetry CSV 路径。",
    )
    parser.add_argument(
        "--output-dir",
        default="/tmp/epsilon_risk_experiment_report",
        help="summary CSV/JSON 和可选趋势图输出目录。",
    )
    parser.add_argument(
        "--experiment-name",
        default="",
        help="实验组名称，例如 baseline_ssc 或 safety_fallback。",
    )
    parser.add_argument(
        "--scenario-name",
        default="",
        help="场景名称，例如 cut_in、follow 或 merge。",
    )
    parser.add_argument(
        "--git-ref",
        default="",
        help="实验对应的 commit 或 tag，用于论文数据追溯。",
    )
    parser.add_argument(
        "--scripted-risk-actors-enabled",
        action="store_true",
        help="记录本实验是否启用了 MVP-16 脚本化周车。",
    )
    parser.add_argument(
        "--scripted-risk-actor-count",
        type=int,
        default=0,
        help="记录脚本化周车数量。",
    )
    parser.add_argument(
        "--risk-actor-script-path",
        default="",
        help="记录脚本化周车 JSON 路径，便于论文实验追溯。",
    )
    return parser


def main() -> int:
    """脚本入口：读取 CSV，输出 summary 和可选图表。"""
    args = build_argument_parser().parse_args()
    output_dir = Path(args.output_dir)
    risk_grid_rows = read_csv(Path(args.risk_grid_csv))
    risk_exposure_rows = read_csv(Path(args.risk_exposure_csv))
    scripted_actor_telemetry_rows = read_csv(Path(args.scripted_actor_telemetry_csv))

    summary: Dict[str, SummaryValue] = {
        "experiment_name": args.experiment_name,
        "scenario_name": args.scenario_name,
        "git_ref": args.git_ref,
        "risk_grid_csv": str(Path(args.risk_grid_csv)),
        "risk_exposure_csv": str(Path(args.risk_exposure_csv)),
        "scripted_actor_telemetry_csv": str(Path(args.scripted_actor_telemetry_csv)),
        "scripted_risk_actors_enabled": int(args.scripted_risk_actors_enabled),
        "scripted_risk_actor_count": args.scripted_risk_actor_count,
        "risk_actor_script_path": args.risk_actor_script_path,
    }
    summary.update(summarize_risk_grid(risk_grid_rows))
    summary.update(summarize_exposure(risk_exposure_rows))
    summary.update(summarize_scripted_actor_telemetry(scripted_actor_telemetry_rows))

    output_dir.mkdir(parents=True, exist_ok=True)
    write_summary_csv(summary, output_dir / "summary.csv")
    with (output_dir / "summary.json").open("w", encoding="utf-8") as json_file:
        json.dump(summary, json_file, indent=2, sort_keys=True)

    plot_series(risk_grid_rows, "sum_risk", output_dir / "risk_grid_sum.png")
    plot_series(risk_grid_rows, "nonzero_cells", output_dir / "risk_grid_nonzero.png")
    plot_series(risk_exposure_rows, "exposure_max", output_dir / "trajectory_exposure_max.png")
    plot_series(risk_exposure_rows, "risk_score", output_dir / "trajectory_risk_score.png")
    plot_series(scripted_actor_telemetry_rows, "gap", output_dir / "scripted_actor_gap.png")
    plot_series(
        scripted_actor_telemetry_rows,
        "command_acceleration",
        output_dir / "scripted_actor_command_acceleration.png",
    )

    print(f"summary_csv={output_dir / 'summary.csv'}")
    print(f"summary_json={output_dir / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
