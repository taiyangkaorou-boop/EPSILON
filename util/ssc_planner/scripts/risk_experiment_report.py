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
from typing import Dict, Iterable, List, Optional


Number = Optional[float]


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
    with path.open("r", newline="") as csv_file:
        return list(csv.DictReader(csv_file))


def values(rows: Iterable[Dict[str, str]], field: str) -> List[float]:
    """提取某个数值字段，过滤空值和非法值。"""
    result: List[float] = []
    for row in rows:
        parsed = parse_float(row.get(field, ""))
        if parsed is not None:
            result.append(parsed)
    return result


def summarize_numeric(rows: List[Dict[str, str]], field: str) -> Dict[str, Number]:
    """计算单个数值字段的 count/mean/max/sum。"""
    field_values = values(rows, field)
    if not field_values:
        return {
            f"{field}_count": 0,
            f"{field}_mean": None,
            f"{field}_max": None,
            f"{field}_sum": None,
        }
    return {
        f"{field}_count": len(field_values),
        f"{field}_mean": mean(field_values),
        f"{field}_max": max(field_values),
        f"{field}_sum": sum(field_values),
    }


def summarize_risk_grid(rows: List[Dict[str, str]]) -> Dict[str, Number]:
    """汇总 risk grid 统计 CSV。"""
    summary: Dict[str, Number] = {"risk_grid_cycles": len(rows)}
    for field in [
        "nonzero_cells",
        "max_risk",
        "sum_risk",
        "active_time_layers",
        "total_time_layers",
    ]:
        summary.update(summarize_numeric(rows, field))
    return summary


def summarize_exposure(rows: List[Dict[str, str]]) -> Dict[str, Number]:
    """汇总候选轨迹 risk exposure CSV。"""
    summary: Dict[str, Number] = {"risk_exposure_rows": len(rows)}
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
    summary.update(
        {
            f"selected_{key}": value
            for key, value in summarize_numeric(selected_rows, "exposure_max").items()
        }
    )

    fallback_rows = [
        row for row in rows if parse_int(row.get("safety_fallback_triggered", "")) == 1
    ]
    switched_rows = [
        row for row in rows if parse_int(row.get("safety_fallback_switched", "")) == 1
    ]
    summary["safety_fallback_triggered_rows"] = len(fallback_rows)
    summary["safety_fallback_switched_rows"] = len(switched_rows)
    return summary


def write_summary_csv(summary: Dict[str, Number], output_path: Path) -> None:
    """将 summary 写成两列表格，便于论文表格或电子表格读取。"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as csv_file:
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
        description="Generate EPSILON risk-aware SSC offline experiment reports."
    )
    parser.add_argument(
        "--risk-grid-csv",
        default="/tmp/epsilon_risk_grid_stats.csv",
        help="MVP-1B risk grid statistics CSV path.",
    )
    parser.add_argument(
        "--risk-exposure-csv",
        default="/tmp/epsilon_mvp6_risk_exposure.csv",
        help="MVP-6/7/8 risk exposure CSV path.",
    )
    parser.add_argument(
        "--output-dir",
        default="/tmp/epsilon_risk_experiment_report",
        help="Directory for summary CSV/JSON and optional plots.",
    )
    return parser


def main() -> int:
    """脚本入口：读取 CSV，输出 summary 和可选图表。"""
    args = build_argument_parser().parse_args()
    output_dir = Path(args.output_dir)
    risk_grid_rows = read_csv(Path(args.risk_grid_csv))
    risk_exposure_rows = read_csv(Path(args.risk_exposure_csv))

    summary: Dict[str, Number] = {}
    summary.update(summarize_risk_grid(risk_grid_rows))
    summary.update(summarize_exposure(risk_exposure_rows))

    output_dir.mkdir(parents=True, exist_ok=True)
    write_summary_csv(summary, output_dir / "summary.csv")
    with (output_dir / "summary.json").open("w") as json_file:
        json.dump(summary, json_file, indent=2, sort_keys=True)

    plot_series(risk_grid_rows, "sum_risk", output_dir / "risk_grid_sum.png")
    plot_series(risk_grid_rows, "nonzero_cells", output_dir / "risk_grid_nonzero.png")
    plot_series(risk_exposure_rows, "exposure_max", output_dir / "trajectory_exposure_max.png")
    plot_series(risk_exposure_rows, "risk_score", output_dir / "trajectory_risk_score.png")

    print(f"summary_csv={output_dir / 'summary.csv'}")
    print(f"summary_json={output_dir / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
