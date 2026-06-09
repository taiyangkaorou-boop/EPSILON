#!/usr/bin/env python3
"""EPSILON 风险感知 SSC 多实验 summary 矩阵汇总工具。

该脚本读取多个 `risk_experiment_report.py` 生成的 summary.csv，将每个实验
压缩成一行对比矩阵，便于论文消融实验直接制表。脚本只做离线 CSV/JSON 处理，
不依赖 ROS 运行时，也不改变规划行为。
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, Iterable, List, Optional


SummaryRow = Dict[str, str]


DEFAULT_METRICS = [
    "risk_grid_rows",
    "sum_risk_mean",
    "max_risk_max",
    "nonzero_cells_mean",
    "risk_grid_behavior_variant_ratio",
    "risk_grid_behavior_sum_risk_range_max",
    "risk_exposure_cycles",
    "cycle_selected_exposure_sum_mean",
    "cycle_selected_exposure_max_max",
    "cycle_baseline_exposure_sum_mean",
    "safety_fallback_triggered_cycles",
    "safety_fallback_switched_cycles",
    "adaptive_enabled_cycles",
]


def read_summary_csv(path: Path) -> SummaryRow:
    """读取两列形式的 summary.csv，返回 metric -> value 字典。"""
    summary: SummaryRow = {}
    with path.open("r", newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            metric = row.get("metric", "").strip()
            if not metric:
                continue
            summary[metric] = row.get("value", "").strip()
    return summary


def infer_summary_path(item: str) -> Path:
    """支持传入 summary.csv 文件或包含 summary.csv 的目录。"""
    path = Path(item)
    if path.is_dir():
        return path / "summary.csv"
    return path


def parse_named_input(item: str) -> tuple[Optional[str], Path]:
    """解析 name=path 形式的输入，方便给实验组手动命名。"""
    if "=" not in item:
        return None, infer_summary_path(item)
    name, path_text = item.split("=", 1)
    return name.strip() or None, infer_summary_path(path_text.strip())


def matrix_row(name: Optional[str], path: Path, metrics: Iterable[str]) -> SummaryRow:
    """将单个 summary.csv 转为矩阵中的一行。"""
    summary = read_summary_csv(path)
    experiment_name = name or summary.get("experiment_name") or path.parent.name
    row: SummaryRow = {
        "experiment_name": experiment_name,
        "scenario_name": summary.get("scenario_name", ""),
        "git_ref": summary.get("git_ref", ""),
        "summary_csv": str(path),
    }
    for metric in metrics:
        row[metric] = summary.get(metric, "")
    return row


def write_matrix_csv(rows: List[SummaryRow], output_path: Path, fields: List[str]) -> None:
    """写出实验矩阵 CSV。"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def build_argument_parser() -> argparse.ArgumentParser:
    """构造命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        description="汇总多个 EPSILON risk experiment summary.csv 为论文消融矩阵。"
    )
    parser.add_argument(
        "--summary",
        action="append",
        required=True,
        help=(
            "输入 summary.csv 或其所在目录；可写成 name=/path/to/summary.csv。"
            "该参数可重复。"
        ),
    )
    parser.add_argument(
        "--metric",
        action="append",
        default=[],
        help="额外输出的 metric 名称，可重复；默认指标会保留。",
    )
    parser.add_argument(
        "--output-dir",
        default="/tmp/epsilon_risk_experiment_matrix",
        help="matrix.csv / matrix.json 输出目录。",
    )
    return parser


def main() -> int:
    """脚本入口：读取多个 summary，输出矩阵 CSV/JSON。"""
    args = build_argument_parser().parse_args()
    metrics = list(dict.fromkeys(DEFAULT_METRICS + args.metric))
    rows: List[SummaryRow] = []

    for item in args.summary:
        name, path = parse_named_input(item)
        if not path.exists():
            raise FileNotFoundError(f"summary CSV not found: {path}")
        rows.append(matrix_row(name, path, metrics))

    output_dir = Path(args.output_dir)
    fields = ["experiment_name", "scenario_name", "git_ref", "summary_csv"] + metrics
    write_matrix_csv(rows, output_dir / "matrix.csv", fields)
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "matrix.json").open("w", encoding="utf-8") as json_file:
        json.dump(rows, json_file, indent=2, ensure_ascii=False)

    print(f"matrix_csv={output_dir / 'matrix.csv'}")
    print(f"matrix_json={output_dir / 'matrix.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
