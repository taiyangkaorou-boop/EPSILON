#!/usr/bin/env python3
"""EPSILON 风险实验论文图表包生成工具。

该脚本读取 risk_experiment_matrix.py 生成的 matrix.csv，整理常用论文指标，
并在本机安装 matplotlib 时额外生成 PNG 图表。脚本只处理离线文件，不启动 ROS。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


MatrixRow = Dict[str, str]
PlotRow = Dict[str, str]


DEFAULT_METRICS = [
    "sum_risk_mean",
    "max_risk_max",
    "nonzero_cells_mean",
    "cycle_selected_exposure_sum_mean",
    "cycle_selected_exposure_max_max",
    "cycle_baseline_exposure_sum_mean",
    "safety_fallback_triggered_cycles",
    "safety_fallback_switched_cycles",
    "adaptive_enabled_cycles",
    "scripted_actor_gap_min",
    "scripted_actor_idm_acceleration_min",
    "scripted_actor_command_acceleration_min",
]


@dataclass(frozen=True)
class MatrixInput:
    """单个 matrix.csv 输入及其显示名称。"""

    name: str
    path: Path


def parse_float(value: str) -> Optional[float]:
    """安全解析浮点数；空值、nan、inf 都视为缺失。"""
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


def slugify(text: str) -> str:
    """把场景/指标名称转为适合文件名的短字符串。"""
    result = []
    for char in text.lower():
        if char.isalnum():
            result.append(char)
        else:
            result.append("_")
    slug = "".join(result).strip("_")
    while "__" in slug:
        slug = slug.replace("__", "_")
    return slug or "metric"


def read_matrix(path: Path) -> List[MatrixRow]:
    """读取 matrix.csv；不存在或无行时由调用方报错。"""
    with path.open("r", newline="", encoding="utf-8") as csv_file:
        return list(csv.DictReader(csv_file))


def infer_matrix_path(item: str) -> Path:
    """支持传入 matrix.csv 或包含 matrix/matrix.csv 的目录。"""
    path = Path(item)
    if path.is_dir():
        nested = path / "matrix" / "matrix.csv"
        if nested.exists():
            return nested
        return path / "matrix.csv"
    return path


def parse_named_matrix(item: str) -> MatrixInput:
    """解析 name=path 形式的 matrix 输入。"""
    if "=" not in item:
        path = infer_matrix_path(item)
        return MatrixInput(name=path.parent.name, path=path)
    name, path_text = item.split("=", 1)
    path = infer_matrix_path(path_text.strip())
    return MatrixInput(name=name.strip() or path.parent.name, path=path)


def discover_matrices(root: Path) -> List[MatrixInput]:
    """从 batch 或 scenario suite 输出目录自动发现 matrix.csv。"""
    candidates: List[MatrixInput] = []
    if root.is_file() and root.name == "matrix.csv":
        return [MatrixInput(name=root.parent.name, path=root)]
    direct = root / "matrix" / "matrix.csv"
    if direct.exists():
        candidates.append(MatrixInput(name=root.name, path=direct))
    matrix_by_scenario = root / "matrix_by_scenario"
    if matrix_by_scenario.is_dir():
        for path in sorted(matrix_by_scenario.glob("*.csv")):
            candidates.append(MatrixInput(name=path.stem, path=path))
    for path in sorted(root.glob("*/matrix/matrix.csv")):
        scenario_name = path.parents[1].name
        candidates.append(MatrixInput(name=scenario_name, path=path))

    # 去重时保留先发现的显式 batch matrix，避免同一路径重复入表。
    unique: Dict[Path, MatrixInput] = {}
    for candidate in candidates:
        unique.setdefault(candidate.path.resolve(), candidate)
    return list(unique.values())


def collect_plot_rows(matrix_inputs: Iterable[MatrixInput], metrics: List[str]) -> List[PlotRow]:
    """把多个 matrix 展开成长表，便于画图和导入电子表格。"""
    plot_rows: List[PlotRow] = []
    for matrix_input in matrix_inputs:
        rows = read_matrix(matrix_input.path)
        for row in rows:
            experiment = row.get("experiment_name", "").strip() or "experiment"
            scenario = row.get("scenario_name", "").strip() or matrix_input.name
            for metric in metrics:
                value = parse_float(row.get(metric, ""))
                if value is None:
                    continue
                plot_rows.append(
                    {
                        "matrix_name": matrix_input.name,
                        "matrix_csv": str(matrix_input.path),
                        "scenario_name": scenario,
                        "experiment_name": experiment,
                        "metric": metric,
                        "value": f"{value:.9g}",
                    }
                )
    return plot_rows


def write_plot_values(rows: List[PlotRow], output_path: Path) -> None:
    """写出长表 CSV，作为论文绘图和复核的稳定中间产物。"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "matrix_name",
        "matrix_csv",
        "scenario_name",
        "experiment_name",
        "metric",
        "value",
    ]
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def group_values_by_metric(rows: List[PlotRow]) -> Dict[str, List[Tuple[str, float]]]:
    """按 metric 聚合绘图数据，标签使用 scenario/experiment 组合。"""
    grouped: Dict[str, List[Tuple[str, float]]] = {}
    for row in rows:
        value = parse_float(row["value"])
        if value is None:
            continue
        label = f"{row['scenario_name']}:{row['experiment_name']}"
        grouped.setdefault(row["metric"], []).append((label, value))
    return grouped


def maybe_plot_metric_bars(rows: List[PlotRow], output_dir: Path) -> List[str]:
    """如果 matplotlib 可用，则为每个指标生成横向柱状图。"""
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        return []

    image_paths: List[str] = []
    grouped = group_values_by_metric(rows)
    output_dir.mkdir(parents=True, exist_ok=True)
    for metric, label_values in grouped.items():
        if not label_values:
            continue
        label_values = sorted(label_values, key=lambda item: item[0])
        labels = [item[0] for item in label_values]
        values = [item[1] for item in label_values]
        height = max(3.5, 0.35 * len(labels) + 1.5)
        plt.figure(figsize=(10, height))
        plt.barh(range(len(values)), values, color="#4C78A8")
        plt.yticks(range(len(labels)), labels, fontsize=8)
        plt.xlabel(metric)
        plt.grid(axis="x", alpha=0.25)
        plt.tight_layout()
        output_path = output_dir / f"{slugify(metric)}.png"
        plt.savefig(output_path, dpi=160)
        plt.close()
        image_paths.append(str(output_path))
    return image_paths


def write_manifest(
    output_dir: Path,
    matrix_inputs: List[MatrixInput],
    metrics: List[str],
    plot_rows: List[PlotRow],
    image_paths: List[str],
) -> None:
    """写出图表包 manifest，记录输入、指标和生成物。"""
    manifest = {
        "matrix_inputs": [
            {"name": item.name, "path": str(item.path)} for item in matrix_inputs
        ],
        "metrics": metrics,
        "plot_values_csv": str(output_dir / "plot_values.csv"),
        "plot_row_count": len(plot_rows),
        "image_count": len(image_paths),
        "image_paths": image_paths,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "figure_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


def build_argument_parser() -> argparse.ArgumentParser:
    """构造命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        description="从 EPSILON risk experiment matrix 生成论文图表包。"
    )
    parser.add_argument(
        "--matrix",
        action="append",
        default=[],
        help="输入 matrix.csv 或目录；可写成 name=/path/to/matrix.csv，可重复。",
    )
    parser.add_argument(
        "--input-root",
        type=Path,
        default=None,
        help="自动发现 matrix 的 batch/suite 输出根目录。",
    )
    parser.add_argument(
        "--metric",
        action="append",
        default=[],
        help="额外绘制的 metric 名称，可重复；默认指标会保留。",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/tmp/epsilon_risk_figure_pack"),
        help="图表包输出目录。",
    )
    return parser


def main() -> int:
    """脚本入口：读取 matrix，输出 plot_values、manifest 和可选 PNG。"""
    args = build_argument_parser().parse_args()
    matrix_inputs = [parse_named_matrix(item) for item in args.matrix]
    if args.input_root is not None:
        matrix_inputs.extend(discover_matrices(args.input_root))
    if not matrix_inputs:
        raise ValueError("at least one --matrix or --input-root is required")

    missing = [item.path for item in matrix_inputs if not item.path.exists()]
    if missing:
        raise FileNotFoundError(f"matrix CSV not found: {missing[0]}")

    metrics = list(dict.fromkeys(DEFAULT_METRICS + args.metric))
    plot_rows = collect_plot_rows(matrix_inputs, metrics)
    output_dir = args.output_dir
    write_plot_values(plot_rows, output_dir / "plot_values.csv")
    image_paths = maybe_plot_metric_bars(plot_rows, output_dir / "figures")
    write_manifest(output_dir, matrix_inputs, metrics, plot_rows, image_paths)

    print(f"figure_manifest={output_dir / 'figure_manifest.json'}")
    print(f"plot_values_csv={output_dir / 'plot_values.csv'}")
    print(f"image_count={len(image_paths)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
