#!/usr/bin/env python3
"""EPSILON 风险实验输出完整性校验工具。

该脚本只读取 batch/suite 产生的离线文件，不启动 ROS，也不改变规划行为。
它用于在论文实验入表前检查 manifest、原始 CSV、summary 和 matrix 是否齐全。
"""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional


ReportRow = Dict[str, object]


@dataclass(frozen=True)
class CsvInfo:
    """记录 CSV 文件是否存在、是否非空以及数据行数量。"""

    path: Path
    exists: bool
    nonempty: bool
    rows: int


@dataclass(frozen=True)
class PathResolver:
    """把 manifest 中的路径解析到当前被校验的输出目录，支持归档目录搬迁。"""

    base_dir: Path
    manifest_output_root: Optional[Path] = None

    def resolve(self, path_text: object) -> Path:
        """解析路径；若绝对路径位于旧 output_root 下，则重定位到当前 base_dir。"""
        path = Path(str(path_text))
        if not path.is_absolute():
            return self.base_dir / path
        if self.manifest_output_root is not None:
            try:
                relative_path = path.relative_to(self.manifest_output_root)
            except ValueError:
                return path
            return self.base_dir / relative_path
        return path


def read_json(path: Path) -> Dict[str, object]:
    """读取 JSON 文件；调用方负责保证文件存在。"""
    return json.loads(path.read_text(encoding="utf-8"))


def csv_info(path: Path) -> CsvInfo:
    """读取 CSV 元信息；空文件或无法解析时按 0 行处理。"""
    if not path.exists():
        return CsvInfo(path=path, exists=False, nonempty=False, rows=0)
    nonempty = path.stat().st_size > 0
    if not nonempty:
        return CsvInfo(path=path, exists=True, nonempty=False, rows=0)
    try:
        with path.open("r", newline="", encoding="utf-8") as csv_file:
            rows = list(csv.DictReader(csv_file))
    except Exception:
        return CsvInfo(path=path, exists=True, nonempty=nonempty, rows=0)
    return CsvInfo(path=path, exists=True, nonempty=nonempty, rows=len(rows))


def bool_from_manifest(value: object) -> bool:
    """兼容 manifest 中的 bool、int 和字符串布尔值。"""
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return value != 0
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def row_status(ok: bool) -> str:
    """把布尔校验结果转为稳定状态字符串。"""
    return "ok" if ok else "failed"


def append_check(
    rows: List[ReportRow],
    scope: str,
    item: str,
    check: str,
    ok: bool,
    detail: str,
) -> None:
    """向报告追加一条校验结果。"""
    rows.append(
        {
            "scope": scope,
            "item": item,
            "check": check,
            "status": row_status(ok),
            "detail": detail,
        }
    )


def verify_csv_file(
    rows: List[ReportRow],
    scope: str,
    item: str,
    check_name: str,
    path: Path,
    required: bool,
) -> CsvInfo:
    """校验单个 CSV 文件；required 为 False 时缺失不视为失败。"""
    info = csv_info(path)
    ok = (info.exists and info.nonempty and info.rows > 0) if required else True
    append_check(
        rows,
        scope,
        item,
        check_name,
        ok,
        f"path={path}; exists={info.exists}; nonempty={info.nonempty}; rows={info.rows}; required={required}",
    )
    return info


def suite_manifest_mode(output_root: Path) -> Optional[str]:
    """读取 scenario suite manifest 的 mode；不存在时返回 None。"""
    if not output_root.is_dir():
        return None
    suite_manifest = output_root / "scenario_suite_manifest.json"
    if not suite_manifest.exists():
        return None
    suite_data = read_json(suite_manifest)
    return str(suite_data.get("mode", ""))


def verify_batch_manifest(manifest_path: Path) -> tuple[List[ReportRow], bool]:
    """校验单个 risk_experiment_batch.py 输出目录。"""
    rows: List[ReportRow] = []
    batch_dir = manifest_path.parent
    append_check(
        rows,
        "batch",
        str(batch_dir),
        "manifest_exists",
        manifest_path.exists(),
        f"path={manifest_path}",
    )
    if not manifest_path.exists():
        return rows, False

    manifest = read_json(manifest_path)
    manifest_output_root = manifest.get("output_root")
    resolver = PathResolver(
        base_dir=batch_dir,
        manifest_output_root=Path(str(manifest_output_root))
        if manifest_output_root
        else None,
    )
    mode = str(manifest.get("mode", ""))
    dry_run_mode = mode == "dry_run"
    append_check(
        rows,
        "batch",
        str(batch_dir),
        "mode_valid",
        mode in {"dry_run", "execute"},
        f"mode={mode}",
    )
    run_plan = resolver.resolve(manifest.get("run_plan", batch_dir / "run_plan.sh"))
    append_check(
        rows,
        "batch",
        str(batch_dir),
        "run_plan_exists",
        run_plan.exists() and run_plan.stat().st_size > 0,
        f"path={run_plan}",
    )
    experiments = manifest.get("experiments", [])
    experiments_ok = isinstance(experiments, list)
    append_check(
        rows,
        "batch",
        str(batch_dir),
        "experiments_list",
        experiments_ok,
        f"count={len(experiments) if isinstance(experiments, list) else 'invalid'}",
    )
    if not experiments_ok:
        return rows, False

    for raw_experiment in experiments:
        if not isinstance(raw_experiment, dict):
            append_check(rows, "experiment", "invalid", "experiment_object", False, "")
            continue
        name = str(raw_experiment.get("name", ""))
        status = str(raw_experiment.get("status", ""))
        experiment_dir = resolver.resolve(raw_experiment.get("output_dir", ""))
        expected_ok = status == "dry_run" if dry_run_mode else status == "ok"
        append_check(
            rows,
            "experiment",
            name,
            "status_valid",
            expected_ok,
            f"mode={mode}; status={status}; output_dir={experiment_dir}",
        )
        if not expected_ok or dry_run_mode:
            continue

        verify_csv_file(
            rows,
            "experiment",
            name,
            "risk_grid_archive",
            resolver.resolve(raw_experiment.get("risk_grid_archive", "")),
            required=True,
        )
        verify_csv_file(
            rows,
            "experiment",
            name,
            "summary_csv",
            resolver.resolve(raw_experiment.get("summary_csv", "")),
            required=True,
        )
        risk_exposure_present = bool_from_manifest(
            raw_experiment.get("risk_exposure_present", False)
        )
        verify_csv_file(
            rows,
            "experiment",
            name,
            "risk_exposure_archive",
            resolver.resolve(raw_experiment.get("risk_exposure_archive", "")),
            required=risk_exposure_present,
        )
        scripted_enabled = bool_from_manifest(
            raw_experiment.get("scripted_risk_actors_enabled", False)
        )
        verify_csv_file(
            rows,
            "experiment",
            name,
            "scripted_actor_telemetry_archive",
            resolver.resolve(raw_experiment.get("scripted_actor_telemetry_archive", "")),
            required=scripted_enabled,
        )

    matrix_csv = batch_dir / "matrix" / "matrix.csv"
    verify_csv_file(
        rows,
        "batch",
        str(batch_dir),
        "matrix_csv",
        matrix_csv,
        required=(
            not dry_run_mode
            and any(
                isinstance(experiment, dict) and experiment.get("status") == "ok"
                for experiment in experiments
            )
        ),
    )
    return rows, all(row["status"] == "ok" for row in rows)


def discover_batch_manifests(output_root: Path) -> List[Path]:
    """从输入路径发现 batch manifest；支持直接传 manifest、batch 目录或 suite 目录。"""
    if output_root.is_file() and output_root.name == "manifest.json":
        return [output_root]
    direct_manifest = output_root / "manifest.json"
    if direct_manifest.exists():
        return [direct_manifest]
    suite_manifest = output_root / "scenario_suite_manifest.json"
    if suite_manifest.exists():
        suite_data = read_json(suite_manifest)
        if str(suite_data.get("mode", "")) == "dry_run":
            return []
        manifests: List[Path] = []
        for raw_scenario in suite_data.get("scenarios", []):
            if not isinstance(raw_scenario, dict):
                continue
            resolver = PathResolver(
                base_dir=output_root,
                manifest_output_root=Path(str(suite_data.get("output_root")))
                if suite_data.get("output_root")
                else None,
            )
            manifest_path = resolver.resolve(raw_scenario.get("manifest_path", ""))
            manifests.append(manifest_path)
        return manifests
    return sorted(output_root.glob("*/manifest.json"))


def verify_suite_manifest(output_root: Path, rows: List[ReportRow]) -> None:
    """如果存在 scenario suite manifest，则校验 suite 顶层结构和场景 matrix。"""
    suite_manifest = output_root / "scenario_suite_manifest.json"
    if not suite_manifest.exists():
        return
    suite_data = read_json(suite_manifest)
    resolver = PathResolver(
        base_dir=output_root,
        manifest_output_root=Path(str(suite_data.get("output_root")))
        if suite_data.get("output_root")
        else None,
    )
    mode = str(suite_data.get("mode", ""))
    dry_run_mode = mode == "dry_run"
    scenarios = suite_data.get("scenarios", [])
    append_check(
        rows,
        "suite",
        str(output_root),
        "mode_valid",
        mode in {"dry_run", "execute"},
        f"mode={mode}",
    )
    scenario_run_plan = resolver.resolve(
        suite_data.get("scenario_run_plan", output_root / "scenario_run_plan.sh")
    )
    append_check(
        rows,
        "suite",
        str(output_root),
        "scenario_run_plan_exists",
        scenario_run_plan.exists() and scenario_run_plan.stat().st_size > 0,
        f"path={scenario_run_plan}",
    )
    append_check(
        rows,
        "suite",
        str(output_root),
        "scenario_suite_manifest",
        isinstance(scenarios, list),
        f"path={suite_manifest}; count={len(scenarios) if isinstance(scenarios, list) else 'invalid'}",
    )
    if not isinstance(scenarios, list):
        return
    for raw_scenario in scenarios:
        if not isinstance(raw_scenario, dict):
            append_check(rows, "suite_scenario", "invalid", "scenario_object", False, "")
            continue
        scenario_name = str(raw_scenario.get("scenario_name", ""))
        status = str(raw_scenario.get("status", ""))
        expected_ok = status == "dry_run" if dry_run_mode else status == "ok"
        append_check(
            rows,
            "suite_scenario",
            scenario_name,
            "status_valid",
            expected_ok,
            f"mode={mode}; status={status}",
        )
        if expected_ok and not dry_run_mode:
            verify_csv_file(
                rows,
                "suite_scenario",
                scenario_name,
                "scenario_matrix_csv",
                resolver.resolve(raw_scenario.get("matrix_csv", "")),
                required=True,
            )
            verify_csv_file(
                rows,
                "suite_scenario",
                scenario_name,
                "copied_matrix_by_scenario",
                output_root / "matrix_by_scenario" / f"{scenario_name}.csv",
                required=True,
            )


def write_report(rows: List[ReportRow], output_dir: Path) -> None:
    """写出 JSON/CSV 两种格式，便于命令行阅读和表格归档。"""
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "verification_report.json"
    csv_path = output_dir / "verification_report.csv"
    json_path.write_text(json.dumps(rows, indent=2, ensure_ascii=False), encoding="utf-8")
    fields = ["scope", "item", "check", "status", "detail"]
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"verification_json={json_path}")
    print(f"verification_csv={csv_path}")


def build_argument_parser() -> argparse.ArgumentParser:
    """构造命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        description="校验 EPSILON risk experiment batch/suite 输出完整性。"
    )
    parser.add_argument(
        "output_root",
        type=Path,
        help="batch 输出目录、suite 输出目录，或单个 manifest.json。",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="校验报告输出目录；默认写入 output_root/verification。",
    )
    return parser


def main() -> int:
    """脚本入口：发现 manifest，逐个校验并输出汇总报告。"""
    args = build_argument_parser().parse_args()
    output_root = args.output_root.resolve()
    report_output_dir = args.output_dir or (
        output_root.parent / "verification" if output_root.is_file() else output_root / "verification"
    )

    rows: List[ReportRow] = []
    append_check(
        rows,
        "root",
        str(output_root),
        "output_root_exists",
        output_root.exists(),
        f"path={output_root}",
    )
    if not output_root.exists():
        write_report(rows, report_output_dir)
        return 1

    verify_suite_manifest(output_root if output_root.is_dir() else output_root.parent, rows)
    manifests = discover_batch_manifests(output_root)
    suite_mode = suite_manifest_mode(output_root)
    batch_manifest_required = suite_mode != "dry_run"
    append_check(
        rows,
        "root",
        str(output_root),
        "batch_manifest_count",
        len(manifests) > 0 or not batch_manifest_required,
        f"count={len(manifests)}; required={batch_manifest_required}",
    )

    all_ok = True
    for manifest_path in manifests:
        batch_rows, batch_ok = verify_batch_manifest(manifest_path)
        rows.extend(batch_rows)
        all_ok = all_ok and batch_ok

    all_ok = all_ok and all(row["status"] == "ok" for row in rows)
    write_report(rows, report_output_dir)
    failed_count = sum(1 for row in rows if row["status"] != "ok")
    print(f"verification_status={row_status(all_ok)}")
    print(f"failed_checks={failed_count}")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
