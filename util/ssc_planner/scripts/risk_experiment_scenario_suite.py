#!/usr/bin/env python3
"""EPSILON 风险感知 SSC 多场景批量实验套件。

该脚本在 MVP-13 单场景闭环 batch runner 之上再包一层“场景维度”：
默认只生成 dry-run 计划；显式传入 --execute 时，才按多个 playground 依次调用
risk_experiment_batch.py，并把各场景 matrix 复制到总输出目录，便于论文第 7 章复现实验。
"""

from __future__ import annotations

import argparse
import json
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Optional


DEFAULT_SCENARIOS = [
    "highway_v1.0",
    "highway_lite",
    "risk_dense_following_v1.0",
    "risk_merge_pressure_v1.0",
    "risk_lane_change_conflict_v1.0",
    "ring_small_v1.0",
    "ring_tiny_v1.0",
]


@dataclass
class ScenarioResult:
    """单个场景的批量实验结果。"""

    scenario_name: str
    playground: str
    output_dir: str
    command: List[str]
    return_code: Optional[int]
    manifest_path: str
    matrix_csv: str
    status: str


def repo_root_from_script() -> Path:
    """从脚本位置推导 EPSILON 仓库根目录。"""
    return Path(__file__).resolve().parents[3]


def workspace_root_from_repo(repo_root: Path) -> Path:
    """从 EPSILON 仓库根目录推导 colcon workspace 根目录。"""
    return repo_root.parent.parent


def script_dir_from_script() -> Path:
    """返回当前脚本所在目录，用于定位 batch runner。"""
    return Path(__file__).resolve().parent


def default_playgrounds_dir(repo_root: Path) -> Path:
    """返回源码内 playground 根目录，用于校验场景是否存在。"""
    return repo_root / "core" / "playgrounds"


def validate_scenarios(scenarios: Iterable[str], playgrounds_dir: Path) -> None:
    """校验场景目录和三类核心 JSON 是否存在。"""
    required_files = [
        "vehicle_set.json",
        "obstacles_norm.json",
        "lane_net_norm.json",
        "agent_config.json",
    ]
    for scenario in scenarios:
        scenario_dir = playgrounds_dir / scenario
        if not scenario_dir.is_dir():
            raise FileNotFoundError(f"playground not found: {scenario_dir}")
        for file_name in required_files:
            path = scenario_dir / file_name
            if not path.exists():
                raise FileNotFoundError(f"playground file not found: {path}")


def build_batch_command(
    batch_script: Path,
    scenario: str,
    output_dir: Path,
    args: argparse.Namespace,
) -> List[str]:
    """生成调用单场景 batch runner 的命令。"""
    command = [
        sys.executable,
        str(batch_script),
        "--output-root",
        str(output_dir),
        "--scenario-name",
        scenario,
        "--playground",
        scenario,
        "--planner-backend",
        args.planner_backend,
        "--duration-sec",
        str(args.duration_sec),
        "--repo-root",
        str(args.repo_root),
        "--setup-bash",
        str(args.setup_bash),
    ]
    for experiment in args.experiment:
        command.extend(["--experiment", experiment])
    if args.git_ref:
        command.extend(["--git-ref", args.git_ref])
    if args.execute:
        command.append("--execute")
    if args.continue_on_error:
        command.append("--continue-on-error")
    return command


def run_command(command: List[str], cwd: Path, log_path: Path) -> int:
    """执行子命令，并把输出写入指定日志文件。"""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log_file:
        process = subprocess.run(
            command,
            cwd=cwd,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
        )
    return process.returncode


def copy_matrix_if_exists(source: Path, target: Path) -> bool:
    """如果场景 matrix.csv 存在，则复制到总 matrix 目录。"""
    if not source.exists():
        return False
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    return True


def write_shell_plan(plan_path: Path, commands: Iterable[List[str]]) -> None:
    """写出跨场景 dry-run 命令计划。"""
    lines = [
        "#!/usr/bin/env bash",
        "set -euo pipefail",
        "",
        "# MVP-14 dry-run 生成的多场景实验计划。",
        "# 每条命令会继续调用 risk_experiment_batch.py；默认仍为 dry-run。",
        "",
    ]
    for command in commands:
        lines.append(shlex.join(command))
        lines.append("")
    plan_path.write_text("\n".join(lines), encoding="utf-8")
    plan_path.chmod(0o755)


def build_argument_parser() -> argparse.ArgumentParser:
    """构造命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        description="跨多个 playground 编排 EPSILON risk-aware SSC 实验。"
    )
    parser.add_argument(
        "--scenario",
        action="append",
        default=[],
        help="要运行的 playground 名称；可重复。不填写时使用内置四个场景。",
    )
    parser.add_argument(
        "--experiment",
        action="append",
        default=[],
        help="透传给 risk_experiment_batch.py 的实验组定义，格式 name=config_path。",
    )
    parser.add_argument(
        "--output-root",
        default="/tmp/epsilon_risk_scenario_suite",
        help="多场景实验输出根目录。",
    )
    parser.add_argument(
        "--planner-backend",
        choices=["eudm", "mpdm"],
        default="eudm",
        help="透传给闭环 batch runner 的规划后端。",
    )
    parser.add_argument(
        "--duration-sec",
        type=int,
        default=60,
        help="每组单场景实验的 timeout 秒数。",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="真正执行每个场景的 batch runner；默认只生成 dry-run 计划。",
    )
    parser.add_argument(
        "--continue-on-error",
        action="store_true",
        help="某个场景失败后继续后续场景，并在 suite manifest 中记录失败。",
    )
    parser.add_argument(
        "--git-ref",
        default="",
        help="透传给 batch runner 的代码版本字段。",
    )
    parser.add_argument(
        "--setup-bash",
        type=Path,
        default=None,
        help="ROS2 workspace setup.bash；默认使用当前 colcon workspace 的 install/setup.bash。",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help="EPSILON 源码仓库根目录；安装后运行脚本时可显式指定。",
    )
    return parser


def main() -> int:
    """脚本入口：按场景维度编排单场景 batch runner。"""
    args = build_argument_parser().parse_args()
    repo_root = args.repo_root.resolve() if args.repo_root else repo_root_from_script()
    args.repo_root = repo_root
    if args.setup_bash is None:
        args.setup_bash = workspace_root_from_repo(repo_root) / "install" / "setup.bash"
    else:
        args.setup_bash = args.setup_bash.resolve()
    if args.execute and not args.setup_bash.exists():
        raise FileNotFoundError(f"setup.bash not found: {args.setup_bash}")

    scenarios = args.scenario or DEFAULT_SCENARIOS
    playgrounds_dir = default_playgrounds_dir(repo_root)
    validate_scenarios(scenarios, playgrounds_dir)

    output_root = Path(args.output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    batch_script = script_dir_from_script() / "risk_experiment_batch.py"
    if not batch_script.exists():
        raise FileNotFoundError(f"batch script not found: {batch_script}")

    results: List[ScenarioResult] = []
    plan_commands: List[List[str]] = []
    had_error = False

    for scenario in scenarios:
        scenario_output_dir = output_root / scenario
        command = build_batch_command(batch_script, scenario, scenario_output_dir, args)
        plan_commands.append(command)

        manifest_path = scenario_output_dir / "manifest.json"
        matrix_csv = scenario_output_dir / "matrix" / "matrix.csv"
        status = "dry_run"
        return_code: Optional[int] = None

        if args.execute:
            return_code = run_command(
                command, repo_root, scenario_output_dir / "scenario_suite.log"
            )
            status = "ok" if return_code == 0 else "run_failed"
            copied = copy_matrix_if_exists(
                matrix_csv, output_root / "matrix_by_scenario" / f"{scenario}.csv"
            )
            if return_code == 0 and not copied:
                status = "matrix_missing"
                had_error = True
            elif return_code != 0:
                had_error = True

        results.append(
            ScenarioResult(
                scenario_name=scenario,
                playground=scenario,
                output_dir=str(scenario_output_dir),
                command=command,
                return_code=return_code,
                manifest_path=str(manifest_path),
                matrix_csv=str(matrix_csv),
                status=status,
            )
        )
        if had_error and not args.continue_on_error:
            break

    plan_path = output_root / "scenario_run_plan.sh"
    write_shell_plan(plan_path, plan_commands)

    suite_manifest: Dict[str, object] = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "mode": "execute" if args.execute else "dry_run",
        "repo_root": str(repo_root),
        "setup_bash": str(args.setup_bash),
        "output_root": str(output_root),
        "scenario_run_plan": str(plan_path),
        "planner_backend": args.planner_backend,
        "duration_sec": args.duration_sec,
        "scenarios": [result.__dict__ for result in results],
    }
    suite_manifest_path = output_root / "scenario_suite_manifest.json"
    suite_manifest_path.write_text(
        json.dumps(suite_manifest, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    print(f"suite_manifest={suite_manifest_path}")
    print(f"scenario_run_plan={plan_path}")
    return 1 if had_error and not args.continue_on_error else 0


if __name__ == "__main__":
    raise SystemExit(main())
