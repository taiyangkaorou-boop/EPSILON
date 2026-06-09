#!/usr/bin/env python3
"""EPSILON 风险感知 SSC 批量实验编排工具。

该脚本把“清理 CSV -> 运行一组实验 -> 归档原始 CSV -> 生成 summary ->
汇总 matrix”的流程固化下来。默认只生成 dry-run 计划，不会启动 ROS；
只有显式传入 --execute 时才会执行实验命令，避免误跑长时间仿真。
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Optional


DEFAULT_RISK_GRID_CSV = Path("/tmp/epsilon_risk_grid_stats.csv")
DEFAULT_RISK_EXPOSURE_CSV = Path("/tmp/epsilon_mvp6_risk_exposure.csv")
TIMEOUT_RETURN_CODE = 124


@dataclass(frozen=True)
class ExperimentSpec:
    """单个消融实验组定义。"""

    name: str
    config_path: Path


@dataclass
class ExperimentResult:
    """单个实验组的执行与汇总结果。"""

    name: str
    config_path: str
    output_dir: str
    command: str
    return_code: Optional[int]
    summary_csv: str
    risk_grid_archive: str
    risk_exposure_archive: str
    risk_grid_present: bool
    risk_exposure_present: bool
    timed_out: bool
    scripted_risk_actors_enabled: bool
    scripted_risk_actor_count: int
    risk_actor_script_path: str
    status: str


@dataclass(frozen=True)
class ScriptedRiskActorMetadata:
    """脚本化周车元数据，贯穿 manifest、summary 和 matrix。"""

    enabled: bool
    actor_count: int
    script_path: Path


def repo_root_from_script() -> Path:
    """从脚本位置推导 EPSILON 仓库根目录。"""
    return Path(__file__).resolve().parents[3]


def workspace_root_from_repo(repo_root: Path) -> Path:
    """从 EPSILON 仓库根目录推导 colcon workspace 根目录。"""
    return repo_root.parent.parent


def config_dir_from_script() -> Path:
    """返回 ssc_planner 源码配置目录。"""
    return Path(__file__).resolve().parents[1] / "config"


def script_dir_from_script() -> Path:
    """返回当前脚本所在目录，用于定位 report/matrix 工具。"""
    return Path(__file__).resolve().parent


def default_experiments(config_dir: Path) -> List[ExperimentSpec]:
    """构造 MVP-11 推荐的四组消融实验。"""
    return [
        ExperimentSpec("baseline", config_dir / "ssc_config.pb.txt"),
        ExperimentSpec("observe", config_dir / "ssc_config_risk_observe.pb.txt"),
        ExperimentSpec("corridor", config_dir / "ssc_config_risk_corridor.pb.txt"),
        ExperimentSpec("full", config_dir / "ssc_config_risk_full.pb.txt"),
    ]


def count_scripted_risk_actors(script_path: Path) -> int:
    """读取 risk_actor_script.json 中的 actor 数量；不存在时返回 0。"""
    if not script_path.exists():
        return 0
    data = json.loads(script_path.read_text(encoding="utf-8"))
    actors = data.get("actors", [])
    if not isinstance(actors, list):
        raise ValueError(f"risk actor script actors must be a list: {script_path}")
    return len(actors)


def scripted_risk_actor_metadata(args: argparse.Namespace) -> ScriptedRiskActorMetadata:
    """根据命令行参数和 playground 默认脚本推导脚本化周车元数据。"""
    default_script_path = (
        args.repo_root / "core" / "playgrounds" / args.playground / "risk_actor_script.json"
    )
    script_path = args.risk_actor_script_path or default_script_path
    actor_count = count_scripted_risk_actors(script_path)
    enabled = args.enable_scripted_risk_actors or actor_count > 0
    return ScriptedRiskActorMetadata(enabled, actor_count, script_path)


def parse_experiment_item(item: str, config_dir: Path) -> ExperimentSpec:
    """解析 name=config.pb.txt 形式的实验组参数。"""
    if "=" not in item:
        raise ValueError(f"experiment must be name=config_path: {item}")
    name, config_text = item.split("=", 1)
    name = name.strip()
    config_text = config_text.strip()
    if not name or not config_text:
        raise ValueError(f"invalid experiment item: {item}")

    config_path = Path(config_text)
    if not config_path.is_absolute():
        config_path = config_dir / config_path
    return ExperimentSpec(name, config_path)


def git_ref(repo_root: Path) -> str:
    """读取当前 git 短哈希；失败时返回空字符串。"""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=repo_root,
            check=True,
            capture_output=True,
            text=True,
        )
    except Exception:
        return ""
    return result.stdout.strip()


def remove_file_if_exists(path: Path) -> None:
    """删除旧 CSV，避免不同实验组混写。"""
    try:
        path.unlink()
    except FileNotFoundError:
        return


def archive_csv(source: Path, target: Path) -> bool:
    """归档运行时 CSV；返回源 CSV 是否存在且非空。"""
    target.parent.mkdir(parents=True, exist_ok=True)
    if source.exists() and source.stat().st_size > 0:
        shutil.copy2(source, target)
        return True
    target.write_text("", encoding="utf-8")
    return False


def command_context(
    experiment: ExperimentSpec,
    output_dir: Path,
    args: argparse.Namespace,
) -> Dict[str, str]:
    """生成命令模板可使用的占位符上下文。"""
    # 闭环实验默认使用 MVP-13 新增入口，同时保留后端 planning launch 占位符，方便外部模板复用。
    launch_file = "risk_experiment_closed_loop_launch.py"
    planning_launch_file = (
        "test_ssc_with_mpdm_ros_launch.py"
        if args.planner_backend == "mpdm"
        else "test_ssc_with_eudm_ros_launch.py"
    )
    risk_actor_metadata = scripted_risk_actor_metadata(args)
    return {
        "experiment_name": experiment.name,
        "scenario_name": args.scenario_name,
        "config_path": str(experiment.config_path),
        "playground": args.playground,
        "planner_backend": args.planner_backend,
        "launch_file": launch_file,
        "planning_launch_file": planning_launch_file,
        "duration_sec": str(args.duration_sec),
        "risk_grid_csv": str(args.risk_grid_csv),
        "risk_exposure_csv": str(args.risk_exposure_csv),
        "output_dir": str(output_dir),
        "setup_bash": str(args.setup_bash),
        "enable_scripted_risk_actors": "true" if risk_actor_metadata.enabled else "false",
        "risk_actor_script_path": str(risk_actor_metadata.script_path),
        "scripted_risk_actor_count": str(risk_actor_metadata.actor_count),
        "risk_actor_publish_rate_hz": str(args.risk_actor_publish_rate_hz),
    }


def default_run_command(context: Dict[str, str]) -> str:
    """生成默认 ros2 launch 命令，默认启动仿真器和规划器闭环。"""
    setup_bash = shlex.quote(context["setup_bash"])
    launch_file = shlex.quote(context["launch_file"])
    planner_backend = shlex.quote(context["planner_backend"])
    playground = shlex.quote(context["playground"])
    config_path = shlex.quote(context["config_path"])
    duration_sec = shlex.quote(context["duration_sec"])
    enable_scripted_risk_actors = shlex.quote(context["enable_scripted_risk_actors"])
    risk_actor_script_path = shlex.quote(context["risk_actor_script_path"])
    risk_actor_publish_rate_hz = shlex.quote(context["risk_actor_publish_rate_hz"])
    return (
        f"source {setup_bash} && "
        f"timeout {duration_sec}s ros2 launch planning_integrated {launch_file} "
        f"planner_backend:={planner_backend} playground:={playground} "
        f"ssc_config_path:={config_path} "
        f"enable_scripted_risk_actors:={enable_scripted_risk_actors} "
        f"risk_actor_script_path:={risk_actor_script_path} "
        f"risk_actor_publish_rate_hz:={risk_actor_publish_rate_hz}"
    )


def build_run_command(
    experiment: ExperimentSpec,
    output_dir: Path,
    args: argparse.Namespace,
) -> str:
    """根据用户模板或默认模板生成单组实验命令。"""
    context = command_context(experiment, output_dir, args)
    template = args.run_command_template or default_run_command(context)
    return template.format(**context)


def run_shell_command(command: str, cwd: Path, log_path: Path) -> int:
    """执行 shell 命令，并把 stdout/stderr 合并写入日志。"""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log_file:
        process = subprocess.run(
            command,
            cwd=cwd,
            shell=True,
            executable="/bin/bash",
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            env=os.environ.copy(),
        )
    return process.returncode


def run_python_command(command: List[str], cwd: Path, log_path: Path) -> int:
    """执行 Python 辅助脚本，并把输出写入日志。"""
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


def write_run_plan(output_root: Path, commands: Iterable[str]) -> Path:
    """写出可人工复核/执行的 shell 计划。"""
    plan_path = output_root / "run_plan.sh"
    output_root.mkdir(parents=True, exist_ok=True)
    lines = [
        "#!/usr/bin/env bash",
        "set -euo pipefail",
        "",
        "# MVP-13 dry-run 生成的闭环实验命令计划。",
        "# 默认命令已包含 source install/setup.bash；自定义命令请自行保证 ROS 环境。",
        "# 注意：默认命令由 timeout 控制实验时长，到时退出码 124 属于预期停止。",
        "",
    ]
    for command in commands:
        lines.append(command)
        lines.append("")
    plan_path.write_text("\n".join(lines), encoding="utf-8")
    plan_path.chmod(0o755)
    return plan_path


def summarize_experiment(
    experiment: ExperimentSpec,
    output_dir: Path,
    args: argparse.Namespace,
    report_script: Path,
    git_reference: str,
    repo_root: Path,
) -> Path:
    """调用 risk_experiment_report.py 生成单组 summary。"""
    summary_dir = output_dir / "report"
    command = [
        sys.executable,
        str(report_script),
        "--risk-grid-csv",
        str(output_dir / "raw_risk_grid_stats.csv"),
        "--risk-exposure-csv",
        str(output_dir / "raw_risk_exposure.csv"),
        "--output-dir",
        str(summary_dir),
        "--experiment-name",
        experiment.name,
        "--scenario-name",
        args.scenario_name,
        "--git-ref",
        args.git_ref or git_reference,
    ]
    risk_actor_metadata = scripted_risk_actor_metadata(args)
    if risk_actor_metadata.enabled:
        command.append("--scripted-risk-actors-enabled")
    command.extend([
        "--scripted-risk-actor-count",
        str(risk_actor_metadata.actor_count),
        "--risk-actor-script-path",
        str(risk_actor_metadata.script_path),
    ])
    return_code = run_python_command(command, repo_root, output_dir / "report.log")
    if return_code != 0:
        raise RuntimeError(f"risk_experiment_report.py failed for {experiment.name}")
    return summary_dir / "summary.csv"


def build_argument_parser() -> argparse.ArgumentParser:
    """构造命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        description="批量编排 EPSILON risk-aware SSC 消融实验。"
    )
    parser.add_argument(
        "--experiment",
        action="append",
        default=[],
        help=(
            "实验组定义，格式 name=config_path；可重复。"
            "不填写时使用 baseline/observe/corridor/full 默认组。"
        ),
    )
    parser.add_argument(
        "--output-root",
        default="/tmp/epsilon_risk_batch",
        help="批量实验输出根目录。",
    )
    parser.add_argument(
        "--scenario-name",
        default="highway_v1.0",
        help="写入 summary 的场景名，也作为默认 playground。",
    )
    parser.add_argument(
        "--playground",
        default="",
        help="传给 launch 的 playground；默认复用 scenario-name。",
    )
    parser.add_argument(
        "--planner-backend",
        choices=["eudm", "mpdm"],
        default="eudm",
        help="默认 launch 后端。",
    )
    parser.add_argument(
        "--duration-sec",
        type=int,
        default=30,
        help="默认 ros2 launch 命令的 timeout 秒数。",
    )
    parser.add_argument(
        "--run-command-template",
        default="",
        help=(
            "自定义实验运行命令模板；支持 {experiment_name}、{config_path}、"
            "{playground}、{planner_backend}、{launch_file}、"
            "{planning_launch_file}、{duration_sec}、{output_dir}、{setup_bash} 等占位符。"
        ),
    )
    parser.add_argument(
        "--enable-scripted-risk-actors",
        action="store_true",
        help="显式启用 MVP-16 脚本化周车；默认仅在 playground 带 risk_actor_script.json 时自动启用。",
    )
    parser.add_argument(
        "--risk-actor-script-path",
        type=Path,
        default=None,
        help="脚本化周车 JSON 路径；默认使用当前 playground/risk_actor_script.json。",
    )
    parser.add_argument(
        "--risk-actor-publish-rate-hz",
        type=float,
        default=50.0,
        help="脚本化周车控制信号发布频率。",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="真正执行每组实验命令；默认只生成 dry-run 计划。",
    )
    parser.add_argument(
        "--continue-on-error",
        action="store_true",
        help="某组实验失败后继续后续组，并在 manifest 中记录失败。",
    )
    parser.add_argument(
        "--risk-grid-csv",
        type=Path,
        default=DEFAULT_RISK_GRID_CSV,
        help="运行时 risk grid CSV 路径。",
    )
    parser.add_argument(
        "--risk-exposure-csv",
        type=Path,
        default=DEFAULT_RISK_EXPOSURE_CSV,
        help="运行时 risk exposure CSV 路径。",
    )
    parser.add_argument(
        "--git-ref",
        default="",
        help="写入 summary 的代码版本；默认自动读取当前 git short hash。",
    )
    parser.add_argument(
        "--setup-bash",
        type=Path,
        default=None,
        help="ROS2 workspace setup.bash 路径；默认使用当前 colcon workspace 的 install/setup.bash。",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help="EPSILON 源码仓库根目录；安装后运行脚本时可用该参数保证 git_ref 可追溯。",
    )
    return parser


def main() -> int:
    """脚本入口：按实验矩阵执行或生成 dry-run 计划。"""
    args = build_argument_parser().parse_args()
    if not args.playground:
        args.playground = args.scenario_name

    repo_root = args.repo_root.resolve() if args.repo_root else repo_root_from_script()
    args.repo_root = repo_root
    if args.setup_bash is None:
        args.setup_bash = workspace_root_from_repo(repo_root) / "install" / "setup.bash"
    if args.execute and not args.setup_bash.exists():
        raise FileNotFoundError(f"setup.bash not found: {args.setup_bash}")
    config_dir = config_dir_from_script()
    script_dir = script_dir_from_script()
    report_script = script_dir / "risk_experiment_report.py"
    matrix_script = script_dir / "risk_experiment_matrix.py"
    output_root = Path(args.output_root)
    output_root.mkdir(parents=True, exist_ok=True)

    experiments = (
        [parse_experiment_item(item, config_dir) for item in args.experiment]
        if args.experiment
        else default_experiments(config_dir)
    )
    for experiment in experiments:
        if not experiment.config_path.exists():
            raise FileNotFoundError(f"config not found: {experiment.config_path}")

    git_reference = args.git_ref or git_ref(repo_root)
    risk_actor_metadata = scripted_risk_actor_metadata(args)
    results: List[ExperimentResult] = []
    dry_run_commands: List[str] = []
    had_error = False

    for experiment in experiments:
        experiment_dir = output_root / experiment.name
        experiment_dir.mkdir(parents=True, exist_ok=True)
        run_command = build_run_command(experiment, experiment_dir, args)
        dry_run_commands.append(run_command)

        if not args.execute:
            results.append(
                ExperimentResult(
                    name=experiment.name,
                    config_path=str(experiment.config_path),
                    output_dir=str(experiment_dir),
                    command=run_command,
                    return_code=None,
                    summary_csv=str(experiment_dir / "report" / "summary.csv"),
                    risk_grid_archive=str(experiment_dir / "raw_risk_grid_stats.csv"),
                    risk_exposure_archive=str(experiment_dir / "raw_risk_exposure.csv"),
                    risk_grid_present=False,
                    risk_exposure_present=False,
                    timed_out=False,
                    scripted_risk_actors_enabled=risk_actor_metadata.enabled,
                    scripted_risk_actor_count=risk_actor_metadata.actor_count,
                    risk_actor_script_path=str(risk_actor_metadata.script_path),
                    status="dry_run",
                )
            )
            continue

        remove_file_if_exists(args.risk_grid_csv)
        remove_file_if_exists(args.risk_exposure_csv)
        return_code = run_shell_command(run_command, repo_root, experiment_dir / "run.log")
        risk_grid_present = archive_csv(
            args.risk_grid_csv, experiment_dir / "raw_risk_grid_stats.csv"
        )
        risk_exposure_present = archive_csv(
            args.risk_exposure_csv, experiment_dir / "raw_risk_exposure.csv"
        )

        timed_out = return_code == TIMEOUT_RETURN_CODE
        command_completed = return_code == 0 or timed_out
        status = "ok" if command_completed else "run_failed"
        summary_csv = experiment_dir / "report" / "summary.csv"
        if command_completed and not risk_grid_present:
            status = "data_missing"
            had_error = True
        elif command_completed:
            try:
                summary_csv = summarize_experiment(
                    experiment, experiment_dir, args, report_script, git_reference, repo_root
                )
            except Exception:
                status = "report_failed"
                had_error = True
        else:
            had_error = True

        results.append(
            ExperimentResult(
                name=experiment.name,
                config_path=str(experiment.config_path),
                output_dir=str(experiment_dir),
                command=run_command,
                return_code=return_code,
                summary_csv=str(summary_csv),
                risk_grid_archive=str(experiment_dir / "raw_risk_grid_stats.csv"),
                risk_exposure_archive=str(experiment_dir / "raw_risk_exposure.csv"),
                risk_grid_present=risk_grid_present,
                risk_exposure_present=risk_exposure_present,
                timed_out=timed_out,
                scripted_risk_actors_enabled=risk_actor_metadata.enabled,
                scripted_risk_actor_count=risk_actor_metadata.actor_count,
                risk_actor_script_path=str(risk_actor_metadata.script_path),
                status=status,
            )
        )
        if had_error and not args.continue_on_error:
            break

    plan_path = write_run_plan(output_root, dry_run_commands)

    summary_args = [
        f"{result.name}={result.summary_csv}"
        for result in results
        if result.status == "ok" and Path(result.summary_csv).exists()
    ]
    matrix_command: List[str] = []
    if args.execute and summary_args:
        matrix_output_dir = output_root / "matrix"
        matrix_command = [
            sys.executable,
            str(matrix_script),
        ]
        for summary in summary_args:
            matrix_command.extend(["--summary", summary])
        matrix_command.extend(["--output-dir", str(matrix_output_dir)])
        matrix_return = run_python_command(
            matrix_command, repo_root, output_root / "matrix.log"
        )
        if matrix_return != 0:
            had_error = True

    manifest = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "mode": "execute" if args.execute else "dry_run",
        "git_ref": git_reference,
        "scenario_name": args.scenario_name,
        "playground": args.playground,
        "planner_backend": args.planner_backend,
        "repo_root": str(repo_root),
        "setup_bash": str(args.setup_bash),
        "scripted_risk_actors_enabled": risk_actor_metadata.enabled,
        "scripted_risk_actor_count": risk_actor_metadata.actor_count,
        "risk_actor_script_path": str(risk_actor_metadata.script_path),
        "output_root": str(output_root),
        "run_plan": str(plan_path),
        "matrix_command": matrix_command,
        "experiments": [result.__dict__ for result in results],
    }
    manifest_path = output_root / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"manifest={manifest_path}")
    print(f"run_plan={plan_path}")
    if args.execute and summary_args:
        print(f"matrix_dir={output_root / 'matrix'}")
    return 1 if had_error and not args.continue_on_error else 0


if __name__ == "__main__":
    raise SystemExit(main())
