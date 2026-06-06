from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable


SIMULATOR_DIFF_PATHS = [
    "src/simulator",
    "docs/sentry/internal/simulator.md",
    "docs/sentry/internal/simulator_offline_debug_roadmap.md",
    "docs/sentry/internal/PROGRESS_REPORT.md",
    "docs/sentry/internal/QUALITY_REVIEW.md",
]
TEXT_SUFFIXES = {".cfg", ".json", ".jsonl", ".md", ".py", ".txt", ".xml", ".yaml", ".yml"}
SKIP_DIR_NAMES = {".pytest_cache", "__pycache__"}


@dataclass(frozen=True)
class QualityStep:
    name: str
    command: list[str]
    env: dict[str, str] = field(default_factory=dict)


StepRunner = Callable[[QualityStep, Path], int]


def simulator_env(repo_root: Path) -> dict[str, str]:
    pythonpath = str(repo_root / "src" / "simulator")
    return {"PYTHONPATH": pythonpath}


def pytest_env(repo_root: Path) -> dict[str, str]:
    env = simulator_env(repo_root)
    env["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    return env


def build_plan(
    repo_root: Path,
    *,
    with_build: bool = False,
    with_browser_visual: bool = False,
    smoke_port: int = 9021,
) -> list[QualityStep]:
    simulator_modules = sorted((repo_root / "src" / "simulator" / "simulator").glob("*.py"))
    steps = [
        QualityStep(
            "pytest",
            [sys.executable, "-m", "pytest", "src/simulator/test", "-q"],
            pytest_env(repo_root),
        ),
        QualityStep(
            "py_compile",
            [sys.executable, "-m", "py_compile", *[path.as_posix() for path in simulator_modules]],
        ),
        QualityStep(
            "validate_sample",
            [sys.executable, "-m", "simulator.main", "--validate-only"],
            simulator_env(repo_root),
        ),
        QualityStep(
            "validate_route_churn_json",
            [
                sys.executable,
                "-m",
                "simulator.main",
                "src/simulator/sample/scenarios/route_churn_warning.jsonl",
                "--validate-only",
                "--validate-format",
                "json",
            ],
            simulator_env(repo_root),
        ),
        QualityStep(
            "list_unit_scenes",
            [sys.executable, "-m", "simulator.unit_scene", "--list-samples"],
            simulator_env(repo_root),
        ),
        QualityStep(
            "summarize_full_roster_unit_scene",
            [
                sys.executable,
                "-m",
                "simulator.unit_scene",
                "src/simulator/sample/unit_scenes/full_roster.json",
                "--team",
                "red",
                "--json",
            ],
            simulator_env(repo_root),
        ),
        QualityStep(
            "smoke_asset_render",
            [
                sys.executable,
                "-m",
                "pytest",
                "src/simulator/test/test_assets.py::test_default_assets_decode_scale_and_blit_with_pygame",
                "-q",
            ],
            pytest_env(repo_root),
        ),
        QualityStep(
            "check_multi_unit_trace_evidence",
            [
                sys.executable,
                "-m",
                "simulator.unit_trace",
                "src/simulator/sample/scenarios/multi_unit_decision_context.jsonl",
                "--unit-scene",
                "src/simulator/sample/unit_scenes/multi_unit_trace_contract.json",
                "--json",
            ],
            simulator_env(repo_root),
        ),
        QualityStep(
            "list_offline_workflows",
            [sys.executable, "-m", "simulator.offline_workflow", "--json"],
            simulator_env(repo_root),
        ),
        QualityStep(
            "decision_input_coverage",
            [sys.executable, "-m", "simulator.decision_input_coverage", "--json"],
            simulator_env(repo_root),
        ),
        QualityStep(
            "smoke_web",
            [sys.executable, "-m", "simulator.main", "--smoke-test", "--web-port", str(smoke_port)],
            {**simulator_env(repo_root), "SDL_VIDEODRIVER": "dummy"},
        ),
        QualityStep(
            "smoke_full_roster_scene",
            [
                sys.executable,
                "-m",
                "simulator.main",
                "--smoke-test",
                "--web-port",
                str(smoke_port + 1),
                "--unit-scene",
                "src/simulator/sample/unit_scenes/full_roster.json",
                "--smoke-screenshot",
                "/tmp/ly-simulator-full-roster-smoke.png",
            ],
            {**simulator_env(repo_root), "SDL_VIDEODRIVER": "dummy"},
        ),
        QualityStep(
            "smoke_full_roster_visual_asset_qa_scene",
            [
                sys.executable,
                "-m",
                "simulator.main",
                "--smoke-test",
                "--web-port",
                str(smoke_port + 2),
                "--config",
                "src/simulator/config/visual_asset_qa.yaml",
                "--unit-scene",
                "src/simulator/sample/unit_scenes/full_roster.json",
                "--smoke-screenshot",
                "/tmp/ly-simulator-full-roster-visual-qa.png",
            ],
            {**simulator_env(repo_root), "SDL_VIDEODRIVER": "dummy"},
        ),
        QualityStep(
            "full_roster_visual_asset_qa",
            [
                sys.executable,
                "-m",
                "simulator.visual_asset_qa",
                "--screenshot",
                "/tmp/ly-simulator-full-roster-visual-qa.png",
                "--unit-scene",
                "src/simulator/sample/unit_scenes/full_roster.json",
                "--config",
                "src/simulator/config/visual_asset_qa.yaml",
                "--team",
                "red",
                "--json",
            ],
            {**simulator_env(repo_root), "SDL_VIDEODRIVER": "dummy"},
        ),
    ]
    if with_browser_visual:
        steps.append(
            QualityStep(
                "browser_visual_check",
                [
                    sys.executable,
                    "-m",
                    "simulator.web_visual_check",
                    "--output-dir",
                    "/tmp/ly-simulator-web-visual",
                    "--require-browser",
                ],
                simulator_env(repo_root),
            )
        )
    steps.append(
        QualityStep(
            "text_whitespace",
            [
                sys.executable,
                "-m",
                "simulator.quality",
                "--repo-root",
                repo_root.as_posix(),
                "--check-text-whitespace",
            ],
            simulator_env(repo_root),
        )
    )
    if with_build:
        steps.append(QualityStep("colcon_build", ["colcon", "build", "--packages-select", "simulator"]))
    return steps


def format_command(step: QualityStep) -> str:
    prefix = " ".join(f"{key}={shlex.quote(value)}" for key, value in sorted(step.env.items()))
    command = " ".join(shlex.quote(part) for part in step.command)
    return f"{prefix} {command}".strip()


def subprocess_runner(step: QualityStep, repo_root: Path) -> int:
    env = os.environ.copy()
    for key, value in step.env.items():
        if key == "PYTHONPATH" and env.get("PYTHONPATH"):
            env[key] = value + os.pathsep + env[key]
        else:
            env[key] = value
    result = subprocess.run(step.command, cwd=repo_root, env=env, check=False)
    return int(result.returncode)


def iter_text_files(repo_root: Path) -> list[Path]:
    files: list[Path] = []
    for relative in SIMULATOR_DIFF_PATHS:
        path = repo_root / relative
        if path.is_file() and path.suffix in TEXT_SUFFIXES:
            files.append(path)
            continue
        if not path.is_dir():
            continue
        for child in path.rglob("*"):
            if any(part in SKIP_DIR_NAMES for part in child.relative_to(path).parts):
                continue
            if child.is_file() and child.suffix in TEXT_SUFFIXES:
                files.append(child)
    return sorted(files)


def check_text_whitespace(repo_root: Path) -> int:
    issues: list[str] = []
    for path in iter_text_files(repo_root):
        try:
            lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
        except UnicodeDecodeError:
            continue
        rel_path = path.relative_to(repo_root).as_posix()
        for line_no, line in enumerate(lines, start=1):
            content = line.rstrip("\r\n")
            if content.rstrip(" \t") != content:
                issues.append(f"{rel_path}:{line_no}: trailing whitespace")
            marker = content.strip()
            if marker.startswith("<<<<<<< ") or marker == "=======" or marker.startswith(">>>>>>> "):
                issues.append(f"{rel_path}:{line_no}: merge conflict marker")
    for issue in issues:
        print(issue, file=sys.stderr)
    return 1 if issues else 0


def run_steps(
    steps: list[QualityStep],
    repo_root: Path,
    *,
    runner: StepRunner = subprocess_runner,
    keep_going: bool = False,
) -> int:
    first_failure = 0
    for index, step in enumerate(steps, start=1):
        print(f"[{index}/{len(steps)}] RUN {step.name}: {format_command(step)}")
        code = runner(step, repo_root)
        if code == 0:
            print(f"[{index}/{len(steps)}] PASS {step.name}")
            continue
        print(f"[{index}/{len(steps)}] FAIL {step.name}: exit {code}")
        if first_failure == 0:
            first_failure = int(code)
        if not keep_going:
            return int(code)
    return first_failure


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the offline simulator quality gate.")
    parser.add_argument(
        "--repo-root",
        default=".",
        help="Workspace root. Defaults to the current working directory.",
    )
    parser.add_argument("--with-build", action="store_true", help="Also run `colcon build --packages-select simulator`.")
    parser.add_argument(
        "--with-browser-visual",
        action="store_true",
        help="Also run the optional Playwright/Chromium browser visual dashboard check.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print the quality plan without running commands.")
    parser.add_argument("--keep-going", action="store_true", help="Run remaining steps after a failure.")
    parser.add_argument("--smoke-port", type=int, default=9021, help="HTTP port used by the headless web smoke test.")
    parser.add_argument(
        "--check-text-whitespace",
        action="store_true",
        help="Check simulator-related text files for trailing whitespace and merge markers, then exit.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    repo_root = Path(args.repo_root).expanduser().resolve()
    if not (repo_root / "src" / "simulator").is_dir():
        print(f"--repo-root must point to the workspace root containing src/simulator: {repo_root}", file=sys.stderr)
        return 2
    if args.check_text_whitespace:
        return check_text_whitespace(repo_root)
    if args.smoke_port <= 0 or args.smoke_port > 65535:
        print("--smoke-port must be in [1, 65535]", file=sys.stderr)
        return 2
    steps = build_plan(
        repo_root,
        with_build=bool(args.with_build),
        with_browser_visual=bool(args.with_browser_visual),
        smoke_port=int(args.smoke_port),
    )
    if args.dry_run:
        print("[dry-run] simulator quality plan")
        for index, step in enumerate(steps, start=1):
            print(f"[{index}/{len(steps)}] {step.name}: {format_command(step)}")
        return 0
    return run_steps(steps, repo_root, keep_going=bool(args.keep_going))


if __name__ == "__main__":
    raise SystemExit(main())
