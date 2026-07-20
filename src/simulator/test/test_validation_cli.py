from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

from simulator.main import trace_status_payload
from simulator.trace import normalize_record

from test_trace_contract import stable_trace_row


REPO_ROOT = Path(__file__).resolve().parents[3]
SCENARIO_DIR = REPO_ROOT / "src" / "simulator" / "sample" / "scenarios"
TACTICAL_BOARD_SCENE = REPO_ROOT / "src" / "simulator" / "sample" / "unit_scenes" / "tactical_board.yaml"


def run_simulator(*args: str, extra_env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env["PYTHONPATH"] = "src/simulator" + os.pathsep + env.get("PYTHONPATH", "")
    if extra_env:
        env.update(extra_env)
    return subprocess.run(
        [sys.executable, "-m", "simulator.main", *args],
        cwd=REPO_ROOT,
        env=env,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def test_validate_only_json_reports_warn_trace_as_machine_readable_status() -> None:
    result = run_simulator(
        str(SCENARIO_DIR / "route_churn_warning.jsonl"),
        "--validate-only",
        "--validate-format",
        "json",
    )

    assert result.returncode == 0
    assert result.stderr == ""
    report = json.loads(result.stdout)
    assert report["schema"] == "ly_simulator_validation_report_v1"
    assert report["status"] == "WARN"
    assert report["summary"]["issues_by_severity"] == {"errors": 0, "warnings": 1}
    assert report["issue_groups"] == {"scenario": ["scenario.route_churn"]}
    assert report["issues"][0]["code"] == "scenario.route_churn"


def test_validate_only_json_returns_failure_status_and_exit_code_for_errors(tmp_path: Path) -> None:
    raw = stable_trace_row()
    raw.pop("decision_output")
    trace_path = tmp_path / "bad_trace.jsonl"
    trace_path.write_text(json.dumps(raw, ensure_ascii=True) + "\n", encoding="utf-8")

    result = run_simulator(str(trace_path), "--validate-only", "--validate-format", "json")

    assert result.returncode == 2
    report = json.loads(result.stdout)
    assert report["status"] == "FAIL"
    assert report["summary"]["issues_by_severity"]["errors"] == 1
    assert report["issues"][0]["code"] == "schema.missing_decision_output"


def test_validate_format_json_requires_validate_only() -> None:
    result = run_simulator("--validate-format", "json")

    assert result.returncode == 2
    assert "--validate-format requires --validate-only" in result.stderr
    assert result.stdout == ""


def test_smoke_screenshot_requires_smoke_test() -> None:
    result = run_simulator("--smoke-screenshot", "/tmp/ly-sim-smoke.png", "--validate-only")

    assert result.returncode == 2
    assert "--smoke-screenshot requires --smoke-test" in result.stderr
    assert result.stdout == ""


def test_smoke_test_can_write_png_screenshot(tmp_path: Path) -> None:
    screenshot = tmp_path / "frame.png"

    result = run_simulator(
        "--smoke-test",
        "--no-web-stream",
        "--smoke-screenshot",
        str(screenshot),
        extra_env={"SDL_VIDEODRIVER": "dummy"},
    )

    assert result.returncode == 0, result.stderr
    assert "smoke screenshot:" in result.stdout
    assert screenshot.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"


def test_smoke_test_renders_tactical_board_scene(tmp_path: Path) -> None:
    screenshot = tmp_path / "tactical-board.png"

    result = run_simulator(
        str(SCENARIO_DIR / "tactical_protect_castle.jsonl"),
        "--unit-scene",
        str(TACTICAL_BOARD_SCENE),
        "--smoke-test",
        "--no-web-stream",
        "--smoke-screenshot",
        str(screenshot),
        extra_env={"SDL_VIDEODRIVER": "dummy"},
    )

    assert result.returncode == 0, result.stderr
    assert screenshot.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"


def test_smoke_test_reports_unwritable_screenshot_path(tmp_path: Path) -> None:
    result = run_simulator(
        "--smoke-test",
        "--no-web-stream",
        "--smoke-screenshot",
        str(tmp_path),
        extra_env={"SDL_VIDEODRIVER": "dummy"},
    )

    assert result.returncode == 2
    assert "failed to write smoke screenshot" in result.stderr


def test_web_fps_rejects_non_finite_values() -> None:
    result = run_simulator("--web-fps", "nan", "--validate-only")

    assert result.returncode == 2
    assert "--web-fps must be > 0" in result.stderr
    assert result.stdout == ""


def test_trace_status_payload_uses_trace_basename_without_local_path() -> None:
    record = normalize_record(stable_trace_row(), 0, {18: "OccupyArea"})
    trace_path = Path("/tmp/private-workspace/decision_trace.jsonl")

    payload = trace_status_payload(trace_path, [record], bad_lines=2, follow=True)

    assert payload == {
        "name": "decision_trace.jsonl",
        "follow": True,
        "records": 1,
        "bad_lines": 2,
        "duration_sec": 0.0,
        "tick_range": {"first": 7, "last": 7},
    }
    assert "/tmp/private-workspace" not in json.dumps(payload)
