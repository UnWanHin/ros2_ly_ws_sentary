from __future__ import annotations

from pathlib import Path

from simulator.quality import QualityStep, build_plan, check_text_whitespace, main, run_steps


REPO_ROOT = Path(__file__).resolve().parents[3]


def command_text(step: QualityStep) -> str:
    return " ".join(step.command)


def test_build_plan_includes_fast_default_quality_gate() -> None:
    steps = build_plan(REPO_ROOT, with_build=False, smoke_port=9021)
    commands = "\n".join(command_text(step) for step in steps)

    assert [step.name for step in steps] == [
        "pytest",
        "py_compile",
        "validate_sample",
        "validate_route_churn_json",
        "list_unit_scenes",
        "summarize_full_roster_unit_scene",
        "smoke_asset_render",
        "check_multi_unit_trace_evidence",
        "list_offline_workflows",
        "decision_input_coverage",
        "smoke_web",
        "smoke_full_roster_scene",
        "smoke_full_roster_visual_asset_qa_scene",
        "full_roster_visual_asset_qa",
        "text_whitespace",
    ]
    assert "PYTEST_DISABLE_PLUGIN_AUTOLOAD" in steps[0].env
    assert "src/simulator/test" in commands
    assert "py_compile" in commands
    assert "simulator.main --validate-only" in commands
    assert "route_churn_warning.jsonl --validate-only --validate-format json" in commands
    assert "simulator.unit_scene --list-samples" in commands
    assert "full_roster.json --team red --json" in commands
    assert "test_assets.py::test_default_assets_decode_scale_and_blit_with_pygame" in commands
    assert "simulator.unit_trace" in commands
    assert "multi_unit_decision_context.jsonl --unit-scene src/simulator/sample/unit_scenes/multi_unit_trace_contract.json --json" in commands
    assert "simulator.offline_workflow --json" in commands
    assert "simulator.decision_input_coverage --json" in commands
    assert "--smoke-test --web-port 9021" in commands
    assert "--smoke-test --web-port 9022 --unit-scene src/simulator/sample/unit_scenes/full_roster.json" in commands
    assert "--smoke-screenshot /tmp/ly-simulator-full-roster-smoke.png" in commands
    assert "--smoke-test --web-port 9023 --config src/simulator/config/visual_asset_qa.yaml" in commands
    assert "--smoke-screenshot /tmp/ly-simulator-full-roster-visual-qa.png" in commands
    assert "simulator.visual_asset_qa" in commands
    assert "--screenshot /tmp/ly-simulator-full-roster-visual-qa.png" in commands
    assert "--config src/simulator/config/visual_asset_qa.yaml" in commands
    assert "simulator.quality" in commands
    assert "--check-text-whitespace" in commands
    assert "colcon build" not in commands


def test_build_plan_can_add_package_build_gate() -> None:
    steps = build_plan(REPO_ROOT, with_build=True, smoke_port=9021)

    assert steps[-1].name == "colcon_build"
    assert command_text(steps[-1]) == "colcon build --packages-select simulator"


def test_build_plan_can_add_optional_browser_visual_gate() -> None:
    steps = build_plan(REPO_ROOT, with_browser_visual=True, smoke_port=9021)

    assert [step.name for step in steps][-2:] == ["browser_visual_check", "text_whitespace"]
    browser_step = steps[-2]
    assert command_text(browser_step) == (
        " ".join(
            [
                browser_step.command[0],
                "-m",
                "simulator.web_visual_check",
                "--output-dir",
                "/tmp/ly-simulator-web-visual",
                "--require-browser",
            ]
        )
    )
    assert browser_step.env["PYTHONPATH"].endswith("src/simulator")


def test_dry_run_prints_quality_plan_without_running_commands(capsys) -> None:
    code = main(["--repo-root", str(REPO_ROOT), "--dry-run", "--with-build", "--with-browser-visual"])
    out = capsys.readouterr().out

    assert code == 0
    assert "[dry-run] simulator quality plan" in out
    assert "simulator.web_visual_check" in out
    assert "pytest" in out
    assert "colcon build --packages-select simulator" in out


def test_main_rejects_invalid_repo_root(tmp_path: Path, capsys) -> None:
    code = main(["--repo-root", str(tmp_path), "--dry-run"])
    captured = capsys.readouterr()

    assert code == 2
    assert "--repo-root must point to the workspace root" in captured.err


def test_text_whitespace_check_reports_untracked_text_files(tmp_path: Path, capsys) -> None:
    src_dir = tmp_path / "src" / "simulator"
    src_dir.mkdir(parents=True)
    bad_file = src_dir / "new_quality_case.py"
    bad_file.write_text("ok = 1  \n<<<<<<< HEAD\n", encoding="utf-8")

    code = check_text_whitespace(tmp_path)
    captured = capsys.readouterr()

    assert code == 1
    assert "src/simulator/new_quality_case.py:1: trailing whitespace" in captured.err
    assert "src/simulator/new_quality_case.py:2: merge conflict marker" in captured.err


def test_simulator_docs_describe_tactical_board_trace_and_input_ownership() -> None:
    docs = (REPO_ROOT / "docs" / "sentry" / "internal" / "simulator.md").read_text(encoding="utf-8")
    readme = (REPO_ROOT / "src" / "simulator" / "README.md").read_text(encoding="utf-8")

    for text in (docs, readme):
        assert "/tactical" in text
        assert "manual_ros" in text
        assert "control_output" in text
        assert "tactical_protect_castle" in text
        assert "Zoom to Selection" in text
        assert "Activity rail" in text
        assert "Operations shelf" in text
        assert "official centimeter" in text
        assert "Sentinel Flight Deck" in text
        assert "#4DB7FF" in text
        assert "presentation-only" in text


def test_run_steps_returns_failing_step_code(capsys) -> None:
    calls: list[str] = []

    def runner(step: QualityStep, repo_root: Path) -> int:
        calls.append(step.name)
        return 7 if step.name == "bad" else 0

    steps = [
        QualityStep("good", ["true"]),
        QualityStep("bad", ["false"]),
        QualityStep("skipped", ["true"]),
    ]

    code = run_steps(steps, REPO_ROOT, runner=runner, keep_going=False)
    out = capsys.readouterr().out

    assert code == 7
    assert calls == ["good", "bad"]
    assert "FAIL bad" in out


def test_run_steps_can_keep_going_after_failure() -> None:
    calls: list[str] = []

    def runner(step: QualityStep, repo_root: Path) -> int:
        calls.append(step.name)
        return 3 if step.name == "bad" else 0

    steps = [
        QualityStep("bad", ["false"]),
        QualityStep("after", ["true"]),
    ]

    code = run_steps(steps, REPO_ROOT, runner=runner, keep_going=True)

    assert code == 3
    assert calls == ["bad", "after"]
