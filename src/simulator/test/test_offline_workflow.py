from __future__ import annotations

import json
from pathlib import Path

from simulator.offline_workflow import (
    SCHEMA,
    WORKFLOWS,
    catalog_payload,
    default_trace_path,
    main,
    post_run_commands,
    preflight_commands,
    start_command,
    validate_workflow,
    workflow_by_key,
)


def test_workflow_catalog_has_unique_ids_and_valid_references() -> None:
    keys = [workflow.key for workflow in WORKFLOWS]

    assert len(keys) == len(set(keys))
    assert {
        "regional-buff-timeout",
        "regional-outpost-collapse",
        "official-target-fallback",
        "uwb-position-fusion",
        "bullet-info-resource-snapshot",
        "multi-unit-target-priority",
        "low-resource-recovery-exit",
        "full-roster-visual-inputs",
        "tactical-protection",
    } <= set(keys)
    for workflow in WORKFLOWS:
        assert validate_workflow(workflow) == []
        assert workflow.expected_evidence


def test_catalog_payload_contains_runnable_command_groups() -> None:
    payload = catalog_payload(live_view=True)

    assert payload["schema"] == SCHEMA
    assert len(payload["workflows"]) == len(WORKFLOWS)
    for workflow in payload["workflows"]:
        commands = workflow["commands"]
        assert workflow["issues"] == []
        assert "PYTHONPATH=src/simulator python3 -m simulator.start" in commands["start"]
        assert "--offline-decision" in commands["start"]
        assert "--trace-on" in commands["start"]
        assert "--live-view" in commands["start"]
        assert commands["preflight"]
        assert commands["post_run"][0].endswith("--validate-only")


def test_workflow_commands_for_multi_unit_target_priority() -> None:
    workflow = workflow_by_key()["multi-unit-target-priority"]
    trace = default_trace_path(workflow)

    start = start_command(
        workflow,
        trace=trace,
        live_view=False,
        control_file="/tmp/custom_control.jsonl",
        trace_on=True,
    )
    preflight = preflight_commands(workflow, trace=trace, control_file="/tmp/custom_control.jsonl", team="blue")
    post_run = post_run_commands(workflow, trace=trace)

    assert "--mock-preset multi-unit-regional" in start
    assert "--mock-sequence src/simulator/sample/mock_sequences/multi_unit_target_priority_rehearsal.json" in start
    assert "--unit-scene src/simulator/sample/unit_scene.json" in start
    assert "--live-view" not in start
    assert "/tmp/custom_control.jsonl" in start
    assert any("simulator.mock_sequence" in command and "--dry-run" in command for command in preflight)
    assert any("simulator.unit_scene src/simulator/sample/unit_scene.json --team blue" in command for command in preflight)
    assert any("simulator.unit_trace" in command and trace in command for command in post_run)


def test_workflow_commands_for_uwb_position_fusion() -> None:
    workflow = workflow_by_key()["uwb-position-fusion"]
    trace = default_trace_path(workflow)

    start = start_command(
        workflow,
        trace=trace,
        live_view=False,
        control_file="/tmp/uwb_control.jsonl",
        trace_on=True,
    )
    preflight = preflight_commands(workflow, trace=trace, control_file="/tmp/uwb_control.jsonl", team="red")

    assert "--mock-preset uwb-fusion" in start
    assert "--live-view" not in start
    assert "/tmp/uwb_control.jsonl" in start
    assert any("simulator.start" in command and "--dry-run" in command for command in preflight)
    assert not any("simulator.mock_sequence" in command for command in preflight)


def test_workflow_commands_for_bullet_info_resource_snapshot() -> None:
    workflow = workflow_by_key()["bullet-info-resource-snapshot"]
    trace = default_trace_path(workflow)

    start = start_command(
        workflow,
        trace=trace,
        live_view=False,
        control_file="/tmp/bullet_control.jsonl",
        trace_on=True,
    )
    post_run = post_run_commands(workflow, trace=trace)

    assert "--mock-preset bullet-resource" in start
    assert "--live-view" not in start
    assert "/tmp/bullet_control.jsonl" in start
    assert any("bullet_info_resource.jsonl --validate-only" in command for command in post_run)


def test_workflow_commands_for_full_roster_visual_inputs_include_clean_asset_qa() -> None:
    workflow = workflow_by_key()["full-roster-visual-inputs"]
    trace = default_trace_path(workflow)

    post_run = post_run_commands(workflow, trace=trace)

    assert any("/tmp/ly-simulator-full-roster-smoke.png" in command for command in post_run)
    assert any("src/simulator/config/visual_asset_qa.yaml" in command for command in post_run)
    assert any("/tmp/ly-simulator-full-roster-visual-qa.png" in command for command in post_run)
    assert any("simulator.visual_asset_qa" in command for command in post_run)


def test_workflow_commands_for_tactical_protection_include_scene_and_evidence() -> None:
    workflow = workflow_by_key()["tactical-protection"]
    trace = default_trace_path(workflow)

    start = start_command(
        workflow,
        trace=trace,
        live_view=False,
        control_file="/tmp/tactical-protection.jsonl",
        trace_on=True,
    )
    post_run = post_run_commands(workflow, trace=trace)

    assert "--mock-sequence src/simulator/sample/mock_sequences/tactical_protection.json" in start
    assert "--unit-scene src/simulator/sample/unit_scenes/tactical_board.yaml" in start
    assert any("tactical_protect_castle.jsonl --validate-only" in command for command in post_run)
    assert any("tactical_follow_rotate.jsonl --validate-only" in command for command in post_run)


def test_main_lists_workflows(capsys) -> None:
    assert main([]) == 0
    output = capsys.readouterr().out

    assert "Offline decision workflows:" in output
    assert "regional-buff-timeout" in output
    assert "uwb-position-fusion" in output
    assert "bullet-info-resource-snapshot" in output
    assert "full-roster-visual-inputs" in output


def test_main_prints_selected_workflow_detail(capsys) -> None:
    assert main(["official-target-fallback", "--no-live-view"]) == 0
    output = capsys.readouterr().out

    assert "Workflow: official-target-fallback" in output
    assert "--mock-preset official-target-sentry" in output
    assert "--live-view" not in output
    assert "Expected evidence:" in output


def test_main_prints_json_for_selected_workflow(capsys) -> None:
    assert main(["regional-buff-timeout", "--json", "--trace", "log/custom.jsonl"]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["schema"] == SCHEMA
    assert len(payload["workflows"]) == 1
    workflow = payload["workflows"][0]
    assert workflow["id"] == "regional-buff-timeout"
    assert workflow["commands"]["start"].find("log/custom.jsonl") != -1


def test_main_rejects_unknown_workflow(capsys) -> None:
    assert main(["missing-workflow"]) == 2
    output = capsys.readouterr().out

    assert "unknown workflow: missing-workflow" in output
    assert "available workflows:" in output


def test_installed_entry_point_name_is_packaged() -> None:
    setup_py = Path(__file__).resolve().parents[1] / "setup.py"

    assert "simulator-offline-workflows = simulator.offline_workflow:main" in setup_py.read_text(encoding="utf-8")
