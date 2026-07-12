from __future__ import annotations

import argparse
import json
import shlex
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .config import resolve_path
from .mock_sequence import load_mock_sequence_sample, sample_sequence_dir
from .start import MOCK_PRESET_DESCRIPTIONS, MOCK_PRESETS


SCHEMA = "ly_simulator_offline_workflow_catalog_v1"
DEFAULT_CONTROL_FILE = "/tmp/simulator_match_control.jsonl"
SOURCE_PACKAGE_ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class OfflineWorkflow:
    key: str
    title: str
    purpose: str
    mode: str
    bt_config: str
    mock_preset: str
    mock_sequence: str = ""
    unit_scene: str = ""
    expected_evidence: tuple[str, ...] = ()
    review_notes: tuple[str, ...] = ()
    fixture_checks: tuple[str, ...] = ()


WORKFLOWS: tuple[OfflineWorkflow, ...] = (
    OfflineWorkflow(
        key="regional-buff-timeout",
        title="Regional Buff Window And Timeout",
        purpose=(
            "Rehearse the regional energy window with sentry activation, buff target lock, posture/ammo "
            "changes, and target disappearance."
        ),
        mode="regional",
        bt_config="regional_competition.json",
        mock_preset="buff-ready",
        mock_sequence="buff_timeout_context.json",
        expected_evidence=(
            "Trace rows should show buff/referee energy readiness before the timeout.",
            "Current-record status should expose buff target state, posture changes, ammo changes, and match-time jumps.",
            "Validation should stay PASS or produce only reviewed scenario WARN items.",
        ),
        fixture_checks=(
            "PYTHONPATH=src/simulator python3 -m simulator.main src/simulator/sample/scenarios/buff_activation.jsonl --validate-only",
        ),
    ),
    OfflineWorkflow(
        key="regional-outpost-collapse",
        title="Regional Multi-Unit Outpost Collapse",
        purpose=(
            "Rehearse a regional context where unit positions, resource state, self resources, and enemy "
            "outpost HP change over time."
        ),
        mode="regional",
        bt_config="regional_competition.json",
        mock_preset="multi-unit-regional",
        mock_sequence="regional_timed_context.json",
        unit_scene="src/simulator/sample/unit_scene.json",
        expected_evidence=(
            "Trace rows should include friend/enemy HP, UnitInfo positions, team buff/RFID facts, and enemy outpost HP.",
            "Events tab and /status.json should surface low HP units and destroyed enemy outpost state after the sequence.",
            "Generated trace should be suitable for unit-scene evidence review when UnitInfo rows are present.",
        ),
        fixture_checks=(
            (
                "PYTHONPATH=src/simulator python3 -m simulator.unit_trace "
                "src/simulator/sample/scenarios/multi_unit_decision_context.jsonl "
                "--unit-scene src/simulator/sample/unit_scenes/multi_unit_trace_contract.json"
            ),
        ),
    ),
    OfflineWorkflow(
        key="official-target-fallback",
        title="Official Target Fallback",
        purpose=(
            "Rehearse /ly/navi/target_official fallback for a sentry armor position while nearby unit facts "
            "and self resources change."
        ),
        mode="regional",
        bt_config="regional_competition.json",
        mock_preset="official-target-sentry",
        mock_sequence="official_target_fallback_companion.json",
        expected_evidence=(
            "Trace rows should expose official target fallback metadata and ArmorType Sentry ID 6.",
            "Current-record output should make the chosen final topic/frame visible.",
            "Docs/tests must keep ArmorType Sentry ID 6 distinct from UnitType Drone ID 6.",
        ),
        fixture_checks=(
            "PYTHONPATH=src/simulator python3 -m simulator.main src/simulator/sample/scenarios/chase_goal_pos_raw_bridge.jsonl --validate-only",
        ),
    ),
    OfflineWorkflow(
        key="uwb-position-fusion",
        title="UWB Self-Position Fusion",
        purpose=(
            "Rehearse the dedicated /ly/friend/uwb_pos self-position input alongside normal navigation "
            "position state without changing the formal behavior-tree subscriber chain."
        ),
        mode="regional",
        bt_config="regional_competition.json",
        mock_preset="uwb-fusion",
        expected_evidence=(
            "Dry-run mock command should publish /ly/friend/uwb_pos with CLI coordinates expressed in official field centimeters.",
            "behavior_tree reconstructs official y from the raw UWB payload as 1500 - raw_y.",
            "Trace UnitInfo/self-position rows should be reviewed for source/fusion behavior in a launched run.",
        ),
        review_notes=(
            "The behavior-tree fusion priority considers UWB before PositionData and Navi when it is fresh.",
        ),
    ),
    OfflineWorkflow(
        key="bullet-info-resource-snapshot",
        title="BulletInfo Resource Snapshot",
        purpose=(
            "Rehearse cached /ly/game/bullet resource facts including initial speed, shoot data, projectile "
            "allowance, and remaining gold coin without changing the formal behavior-tree subscriber chain."
        ),
        mode="regional",
        bt_config="regional_competition.json",
        mock_preset="bullet-resource",
        expected_evidence=(
            "Trace rows should include bullet_info.has_received, speed, shoot data, projectile allowance, and gold coin fields.",
            "Runtime tab, /status.json.current_record.bullet_info, and Foxglove decision frame should expose the same BulletInfo snapshot.",
            "Decisions should still use the existing legacy ammo/speed gates unless formal BT logic changes separately.",
        ),
        fixture_checks=(
            "PYTHONPATH=src/simulator python3 -m simulator.main src/simulator/sample/scenarios/bullet_info_resource.jsonl --validate-only",
        ),
    ),
    OfflineWorkflow(
        key="multi-unit-target-priority",
        title="Multi-Unit Target Priority Rehearsal",
        purpose=(
            "Rehearse target-priority review with multiple enemy classes, changing HP, self resources, "
            "and a late Sentry target."
        ),
        mode="regional",
        bt_config="regional_competition.json",
        mock_preset="multi-unit-regional",
        mock_sequence="multi_unit_target_priority_rehearsal.json",
        unit_scene="src/simulator/sample/unit_scene.json",
        expected_evidence=(
            "Trace rows should show fresh UnitInfo for traceable Hero/Engineer/Infantry/Sentry classes.",
            "Visual-only Drone and formal UnitInfo-excluded Infantry3 distinctions should remain explicit.",
            "Generated trace should be compared against unit-scene evidence before using target priority conclusions.",
        ),
        fixture_checks=(
            (
                "PYTHONPATH=src/simulator python3 -m simulator.unit_trace "
                "src/simulator/sample/scenarios/multi_unit_decision_context.jsonl "
                "--unit-scene src/simulator/sample/unit_scenes/multi_unit_trace_contract.json --json"
            ),
        ),
    ),
    OfflineWorkflow(
        key="low-resource-recovery-exit",
        title="Low Resource Recovery Exit",
        purpose=(
            "Rehearse low HP/ammo recovery followed by resource restoration, forward self-position movement, "
            "and renewed enemy context."
        ),
        mode="regional",
        bt_config="regional_competition.json",
        mock_preset="low-resource",
        mock_sequence="low_resource_recovery_exit.json",
        expected_evidence=(
            "Early trace rows should expose low self HP/ammo and recovery intent.",
            "Later rows should show restored self resources and forward self position.",
            "Validation should flag suspicious match-time/resource jumps only as reviewed WARN items.",
        ),
        fixture_checks=(
            "PYTHONPATH=src/simulator python3 -m simulator.main src/simulator/sample/scenarios/low_resource_recovery.jsonl --validate-only",
        ),
    ),
    OfflineWorkflow(
        key="full-roster-visual-inputs",
        title="Full Roster Visual And Mock Input QA",
        purpose=(
            "Check all packaged red/blue unit art, HP mapping for formal health units, PositionData "
            "mapping for placed units, and full-roster mock input coverage before a longer offline decision run."
        ),
        mode="regional",
        bt_config="regional_competition.json",
        mock_preset="full-roster-regional",
        unit_scene="src/simulator/sample/unit_scenes/full_roster.json",
        expected_evidence=(
            "Unit-scene summary should include red/blue Hero, Engineer, Infantry, Sentry, and visual Drone pieces.",
            "Headless smoke screenshot should render the full roster without missing sprites.",
            "Drone should report PUB:POS noUI: PositionData is published, with no formal Health field or current UnitInfo output.",
        ),
        fixture_checks=(
            (
                "SDL_VIDEODRIVER=dummy PYTHONPATH=src/simulator python3 -m simulator.main --smoke-test "
                "--unit-scene src/simulator/sample/unit_scenes/full_roster.json "
                "--smoke-screenshot /tmp/ly-simulator-full-roster-smoke.png"
            ),
            (
                "SDL_VIDEODRIVER=dummy PYTHONPATH=src/simulator python3 -m simulator.main --smoke-test "
                "--config src/simulator/config/visual_asset_qa.yaml "
                "--unit-scene src/simulator/sample/unit_scenes/full_roster.json "
                "--smoke-screenshot /tmp/ly-simulator-full-roster-visual-qa.png"
            ),
            (
                "SDL_VIDEODRIVER=dummy PYTHONPATH=src/simulator python3 -m simulator.visual_asset_qa "
                "--screenshot /tmp/ly-simulator-full-roster-visual-qa.png "
                "--unit-scene src/simulator/sample/unit_scenes/full_roster.json "
                "--config src/simulator/config/visual_asset_qa.yaml --team red --json"
            ),
        ),
    ),
)


def workflow_by_key() -> dict[str, OfflineWorkflow]:
    return {workflow.key: workflow for workflow in WORKFLOWS}


def shell_join(parts: list[str]) -> str:
    return " ".join(shlex.quote(part) for part in parts)


def py_module_command(module: str, *args: str) -> str:
    return "PYTHONPATH=src/simulator " + shell_join(["python3", "-m", module, *args])


def default_trace_path(workflow: OfflineWorkflow) -> str:
    return f"log/offline_workflow_{workflow.key}.jsonl"


def build_start_args(
    workflow: OfflineWorkflow,
    *,
    trace: str,
    live_view: bool,
    control_file: str,
    trace_on: bool,
) -> list[str]:
    args = [
        "--offline-decision",
        "--mode",
        workflow.mode,
        "--bt-config",
        workflow.bt_config,
        "--mock-preset",
        workflow.mock_preset,
        "--trace",
        trace,
        "--control-file",
        control_file,
    ]
    if trace_on:
        args.append("--trace-on")
    if workflow.mock_sequence:
        args.extend(["--mock-sequence", sequence_command_path(workflow)])
    if workflow.unit_scene:
        args.extend(["--unit-scene", workflow.unit_scene])
    if live_view:
        args.append("--live-view")
    return args


def start_command(
    workflow: OfflineWorkflow,
    *,
    trace: str,
    live_view: bool,
    control_file: str,
    trace_on: bool,
) -> str:
    return py_module_command(
        "simulator.start",
        *build_start_args(
            workflow,
            trace=trace,
            live_view=live_view,
            control_file=control_file,
            trace_on=trace_on,
        ),
    )


def preflight_commands(workflow: OfflineWorkflow, *, trace: str, control_file: str, team: str) -> list[str]:
    start_args = build_start_args(
        workflow,
        trace=trace,
        live_view=False,
        control_file=control_file,
        trace_on=True,
    )
    commands = [py_module_command("simulator.start", *start_args, "--dry-run")]
    if workflow.mock_sequence:
        commands.append(
            py_module_command(
                "simulator.mock_sequence",
                sequence_command_path(workflow),
                "--control-file",
                control_file,
                "--dry-run",
            )
        )
    if workflow.unit_scene:
        commands.append(py_module_command("simulator.unit_scene", workflow.unit_scene, "--team", team))
    return commands


def post_run_commands(workflow: OfflineWorkflow, *, trace: str) -> list[str]:
    commands = [py_module_command("simulator.main", trace, "--validate-only")]
    if workflow.unit_scene:
        commands.append(
            py_module_command(
                "simulator.unit_trace",
                trace,
                "--unit-scene",
                workflow.unit_scene,
            )
        )
    commands.extend(workflow.fixture_checks)
    return commands


def sequence_path(workflow: OfflineWorkflow) -> Path:
    return sample_sequence_dir() / workflow.mock_sequence


def sequence_command_path(workflow: OfflineWorkflow) -> str:
    return f"src/simulator/sample/mock_sequences/{workflow.mock_sequence}"


def validate_workflow(workflow: OfflineWorkflow) -> list[str]:
    issues: list[str] = []
    if workflow.mock_preset not in MOCK_PRESETS:
        issues.append(f"{workflow.key}: unknown mock preset {workflow.mock_preset}")
    if workflow.mock_sequence and not sequence_path(workflow).is_file():
        issues.append(f"{workflow.key}: missing mock sequence {sequence_path(workflow)}")
    if workflow.mock_sequence:
        try:
            load_mock_sequence_sample(sequence_path(workflow))
        except (OSError, RuntimeError, ValueError) as exc:
            issues.append(f"{workflow.key}: invalid mock sequence {workflow.mock_sequence}: {exc}")
    if workflow.unit_scene:
        scene = resolve_path(workflow.unit_scene)
        if not scene.is_file():
            issues.append(f"{workflow.key}: missing unit scene {workflow.unit_scene}")
    return issues


def workflow_payload(
    workflow: OfflineWorkflow,
    *,
    trace: str,
    live_view: bool,
    control_file: str,
    team: str,
) -> dict[str, Any]:
    preset_description = MOCK_PRESET_DESCRIPTIONS.get(workflow.mock_preset, "")
    issues = validate_workflow(workflow)
    return {
        "id": workflow.key,
        "title": workflow.title,
        "purpose": workflow.purpose,
        "mode": workflow.mode,
        "bt_config": workflow.bt_config,
        "mock_preset": workflow.mock_preset,
        "mock_preset_description": preset_description,
        "mock_sequence": workflow.mock_sequence,
        "unit_scene": workflow.unit_scene,
        "expected_evidence": list(workflow.expected_evidence),
        "review_notes": list(workflow.review_notes),
        "commands": {
            "start": start_command(
                workflow,
                trace=trace,
                live_view=live_view,
                control_file=control_file,
                trace_on=True,
            ),
            "preflight": preflight_commands(workflow, trace=trace, control_file=control_file, team=team),
            "post_run": post_run_commands(workflow, trace=trace),
        },
        "issues": issues,
    }


def catalog_payload(
    *,
    selected: OfflineWorkflow | None = None,
    trace: str = "",
    live_view: bool = True,
    control_file: str = DEFAULT_CONTROL_FILE,
    team: str = "red",
) -> dict[str, Any]:
    workflows = [selected] if selected is not None else list(WORKFLOWS)
    return {
        "schema": SCHEMA,
        "workflows": [
            workflow_payload(
                workflow,
                trace=trace or default_trace_path(workflow),
                live_view=live_view,
                control_file=control_file,
                team=team,
            )
            for workflow in workflows
        ],
    }


def print_workflow_summary(payload: dict[str, Any]) -> int:
    print("Offline decision workflows:")
    for workflow in payload.get("workflows", []):
        item = workflow if isinstance(workflow, dict) else {}
        issue_count = len(item.get("issues", []))
        status = "ok" if issue_count == 0 else f"{issue_count} issue(s)"
        print(f"  {item.get('id')}: {item.get('title')} [{status}]")
        print(f"    purpose: {item.get('purpose')}")
        print(
            "    inputs: "
            f"mode={item.get('mode')} "
            f"preset={item.get('mock_preset')} "
            f"sequence={item.get('mock_sequence') or '-'} "
            f"unit_scene={item.get('unit_scene') or '-'}"
        )
    return 0


def print_workflow_detail(payload: dict[str, Any], *, commands_only: bool = False) -> int:
    workflows = payload.get("workflows", [])
    if len(workflows) != 1 or not isinstance(workflows[0], dict):
        return print_workflow_summary(payload)
    workflow = workflows[0]
    commands = workflow.get("commands", {}) if isinstance(workflow.get("commands"), dict) else {}
    if commands_only:
        print(commands.get("start", ""))
        for command in commands.get("preflight", []):
            print(command)
        for command in commands.get("post_run", []):
            print(command)
        return 0

    print(f"Workflow: {workflow.get('id')} - {workflow.get('title')}")
    print(f"Purpose: {workflow.get('purpose')}")
    print(
        "Inputs: "
        f"mode={workflow.get('mode')} "
        f"bt_config={workflow.get('bt_config')} "
        f"preset={workflow.get('mock_preset')} "
        f"sequence={workflow.get('mock_sequence') or '-'} "
        f"unit_scene={workflow.get('unit_scene') or '-'}"
    )
    print("\nStart:")
    print(f"  {commands.get('start', '')}")
    print("\nPreflight:")
    for command in commands.get("preflight", []):
        print(f"  {command}")
    print("\nPost-run checks:")
    for command in commands.get("post_run", []):
        print(f"  {command}")
    print("\nExpected evidence:")
    for evidence in workflow.get("expected_evidence", []):
        print(f"  - {evidence}")
    issues = workflow.get("issues", [])
    if issues:
        print("\nCatalog issues:")
        for issue in issues:
            print(f"  - {issue}")
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="List offline decision simulator workflow playbooks.")
    parser.add_argument("workflow", nargs="?", help="Workflow id. Omit to list all workflows.")
    parser.add_argument("--json", action="store_true", help="Print machine-readable workflow catalog JSON.")
    parser.add_argument("--commands", action="store_true", help="Print only shell commands for one workflow.")
    parser.add_argument("--no-live-view", action="store_true", help="Omit --live-view from generated start commands.")
    parser.add_argument(
        "--trace",
        default="",
        help="Trace path used in generated commands. Defaults to log/offline_workflow_<id>.jsonl.",
    )
    parser.add_argument(
        "--control-file",
        default=DEFAULT_CONTROL_FILE,
        help=f"Control bus path used in generated commands (default: {DEFAULT_CONTROL_FILE}).",
    )
    parser.add_argument("--team", choices=("red", "blue"), default="red", help="Team used for unit-scene summaries.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    workflows = workflow_by_key()
    selected = None
    if str(args.workflow or "").strip():
        selected = workflows.get(str(args.workflow).strip())
        if selected is None:
            print(f"unknown workflow: {args.workflow}")
            print("available workflows: " + ", ".join(sorted(workflows)))
            return 2

    payload = catalog_payload(
        selected=selected,
        trace=str(args.trace),
        live_view=not bool(args.no_live_view),
        control_file=str(args.control_file),
        team=str(args.team),
    )
    has_issues = any(workflow.get("issues") for workflow in payload.get("workflows", []) if isinstance(workflow, dict))
    if args.json:
        print(json.dumps(payload, ensure_ascii=True, indent=2, sort_keys=True))
    elif selected is None:
        print_workflow_summary(payload)
    else:
        print_workflow_detail(payload, commands_only=bool(args.commands))
    return 1 if has_issues else 0


if __name__ == "__main__":
    raise SystemExit(main())
