from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import math
from typing import Any

from .model import TraceRecord
from .trace import as_dict


ROUTE_CHURN_MIN_SELECTIONS = 4
ROUTE_CHURN_WINDOW_SEC = 5.0
POSTURE_LAG_WINDOW_SEC = 5.0
MATCH_TIME_JUMP_TOLERANCE_SEC = 5.0
REFEREE_RESOURCE_FIELDS = (
    "self_outpost_hp",
    "enemy_outpost_hp",
    "self_base_hp",
    "enemy_base_hp",
)
UNIT_LABEL_ALIASES = {
    "hero": {"hero", "hero (1)", "1"},
    "engineer": {"engineer", "engineer (2)", "2"},
    "infantry1": {"infantry1", "infantry1 (3)", "3"},
    "infantry2": {"infantry2", "infantry2 (4)", "4"},
    "infantry3": {"infantry3", "infantry3 (5)", "5", "reserve"},
    "drone": {"drone", "drone (6)", "6"},
    "sentry": {"sentry", "sentry (7)", "7"},
}
FORMAL_UNIT_INFO_TYPE_IDS = {1, 2, 3, 4, 7}
FORMAL_UNIT_INFO_TYPE_NAMES = {
    "hero": 1,
    "engineer": 2,
    "infantry1": 3,
    "infantry2": 4,
    "sentry": 7,
}
ISSUE_GROUP_ORDER = ("schema", "trace", "output", "unit", "runtime", "referee", "scenario", "validation")


@dataclass(frozen=True)
class ValidationIssue:
    severity: str
    index: int | None
    message: str
    code: str = "validation.issue"
    suggestion: str = ""


def make_issue(
    severity: str,
    index: int | None,
    code: str,
    message: str,
    suggestion: str,
) -> ValidationIssue:
    return ValidationIssue(
        severity=severity,
        index=index,
        message=message,
        code=code,
        suggestion=suggestion,
    )


def config_float(value: Any, default: float) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if out > 0 else default


def validate_records(records: list[TraceRecord], config: dict[str, Any], bad_lines: int = 0) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    if bad_lines:
        issues.append(
            make_issue(
                "warning",
                None,
                "trace.bad_line",
                f"skipped {bad_lines} bad trace line(s)",
                "Inspect the JSONL around the reported offset; partially written live-follow rows should end with a newline.",
            )
        )

    field = as_dict(config.get("field_cm"))
    field_w = config_float(field.get("width"), 2800.0)
    field_h = config_float(field.get("height"), 1500.0)
    last_t: float | None = None
    legacy_schema_seen = False

    for record in records:
        schema_version = record.schema_version
        if schema_version < 2:
            legacy_schema_seen = True
        elif not record.has_decision_output:
            issues.append(
                make_issue(
                    "error",
                    record.index,
                    "schema.missing_decision_output",
                    "schema_version >= 2 missing decision_output",
                    "Update the trace writer or adapter so schema v2 rows expose the stable simulator decision_output contract.",
                )
            )
        elif not record.has_decision_intent:
            issues.append(
                make_issue(
                    "warning",
                    record.index,
                    "schema.missing_decision_intent",
                    "schema_version >= 2 missing decision_intent",
                    "Add decision_intent to explain why the current output was selected, or keep the row on schema v1.",
                )
            )

        if last_t is not None and record.t < last_t:
            issues.append(
                make_issue(
                    "error",
                    record.index,
                    "trace.time_monotonic",
                    "trace time is not monotonic",
                    "Keep recorder output ordered by monotonic time; sort only offline merged traces before replay.",
                )
            )
        last_t = record.t

        pos = record.output.goal_pos_cm
        if pos is None and record.output.uses_goal_pos:
            issues.append(
                make_issue(
                    "error",
                    record.index,
                    "output.missing_goal_pos",
                    "goal_pos output has no goal_pos_cm",
                    "For goal_pos outputs, write explicit field centimeters in decision_output.goal_pos_cm.",
                )
            )
        if pos is not None and not (0 <= pos[0] <= field_w and 0 <= pos[1] <= field_h):
            issues.append(
                make_issue(
                    "error",
                    record.index,
                    "output.goal_pos_bounds",
                    f"goal_pos_cm out of field: x={pos[0]:.0f} y={pos[1]:.0f}",
                    "Check field_cm, goal point coordinates, and any ToNavi/map-frame conversion before replaying this trace.",
                )
            )
        if record.output.publish_enabled and record.output.publish_allowed and not record.output.output_topic:
            issues.append(
                make_issue(
                    "warning",
                    record.index,
                    "output.missing_topic",
                    "published output has no output_topic",
                    "Record output_topic so reviewers can distinguish /ly/navi/goal, /ly/navi/goal_pos, and bridge output.",
                )
            )
        if record.has_decision_output and record.goal_reach.has_detail:
            if (
                record.goal_reach.goal_id != record.output.goal_id
                or record.goal_reach.base_goal_id != record.output.goal_base_id
            ):
                issues.append(
                    make_issue(
                        "warning",
                        record.index,
                        "output.goal_reach_mismatch",
                        (
                            "goal_reach_state goal does not match decision_output: "
                            f"reach={record.goal_reach.goal_id}/{record.goal_reach.base_goal_id} "
                            f"output={record.output.goal_id}/{record.output.goal_base_id}"
                        ),
                        "Record goal reach state after resolving the same navigation output that is written to decision_output.",
                    )
                )
            if record.goal_reach.distance_cm is not None and record.goal_reach.distance_cm < 0.0:
                issues.append(
                    make_issue(
                        "warning",
                        record.index,
                        "output.goal_reach_distance",
                        f"goal_reach_state distance_cm is negative: {record.goal_reach.distance_cm:.1f}",
                        "Distance-to-goal should be a non-negative centimeter scalar or null when unavailable.",
                    )
                )
        if record.output.kind == "relative_target_bridge" and not record.navi_relative_target.valid:
            issues.append(
                make_issue(
                    "warning",
                    record.index,
                    "output.relative_target_bridge_missing_target",
                    "relative_target_bridge output has no valid navi_relative_target",
                    "Bridge outputs should include the relative target payload used to produce /goal_pose.",
                )
            )
        if record.navi_relative_target.valid and not record.navi_relative_target.frame_id.strip():
            issues.append(
                make_issue(
                    "warning",
                    record.index,
                    "output.relative_target_frame_missing",
                    "valid navi_relative_target has no frame_id",
                    "Record navi_relative_target.frame_id so offline bridge replay can distinguish base_link, gimbal_world, and map-frame data.",
                )
            )
        if record.navi_velocity.raw_to_mps is not None and record.navi_velocity.raw_to_mps <= 0.0:
            issues.append(
                make_issue(
                    "warning",
                    record.index,
                    "output.navi_velocity_scale",
                    f"navi_velocity raw_to_mps must be positive when present: {record.navi_velocity.raw_to_mps:g}",
                    "Use the trace writer's raw-to-m/s conversion scale for /ly/control/vel, or omit raw_to_mps when unknown.",
                )
            )
        if record.navi_status.should_rotate_fresh is True and record.navi_status.should_rotate is None:
            issues.append(
                make_issue(
                    "warning",
                    record.index,
                    "output.should_rotate_missing",
                    "navi_status marks should_rotate fresh but has no should_rotate value",
                    "Write the effective /ly/navi/should_rotate value together with its freshness flag.",
                )
            )

        add_control_output_validation(record, issues)

        for unit in record.units:
            if unit.max_hp > 0 and not (0 <= unit.hp <= unit.max_hp):
                issues.append(
                    make_issue(
                        "warning",
                        record.index,
                        "unit.hp_bounds",
                        f"{unit.side}:{unit.type_name} hp={unit.hp} outside 0..{unit.max_hp}",
                        "Clamp mock/unit-scene HP or fix the trace producer before comparing decision resource behavior.",
                    )
                )
            if unit.position_cm is None:
                continue
            x, y = unit.position_cm
            if not (0 <= x <= field_w and 0 <= y <= field_h):
                issues.append(
                    make_issue(
                        "warning",
                        record.index,
                        "unit.position_bounds",
                        f"{unit.side}:{unit.type_name} position out of field: x={x:.0f} y={y:.0f}",
                        "Keep unit positions in official field centimeters with left-bottom origin.",
                    )
                )
        for unit in record.unit_info:
            if unit.has_position and unit.position_cm is None:
                issues.append(
                    make_issue(
                        "warning",
                        record.index,
                        "unit.info_missing_position",
                        f"{unit.side}:{unit.type_name} unit_info has_position=true but no parseable position",
                        "Record unit_info position_x/position_y or position_cm in official field centimeters.",
                    )
                )
                continue
            if unit.position_cm is None:
                continue
            x, y = unit.position_cm
            if not (0 <= x <= field_w and 0 <= y <= field_h):
                issues.append(
                    make_issue(
                        "warning",
                        record.index,
                        "unit.info_position_bounds",
                        f"{unit.side}:{unit.type_name} unit_info position out of field: x={x:.0f} y={y:.0f}",
                        "Keep UnitInfo positions aligned with official-map centimeters before using area/reliable-position decisions offline.",
                    )
                )

    if legacy_schema_seen:
        issues.insert(
            0,
            make_issue(
                "warning",
                None,
                "schema.legacy",
                "legacy schema_version < 2 records are readable but lack stable decision_output/decision_intent fields",
                "Prefer schema v2 traces for decision review; legacy rows are kept only for old recordings.",
            ),
        )

    add_scenario_diagnostics(records, issues)
    return issues


def add_control_output_validation(record: TraceRecord, issues: list[ValidationIssue]) -> None:
    trajectory = record.control_output.trajectory
    if not trajectory.available:
        return

    values = {
        "yaw": trajectory.yaw,
        "pitch": trajectory.pitch,
        "yaw_omega": trajectory.yaw_omega,
        "pitch_omega": trajectory.pitch_omega,
        "yaw_alpha": trajectory.yaw_alpha,
        "pitch_alpha": trajectory.pitch_alpha,
    }
    invalid = [name for name, value in values.items() if value is None or not math.isfinite(value)]
    if invalid:
        issues.append(
            make_issue(
                "error",
                record.index,
                "control.trajectory_nonfinite",
                "available control trajectory has missing/non-finite field(s): " + ", ".join(invalid),
                "Record all six finite trajectory fields when control_output.trajectory.available=true; otherwise set available=false with an unavailable_reason.",
            )
        )


def add_scenario_diagnostics(records: list[TraceRecord], issues: list[ValidationIssue]) -> None:
    add_route_churn_warning(records, issues)
    add_stale_chase_target_warnings(records, issues)
    add_reliable_enemy_unit_info_warnings(records, issues)
    add_zero_hp_runtime_mismatch_warnings(records, issues)
    add_zero_self_hp_runtime_mismatch_warnings(records, issues)
    add_posture_lag_warnings(records, issues)
    add_outpost_engagement_lock_warnings(records, issues)
    add_match_time_jump_warnings(records, issues)
    add_missing_referee_resource_warnings(records, issues)


def add_route_churn_warning(records: list[TraceRecord], issues: list[ValidationIssue]) -> None:
    route_selections: list[tuple[TraceRecord, tuple[int, str, object]]] = []
    last_route: tuple[int, str, object] | None = None
    for record in records:
        route = record.output.route_key
        if route != last_route:
            route_selections.append((record, route))
            last_route = route

    for start in range(len(route_selections)):
        end = start + ROUTE_CHURN_MIN_SELECTIONS - 1
        if end >= len(route_selections):
            break
        first = route_selections[start][0]
        last = route_selections[end][0]
        span = last.t - first.t
        if 0 <= span <= ROUTE_CHURN_WINDOW_SEC:
            route_text = " -> ".join(selection[0].goal_name for selection in route_selections[start : end + 1])
            issues.append(
                make_issue(
                    "warning",
                    last.index,
                    "scenario.route_churn",
                    f"{ROUTE_CHURN_MIN_SELECTIONS} route selections in {span:.1f}s: {route_text}",
                    "Inspect decision_intent and event changes; rapid route churn can hide unstable priority or reachability logic.",
                )
            )
            return


def add_stale_chase_target_warnings(records: list[TraceRecord], issues: list[ValidationIssue]) -> None:
    for record in records:
        if record.output.kind != "relative_target_bridge" and record.decision_intent.layer.lower() != "chase":
            continue
        target_fresh = (
            record.target_state.has_recent_target
            or record.target_state.fresh_current_aim is True
            or record.target_state.fresh_auto_aim
            or record.target_state.fresh_buff
            or record.target_state.fresh_outpost
        )
        if target_fresh:
            continue
        issues.append(
            make_issue(
                "warning",
                record.index,
                "scenario.stale_chase_target",
                "chase/relative-target output is active while target freshness flags are all false",
                "Check target_state freshness and bridge source timing before using this trace as chase behavior evidence.",
            )
        )


def add_reliable_enemy_unit_info_warnings(records: list[TraceRecord], issues: list[ValidationIssue]) -> None:
    for record in records:
        if not record.unit_info or not record.target_state.reliable_enemy_positions:
            continue
        fresh_enemy = [
            unit for unit in record.unit_info
            if unit.side == "enemy" and unit.has_position and unit.position_fresh
        ]
        missing = [
            label for label in record.target_state.reliable_enemy_positions
            if not any(unit_label_matches(unit.type_name, unit.type_id, label) for unit in fresh_enemy)
        ]
        if not missing:
            continue
        issues.append(
            make_issue(
                "warning",
                record.index,
                "unit.reliable_position_missing",
                "reliable enemy position target(s) missing fresh UnitInfo position: " + ", ".join(missing),
                "Check /ly/position/data or /ly/navi/target_official mock inputs; reliable_enemy_positions should be explainable from fresh enemy UnitInfo.",
            )
        )
        return


def unit_label_matches(type_name: str, type_id: int, label: str) -> bool:
    type_key = str(type_name).strip().lower()
    label_key = str(label).strip().lower()
    aliases = UNIT_LABEL_ALIASES.get(type_key, {type_key})
    return label_key in aliases or label_key == str(type_id)


def add_zero_hp_runtime_mismatch_warnings(records: list[TraceRecord], issues: list[ValidationIssue]) -> None:
    for record in records:
        for unit in record.units:
            if unit.hp != 0 or not is_formal_unit_info_type(unit.type_name, unit.type_id):
                continue
            matching = [
                info for info in record.unit_info
                if info.side == unit.side and unit_info_matches_unit(unit.type_name, unit.type_id, info.type_name, info.type_id)
            ]
            if any(info.has_hp and info.hp_fresh and info.hp == 0 for info in matching):
                continue
            if matching:
                detail = format_zero_hp_unit_info_detail(matching[0])
            else:
                detail = "no matching fresh unit_info HP"
            issues.append(
                make_issue(
                    "warning",
                    record.index,
                    "unit.zero_hp_runtime_mismatch",
                    f"{unit.side}:{unit.type_name} offline unit hp=0, but {detail}",
                    (
                        "Do not treat this trace row as confirmed-dead formal BT evidence. Current "
                        "behavior_tree Health.msg unit subscribers ignore zero HP; record fresh unit_info "
                        "hp=0 or change the formal runtime contract in a separate BT patch before relying on it."
                    ),
                )
            )


def is_formal_unit_info_type(type_name: str, type_id: int) -> bool:
    if type_id in FORMAL_UNIT_INFO_TYPE_IDS:
        return True
    return str(type_name).strip().lower() in FORMAL_UNIT_INFO_TYPE_NAMES


def unit_info_matches_unit(unit_type_name: str, unit_type_id: int, info_type_name: str, info_type_id: int) -> bool:
    if unit_type_id and info_type_id:
        return unit_type_id == info_type_id
    return unit_label_matches(unit_type_name, unit_type_id, info_type_name) or unit_label_matches(
        info_type_name,
        info_type_id,
        unit_type_name,
    )


def format_zero_hp_unit_info_detail(info: object) -> str:
    has_hp = bool(getattr(info, "has_hp", False))
    hp_fresh = bool(getattr(info, "hp_fresh", False))
    hp = int(getattr(info, "hp", 0))
    if has_hp and hp_fresh:
        return f"unit_info still reports hp={hp}"
    if has_hp:
        return f"unit_info reports stale hp={hp}"
    return "unit_info has no HP evidence"


def add_zero_self_hp_runtime_mismatch_warnings(records: list[TraceRecord], issues: list[ValidationIssue]) -> None:
    for record in records:
        if record.referee.self_hp != 0 or not raw_has_self_hp_field(record):
            continue
        issues.append(
            make_issue(
                "warning",
                record.index,
                "referee.zero_self_hp_runtime_mismatch",
                "referee.self_hp=0 appears in trace/mock context, but current behavior_tree selfhealth subscriber ignores zero",
                (
                    "Do not treat this row as proof that formal self-health state reached zero. "
                    "Current /ly/game/all.selfhealth handling updates only when selfhealth > 0; use a positive "
                    "low-health value for recovery/posture rehearsal or change the formal runtime contract separately."
                ),
            )
        )


def raw_has_self_hp_field(record: TraceRecord) -> bool:
    referee = as_dict(record.raw.get("referee"))
    return "self_hp" in referee or "self_hp" in record.raw


def add_posture_lag_warnings(records: list[TraceRecord], issues: list[ValidationIssue]) -> None:
    start: TraceRecord | None = None
    end: TraceRecord | None = None
    for record in records:
        active = (
            record.posture_runtime.has_pending
            and record.posture_current != record.posture_desired
            and record.posture_desired not in {"-", "Unknown", "Unknown (0)"}
        )
        if active:
            if start is None:
                start = record
            end = record
            continue
        maybe_add_posture_lag_warning(start, end, issues)
        start = None
        end = None
    maybe_add_posture_lag_warning(start, end, issues)


def add_outpost_engagement_lock_warnings(records: list[TraceRecord], issues: list[ValidationIssue]) -> None:
    for record in records:
        lock = record.outpost_engagement_lock
        if lock.active and (lock.normal_exit_hp < 0 or lock.enhanced_exit_hp < lock.normal_exit_hp):
            issues.append(make_issue("warning", record.index, "outpost_lock.threshold", "active outpost lock has invalid exit thresholds", "Keep normal threshold non-negative and enhanced threshold no lower than normal."))
        if lock.enhanced_active and not lock.hold_target:
            issues.append(make_issue("warning", record.index, "outpost_lock.active_without_hold", "enhanced outpost attack is active without target-7 hold", "Keep target hold until the enhanced lock exits."))


def maybe_add_posture_lag_warning(
    start: TraceRecord | None,
    end: TraceRecord | None,
    issues: list[ValidationIssue],
) -> None:
    if start is None or end is None:
        return
    span = end.t - start.t
    if span < POSTURE_LAG_WINDOW_SEC:
        return
    issues.append(
        make_issue(
            "warning",
            end.index,
            "runtime.posture_lag",
            f"posture pending for {span:.1f}s: current={end.posture_current} desired={end.posture_desired}",
            "Check posture feedback freshness, retry behavior, and whether degraded posture mode should be visible in this scenario.",
        )
    )


def add_match_time_jump_warnings(records: list[TraceRecord], issues: list[ValidationIssue]) -> None:
    for previous, record in zip(records, records[1:]):
        if not raw_has_time_left(previous) or not raw_has_time_left(record):
            continue
        dt = record.t - previous.t
        if dt < 0:
            continue
        expected_decrease = dt
        actual_decrease = previous.time_left - record.time_left
        mismatch = abs(actual_decrease - expected_decrease)
        if mismatch <= MATCH_TIME_JUMP_TOLERANCE_SEC:
            continue
        issues.append(
            make_issue(
                "warning",
                record.index,
                "scenario.match_time_jump",
                f"match time changed by {actual_decrease:.1f}s over {dt:.1f}s of trace time",
                "Check match-control commands, mock time_left publishing, or merged trace ordering before comparing timing-sensitive behavior.",
            )
        )
        return


def add_missing_referee_resource_warnings(records: list[TraceRecord], issues: list[ValidationIssue]) -> None:
    for record in records:
        missing = [field for field in REFEREE_RESOURCE_FIELDS if getattr(record.referee, field) is None]
        if not missing:
            continue
        issues.append(
            make_issue(
                "warning",
                record.index,
                "referee.missing_resource_state",
                "missing referee resource fields: " + ", ".join(missing),
                "Record outpost/base HP fields so recovery, outpost, and base-defense decisions can be reviewed offline.",
            )
        )
        return


def raw_has_time_left(record: TraceRecord) -> bool:
    referee = as_dict(record.raw.get("referee"))
    return "time_left" in referee or "time_left" in record.raw


def validation_status(issues: list[ValidationIssue]) -> str:
    errors = sum(1 for issue in issues if issue.severity == "error")
    warnings = sum(1 for issue in issues if issue.severity == "warning")
    if errors:
        return "FAIL"
    if warnings:
        return "WARN"
    return "PASS"


def issue_group(issue: ValidationIssue) -> str:
    return issue.code.split(".", 1)[0] if "." in issue.code else "validation"


def next_actions(issues: list[ValidationIssue]) -> list[str]:
    actions: list[str] = []
    if any(issue.severity == "error" for issue in issues):
        actions.append("Fix ERROR items before trusting replay, Foxglove export, or offline decision comparison.")
    if any(issue.severity == "warning" for issue in issues):
        actions.append("Review WARNING items before using the trace as regression evidence.")
    if issues:
        actions.append("Re-run `PYTHONPATH=src/simulator python3 -m simulator.main <trace> --validate-only` after fixes.")
    return actions


def validation_report(records: list[TraceRecord], issues: list[ValidationIssue]) -> dict[str, Any]:
    duration = records[-1].t - records[0].t if records else 0.0
    first_tick = records[0].tick if records else 0
    last_tick = records[-1].tick if records else 0
    schema_counts = Counter(f"v{record.schema_version}" for record in records)
    output_counts = Counter(record.output.kind for record in records)
    errors = sum(1 for issue in issues if issue.severity == "error")
    warnings = sum(1 for issue in issues if issue.severity == "warning")
    issue_rows = [
        {
            "severity": issue.severity,
            "record_index": issue.index,
            "code": issue.code,
            "group": issue_group(issue),
            "message": issue.message,
            "suggestion": issue.suggestion,
        }
        for issue in issues
    ]
    grouped_codes: dict[str, list[str]] = {}
    for issue in issues:
        grouped_codes.setdefault(issue_group(issue), []).append(issue.code)

    return {
        "schema": "ly_simulator_validation_report_v1",
        "status": validation_status(issues),
        "summary": {
            "records": len(records),
            "duration_sec": duration,
            "tick_range": {"first": first_tick, "last": last_tick},
            "issues_by_severity": {"errors": errors, "warnings": warnings},
            "schema_versions": dict(sorted(schema_counts.items())),
            "output_kinds": dict(sorted(output_counts.items())),
        },
        "issues": issue_rows,
        "issue_groups": grouped_codes,
        "next_actions": next_actions(issues),
    }


def format_validation(records: list[TraceRecord], issues: list[ValidationIssue]) -> str:
    report = validation_report(records, issues)
    summary = report["summary"]
    severity = summary["issues_by_severity"]
    lines = [
        "Simulator Validation Report",
        f"Status: {report['status']}",
        f"Records: {summary['records']}",
        f"Duration: {summary['duration_sec']:.2f}s ticks={summary['tick_range']['first']}..{summary['tick_range']['last']}",
        f"Issues by severity: errors={severity['errors']} warnings={severity['warnings']}",
        "Schema versions: " + format_mapping(summary["schema_versions"]),
        "Output kinds: " + format_mapping(summary["output_kinds"]),
    ]
    if not issues:
        lines.append("No issues detected.")
        return "\n".join(lines)

    lines.append("Issues:")
    rendered = 0
    for group, group_issues in grouped_issues(issues):
        lines.append(f"{group.title()} issues:")
        for issue in group_issues:
            if rendered >= 40:
                break
            rendered += 1
            prefix = issue.severity.upper()
            index = "-" if issue.index is None else str(issue.index)
            lines.append(f"- {prefix} [{issue.code}] record={index}: {issue.message}")
            if issue.suggestion:
                lines.append(f"  suggestion: {issue.suggestion}")
        if rendered >= 40:
            break
    if len(issues) > 40:
        lines.append(f"... {len(issues) - 40} more issue(s)")
    lines.append("Next actions:")
    for action in report["next_actions"]:
        lines.append(f"- {action}")
    return "\n".join(lines)


def format_mapping(mapping: dict[str, int]) -> str:
    if not mapping:
        return "-"
    return " ".join(f"{key}={mapping[key]}" for key in sorted(mapping))


def grouped_issues(issues: list[ValidationIssue]) -> list[tuple[str, list[ValidationIssue]]]:
    groups: dict[str, list[ValidationIssue]] = {}
    for issue in issues:
        group = issue_group(issue)
        groups.setdefault(group, []).append(issue)
    ordered: list[tuple[str, list[ValidationIssue]]] = []
    for group in ISSUE_GROUP_ORDER:
        if group in groups:
            ordered.append((group, groups.pop(group)))
    for group in sorted(groups):
        ordered.append((group, groups[group]))
    return ordered
