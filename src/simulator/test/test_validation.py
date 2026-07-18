from __future__ import annotations

from copy import deepcopy

from simulator.trace import normalize_record
from simulator.validation import format_validation, validation_report, validate_records

from test_trace_contract import stable_trace_row


def make_record(raw: dict, index: int = 0):
    return normalize_record(raw, index, {2: "Recovery", 3: "BuffShoot", 13: "OutpostShoot", 18: "OccupyArea", 23: "BuffOutpost"})


def route_row(goal_id: int, t: float, index: int) -> dict:
    raw = deepcopy(stable_trace_row())
    raw["tick"] = index
    raw["t"] = t
    raw["referee"]["time_left"] = 408 - int(t)
    raw["decision_output"]["goal_id"] = goal_id
    raw["decision_output"]["goal_base_id"] = goal_id
    raw["decision_output"]["goal_name"] = f"Goal{goal_id}"
    raw["decision_output"]["goal_pos_cm"] = {"x": 1000 + goal_id, "y": 800}
    raw["decision_intent"]["base_goal_id"] = goal_id
    raw["decision_intent"]["resolved_goal_id"] = goal_id
    return raw


def test_validation_issues_have_stable_codes_and_suggestions() -> None:
    first = stable_trace_row()
    second = stable_trace_row()
    second["t"] = first["t"] - 1.0
    second.pop("decision_intent")
    second["decision_output"]["goal_pos_cm"] = {"x": 3200, "y": 1600}
    second["units"]["friend"][0]["hp"] = 450

    records = [
        normalize_record(first, 0, {18: "OccupyArea"}),
        normalize_record(second, 1, {18: "OccupyArea"}),
    ]

    issues = validate_records(records, {"field_cm": {"width": 2800, "height": 1500}}, bad_lines=2)
    by_code = {issue.code: issue for issue in issues}

    assert by_code["trace.bad_line"].severity == "warning"
    assert by_code["schema.missing_decision_intent"].severity == "warning"
    assert by_code["trace.time_monotonic"].severity == "error"
    assert by_code["output.goal_pos_bounds"].severity == "error"
    assert by_code["unit.hp_bounds"].severity == "warning"
    assert "sort" in by_code["trace.time_monotonic"].suggestion


def test_validation_report_is_actionable_for_failed_traces() -> None:
    raw = stable_trace_row()
    raw.pop("decision_output")
    record = normalize_record(raw, 0, {18: "OccupyArea"})

    report = format_validation(
        [record],
        validate_records([record], {"field_cm": {"width": 2800, "height": 1500}}),
    )

    assert "Simulator Validation Report" in report
    assert "Status: FAIL" in report
    assert "Records: 1" in report
    assert "Issues by severity: errors=1 warnings=0" in report
    assert "schema.missing_decision_output" in report
    assert "Next actions:" in report


def test_validation_report_summarizes_clean_sample_shape() -> None:
    record = normalize_record(stable_trace_row(), 0, {18: "OccupyArea"})
    report = format_validation(
        [record],
        validate_records([record], {"field_cm": {"width": 2800, "height": 1500}}),
    )

    assert "Status: PASS" in report
    assert "Schema versions: v2=1" in report
    assert "Output kinds: goal_pos=1" in report
    assert "No issues detected." in report


def test_validation_report_has_machine_readable_json_shape() -> None:
    first = stable_trace_row()
    second = stable_trace_row()
    second["tick"] = 8
    second["t"] = first["t"] + 1.0
    second["referee"]["time_left"] = first["referee"]["time_left"] - 20

    records = [
        normalize_record(first, 0, {18: "OccupyArea"}),
        normalize_record(second, 1, {18: "OccupyArea"}),
    ]
    issues = validate_records(records, {"field_cm": {"width": 2800, "height": 1500}})

    report = validation_report(records, issues)

    assert report["schema"] == "ly_simulator_validation_report_v1"
    assert report["status"] == "WARN"
    assert report["summary"] == {
        "records": 2,
        "duration_sec": 1.0,
        "tick_range": {"first": 7, "last": 8},
        "issues_by_severity": {"errors": 0, "warnings": 1},
        "schema_versions": {"v2": 2},
        "output_kinds": {"goal_pos": 2},
    }
    assert report["issues"] == [
        {
            "severity": "warning",
            "record_index": 1,
            "code": "scenario.match_time_jump",
            "group": "scenario",
            "message": "match time changed by 20.0s over 1.0s of trace time",
            "suggestion": "Check match-control commands, mock time_left publishing, or merged trace ordering before comparing timing-sensitive behavior.",
        }
    ]
    assert report["issue_groups"] == {"scenario": ["scenario.match_time_jump"]}
    assert report["next_actions"] == [
        "Review WARNING items before using the trace as regression evidence.",
        "Re-run `PYTHONPATH=src/simulator python3 -m simulator.main <trace> --validate-only` after fixes.",
    ]


def test_validation_warns_on_route_churn_without_blocking_replay() -> None:
    records = [
        make_record(route_row(18, 0.0, 0), 0),
        make_record(route_row(3, 0.5, 1), 1),
        make_record(route_row(18, 1.0, 2), 2),
        make_record(route_row(3, 1.5, 3), 3),
    ]

    issues = validate_records(records, {"field_cm": {"width": 2800, "height": 1500}})
    by_code = {issue.code: issue for issue in issues}

    assert by_code["scenario.route_churn"].severity == "warning"
    assert "4 route selections" in by_code["scenario.route_churn"].message


def test_validation_warns_on_stale_target_while_chasing() -> None:
    raw = deepcopy(stable_trace_row())
    raw["decision_output"].update(
        {
            "kind": "relative_target_bridge",
            "uses_goal_pos": False,
            "uses_to_navi": True,
            "relative_target_valid": True,
            "output_topic": "/ly/navi/target_rel",
            "final_goal_pos_topic": "/goal_pose",
        }
    )
    raw["decision_intent"]["layer"] = "Chase"
    raw["decision_intent"]["reason"] = "RelativeTarget"
    raw["target_state"].update(
        {
            "has_recent_target": False,
            "external_aim_active": False,
            "fresh_current_aim": False,
            "fresh_auto_aim": False,
            "fresh_buff": False,
            "fresh_outpost": False,
        }
    )
    raw["navi_relative_target"] = {"valid": True, "x": 1.0, "y": 0.0, "z": 0.0}

    issues = validate_records([make_record(raw)], {"field_cm": {"width": 2800, "height": 1500}})

    assert any(issue.code == "scenario.stale_chase_target" and issue.severity == "warning" for issue in issues)


def test_validation_warns_on_posture_lag_and_groups_report() -> None:
    records = []
    for index, t in enumerate((0.0, 3.0, 6.0)):
        raw = deepcopy(stable_trace_row())
        raw["tick"] = index
        raw["t"] = t
        raw["posture"]["runtime"].update(
            {
                "current": {"id": 3, "name": "Move"},
                "desired": {"id": 1, "name": "Attack"},
                "pending": {"id": 1, "name": "Attack"},
                "has_pending": True,
            }
        )
        raw["posture"]["command"] = {"id": 1, "name": "Attack"}
        raw["posture"]["state"] = {"id": 3, "name": "Move"}
        raw["referee"]["time_left"] = 408 - int(t)
        records.append(make_record(raw, index))

    issues = validate_records(records, {"field_cm": {"width": 2800, "height": 1500}})
    report = format_validation(records, issues)

    assert any(issue.code == "runtime.posture_lag" and issue.severity == "warning" for issue in issues)
    assert "Runtime issues:" in report
    assert "runtime.posture_lag" in report


def test_validation_warns_on_match_time_jumps_and_missing_referee_resources() -> None:
    first = deepcopy(stable_trace_row())
    second = deepcopy(stable_trace_row())
    second["tick"] = 1
    second["t"] = first["t"] + 1.0
    second["referee"]["time_left"] = first["referee"]["time_left"] - 20
    for key in ("self_outpost_hp", "enemy_outpost_hp", "self_base_hp", "enemy_base_hp"):
        second["referee"].pop(key)

    records = [make_record(first, 0), make_record(second, 1)]
    issues = validate_records(records, {"field_cm": {"width": 2800, "height": 1500}})
    by_code = {issue.code: issue for issue in issues}

    assert by_code["scenario.match_time_jump"].severity == "warning"
    assert by_code["referee.missing_resource_state"].severity == "warning"


def test_validation_checks_unit_info_bounds_and_reliable_enemy_positions() -> None:
    raw = deepcopy(stable_trace_row())
    raw["target_state"]["reliable_enemy_positions"] = [
        {"id": 1, "name": "Hero"},
        {"id": 7, "name": "Sentry"},
    ]
    raw["unit_info"] = {
        "enemy": [
            {
                "car_id": 101,
                "hp": 100,
                "has_hp": True,
                "hp_fresh": True,
                "position_x": 1165,
                "position_y": 1063,
                "has_position": True,
                "position_fresh": True,
                "position_source": "position_data",
                "area_id": 6,
                "area_name": "enemy_ready_roadland",
            },
            {
                "car_id": 107,
                "hp": 400,
                "has_hp": True,
                "hp_fresh": True,
                "position_x": 3100,
                "position_y": 1600,
                "has_position": True,
                "position_fresh": False,
                "position_source": "position_data",
                "area_id": 0,
                "area_name": "unknown",
            },
        ]
    }

    issues = validate_records([make_record(raw)], {"field_cm": {"width": 2800, "height": 1500}})
    by_code = {issue.code: issue for issue in issues}

    assert by_code["unit.info_position_bounds"].severity == "warning"
    assert by_code["unit.reliable_position_missing"].severity == "warning"
    assert "Sentry" in by_code["unit.reliable_position_missing"].message


def test_validation_warns_on_formal_navigation_debug_inconsistencies() -> None:
    raw = deepcopy(stable_trace_row())
    raw["goal_reach_state"]["goal_id"] = 3
    raw["goal_reach_state"]["base_goal_id"] = 3
    raw["goal_reach_state"]["distance_cm"] = -1.0
    raw["navi_relative_target"].update({"valid": True, "frame_id": ""})
    raw["navi_velocity"]["raw_to_mps"] = 0.0
    raw["navi_status"].update({"should_rotate_fresh": True, "should_rotate": None})

    issues = validate_records([make_record(raw)], {"field_cm": {"width": 2800, "height": 1500}})
    by_code = {issue.code: issue for issue in issues}

    assert by_code["output.goal_reach_mismatch"].severity == "warning"
    assert by_code["output.goal_reach_distance"].severity == "warning"
    assert by_code["output.relative_target_frame_missing"].severity == "warning"
    assert by_code["output.navi_velocity_scale"].severity == "warning"
    assert by_code["output.should_rotate_missing"].severity == "warning"


def test_validation_warns_when_zero_unit_hp_is_not_confirmed_by_formal_unit_info() -> None:
    raw = deepcopy(stable_trace_row())
    raw["units"]["enemy"] = [
        {
            "type_id": 1,
            "type": "Hero",
            "side": "enemy",
            "hp": 0,
            "max_hp": 200,
            "position_cm": {"x": 1165, "y": 1063},
        }
    ]
    raw["unit_info"] = {
        "enemy": [
            {
                "car_id": 101,
                "hp": 120,
                "has_hp": True,
                "hp_fresh": True,
                "position_x": 1165,
                "position_y": 1063,
                "has_position": True,
                "position_fresh": True,
                "position_source": "position_data",
            }
        ]
    }

    issues = validate_records([make_record(raw)], {"field_cm": {"width": 2800, "height": 1500}})
    by_code = {issue.code: issue for issue in issues}

    assert by_code["unit.zero_hp_runtime_mismatch"].severity == "warning"
    assert "enemy:Hero" in by_code["unit.zero_hp_runtime_mismatch"].message
    assert "unit_info still reports hp=120" in by_code["unit.zero_hp_runtime_mismatch"].message


def test_validation_allows_zero_unit_hp_when_formal_unit_info_confirms_zero() -> None:
    raw = deepcopy(stable_trace_row())
    raw["units"]["enemy"] = [
        {
            "type_id": 1,
            "type": "Hero",
            "side": "enemy",
            "hp": 0,
            "max_hp": 200,
            "position_cm": {"x": 1165, "y": 1063},
        }
    ]
    raw["unit_info"] = {
        "enemy": [
            {
                "car_id": 101,
                "hp": 0,
                "has_hp": True,
                "hp_fresh": True,
                "position_x": 1165,
                "position_y": 1063,
                "has_position": True,
                "position_fresh": True,
            }
        ]
    }

    issues = validate_records([make_record(raw)], {"field_cm": {"width": 2800, "height": 1500}})

    assert not [issue for issue in issues if issue.code == "unit.zero_hp_runtime_mismatch"]


def test_validation_warns_when_zero_unit_hp_has_no_formal_unit_info_evidence() -> None:
    raw = deepcopy(stable_trace_row())
    raw["units"]["enemy"] = [
        {
            "type_id": 7,
            "type": "Sentry",
            "side": "enemy",
            "hp": 0,
            "max_hp": 400,
            "position_cm": {"x": 1831, "y": 1132},
        },
        {
            "type_id": 6,
            "type": "Drone",
            "side": "enemy",
            "hp": 0,
            "max_hp": 150,
            "position_cm": {"x": 1680, "y": 240},
        },
    ]
    raw["unit_info"] = {"enemy": []}

    issues = validate_records([make_record(raw)], {"field_cm": {"width": 2800, "height": 1500}})
    mismatches = [issue for issue in issues if issue.code == "unit.zero_hp_runtime_mismatch"]

    assert len(mismatches) == 1
    assert "enemy:Sentry" in mismatches[0].message
    assert "no matching fresh unit_info HP" in mismatches[0].message


def test_validation_warns_when_trace_records_explicit_zero_self_hp() -> None:
    raw = deepcopy(stable_trace_row())
    raw["referee"]["self_hp"] = 0
    raw["referee"]["has_self_hp"] = True
    raw["events"]["self_low_hp"] = True

    issues = validate_records([make_record(raw)], {"field_cm": {"width": 2800, "height": 1500}})
    by_code = {issue.code: issue for issue in issues}

    assert by_code["referee.zero_self_hp_runtime_mismatch"].severity == "warning"
    assert "referee.self_hp=0" in by_code["referee.zero_self_hp_runtime_mismatch"].message
