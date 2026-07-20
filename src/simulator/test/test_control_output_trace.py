from __future__ import annotations

from copy import deepcopy

from simulator.trace import normalize_record
from simulator.validation import validate_records
from simulator.viewer import Viewer, record_status_payload

from test_trace_contract import stable_trace_row


def v3_row(
    *,
    feedback_rotate: int = 0,
    final_rotate: int = 3,
    sequence: int = 41,
    age_ms: int = 12,
) -> dict:
    row = deepcopy(stable_trace_row())
    row["schema_version"] = 3
    row["gimbal_feedback"] = {
        "available": True,
        "age_ms": 8,
        "fire_code": {
            "fire_status": 0,
            "cap_state": 1,
            "follow_mode": 0,
            "aim_mode": 0,
            "rotate": feedback_rotate,
        },
    }
    row["control_output"] = {
        "available": True,
        "sequence": sequence,
        "age_ms": age_ms,
        "source": "normal",
        "angles": {"published": True, "yaw": 12.5, "pitch": -3.25},
        "fire_code": {
            "published": True,
            "fire_status": 1,
            "cap_state": 1,
            "follow_mode": True,
            "aim_mode": True,
            "rotate": final_rotate,
        },
        "trajectory": {
            "published": True,
            "available": True,
            "unavailable_reason": "",
            "yaw": 12.5,
            "pitch": -3.25,
            "yaw_omega": 1.0,
            "pitch_omega": -2.0,
            "yaw_alpha": 3.0,
            "pitch_alpha": -4.0,
        },
    }
    return row


def test_v3_trace_keeps_feedback_and_final_command_distinct() -> None:
    record = normalize_record(v3_row(feedback_rotate=0, final_rotate=3), 0, {18: "OccupyArea"})

    assert record.gimbal_feedback.available is True
    assert record.gimbal_feedback.rotate == 0
    assert record.control_output.is_available is True
    assert record.control_output.fire_code.rotate == 3
    assert record.control_output.fire_code.follow_mode is True
    assert record.control_output.trajectory.published is True
    assert record.control_output.trajectory.available is True


def test_web_status_keeps_final_output_and_feedback_as_distinct_evidence() -> None:
    record = normalize_record(v3_row(feedback_rotate=1, final_rotate=0), 0, {18: "OccupyArea"})

    payload = record_status_payload(record)

    assert payload["gimbal_feedback"]["available"] is True
    assert payload["gimbal_feedback"]["fire_code"]["rotate"] == 1
    assert payload["control_output"]["available"] is True
    assert payload["control_output"]["fire_code"]["rotate"] == 0
    assert payload["control_output"]["trajectory"]["available"] is True


def test_pygame_control_panel_keeps_final_output_and_feedback_distinct() -> None:
    record = normalize_record(v3_row(feedback_rotate=1, final_rotate=0), 0, {18: "OccupyArea"})
    viewer = object.__new__(Viewer)

    output_rows = dict(viewer.control_output_rows(record))
    feedback_rows = dict(viewer.gimbal_feedback_rows(record))

    assert output_rows["FireCode"] == "fire=1 cap=1 follow=1 aim=1 rotate=0"
    assert output_rows["Trajectory"] == "published=1 valid=1 yaw=12.5 pitch=-3.2"
    assert feedback_rows["Available"] == "Y age=8ms"
    assert feedback_rows["FireCode"] == "fire=0 cap=1 follow=0 aim=0 rotate=1"


def test_v2_trace_replay_uses_empty_control_output() -> None:
    record = normalize_record(stable_trace_row(), 0, {18: "OccupyArea"})

    assert record.schema_version == 2
    assert record.gimbal_feedback.available is False
    assert record.control_output.is_available is False
    assert record.control_output.sequence is None
    assert record.control_output.angles.published is False
    assert record.control_output.fire_code.published is False
    assert record.control_output.trajectory.available is False


def test_v3_trace_without_new_objects_is_still_explicitly_unavailable() -> None:
    row = stable_trace_row()
    row["schema_version"] = 3
    record = normalize_record(row, 0, {18: "OccupyArea"})

    assert record.gimbal_feedback.available is False
    assert record.gimbal_feedback.rotate is None
    assert record.control_output.is_available is False
    assert record.control_output.source == "not_recorded"


def test_non_publish_event_keeps_latest_output_sequence_and_age() -> None:
    row = v3_row(sequence=91, age_ms=37)
    row["event"] = "game_start"
    record = normalize_record(row, 0, {18: "OccupyArea"})

    assert record.event == "game_start"
    assert record.control_output.sequence == 91
    assert record.control_output.age_ms == 37
    assert record.control_output.fire_code.published is True


def test_safe_control_keeps_its_explicit_trajectory_absence_reason() -> None:
    row = v3_row()
    row["control_output"].update({"source": "safe_control"})
    row["control_output"]["trajectory"] = {
        "published": False,
        "available": False,
        "unavailable_reason": "safe_control",
    }
    record = normalize_record(row, 0, {18: "OccupyArea"})

    assert record.control_output.source == "safe_control"
    assert record.control_output.trajectory.published is False
    assert record.control_output.trajectory.available is False
    assert record.control_output.trajectory.unavailable_reason == "safe_control"


def test_validation_checks_trajectory_values_only_when_available() -> None:
    unavailable = v3_row()
    unavailable["control_output"]["trajectory"] = {
        "published": False,
        "available": False,
        "unavailable_reason": "invalid_dynamics",
        "yaw": float("nan"),
    }
    unavailable_record = normalize_record(unavailable, 0, {18: "OccupyArea"})
    unavailable_issues = validate_records([unavailable_record], {"field_cm": {"width": 2800, "height": 1500}})
    assert not [issue for issue in unavailable_issues if issue.code == "control.trajectory_nonfinite"]

    invalid = v3_row()
    invalid["control_output"]["trajectory"]["yaw_alpha"] = float("nan")
    invalid_record = normalize_record(invalid, 0, {18: "OccupyArea"})
    invalid_issues = validate_records([invalid_record], {"field_cm": {"width": 2800, "height": 1500}})
    assert any(issue.code == "control.trajectory_nonfinite" for issue in invalid_issues)
