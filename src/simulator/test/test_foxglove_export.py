from __future__ import annotations

from simulator.foxglove_export import record_to_decision_frame, record_to_goal_point, record_to_metrics
from simulator.trace import normalize_record

from test_trace_contract import expected_rfid_match_payload, stable_trace_row


def test_foxglove_export_uses_stable_trace_record() -> None:
    raw = stable_trace_row()
    raw["bt_debug"] = {"new_node": "ignored by exporter"}
    record = normalize_record(raw, 0, {18: "OccupyArea"})

    frame = record_to_decision_frame(record)
    metrics = record_to_metrics(record)
    point = record_to_goal_point(record)

    assert frame["schema"] == "ly_simulator_decision_frame_v1"
    assert frame["output"]["goal_id"] == 18
    assert frame["output"]["chase_official_armor_type"] == 1
    assert frame["intent"]["reason"] == "TargetVisible"
    assert frame["goal_reach"]["status"] == "traveling"
    assert frame["goal_reach"]["distance_cm"] == 94.0
    assert frame["navi_status"]["should_rotate"] is True
    assert frame["navi_velocity"]["output_x_mps"] == 0.15000000000000002
    assert frame["navi_relative_target"]["frame_id"] == "base_link"
    assert frame["target_state"]["external_aim_active"] is True
    assert frame["target_state"]["fresh_current_aim"] is True
    assert frame["events"]["regional_defense_active"] is True
    assert frame["referee"]["rfid_status"] == 65537
    assert frame["referee"]["has_rfid_status_2"] is True
    assert frame["referee"]["rfid_status_2"] == 7
    assert frame["referee"]["rfid_match"] == expected_rfid_match_payload()
    assert frame["bullet_info"]["has_received"] is True
    assert frame["bullet_info"]["initial_speed"] == 23.4
    assert frame["bullet_info"]["projectile_allowance_17mm"] == 118
    assert "bt_debug" not in frame
    assert metrics["goal_id"] == 18
    assert metrics["target_visible"] == 1
    assert metrics["goal_reach_status_id"] == 1
    assert metrics["goal_distance_cm"] == 94.0
    assert metrics["should_rotate"] == 1
    assert metrics["navi_velocity_output_x_raw"] == 6
    assert metrics["navi_velocity_output_x_mps"] == 0.15000000000000002
    assert point == {"x": 1075.0, "y": 898.0}
