from __future__ import annotations

import json
from pathlib import Path

from simulator.assets import load_unit_asset_catalog
from simulator.config import load_config
from simulator.panel_scroll import PanelScrollState
from simulator.trace import normalize_record
from simulator.validation import validate_records
from simulator.viewer import Viewer, record_status_payload


def stable_trace_row() -> dict:
    return {
        "schema": "ly_decision_trace_v1",
        "schema_version": 2,
        "event": "tick",
        "tick": 7,
        "t": 12.5,
        "field_cm": {"width": 2800, "height": 1500, "frame": "left_bottom_origin_cm"},
        "competition_profile": "regional",
        "strategy_mode": "Regional",
        "aim_mode": "AutoAim",
        "team": "red",
        "target_armor": {"id": 1, "name": "Hero", "distance_m": 8.0},
        "target_state": {
            "has_recent_target": True,
            "external_aim_active": True,
            "fresh_current_aim": True,
            "fresh_auto_aim": True,
            "fresh_buff": False,
            "fresh_outpost": False,
            "hitable_targets": [{"id": 1, "name": "Hero"}],
            "reliable_enemy_positions": [{"id": 1, "name": "Hero"}],
        },
        "events": {
            "event_data_fresh": True,
            "sentry_info_fresh": True,
            "buff_task_enabled": False,
            "buff_can_activate": False,
            "buff_activating": False,
            "buff_activated": False,
            "outpost_task_enabled": True,
            "enemy_outpost_alive": True,
            "outpost_attack_window_open": True,
            "self_low_hp": False,
            "self_low_ammo": False,
            "recent_damage_over_30": False,
            "armor_target_visible": True,
            "buff_target_locked": False,
            "outpost_target_locked": True,
            "goal_reached": False,
            "goal_unreachable": False,
            "regional_defense_active": True,
        },
        "units": {
            "friend": [
                {
                    "type_id": 7,
                    "type": "Sentry",
                    "side": "friend",
                    "hp": 380,
                    "max_hp": 400,
                    "position_cm": {"x": 1000, "y": 900},
                }
            ],
            "enemy": [],
        },
        "decision_output": {
            "kind": "goal_pos",
            "source": "behavior_tree",
            "publish_allowed": True,
            "publish_enabled": True,
            "uses_goal_pos": True,
            "uses_to_navi": False,
            "relative_target_valid": False,
            "chase_official_target_valid": False,
            "chase_official_armor_type": 1,
            "output_topic": "/ly/navi/goal_pos",
            "output_frame": "map",
            "final_goal_pos_topic": "/ly/navi/goal_pos",
            "goal_id": 18,
            "goal_base_id": 18,
            "goal_name": "OccupyArea",
            "goal_side": "red",
            "speed_level": 1,
            "goal_pos_cm": {"x": 1075, "y": 898},
        },
        "decision_intent": {
            "layer": "RegionalPatrol",
            "reason": "TargetVisible",
            "base_goal_id": 18,
            "resolved_goal_id": 18,
            "goal_team": "red",
            "apply_team_offset": False,
            "priority": 60,
            "detail": "hold occupy area while armor target is fresh",
        },
        "goal_reach_state": {
            "status": "traveling",
            "status_id": 1,
            "reason": "position_distance",
            "reason_id": 3,
            "goal_id": 18,
            "base_goal_id": 18,
            "goal_age_ms": 1250,
            "external_reach_fresh": True,
            "external_reach": False,
            "external_reachable_fresh": True,
            "external_reachable": True,
            "position_fresh": True,
            "has_position": True,
            "distance_cm": 94.0,
            "arrive_distance_cm": 50,
            "face_distance_cm": 120,
            "distance_fallback_allowed": True,
            "within_arrive_distance": False,
            "within_face_distance": True,
            "timeout": False,
        },
        "navi_velocity": {
            "input_x": 8,
            "input_y": -2,
            "output_x": 6,
            "output_y": -1,
            "raw_to_mps": 0.025,
        },
        "navi_status": {
            "should_rotate": True,
            "should_rotate_fresh": True,
            "reached": False,
            "reached_fresh": True,
            "reachable": True,
            "reachable_fresh": True,
        },
        "navi_relative_target": {
            "valid": False,
            "frame_id": "base_link",
            "armor_type": 1,
            "aim_mode": 1,
            "official_target_valid": False,
            "official_armor_type": 1,
        },
        "posture": {
            "command": {"id": 1, "name": "Attack"},
            "state": {"id": 1, "name": "Attack"},
            "last_reason": "target_visible",
            "runtime": {
                "current": {"id": 1, "name": "Attack"},
                "desired": {"id": 1, "name": "Attack"},
                "pending": {"id": 0, "name": "Unknown"},
                "has_pending": False,
                "feedback_stale": False,
                "retry_count": 0,
                "degraded": {"attack": False, "defense": False, "move": False},
            },
        },
        "referee": {
            "self_hp": 380,
            "self_max_hp": 400,
            "self_outpost_hp": 60,
            "enemy_outpost_hp": 55,
            "self_base_hp": 5000,
            "enemy_base_hp": 5000,
            "ammo": 42,
            "time_left": 408,
            "sentry_can_activate_energy": False,
            "energy_activate_confirm_pulse": False,
            "event_self_fortress_gain_point_status": 1,
            "event_self_outpost_gain_point_status": 0,
            "event_self_base_gain_point_status": False,
            "rfid_status": 65537,
            "has_rfid_status_2": True,
            "rfid_status_2": 7,
            "rfid_match": {
                "fresh": True,
                "any": True,
                "raw": 65537,
                "has_rfid_status_2": True,
                "rfid_status_2_raw": 7,
                "self_base_gain_point": False,
                "self_supply": True,
                "self_non_resource_supply": False,
                "self_resource_supply": True,
                "self_highland_gain_point": False,
                "enemy_highland_gain_point": False,
                "self_road_crossing": False,
                "enemy_road_crossing": False,
                "self_central_highland_crossing": False,
                "enemy_central_highland_crossing": False,
                "self_tunnel": False,
                "enemy_tunnel": True,
                "tunnel": True,
                "center_gain_point": True,
                "self_fortress_gain_point": False,
                "enemy_fortress_gain_point": True,
                "self_outpost_gain_point": False,
                "enemy_outpost_gain_point": False,
                "self_assembly_gain_point": False,
                "enemy_assembly_gain_point": False,
                "self_fly_ramp": False,
                "enemy_fly_ramp": False,
                "on_self_side": True,
                "on_enemy_side": True,
            },
            "team_buff": {"attack": 1, "defence": 0, "remaining_energy": 24},
        },
        "gimbal": {
            "yaw_deg": 8.0,
            "pitch_deg": -2.0,
            "yaw_vel_deg_per_sec": 15.0,
            "yaw_angle_deg": 8.0,
            "cap_v": 22,
            "navi_lower_head": 0,
            "fire_code": {
                "fire_status": 1,
                "cap_state": 1,
                "follow_mode": 0,
                "aim_mode": 1,
                "rotate": 0,
            },
        },
        "bullet_info": {
            "has_received": True,
            "age_ms": 120,
            "has_initial_speed": True,
            "initial_speed": 23.4,
            "has_shoot_data": True,
            "bullet_type": 1,
            "shooter_number": 7,
            "launching_frequency": 9,
            "has_projectile_allowance": True,
            "projectile_allowance_17mm": 118,
            "projectile_allowance_42mm": 6,
            "remaining_gold_coin": 14,
            "projectile_allowance_fortress_17mm": 32,
        },
        "runtime_guard": {"fault": "None", "recovery_requested": False, "recovering": False},
    }


def expected_rfid_match_payload() -> dict:
    payload = {
        "fresh": True,
        "any": True,
        "raw": 65537,
        "has_rfid_status_2": True,
        "rfid_status_2_raw": 7,
        "self_base_gain_point": False,
        "self_supply": True,
        "self_non_resource_supply": False,
        "self_resource_supply": True,
        "self_highland_gain_point": False,
        "enemy_highland_gain_point": False,
        "self_road_crossing": False,
        "enemy_road_crossing": False,
        "self_central_highland_crossing": False,
        "enemy_central_highland_crossing": False,
        "self_tunnel": False,
        "enemy_tunnel": True,
        "tunnel": True,
        "center_gain_point": True,
        "self_fortress_gain_point": False,
        "enemy_fortress_gain_point": True,
        "self_outpost_gain_point": False,
        "enemy_outpost_gain_point": False,
        "self_assembly_gain_point": False,
        "enemy_assembly_gain_point": False,
        "self_fly_ramp": False,
        "enemy_fly_ramp": False,
        "on_self_side": True,
        "on_enemy_side": True,
    }
    return payload


def test_trace_record_exposes_stable_simulator_contract() -> None:
    record = normalize_record(stable_trace_row(), 0, {18: "OccupyArea"})

    assert record.schema_version == 2
    assert record.has_decision_output
    assert record.has_decision_intent
    assert record.field.width == 2800
    assert record.competition_profile == "regional"
    assert record.output.goal_name == "OccupyArea"
    assert record.output.chase_official_target_valid is False
    assert record.output.chase_official_armor_type == 1
    assert record.decision_intent.layer == "RegionalPatrol"
    assert record.events.regional_defense_active
    assert record.target_state.external_aim_active is True
    assert record.target_state.fresh_current_aim is True
    assert record.goal_reach.status == "traveling"
    assert record.goal_reach.reason == "position_distance"
    assert record.goal_reach.distance_cm == 94.0
    assert record.navi_status.should_rotate is True
    assert record.navi_status.reachable is True
    assert record.navi_velocity.output_x == 6
    assert record.navi_velocity.raw_to_mps == 0.025
    assert record.navi_relative_target.frame_id == "base_link"
    assert record.referee.self_outpost_hp == 60
    assert record.referee.team_buff_remaining_energy == 24
    assert record.referee.rfid_status == 65537
    assert record.referee.has_rfid_status_2 is True
    assert record.referee.rfid_status_2 == 7
    assert record.referee.rfid_match.as_payload() == expected_rfid_match_payload()
    assert record.bullet_info.has_received
    assert record.bullet_info.initial_speed == 23.4
    assert record.bullet_info.projectile_allowance_17mm == 118
    assert not record.posture_runtime.feedback_stale


def test_goal_reach_preserves_known_but_stale_position_contract() -> None:
    raw = stable_trace_row()
    raw["goal_reach_state"].update(
        {
            "reason": "position_stale",
            "reason_id": 5,
            "position_fresh": False,
            "has_position": True,
            "distance_cm": None,
            "within_arrive_distance": False,
            "within_face_distance": False,
        }
    )

    record = normalize_record(raw, 0, {18: "OccupyArea"})
    payload = record_status_payload(record)

    assert record.goal_reach.has_position is True
    assert record.goal_reach.position_fresh is False
    assert record.goal_reach.distance_cm is None
    assert payload["goal_reach"]["has_position"] is True
    assert payload["goal_reach"]["position_fresh"] is False


def test_missing_rfid_trace_fields_remain_unknown_not_false() -> None:
    raw = stable_trace_row()
    raw["referee"].pop("rfid_status")
    raw["referee"].pop("has_rfid_status_2")
    raw["referee"].pop("rfid_status_2")
    raw["referee"].pop("rfid_match")

    record = normalize_record(raw, 0, {18: "OccupyArea"})
    payload = record_status_payload(record)

    assert record.referee.rfid_status is None
    assert record.referee.has_rfid_status_2 is None
    assert record.referee.rfid_status_2 is None
    assert record.referee.rfid_match.as_payload() == {key: None for key in expected_rfid_match_payload()}
    assert payload["referee"]["rfid_match"]["fresh"] is None
    assert payload["referee"]["rfid_match"]["center_gain_point"] is None


def test_viewer_referee_rows_expose_full_rfid_groups_without_rendering() -> None:
    record = normalize_record(stable_trace_row(), 0, {18: "OccupyArea"})
    viewer = object.__new__(Viewer)

    rows = dict(viewer.referee_rows(record))

    assert rows["RFID raw"] == "fresh=1 any=1 raw=65537 r2=7"
    assert rows["RFID self"] == "base=0 supply=1 high=0 road=0 tunnel=0"
    assert rows["RFID enemy"] == "high=0 road=0 tunnel=1 fortress=1 outpost=0"
    assert rows["RFID zone"] == "center=1 self_side=1 enemy_side=1 fly=0/0"


def test_record_status_payload_is_json_safe_debug_summary() -> None:
    record = normalize_record(stable_trace_row(), 0, {18: "OccupyArea"})

    payload = record_status_payload(record)

    assert payload == {
        "index": 0,
        "tick": 7,
        "event": "tick",
        "t": 12.5,
        "team": "red",
        "strategy": "Regional",
        "aim": "AutoAim",
        "target": "Hero (1)",
        "goal": {"id": 18, "base_id": 18, "name": "OccupyArea", "side": "red", "pos_cm": [1075.0, 898.0]},
        "output": {
            "kind": "goal_pos",
            "topic": "/ly/navi/goal_pos",
            "final_topic": "/ly/navi/goal_pos",
            "frame": "map",
            "publish_enabled": True,
            "publish_allowed": True,
            "speed_level": 1,
            "uses_goal_pos": True,
            "uses_to_navi": False,
            "relative_target_valid": False,
            "chase_official_target_valid": False,
            "chase_official_armor_type": 1,
        },
        "goal_reach": {
            "status": "traveling",
            "status_id": 1,
            "reason": "position_distance",
            "reason_id": 3,
            "goal_id": 18,
            "base_goal_id": 18,
            "goal_age_ms": 1250,
            "distance_cm": 94.0,
            "external_reach_fresh": True,
            "external_reach": False,
            "external_reachable_fresh": True,
            "external_reachable": True,
            "position_fresh": True,
            "has_position": True,
            "timeout": False,
        },
        "navi_status": {
            "should_rotate": True,
            "should_rotate_fresh": True,
            "reached": False,
            "reached_fresh": True,
            "reachable": True,
            "reachable_fresh": True,
        },
        "navi_velocity": {
            "input_x": 8,
            "input_y": -2,
            "output_x": 6,
            "output_y": -1,
            "raw_to_mps": 0.025,
        },
        "relative_target": {
            "valid": False,
            "frame_id": "base_link",
            "x": None,
            "y": None,
            "z": None,
            "distance": None,
            "yaw_error_deg": None,
            "pitch_error_deg": None,
            "armor_type": 1,
            "aim_mode": 1,
            "official_target_valid": False,
            "official_armor_type": 1,
        },
        "intent": {"layer": "RegionalPatrol", "reason": "TargetVisible", "priority": 60},
        "posture": {"command": "Attack (1)", "state": "Attack (1)", "current": "Attack (1)", "desired": "Attack (1)"},
        "referee": {
            "hp": 380,
            "ammo": 42,
            "time_left": 408,
            "rfid_status": 65537,
            "has_rfid_status_2": True,
            "rfid_status_2": 7,
            "rfid_match": expected_rfid_match_payload(),
        },
        "bullet_info": {
            "has_received": True,
            "age_ms": 120,
            "has_initial_speed": True,
            "initial_speed": 23.4,
            "has_shoot_data": True,
            "bullet_type": 1,
            "shooter_number": 7,
            "launching_frequency": 9,
            "has_projectile_allowance": True,
            "projectile_allowance_17mm": 118,
            "projectile_allowance_42mm": 6,
            "remaining_gold_coin": 14,
            "projectile_allowance_fortress_17mm": 32,
        },
        "runtime_guard": {"fault": "None", "recovering": False},
    }


def test_unit_info_records_are_parsed_when_trace_exposes_formal_unit_state() -> None:
    raw = stable_trace_row()
    raw["unit_info"] = {
        "friend": [
            {
                "car_id": 7,
                "hp": 380,
                "has_hp": True,
                "hp_fresh": True,
                "position_x": 1000,
                "position_y": 900,
                "has_position": True,
                "position_fresh": True,
                "position_source": "position_data",
                "area_id": 3,
                "area_name": "my_roadland",
            }
        ],
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
                "position_source": "navi_target_official",
                "area_id": 6,
                "area_name": "enemy_roadland",
            }
        ],
    }

    record = normalize_record(raw, 0, {18: "OccupyArea"})
    payload = record_status_payload(record)

    assert len(record.unit_info) == 2
    enemy = [unit for unit in record.unit_info if unit.side == "enemy"][0]
    assert enemy.car_id == 101
    assert enemy.type_id == 1
    assert enemy.type_name == "Hero"
    assert enemy.hp_fresh
    assert enemy.position_cm == (1165.0, 1063.0)
    assert enemy.position_fresh
    assert enemy.position_source == "navi_target_official"
    assert enemy.area_name == "enemy_roadland"
    assert payload["unit_info"] == {
        "friend": 1,
        "enemy": 1,
        "fresh_friend_positions": ["Sentry"],
        "fresh_enemy_positions": ["Hero"],
    }


def test_viewer_web_status_metadata_exposes_replay_state_without_absolute_paths() -> None:
    record = normalize_record(stable_trace_row(), 0, {18: "OccupyArea"})
    viewer = object.__new__(Viewer)
    viewer.records = [record]
    viewer.current_index = 0
    viewer.trace_path = Path("/tmp/private-workspace/decision_trace.jsonl")
    viewer.follow = False
    viewer.bad_lines = 1
    viewer.playing = False
    viewer.playback_speed = 1.5
    viewer.panel_tab = "decision"
    viewer.match_time_left_sec = 408.0
    viewer.match_running = False
    viewer.match_control_enabled = False
    viewer.control_path = None
    viewer.simulator_inputs_enabled = True
    viewer.last_control_status = "idle"
    from simulator.field import FieldGeometry
    from simulator.interactive_inputs import SimulatorInputState

    viewer.sim_input_state = SimulatorInputState.with_defaults(field=FieldGeometry())
    viewer.sim_input_state.apply_command("set_unit", {"side": "enemy", "type": "Hero", "hp": 120, "x": 2555, "y": 900})
    viewer.goals = {}

    payload = viewer.web_status_metadata()

    assert payload["trace"] == {
        "name": "decision_trace.jsonl",
        "follow": False,
        "records": 1,
        "bad_lines": 1,
        "duration_sec": 0.0,
        "tick_range": {"first": 7, "last": 7},
    }
    assert payload["replay"]["current_record"] == 1
    assert payload["replay"]["total_records"] == 1
    assert payload["current_record"]["goal"]["name"] == "OccupyArea"
    assert payload["simulator_inputs"]["enabled"] is True
    assert payload["simulator_inputs"]["state"]["summary"]["enemy_units"] == 1
    assert payload["simulator_inputs"]["state"]["units"][0]["position_data"] == {
        "car_id": 101,
        "raw_x": 2555,
        "raw_y": 600,
    }
    assert "/tmp/private-workspace" not in json.dumps(payload)


def test_viewer_layer_and_asset_status_rows_are_debuggable_without_rendering() -> None:
    viewer = object.__new__(Viewer)
    config = load_config(None)
    viewer.unit_assets = load_unit_asset_catalog(config.get("assets"))
    viewer.layers = {"terrain": True, "grid": False, "units": True}
    viewer.goal_tags_expanded = False
    viewer.scripted_enabled = False

    asset_rows = dict(viewer.asset_status_rows())
    layer_rows = dict(viewer.layer_summary_rows())

    assert asset_rows["Enabled"] == "1"
    assert asset_rows["Manifest"] == "manifest.yaml"
    assert asset_rows["License"] == "unknown"
    assert "素材.zip" in asset_rows["Source"]
    assert "infantry1->infantry" in asset_rows["Aliases"]
    assert layer_rows["Enabled"] == "terrain, units"
    assert layer_rows["Disabled"] == "grid"
    assert layer_rows["Tags"] == "hover"
    assert Viewer.scaled_velocity_text(6, 0.025) == "0.15"


def test_viewer_panel_scroll_is_clamped_per_tab_without_rendering() -> None:
    viewer = object.__new__(Viewer)
    viewer.panel_tab = "events"
    viewer.panel_scroll = PanelScrollState()
    viewer.panel_scroll.set_content_height("events", 1000)
    viewer.panel_scroll.set_body_height("events", 300)
    viewer.panel_scroll.set_content_height("runtime", 260)
    viewer.panel_scroll.set_body_height("runtime", 400)
    viewer.panel_scroll.set_offset("runtime", 32)

    viewer.scroll_panel(9999)
    assert viewer.panel_scroll.offsets["events"] == 712

    viewer.scroll_panel(-9999)
    assert viewer.panel_scroll.offsets["events"] == 0

    viewer.panel_scroll.set_offset("events", 700)
    viewer.panel_scroll.set_content_height("events", 320)
    viewer.clamp_panel_scroll()
    assert viewer.panel_scroll.offsets["events"] == 32

    viewer.clamp_panel_scroll("runtime")
    assert viewer.panel_scroll.offsets["runtime"] == 0


def test_unknown_bt_internal_fields_do_not_change_stable_contract() -> None:
    raw = stable_trace_row()
    raw["bt_debug"] = {"new_internal_node": {"private_counter": 123}}
    raw["events"]["debug_only_future_flag"] = True
    raw["decision_output"]["debug_detail"] = {"producer": "future_bt"}

    record = normalize_record(raw, 0, {18: "OccupyArea"})
    issues = validate_records([record], {"field_cm": {"width": 2800, "height": 1500}})

    assert not [issue for issue in issues if issue.severity == "error"]
    assert record.output.goal_id == 18
    assert record.events.compact_text() == "RD Outpost=alive Window=open"


def test_new_aim_source_fields_are_optional_for_legacy_traces() -> None:
    raw = stable_trace_row()
    raw["target_state"].pop("external_aim_active")
    raw["target_state"].pop("fresh_current_aim")

    record = normalize_record(raw, 0, {18: "OccupyArea"})
    issues = validate_records([record], {"field_cm": {"width": 2800, "height": 1500}})

    assert not [issue for issue in issues if issue.severity == "error"]
    assert record.target_state.external_aim_active is None
    assert record.target_state.fresh_current_aim is None
    assert "external=-" in record.target_state.fresh_text()
    assert "current=-" in record.target_state.fresh_text()


def test_missing_schema_v2_decision_output_is_contract_error() -> None:
    raw = stable_trace_row()
    raw.pop("decision_output")

    record = normalize_record(raw, 0, {18: "OccupyArea"})
    issues = validate_records([record], {"field_cm": {"width": 2800, "height": 1500}})

    assert any(issue.severity == "error" and "missing decision_output" in issue.message for issue in issues)
