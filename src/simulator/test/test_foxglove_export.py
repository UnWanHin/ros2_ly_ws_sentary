from __future__ import annotations

import json
from pathlib import Path

import pytest

from simulator.foxglove_export import (
    export_records_to_mcap,
    export_records_with_writer,
    record_to_decision_frame,
    record_to_goal_point,
    record_to_metrics,
)
from simulator.trace import normalize_record

from test_control_output_trace import v3_row
from test_trace_contract import expected_rfid_match_payload, stable_trace_row


class RecordingWriter:
    def __init__(self) -> None:
        self.schemas: list[dict] = []
        self.channels: dict[int, str] = {}
        self.messages: list[tuple[int, bytes]] = []

    def register_schema(self, **kwargs: object) -> int:
        self.schemas.append(dict(kwargs))
        return len(self.schemas)

    def register_channel(self, **kwargs: object) -> int:
        channel_id = len(self.channels) + 1
        self.channels[channel_id] = str(kwargs["topic"])
        return channel_id

    def add_message(self, **kwargs: object) -> None:
        self.messages.append((int(kwargs["channel_id"]), bytes(kwargs["data"])))


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


def test_export_registers_final_control_tactical_and_scene_channels() -> None:
    raw = v3_row(feedback_rotate=1, final_rotate=0)
    raw["schema_version"] = 4
    raw["tactical"] = {
        "available": True,
        "protect_castle": {
            "enabled": True,
            "rfid_enabled": True,
            "enemy_pos_enabled": True,
            "rfid_event_raw_active": False,
            "rfid_event_active": False,
            "enemy_pos_active": True,
        },
        "protect_hero": {"enabled": True, "active": False},
        "regional_defense": {
            "threat_active": True,
            "search_kind": "own_base",
            "fortress_enemy_count": 0,
            "own_base_enemy_count": 1,
        },
    }
    record = normalize_record(raw, 0, {18: "OccupyArea"})
    writer = RecordingWriter()

    export_records_with_writer(writer, [record], topic_prefix="/sentry/simulator")

    assert set(writer.channels.values()) == {
        "/sentry/simulator/decision",
        "/sentry/simulator/metrics",
        "/sentry/simulator/goal_point_cm",
        "/sentry/simulator/decision/control_output",
        "/sentry/simulator/decision/tactical",
        "/sentry/simulator/scene",
    }
    payloads = {
        writer.channels[channel_id]: json.loads(data.decode("utf-8"))
        for channel_id, data in writer.messages
    }
    assert payloads["/sentry/simulator/decision/control_output"]["fire_code"]["rotate"] == 0
    assert payloads["/sentry/simulator/decision/tactical"]["protect_castle"]["enemy_pos_active"] is True
    assert payloads["/sentry/simulator/scene"]["units"][0]["type"] == "Sentry"


def test_mcap_export_contains_each_tactical_evidence_channel(tmp_path: Path) -> None:
    pytest.importorskip("mcap")
    from mcap.reader import make_reader

    raw = v3_row(feedback_rotate=2, final_rotate=0)
    raw["schema_version"] = 4
    raw["tactical"] = {
        "available": True,
        "protect_castle": {
            "enabled": True,
            "rfid_enabled": True,
            "enemy_pos_enabled": True,
            "rfid_event_raw_active": False,
            "rfid_event_active": False,
            "enemy_pos_active": True,
        },
        "protect_hero": {"enabled": True, "active": False},
        "regional_defense": {
            "threat_active": True,
            "search_kind": "own_base",
            "fortress_enemy_count": 0,
            "own_base_enemy_count": 1,
        },
    }
    record = normalize_record(raw, 0, {18: "OccupyArea"})
    output = tmp_path / "tactical.mcap"

    export_records_to_mcap([record], output)

    with output.open("rb") as stream:
        channels = {channel.topic for _, channel, _ in make_reader(stream).iter_messages()}
    assert {
        "/sentry/simulator/decision/control_output",
        "/sentry/simulator/decision/tactical",
        "/sentry/simulator/scene",
    } <= channels
