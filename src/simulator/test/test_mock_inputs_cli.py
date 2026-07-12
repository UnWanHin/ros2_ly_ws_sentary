from __future__ import annotations

import pytest

from simulator.field import FieldGeometry
from simulator.mock_inputs import (
    clamp_u8,
    clamp_u16,
    clamp_u32,
    default_self_position,
    default_uwb_position,
    official_bt_point,
    parse_args,
    payload_position_cm,
    uwb_raw_point,
)


def test_parse_args_accepts_decision_context_knobs() -> None:
    args = parse_args(
        [
            "--team",
            "blue",
            "--team-buff-attack",
            "2",
            "--team-buff-defence",
            "1",
            "--team-buff-remaining-energy",
            "37",
            "--event-self-small-energy-status",
            "2",
            "--event-self-base-gain-point-status",
            "true",
            "--sentry-can-activate-energy",
            "true",
            "--rfid-center-gain-point",
            "true",
            "--rfid-self-outpost",
            "true",
            "--rfid-enemy-tunnel",
            "true",
            "--navi-reached",
            "true",
            "--navi-reachable",
            "false",
            "--navi-should-rotate",
            "false",
            "--self-position-x",
            "3000",
            "--self-position-y",
            "-5",
            "--publish-uwb-position",
            "true",
            "--uwb-position-x",
            "1220",
            "--uwb-position-y",
            "760",
            "--official-target-valid",
            "true",
            "--official-target-x",
            "1500",
            "--official-target-y",
            "900",
            "--official-target-armor-type",
            "6",
            "--gimbal-fire-status",
            "1",
            "--gimbal-cap-state",
            "2",
            "--gimbal-follow-mode",
            "true",
            "--gimbal-aim-mode",
            "true",
            "--gimbal-rotate",
            "3",
            "--gimbal-yaw-velocity",
            "14.5",
            "--gimbal-yaw-angle",
            "27.5",
            "--mock-cap-v",
            "22",
            "--mock-navi-lower-head",
            "1",
            "--mock-navi-vel-x",
            "0.35",
            "--mock-navi-vel-y",
            "-0.2",
            "--mock-bullet-initial-speed",
            "23.4",
            "--mock-bullet-has-shoot-data",
            "true",
            "--mock-bullet-type",
            "1",
            "--mock-bullet-shooter-number",
            "7",
            "--mock-bullet-launching-frequency",
            "8",
            "--mock-bullet-projectile-allowance-17mm",
            "120",
            "--mock-external-aim",
            "true",
            "--mock-external-aim-follow",
            "true",
            "--mock-external-aim-fire",
            "false",
            "--mock-external-aim-yaw",
            "7.5",
            "--mock-external-aim-pitch",
            "-1.2",
            "--mock-external-aim-target-id",
            "6",
            "--mock-external-aim-target-x",
            "4.2",
            "--mock-external-aim-target-y",
            "1.1",
            "--mock-external-aim-target-z",
            "0.3",
            "--mock-external-aim-frame",
            "gimbal_world",
        ]
    )

    assert args.team == "blue"
    assert args.team_buff_attack == 2
    assert args.team_buff_defence == 1
    assert args.team_buff_remaining_energy == 37
    assert args.event_self_small_energy_status == 2
    assert args.event_self_base_gain_point_status is True
    assert args.sentry_can_activate_energy is True
    assert args.rfid_center_gain_point is True
    assert args.rfid_self_outpost is True
    assert args.rfid_enemy_tunnel is True
    assert args.navi_reached is True
    assert args.navi_reachable is False
    assert args.navi_should_rotate is False
    assert args.self_position_x == 3000
    assert args.self_position_y == -5
    assert args.publish_uwb_position is True
    assert args.uwb_position_x == 1220
    assert args.uwb_position_y == 760
    assert args.official_target_valid is True
    assert args.official_target_armor_type == 6
    assert args.gimbal_fire_status == 1
    assert args.gimbal_cap_state == 2
    assert args.gimbal_follow_mode is True
    assert args.gimbal_aim_mode is True
    assert args.gimbal_rotate == 3
    assert args.gimbal_yaw_velocity == 14.5
    assert args.gimbal_yaw_angle == 27.5
    assert args.mock_cap_v == 22
    assert args.mock_navi_lower_head == 1
    assert args.mock_navi_vel_x == 0.35
    assert args.mock_navi_vel_y == -0.2
    assert args.mock_bullet_initial_speed == 23.4
    assert args.mock_bullet_has_shoot_data is True
    assert args.mock_bullet_type == 1
    assert args.mock_bullet_shooter_number == 7
    assert args.mock_bullet_launching_frequency == 8
    assert args.mock_bullet_projectile_allowance_17mm == 120
    assert args.mock_external_aim is True
    assert args.mock_external_aim_follow is True
    assert args.mock_external_aim_fire is False
    assert args.mock_external_aim_yaw == 7.5
    assert args.mock_external_aim_pitch == -1.2
    assert args.mock_external_aim_target_id == 6
    assert args.mock_external_aim_target_x == 4.2
    assert args.mock_external_aim_target_y == 1.1
    assert args.mock_external_aim_target_z == 0.3
    assert args.mock_external_aim_frame == "gimbal_world"


def test_parse_args_rejects_bad_bool() -> None:
    with pytest.raises(SystemExit):
        parse_args(["--navi-reachable", "maybe"])


def test_default_self_position_uses_team_base_or_clamped_override() -> None:
    field = FieldGeometry()

    assert default_self_position("red", field, -1, -1) == (245, 750)
    assert default_self_position("blue", field, -1, -1) == (2555, 750)
    assert default_self_position("red", field, 3000, -5) == (245, 750)
    assert default_self_position("red", field, 3000, 1600) == (2800, 1500)


def test_default_uwb_position_follows_self_position_unless_axis_is_overridden() -> None:
    field = FieldGeometry()
    self_position = (245, 750)

    assert default_uwb_position(field, self_position, -1, -1) == (245, 750)
    assert default_uwb_position(field, self_position, 1220, -1) == (1220, 750)
    assert default_uwb_position(field, self_position, -1, 760) == (245, 760)
    assert default_uwb_position(field, self_position, 3000, 0) == (2800, 1)


def test_uwb_raw_point_inverts_official_y_for_behavior_tree_subscriber() -> None:
    field = FieldGeometry()

    assert uwb_raw_point(field, 1220, 760) == (1220, 740)
    assert uwb_raw_point(field, 3000, 1600) == (2800, 0)
    assert uwb_raw_point(field, 0, 0) == (1, 1499)


def test_official_bt_point_never_publishes_zero_coordinates() -> None:
    field = FieldGeometry()

    assert official_bt_point(field, 0, 0) == (1, 1)
    assert official_bt_point(field, -20, -30) == (1, 1)
    assert official_bt_point(field, 3000, 1600) == (2800, 1500)


def test_payload_position_cm_accepts_supported_shapes_and_rejects_invalid_values() -> None:
    field = FieldGeometry()

    assert payload_position_cm({"x": 820, "y": 830}, field) == (820, 830)
    assert payload_position_cm({"position_cm": {"x": 3000, "y": -5}}, field) == (2800, 1)
    assert payload_position_cm({"position": [12.3, 45.6]}, field) == (12, 46)
    assert payload_position_cm({"pos": (0, 0)}, field) == (1, 1)

    assert payload_position_cm({"x": None, "y": 500}, field) is None
    assert payload_position_cm({"position": {"x": "nan", "y": 500}}, field) is None
    assert payload_position_cm({"position": {"x": 100, "y": "inf"}}, field) is None
    assert payload_position_cm({}, field) is None


def test_numeric_clamps_match_ros_message_widths() -> None:
    assert clamp_u8(-1) == 0
    assert clamp_u8(300) == 255
    assert clamp_u16(-1) == 0
    assert clamp_u16(70000) == 65535
    assert clamp_u32(-1) == 0
    assert clamp_u32(0x1FFFFFFFF) == 0xFFFFFFFF
