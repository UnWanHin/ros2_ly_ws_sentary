from simulator.control_bus import normalize_api_control_payload
from simulator.field import FieldGeometry, relative_side_to_field_side
from simulator.interactive_inputs import SimulatorInputState


def test_field_geometry_converts_position_data_y() -> None:
    field = FieldGeometry()

    assert field.position_data_raw_y(900) == 600
    assert field.position_data_official_y(600) == 900
    assert field.clamp_point((-10, 1800)) == (0.0, 1500.0)


def test_relative_side_maps_to_absolute_team_side() -> None:
    assert relative_side_to_field_side("friend", "red") == "red"
    assert relative_side_to_field_side("enemy", "red") == "blue"
    assert relative_side_to_field_side("friend", "blue") == "blue"
    assert relative_side_to_field_side("enemy", "blue") == "red"


def test_structure_health_accepts_zero_and_uses_base_points() -> None:
    state = SimulatorInputState.with_defaults()

    assert state.apply_command("set_structure_health", {"side": "enemy", "structure": "outpost", "hp": 0})
    assert state.structure_hp("enemy", "outpost") == 0

    friend_base = state.find_structure("friend", "base")
    enemy_base = state.find_structure("enemy", "base")

    assert friend_base is not None
    assert enemy_base is not None
    assert state.structure_position(friend_base, "red", {}) == (245.0, 750.0)
    assert state.structure_position(enemy_base, "red", {}) == (2555.0, 750.0)


def test_unit_commands_feed_health_and_position_rows() -> None:
    state = SimulatorInputState.with_defaults(field=FieldGeometry())

    assert state.apply_command(
        "set_unit",
        {
            "side": "enemy",
            "type": "Hero",
            "hp": 123,
            "max_hp": 200,
            "x": 2555,
            "y": 900,
        },
    )

    health = state.health_fields("enemy", 400)
    assert health["hero"] == 123
    assert health["engineer"] == 400

    rows = state.position_rows()
    assert len(rows) == 1
    assert rows[0].enemy_car_id == 101
    assert rows[0].raw_x == 2555
    assert rows[0].raw_y == 600

    assert state.apply_command("set_unit_hp", {"side": "enemy", "type_id": 1, "hp": 0})
    assert state.health_fields("enemy", 400)["hero"] == 0


def test_control_bus_accepts_simulator_input_commands() -> None:
    command, payload, error = normalize_api_control_payload(
        {"command": "set_unit", "side": "friend", "type_id": 7, "x": 300, "y": 400},
        default_step_sec=10,
    )

    assert error is None
    assert command == "set_unit"
    assert payload == {"side": "friend", "type_id": 7, "x": 300, "y": 400}
