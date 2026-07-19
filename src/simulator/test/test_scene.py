from __future__ import annotations

from simulator.scene import SceneCommand, SceneState, normalize_scene_command
from simulator.tactical_catalog import SceneCatalog, default_catalog_path


def load_catalog() -> SceneCatalog:
    return SceneCatalog.load(default_catalog_path())


def test_scene_keeps_two_placed_hero_instances_and_selects_one() -> None:
    state = SceneState.from_catalog(load_catalog(), "red")

    first = state.apply(SceneCommand.place_unit("enemy:hero:a", "enemy", "hero", 1200, 700))
    second = state.apply(SceneCommand.place_unit("enemy:hero:b", "enemy", "hero", 1300, 700))

    assert first.changed and second.changed
    assert len(state.units) == 2
    assert state.snapshot()["selected_entity_id"] == "enemy:hero:b"


def test_manual_mode_rejects_mutating_scene_commands() -> None:
    state = SceneState.from_catalog(load_catalog(), "red", ownership_mode="manual_ros")

    result = state.apply(SceneCommand.set_structure_hp("friend:base", 3000))

    assert not result.changed
    assert result.reason == "manual_ros_observer_mode"


def test_scene_projects_catalog_health_and_position_data() -> None:
    state = SceneState.from_catalog(load_catalog(), "red")
    state.apply(SceneCommand.place_unit("enemy:hero:a", "enemy", "hero", 1200, 700, hp=123))
    state.apply(SceneCommand.set_structure_hp("friend:base", 3000))

    projection = state.project_ros_inputs()

    assert projection.health["enemy"]["hero"] == 123
    assert projection.positions["enemy"][101] == (1200, 800)
    assert projection.structures["friend"]["base"] == 3000
    assert projection.conflicts == ()


def test_mock_projection_uses_catalog_health_and_relative_position_encoding() -> None:
    state = SceneState.from_catalog(load_catalog(), "red", ownership_mode="mock")
    state.apply(SceneCommand.place_unit("enemy:sentry:a", "enemy", "sentry", 1200, 700, hp=300))

    projection = state.project_ros_inputs()

    assert projection.health["enemy"]["sentry"] == 300
    assert projection.positions["enemy"][107] == (1200, 800)
    assert projection.conflicts == ()


def test_scene_reports_same_formal_unit_conflicts_without_overwriting() -> None:
    state = SceneState.from_catalog(load_catalog(), "red")
    state.apply(SceneCommand.place_unit("enemy:hero:a", "enemy", "hero", 1200, 700, hp=123))
    state.apply(SceneCommand.place_unit("enemy:hero:b", "enemy", "hero", 1300, 750, hp=111))

    projection = state.project_ros_inputs()

    assert projection.health["enemy"] == {}
    assert projection.positions["enemy"] == {}
    assert {(item.kind, item.side, item.formal_key) for item in projection.conflicts} == {
        ("health", "enemy", "hero"),
        ("position", "enemy", 101),
    }
    assert all(item.entity_ids == ("enemy:hero:a", "enemy:hero:b") for item in projection.conflicts)


def test_normalize_canonical_place_unit_preserves_its_stable_entity_id() -> None:
    catalog = load_catalog()

    command = normalize_scene_command(
        {
            "command": "place_unit",
            "entity_id": "enemy:hero:alpha",
            "side": "enemy",
            "unit_key": "hero",
            "x": 1200,
            "y": 700,
            "hp": 123,
        },
        catalog,
    )

    assert command.command == "place_unit"
    assert command.entity_id == "enemy:hero:alpha"
    assert command.unit_key == "hero"
    assert command.hp == 123


def test_scene_clamps_position_at_the_catalog_field_boundary() -> None:
    state = SceneState.from_catalog(load_catalog(), "red")

    state.apply(SceneCommand.place_unit("friend:drone:a", "friend", "drone", -10, 1800))

    unit = state.snapshot()["units"][0]
    assert unit["position_cm"] == {"x": 0, "y": 1500}
