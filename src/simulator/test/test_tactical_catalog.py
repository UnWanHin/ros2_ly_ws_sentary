from __future__ import annotations

from copy import deepcopy
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest

from simulator.tactical_catalog import (
    GoalMarker,
    SceneCatalog,
    StructureArchetype,
    UnitArchetype,
    default_catalog_path,
    validate_catalog_payload,
)


def valid_catalog_payload() -> dict:
    return {
        "schema": "ly_simulator_tactical_catalog_v1",
        "field": {
            "width_cm": 2800,
            "height_cm": 1500,
            "frame": "official_map_cm",
        },
        "visual": {
            "asset_manifest": "assets/manifest.yaml",
            "unit_scale_px": 42,
            "trace_unit_scale_px": 34,
            "drag_unit_scale_px": 46,
            "health_bar_height_px": 5,
            "label_rule": "compact_role",
            "team_colors": {"red": "#e85d5d", "blue": "#5d8ee8"},
        },
        "ros_projection": {
            "position_data": {
                "friend_car_id_offset": 0,
                "enemy_car_id_offset": 100,
            }
        },
        "units": [
            {
                "key": "hero",
                "label": "Hero",
                "asset_key": "Hero",
                "default_hp": 200,
                "max_hp": 200,
                "health_field": "hero",
                "position_car_id": 1,
                "project_to_ros": True,
                "decision_consumed": True,
            },
            {
                "key": "drone",
                "label": "Drone",
                "asset_key": "Drone",
                "default_hp": 150,
                "max_hp": 150,
                "health_field": None,
                "position_car_id": 6,
                "project_to_ros": True,
                "decision_consumed": False,
            },
        ],
        "structures": [
            {
                "key": "friend_base",
                "label": "Friend Base",
                "side": "friend",
                "kind": "base",
                "default_hp": 5000,
                "max_hp": 5000,
                "step": 500,
                "health_topic": "/ly/friend/base_hp",
                "positions_cm": {"red": [245, 750], "blue": [2555, 750]},
            }
        ],
        "goals": [
            {
                "id": 1,
                "key": "base",
                "label": "Base",
                "category": "base",
                "marker_style": "objective",
                "positions_cm": {"red": [245, 750], "blue": [2555, 750]},
            }
        ],
    }


def duplicate_position_catalog() -> dict:
    raw = valid_catalog_payload()
    raw["units"].append(
        {
            "key": "engineer",
            "label": "Engineer",
            "asset_key": "Engineer",
            "default_hp": 250,
            "max_hp": 250,
            "health_field": "engineer",
            "position_car_id": 1,
            "project_to_ros": True,
            "decision_consumed": True,
        }
    )
    return raw


def test_catalog_has_single_formal_mapping_per_projectable_unit(tmp_path: Path) -> None:
    catalog = SceneCatalog.load(default_catalog_path())

    assert catalog.field.frame == "left_bottom_origin_cm"
    assert catalog.unit_by_key("hero").health_field == "hero"
    assert catalog.unit_by_key("hero").position_car_id == 1
    assert catalog.unit_by_key("drone").project_to_ros is True
    assert [unit.key for unit in catalog.units] == [
        "hero",
        "engineer",
        "infantry1",
        "infantry2",
        "infantry3",
        "drone",
        "sentry",
    ]


def test_catalog_preserves_drone_position_projection_without_health_or_bt_consumption() -> None:
    catalog = SceneCatalog.load(default_catalog_path())
    drone = catalog.unit_by_key("drone")

    assert drone.health_published is False
    assert drone.position_published is True
    assert drone.decision_consumed is False
    assert catalog.position_car_id_for_side("drone", "friend") == 6
    assert catalog.position_car_id_for_side("drone", "enemy") == 106


def test_catalog_captures_existing_structure_and_goal_positions() -> None:
    catalog = SceneCatalog.load(default_catalog_path())

    friend_base = catalog.structure_by_key("friend_base")
    assert friend_base.health_topic == "/ly/friend/base_hp"
    assert friend_base.position_for_field_side("red") == (245.0, 755.0)
    assert friend_base.position_for_field_side("blue") == (2555.0, 745.0)

    castle = catalog.goal_by_id(6)
    assert castle.key == "castle"
    assert castle.position_for_field_side("red") == (666.0, 749.0)
    assert castle.position_for_field_side("blue") == (2132.0, 749.0)
    assert len(catalog.goals) == 31
    assert catalog.goal_by_id(26).label == "CentralLeft.A"
    assert catalog.goal_by_id(27).label == "CentralLeft.B"
    assert catalog.goal_by_id(29).position_for_field_side("red") == (1000.0, 1007.0)
    assert catalog.goal_by_id(29).position_for_field_side("blue") == (1800.0, 493.0)
    assert catalog.goal_by_id(30).position_for_field_side("red") == (989.0, 496.0)
    assert catalog.goal_by_id(30).position_for_field_side("blue") == (1811.0, 1004.0)


def test_catalog_models_are_immutable() -> None:
    catalog = SceneCatalog.load(default_catalog_path())

    assert isinstance(catalog.unit_by_key("hero"), UnitArchetype)
    assert isinstance(catalog.structure_by_key("friend_base"), StructureArchetype)
    assert isinstance(catalog.goal_by_id(1), GoalMarker)
    with pytest.raises(FrozenInstanceError):
        catalog.unit_by_key("hero").label = "Changed"  # type: ignore[misc]


def test_catalog_rejects_duplicate_formal_position_ids(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="position_car_id"):
        SceneCatalog.from_mapping(duplicate_position_catalog())


def test_catalog_rejects_wrong_position_data_enemy_offset() -> None:
    raw = valid_catalog_payload()
    raw["ros_projection"]["position_data"]["enemy_car_id_offset"] = 99

    with pytest.raises(ValueError, match="enemy_car_id_offset"):
        SceneCatalog.from_mapping(raw)


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda raw: raw["units"].__setitem__(1, deepcopy(raw["units"][0])), "duplicate unit key"),
        (lambda raw: raw["units"][0].__setitem__("max_hp", 0), "max_hp"),
        (lambda raw: raw["goals"][0]["positions_cm"].__setitem__("red", ["bad", 750]), "coordinate"),
        (
            lambda raw: raw["units"][0].update({"health_field": None, "position_car_id": None}),
            "formal mapping",
        ),
    ],
)
def test_catalog_rejects_invalid_boundary_data(mutate, message: str) -> None:
    raw = valid_catalog_payload()
    mutate(raw)

    with pytest.raises(ValueError, match=message):
        validate_catalog_payload(raw)
