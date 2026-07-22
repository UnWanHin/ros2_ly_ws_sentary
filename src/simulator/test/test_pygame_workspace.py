from __future__ import annotations

import pygame

from simulator.config import load_config
from simulator.interactive_inputs import SimulatorInputState
from simulator.viewer import Viewer
from simulator.workspace import Selection


def test_viewer_workspace_selection_switches_between_overview_unit_and_structure() -> None:
    viewer = object.__new__(Viewer)
    viewer.sim_input_state = SimulatorInputState.with_defaults()
    assert viewer.sim_input_state.apply_command(
        "set_unit",
        {
            "entity_id": "enemy:hero:one",
            "side": "enemy",
            "type": "Hero",
            "x": 1800,
            "y": 900,
        },
    )
    viewer.workspace_selection = Selection()
    viewer.selected_structure_key = None

    viewer.select_overview()
    assert viewer.workspace_selection == Selection("overview", "")

    viewer.select_unit("enemy:hero:one")
    assert viewer.workspace_selection == Selection("unit", "enemy:hero:one")
    assert viewer.sim_input_state.scene.selected_entity_id == "enemy:hero:one"

    viewer.select_structure("friend_base")
    assert viewer.workspace_selection == Selection("structure", "friend_base")
    assert viewer.selected_structure_key == "friend_base"
    assert viewer.sim_input_state.scene.selected_entity_id is None


def test_viewer_selection_rejects_unknown_scene_items() -> None:
    viewer = object.__new__(Viewer)
    viewer.sim_input_state = SimulatorInputState.with_defaults()
    viewer.workspace_selection = Selection()
    viewer.selected_structure_key = None

    viewer.select_unit("unknown")
    viewer.select_structure("unknown")

    assert viewer.workspace_selection == Selection("overview", "")


def test_viewer_workspace_layout_keeps_the_battlefield_primary() -> None:
    viewer = object.__new__(Viewer)
    viewer.width = 1500
    viewer.height = 900
    viewer.panel_w = 390
    viewer.timeline_h = 92
    viewer.inspector_width = 320
    viewer.inspector_zone = "right"
    viewer.inspector_collapsed = False
    viewer.shelf_height = 180
    viewer.shelf_collapsed = True

    layout = viewer.workspace_layout()

    assert layout.inspector.zone == "right"
    assert layout.viewport.width / layout.content_width >= 0.75
    assert layout.operations_shelf.collapsed is True


def test_default_pygame_palette_uses_flight_deck_tokens() -> None:
    colors = load_config(None)["colors"]

    assert colors["bg"] == "#111418"
    assert colors["panel"] == "#1D232B"
    assert colors["panel2"] == "#202832"
    assert colors["text"] == "#F5F7FA"
    assert colors["muted"] == "#AEB7C2"
    assert colors["accent"] == "#4DB7FF"


def test_viewer_activity_rail_uses_compact_symbol_and_title_pairs() -> None:
    assert Viewer.activity_rail_items() == (
        ("decision", "◎", "Decision"),
        ("events", "◌", "Events"),
        ("runtime", "⌁", "Runtime"),
        ("control", "⌘", "Control"),
        ("inputs", "◈", "Inputs"),
        ("layers", "▤", "Layers"),
    )


def test_area_selection_only_changes_the_contextual_inspector() -> None:
    viewer = object.__new__(Viewer)
    viewer.sim_input_state = SimulatorInputState.with_defaults()
    viewer.workspace_selection = Selection()
    viewer.selected_structure_key = None
    viewer.config = {
        "terrain": {"zones": [{"name": "High Ground", "rect": [100, 100, 200, 200]}]},
        "structures": {"items": []},
    }

    viewer.select_area("High Ground")

    assert viewer.workspace_selection == Selection("area", "High Ground")
    assert viewer.sim_input_state.scene.selected_entity_id is None


def test_contextual_unit_inspector_preserves_the_existing_unit_hp_command() -> None:
    viewer = object.__new__(Viewer)
    calls: list[tuple[str, dict]] = []
    viewer.inspector_buttons = {
        "enemy:hero:one:destroy": (
            pygame.Rect(10, 10, 50, 20),
            {"_command": "set_unit_hp", "entity_id": "enemy:hero:one", "hp": 0},
        )
    }
    viewer.send_sim_command = lambda command, payload: calls.append((command, payload))

    assert viewer.handle_inspector_mouse_down((20, 15))
    assert calls == [("set_unit_hp", {"entity_id": "enemy:hero:one", "hp": 0})]
