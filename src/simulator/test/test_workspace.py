from __future__ import annotations

import pytest

from simulator.workspace import Rect, ViewportState, WorkspaceLayout


def test_zoom_keeps_pointer_field_coordinate_fixed() -> None:
    state = ViewportState.fit(
        field_width=2800,
        field_height=1500,
        viewport=Rect(0, 0, 1120, 700),
    )

    before = state.screen_to_field(725, 340)
    state.zoom_at(725, 340, factor=1.2)
    after = state.screen_to_field(725, 340)

    assert after == pytest.approx(before)


def test_workspace_layout_keeps_map_primary_with_inspector_open() -> None:
    layout = WorkspaceLayout.desktop(1440, 900, inspector_width=280, shelf_collapsed=True)

    assert layout.viewport.width / layout.content_width >= 0.76
    assert layout.inspector.zone == "right"
    assert layout.activity_rail.width == 56


def test_zoom_to_selection_centers_selection_inside_viewport() -> None:
    state = ViewportState.fit(
        field_width=2800,
        field_height=1500,
        viewport=Rect(0, 0, 1000, 700),
    )
    state.zoom_to_field_bounds(Rect(1880, 780, 160, 160))

    center = state.screen_to_field(500, 350)

    assert center == pytest.approx((1960, 860), abs=2)


def test_pan_moves_the_world_with_the_pointer() -> None:
    state = ViewportState.fit(
        field_width=2800,
        field_height=1500,
        viewport=Rect(0, 0, 1000, 700),
    )
    before = state.field_to_screen(1400, 750)
    state.pan_by(75, -30)
    after = state.field_to_screen(1400, 750)

    assert after == pytest.approx((before[0] + 75, before[1] - 30))
