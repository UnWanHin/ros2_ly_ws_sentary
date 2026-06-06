from __future__ import annotations

from simulator.panel_scroll import PanelScrollState, nonnegative_int


def test_panel_scroll_state_clamps_offsets_per_tab() -> None:
    scroll = PanelScrollState()
    scroll.set_body_height("events", 300)
    scroll.set_content_height("events", 1000)
    scroll.set_body_height("runtime", 400)
    scroll.set_content_height("runtime", 260)
    scroll.set_offset("runtime", 32)

    assert scroll.scroll("events", 9999) == 712
    assert scroll.scroll("events", -9999) == 0

    scroll.set_offset("events", 700)
    scroll.set_content_height("events", 320)
    assert scroll.clamp("events") == 32
    assert scroll.clamp("runtime") == 0


def test_panel_scroll_state_handles_missing_and_invalid_values() -> None:
    scroll = PanelScrollState(bottom_padding=-5)

    assert nonnegative_int("bad", 9) == 9
    assert nonnegative_int("-4") == 0
    assert scroll.max_scroll("missing") == 0

    scroll.set_body_height("events", "bad")
    scroll.set_content_height("events", 100)
    assert scroll.max_scroll("events") == 100

    assert scroll.set_offset("events", "bad") == 0
    assert scroll.scroll("events", "bad") == 0
