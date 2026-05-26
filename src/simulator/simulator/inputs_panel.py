from __future__ import annotations

import math
from typing import Any

from .trace import as_dict


class InputsPanel:
    def __init__(self, viewer: Any) -> None:
        self.viewer = viewer
        self.structure_buttons: dict[str, tuple[Any, dict[str, Any]]] = {}
        self.unit_palette_buttons: dict[int, Any] = {}
        self.unit_hp_buttons: dict[str, tuple[Any, dict[str, Any]]] = {}

    def clear_buttons(self) -> None:
        self.structure_buttons = {}
        self.unit_palette_buttons = {}
        self.unit_hp_buttons = {}

    def handle_mouse_down(self, pos: tuple[int, int]) -> bool:
        viewer = self.viewer
        state = viewer.sim_input_state
        if not viewer.simulator_inputs_enabled:
            return False
        for _, (rect, payload) in self.unit_hp_buttons.items():
            if rect.collidepoint(pos):
                command = str(payload["command"])
                body = {key: value for key, value in payload.items() if key != "command"}
                viewer.send_sim_command(command, body)
                return True
        for _, (rect, payload) in self.structure_buttons.items():
            if rect.collidepoint(pos):
                viewer.send_sim_command("set_structure_health", dict(payload))
                return True
        for index, rect in self.unit_palette_buttons.items():
            if rect.collidepoint(pos):
                if index < 0 or index >= len(state.unit_palette):
                    return True
                item = state.unit_palette[index]
                viewer.dragging_unit = item
                viewer.drag_position = pos
                viewer.last_control_status = f"drag {item.side}:{item.type_name}"
                return True
        return False

    def draw(self, x: int, y: int, max_width: int, panel: Any) -> None:
        viewer = self.viewer
        self.clear_buttons()
        if not viewer.simulator_inputs_enabled:
            viewer.draw_text("Simulator inputs disabled in config.", x, y, viewer.small_font, viewer.palette["muted"], max_width)
            return

        status = "live control bus" if viewer.controls_available() else "needs --offline-decision --live-view"
        y = viewer.draw_text(f"Mock Inputs: {status}", x, y, viewer.small_font, viewer.palette["muted"], max_width)
        y += 8
        y = self.draw_structure_controls(x, y, max_width)
        y = self.draw_unit_palette(x, y, max_width, panel)
        self.draw_placed_units(x, y, max_width, panel)

    def draw_structure_controls(self, x: int, y: int, max_width: int) -> int:
        viewer = self.viewer
        pg = viewer.pg
        state = viewer.sim_input_state
        viewer.draw_text("Structures", x, y, viewer.font, viewer.palette["accent"], max_width)
        y += 24
        button_w = 42
        button_h = 23
        gap = 5
        label_w = max_width - (button_w * 4 + gap * 3) - 8
        for item in state.structures:
            hp = state.structure_health.get(item.key, item.hp)
            label = f"{item.label} {hp}/{item.max_hp}"
            viewer.draw_text(label, x, y + 3, viewer.small_font, viewer.palette["text"], label_w)
            actions = [
                ("0", 0),
                (f"-{item.step}", max(0, hp - item.step)),
                (f"+{item.step}", min(item.max_hp, hp + item.step)),
                ("Max", item.max_hp),
            ]
            bx = x + label_w + 8
            for index, (button_label, next_hp) in enumerate(actions):
                rect = pg.Rect(bx + index * (button_w + gap), y, button_w, button_h)
                viewer.draw_control_button(rect, button_label)
                self.structure_buttons[f"{item.key}:{button_label}"] = (
                    rect,
                    {
                        "side": item.side,
                        "structure": item.structure,
                        "hp": int(next_hp),
                    },
                )
            y += button_h + 6
        y += 4
        pg.draw.line(viewer.screen, viewer.palette["line"], (x, y), (x + max_width, y), 1)
        return y + 8

    def draw_unit_palette(self, x: int, y: int, max_width: int, panel: Any) -> int:
        viewer = self.viewer
        pg = viewer.pg
        state = viewer.sim_input_state
        viewer.draw_text("Drag Unit Pieces", x, y, viewer.font, viewer.palette["accent"], max_width)
        y += 24
        gap = 6
        columns = 2
        chip_w = (max_width - gap * (columns - 1)) // columns
        chip_h = 25
        for index, item in enumerate(state.unit_palette):
            row = index // columns
            col = index % columns
            rect = pg.Rect(x + col * (chip_w + gap), y + row * (chip_h + gap), chip_w, chip_h)
            self.unit_palette_buttons[index] = rect
            label = f"{item.side[:1].upper()} {item.type_name} {item.hp}"
            self.draw_unit_chip(rect, label, item.side)
        rows = math.ceil(len(state.unit_palette) / columns)
        y += rows * (chip_h + gap)
        y += 6
        if y < panel.bottom - 36:
            pg.draw.line(viewer.screen, viewer.palette["line"], (x, y), (x + max_width, y), 1)
            return y + 8
        return y

    def draw_placed_units(self, x: int, y: int, max_width: int, panel: Any) -> None:
        viewer = self.viewer
        pg = viewer.pg
        state = viewer.sim_input_state
        viewer.draw_text("Placed Pieces", x, y, viewer.font, viewer.palette["accent"], max_width)
        y += 24
        if not state.units:
            viewer.draw_text("Drag a piece onto the field map.", x, y, viewer.small_font, viewer.palette["muted"], max_width)
            return
        button_w = 42
        button_h = 22
        gap = 5
        label_w = max_width - button_w * 3 - gap * 2 - 8
        units = sorted(state.units.values(), key=lambda unit: (unit.side, unit.type_id))
        for unit in units:
            if y > panel.bottom - button_h - 8:
                viewer.draw_text("...", x, y, viewer.small_font, viewer.palette["muted"], max_width)
                break
            pos = f"{unit.x},{unit.y}"
            text = f"{unit.side[:1].upper()} {unit.type_name} {unit.hp}/{unit.max_hp} @{pos}"
            viewer.draw_text(text, x, y + 2, viewer.small_font, viewer.palette["text"], label_w)
            step = 50 if unit.max_hp > 100 else 10
            actions = [
                (f"-{step}", "set_unit_hp", max(1, unit.hp - step)),
                (f"+{step}", "set_unit_hp", min(unit.max_hp, unit.hp + step)),
                ("X", "remove_unit", unit.hp),
            ]
            bx = x + label_w + 8
            for index, (label, command, next_hp) in enumerate(actions):
                rect = pg.Rect(bx + index * (button_w + gap), y, button_w, button_h)
                viewer.draw_control_button(rect, label)
                payload = {
                    "command": command,
                    "side": unit.side,
                    "type_id": unit.type_id,
                    "type": unit.type_name,
                    "hp": int(next_hp),
                }
                self.unit_hp_buttons[f"{unit.side}:{unit.type_id}:{label}"] = (rect, payload)
            y += button_h + 6

    def draw_unit_chip(self, rect: Any, label: str, side: str) -> None:
        viewer = self.viewer
        pg = viewer.pg
        side_style = as_dict(viewer.unit_styles.get(side, {}))
        fill = pg.Color(side_style.get("color", viewer.colors_raw.get(side, "#4fb3d9")))
        pg.draw.rect(viewer.screen, viewer.palette["panel2"], rect, border_radius=5)
        pg.draw.rect(viewer.screen, fill, rect, 2, border_radius=5)
        text = viewer.small_font.render(label, True, viewer.palette["text"])
        viewer.screen.blit(text, text.get_rect(center=rect.center))
