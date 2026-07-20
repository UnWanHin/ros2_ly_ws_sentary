from __future__ import annotations

import math
from typing import Any

from .interactive_inputs import unit_decision_summary
from .trace import as_dict


def compact_decision_summary(side: str, type_id: int) -> str:
    summary = unit_decision_summary(side, type_id)
    replacements = {
        "BT:HP,POS,UI": "BT HP/POS/UI",
        "PUB:HP,POS noUI": "PUB HP/POS",
        "PUB:POS noUI": "PUB POS",
        "noUI": "no UI",
    }
    for source, target in replacements.items():
        summary = summary.replace(source, target)
    return summary


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

    def draw(self, x: int, y: int, max_width: int, panel: Any) -> int:
        viewer = self.viewer
        self.clear_buttons()
        if not viewer.simulator_inputs_enabled:
            return viewer.draw_text(
                "Simulator inputs disabled in config.",
                x,
                y,
                viewer.small_font,
                viewer.palette["muted"],
                max_width,
            )

        status = "live control bus" if viewer.controls_available() else "needs --offline-decision --live-view"
        y = viewer.draw_text(f"Mock Inputs: {status}", x, y, viewer.small_font, viewer.palette["muted"], max_width)
        snapshot = viewer.sim_input_state.snapshot(team=viewer.records[viewer.current_index].team, goals=viewer.goals)
        summary = as_dict(snapshot.get("summary"))
        summary_text = (
            f"Units F{summary.get('friend_units', 0)}/E{summary.get('enemy_units', 0)} "
            f"LowHP={len(summary.get('low_hp_units', []))} "
            f"Destroyed={len(summary.get('destroyed_structures', []))}"
        )
        y = viewer.draw_text(summary_text, x, y, viewer.small_font, viewer.palette["muted"], max_width)
        y += 8
        y = self.draw_structure_controls(x, y, max_width)
        y = self.draw_unit_palette(x, y, max_width, panel)
        return self.draw_placed_units(x, y, max_width, panel)

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
        columns = 3 if max_width >= 330 else 2
        chip_w = (max_width - gap * (columns - 1)) // columns
        chip_h = 38
        for index, item in enumerate(state.unit_palette):
            row = index // columns
            col = index % columns
            rect = pg.Rect(x + col * (chip_w + gap), y + row * (chip_h + gap), chip_w, chip_h)
            self.unit_palette_buttons[index] = rect
            self.draw_unit_chip(rect, item)
        rows = math.ceil(len(state.unit_palette) / columns)
        y += rows * chip_h + max(0, rows - 1) * gap
        y += 6
        if y < panel.bottom - 36:
            pg.draw.line(viewer.screen, viewer.palette["line"], (x, y), (x + max_width, y), 1)
            return y + 8
        return y

    def draw_placed_units(self, x: int, y: int, max_width: int, panel: Any) -> int:
        viewer = self.viewer
        pg = viewer.pg
        state = viewer.sim_input_state
        viewer.draw_text("Placed Pieces", x, y, viewer.font, viewer.palette["accent"], max_width)
        y += 24
        snapshot = state.snapshot(team=viewer.records[viewer.current_index].team, goals=viewer.goals)
        units = [item for item in snapshot.get("units", []) if isinstance(item, dict)]
        if not units:
            return viewer.draw_text(
                "Drag a piece onto the field map.",
                x,
                y,
                viewer.small_font,
                viewer.palette["muted"],
                max_width,
            )
        button_w = 42
        button_h = 22
        gap = 5
        icon_w = 34
        label_w = max_width - button_w * 3 - gap * 2 - icon_w - 12
        units.sort(key=lambda unit: (str(unit.get("side", "")), str(unit.get("entity_id", ""))))
        for unit in units:
            row_h = 32
            if y > panel.bottom - row_h - 8:
                viewer.draw_text("...", x, y, viewer.small_font, viewer.palette["muted"], max_width)
                break
            position = as_dict(unit.get("position_cm"))
            side = str(unit.get("side", "friend"))
            type_id = int(unit.get("type_id", 0) or 0)
            type_name = str(unit.get("type", "?"))
            hp = int(unit.get("hp", 0) or 0)
            max_hp = max(1, int(unit.get("max_hp", 1) or 1))
            entity_id = str(unit.get("entity_id", ""))
            pos = f"{position.get('x', 0)},{position.get('y', 0)}"
            channels = str(unit.get("decision_summary", compact_decision_summary(side, type_id)))
            text = f"{side[:1].upper()} {type_name} {hp}/{max_hp} @{pos} {channels}"
            icon_center = (x + 15, y + 14)
            field_side = viewer.unit_field_side(side)
            if not viewer.draw_unit_art(icon_center[0], icon_center[1], field_side, type_name, 25):
                self.draw_unit_dot(icon_center, side)
            text_x = x + icon_w
            self.draw_fitted_text(text, text_x, y + 2, label_w, viewer.palette["text"])
            viewer.draw_health_bar(text_x, y + 21, max(24, label_w - 2), 4, hp / max_hp)
            step = 50 if max_hp > 100 else 10
            actions = [
                (f"-{step}", "set_unit_hp", max(0, hp - step)),
                (f"+{step}", "set_unit_hp", min(max_hp, hp + step)),
                ("X", "remove_unit", hp),
            ]
            bx = x + icon_w + label_w + 8
            for index, (label, command, next_hp) in enumerate(actions):
                rect = pg.Rect(bx + index * (button_w + gap), y, button_w, button_h)
                viewer.draw_control_button(rect, label)
                payload = {
                    "command": command,
                    "entity_id": entity_id,
                    "hp": int(next_hp),
                }
                self.unit_hp_buttons[f"{entity_id}:{label}"] = (rect, payload)
            y += row_h + 4
        return y

    def draw_unit_chip(self, rect: Any, item: Any) -> None:
        viewer = self.viewer
        pg = viewer.pg
        side = item.side
        side_style = as_dict(viewer.unit_styles.get(side, {}))
        fill = pg.Color(side_style.get("color", viewer.colors_raw.get(side, "#4fb3d9")))
        pg.draw.rect(viewer.screen, viewer.palette["panel2"], rect, border_radius=5)
        pg.draw.rect(viewer.screen, fill, rect, 2, border_radius=5)
        icon_center = (rect.x + 17, rect.centery)
        field_side = viewer.unit_field_side(side)
        if not viewer.draw_unit_art(icon_center[0], icon_center[1], field_side, item.type_name, 25):
            self.draw_unit_dot(icon_center, side)
        label = f"{side[:1].upper()} {item.type_name}"
        text_x = rect.x + 35
        self.draw_fitted_text(label, text_x, rect.y + 4, max(8, rect.right - text_x - 6), viewer.palette["text"])
        hp_text = f"{item.hp}/{item.max_hp}"
        self.draw_fitted_text(hp_text, text_x, rect.y + 17, max(8, rect.right - text_x - 6), viewer.palette["muted"])
        self.draw_fitted_text(
            compact_decision_summary(item.side, item.type_id),
            text_x,
            rect.y + 28,
            max(8, rect.right - text_x - 6),
            viewer.palette["muted"],
        )

    def draw_unit_dot(self, center: tuple[int, int], side: str) -> None:
        viewer = self.viewer
        pg = viewer.pg
        side_style = as_dict(viewer.unit_styles.get(side, {}))
        fill = pg.Color(side_style.get("color", viewer.colors_raw.get(side, "#4fb3d9")))
        pg.draw.circle(viewer.screen, viewer.palette["black"], center, 11)
        pg.draw.circle(viewer.screen, fill, center, 9)

    def draw_fitted_text(self, text: str, x: int, y: int, max_width: int, draw_color: Any) -> None:
        viewer = self.viewer
        if max_width <= 0:
            return
        fitted = viewer.fit_word(str(text), viewer.small_font, max_width)
        viewer.screen.blit(viewer.small_font.render(fitted, True, draw_color), (x, y))
