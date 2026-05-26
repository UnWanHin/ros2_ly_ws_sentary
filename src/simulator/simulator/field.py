from __future__ import annotations

from dataclasses import dataclass
from typing import Any


DEFAULT_FIELD_WIDTH_CM = 2800
DEFAULT_FIELD_HEIGHT_CM = 1500
DEFAULT_FIELD_CM = (DEFAULT_FIELD_WIDTH_CM, DEFAULT_FIELD_HEIGHT_CM)


def _number(value: Any, default: float) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out


@dataclass(frozen=True)
class FieldGeometry:
    width: int = DEFAULT_FIELD_WIDTH_CM
    height: int = DEFAULT_FIELD_HEIGHT_CM

    @classmethod
    def from_config(cls, value: Any) -> "FieldGeometry":
        if not isinstance(value, dict):
            return cls()
        width = int(_number(value.get("width"), DEFAULT_FIELD_WIDTH_CM))
        height = int(_number(value.get("height"), DEFAULT_FIELD_HEIGHT_CM))
        return cls(max(1, width), max(1, height))

    def clamp_x(self, value: Any, default: float | None = None) -> int:
        fallback = float(self.width) / 2.0 if default is None else float(default)
        parsed = _number(value, fallback)
        return max(0, min(self.width, int(round(parsed))))

    def clamp_y(self, value: Any, default: float | None = None) -> int:
        fallback = float(self.height) / 2.0 if default is None else float(default)
        parsed = _number(value, fallback)
        return max(0, min(self.height, int(round(parsed))))

    def clamp_point(self, point: tuple[float, float]) -> tuple[float, float]:
        return (
            max(0.0, min(float(self.width), float(point[0]))),
            max(0.0, min(float(self.height), float(point[1]))),
        )

    def position_data_raw_y(self, official_y: Any) -> int:
        return self.height - self.clamp_y(official_y)

    def position_data_official_y(self, raw_y: Any) -> int:
        return self.height - self.clamp_y(raw_y)


def relative_side_to_field_side(relative_side: str, team: str) -> str:
    normalized_team = str(team).strip().lower()
    if normalized_team not in ("red", "blue"):
        normalized_team = "red"
    if str(relative_side).strip().lower() == "friend":
        return normalized_team
    return "blue" if normalized_team == "red" else "red"


def field_to_screen(
    pos: tuple[float, float],
    image_rect: Any,
    field: FieldGeometry,
) -> tuple[int, int]:
    x, y = field.clamp_point(pos)
    sx = image_rect.x + x / field.width * image_rect.width
    sy = image_rect.y + (1.0 - y / field.height) * image_rect.height
    return (round(sx), round(sy))


def screen_to_field(
    pos: tuple[int, int],
    image_rect: Any,
    field: FieldGeometry,
) -> tuple[float, float] | None:
    if image_rect.width <= 0 or image_rect.height <= 0 or not image_rect.collidepoint(pos):
        return None
    x = (pos[0] - image_rect.x) / image_rect.width * field.width
    y = (1.0 - (pos[1] - image_rect.y) / image_rect.height) * field.height
    return field.clamp_point((x, y))
