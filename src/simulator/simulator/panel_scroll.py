from __future__ import annotations

from dataclasses import dataclass, field


def nonnegative_int(value: object, default: int = 0) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        parsed = default
    return max(0, parsed)


@dataclass
class PanelScrollState:
    bottom_padding: int = 12
    offsets: dict[str, int] = field(default_factory=dict)
    content_heights: dict[str, int] = field(default_factory=dict)
    body_heights: dict[str, int] = field(default_factory=dict)

    def set_body_height(self, tab: str, height: object) -> int:
        value = nonnegative_int(height)
        self.body_heights[str(tab)] = value
        return value

    def set_content_height(self, tab: str, height: object) -> int:
        value = nonnegative_int(height)
        self.content_heights[str(tab)] = value
        return value

    def body_height(self, tab: str, default: int = 0) -> int:
        return nonnegative_int(self.body_heights.get(str(tab), default), default)

    def content_height(self, tab: str, default: int = 0) -> int:
        return nonnegative_int(self.content_heights.get(str(tab), default), default)

    def max_scroll(self, tab: str) -> int:
        content = self.content_height(tab)
        body = self.body_height(tab)
        return max(0, content - body + nonnegative_int(self.bottom_padding))

    def offset(self, tab: str) -> int:
        return self.clamp(tab)

    def set_offset(self, tab: str, offset: object) -> int:
        key = str(tab)
        self.offsets[key] = nonnegative_int(offset)
        return self.clamp(key)

    def clamp(self, tab: str) -> int:
        key = str(tab)
        current = nonnegative_int(self.offsets.get(key, 0))
        clamped = min(current, self.max_scroll(key))
        self.offsets[key] = clamped
        return clamped

    def scroll(self, tab: str, delta: object) -> int:
        key = str(tab)
        try:
            amount = int(delta)
        except (TypeError, ValueError):
            amount = 0
        current = int(self.offsets.get(key, 0))
        self.offsets[key] = max(0, min(self.max_scroll(key), current + amount))
        return self.offsets[key]
