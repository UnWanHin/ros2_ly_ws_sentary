"""Presentation-only workspace geometry shared by simulator renderers.

The simulator scene remains the source of truth.  This module only converts
official field coordinates to pixels and assigns non-overlapping dock regions.
"""

from __future__ import annotations

from dataclasses import dataclass


MIN_SCALE = 0.5
MAX_SCALE = 4.0


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class Rect:
    x: float
    y: float
    width: float
    height: float

    @property
    def right(self) -> float:
        return self.x + self.width

    @property
    def bottom(self) -> float:
        return self.y + self.height

    @property
    def center(self) -> tuple[float, float]:
        return (self.x + self.width / 2.0, self.y + self.height / 2.0)

    def inset(self, amount: float) -> "Rect":
        return Rect(
            self.x + amount,
            self.y + amount,
            max(1.0, self.width - 2.0 * amount),
            max(1.0, self.height - 2.0 * amount),
        )


@dataclass(frozen=True)
class DockPanel:
    zone: str
    rect: Rect
    collapsed: bool = False


@dataclass(frozen=True)
class Selection:
    kind: str = "overview"
    key: str = ""


@dataclass(frozen=True)
class WorkspaceLayout:
    command_bar: Rect
    activity_rail: Rect
    viewport: Rect
    inspector: DockPanel
    operations_shelf: DockPanel
    content_width: float

    @classmethod
    def desktop(
        cls,
        width: float,
        height: float,
        *,
        inspector_width: float = 320.0,
        inspector_zone: str = "right",
        inspector_collapsed: bool = False,
        shelf_height: float = 172.0,
        shelf_collapsed: bool = True,
    ) -> "WorkspaceLayout":
        """Build the common desktop shell without renderer-specific objects."""

        outer = 24.0
        gap = 16.0
        command_height = 56.0
        activity_width = 56.0
        collapsed_width = 48.0
        collapsed_shelf_height = 40.0
        minimum_width = 960.0
        minimum_height = 620.0
        work_width = max(minimum_width, float(width))
        work_height = max(minimum_height, float(height))
        command = Rect(outer, outer, work_width - 2.0 * outer, command_height)
        shelf_size = collapsed_shelf_height if shelf_collapsed else _clamp(shelf_height, 140.0, 300.0)
        shelf = Rect(outer, work_height - outer - shelf_size, work_width - 2.0 * outer, shelf_size)
        workspace_top = command.bottom + gap
        workspace_bottom = shelf.y - gap
        workspace_height = max(120.0, workspace_bottom - workspace_top)
        rail = Rect(outer, workspace_top, activity_width, workspace_height)
        panel_width = collapsed_width if inspector_collapsed else _clamp(inspector_width, 280.0, 460.0)
        normalized_zone = "left" if inspector_zone == "left" else "right"

        if normalized_zone == "right":
            inspector_rect = Rect(work_width - outer - panel_width, workspace_top, panel_width, workspace_height)
            viewport = Rect(rail.right + gap, workspace_top, max(160.0, inspector_rect.x - gap - (rail.right + gap)), workspace_height)
        else:
            inspector_rect = Rect(rail.right + gap, workspace_top, panel_width, workspace_height)
            viewport = Rect(inspector_rect.right + gap, workspace_top, max(160.0, work_width - outer - (inspector_rect.right + gap)), workspace_height)

        content_width = viewport.width + gap + inspector_rect.width
        return cls(
            command_bar=command,
            activity_rail=rail,
            viewport=viewport,
            inspector=DockPanel(normalized_zone, inspector_rect, inspector_collapsed),
            operations_shelf=DockPanel("bottom", shelf, shelf_collapsed),
            content_width=content_width,
        )


@dataclass
class ViewportState:
    """A map viewport expressed only in official field centimeters and pixels."""

    field_width: float
    field_height: float
    viewport: Rect
    scale: float
    center_x: float
    center_y: float

    @classmethod
    def fit(
        cls,
        *,
        field_width: float,
        field_height: float,
        viewport: Rect,
        padding_px: float = 24.0,
    ) -> "ViewportState":
        state = cls(
            field_width=max(1.0, float(field_width)),
            field_height=max(1.0, float(field_height)),
            viewport=viewport,
            scale=MIN_SCALE,
            center_x=max(1.0, float(field_width)) / 2.0,
            center_y=max(1.0, float(field_height)) / 2.0,
        )
        state.reset_to_fit(padding_px)
        return state

    def set_viewport(self, viewport: Rect) -> None:
        """Resize without changing the official coordinate at the viewport center."""

        self.viewport = viewport

    def screen_to_field(self, x: float, y: float) -> tuple[float, float]:
        midpoint_x, midpoint_y = self.viewport.center
        return (
            self.center_x + (float(x) - midpoint_x) / self.scale,
            self.center_y - (float(y) - midpoint_y) / self.scale,
        )

    def field_to_screen(self, x: float, y: float) -> tuple[float, float]:
        midpoint_x, midpoint_y = self.viewport.center
        return (
            midpoint_x + (float(x) - self.center_x) * self.scale,
            midpoint_y - (float(y) - self.center_y) * self.scale,
        )

    def zoom_at(self, x: float, y: float, *, factor: float) -> None:
        """Zoom while retaining the pointer's official field coordinate."""

        field_x, field_y = self.screen_to_field(x, y)
        self.scale = _clamp(self.scale * float(factor), MIN_SCALE, MAX_SCALE)
        midpoint_x, midpoint_y = self.viewport.center
        self.center_x = field_x - (float(x) - midpoint_x) / self.scale
        self.center_y = field_y + (float(y) - midpoint_y) / self.scale

    def pan_by(self, dx: float, dy: float) -> None:
        """Move the visible world with the pointer drag delta in pixels."""

        self.center_x -= float(dx) / self.scale
        self.center_y += float(dy) / self.scale

    def reset_to_fit(self, padding_px: float = 24.0) -> None:
        available_width = max(1.0, self.viewport.width - 2.0 * max(0.0, padding_px))
        available_height = max(1.0, self.viewport.height - 2.0 * max(0.0, padding_px))
        self.scale = _clamp(
            min(available_width / self.field_width, available_height / self.field_height),
            MIN_SCALE,
            MAX_SCALE,
        )
        self.center_x = self.field_width / 2.0
        self.center_y = self.field_height / 2.0

    def actual_size(self, map_pixel_width: float) -> None:
        self.scale = _clamp(float(map_pixel_width) / self.field_width, MIN_SCALE, MAX_SCALE)
        self.center_x = self.field_width / 2.0
        self.center_y = self.field_height / 2.0

    def zoom_to_field_bounds(self, bounds: Rect, padding_px: float = 96.0) -> None:
        available_width = max(1.0, self.viewport.width - 2.0 * max(0.0, padding_px))
        available_height = max(1.0, self.viewport.height - 2.0 * max(0.0, padding_px))
        self.scale = _clamp(
            min(available_width / max(1.0, bounds.width), available_height / max(1.0, bounds.height)),
            MIN_SCALE,
            MAX_SCALE,
        )
        self.center_x = bounds.x + bounds.width / 2.0
        self.center_y = bounds.y + bounds.height / 2.0
