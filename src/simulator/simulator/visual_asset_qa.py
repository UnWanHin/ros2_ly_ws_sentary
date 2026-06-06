from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .assets import load_unit_asset_catalog
from .config import load_config, resolve_path
from .field import FieldGeometry, field_to_screen, relative_side_to_field_side
from .interactive_inputs import load_unit_scene_file
from .trace import as_dict


SCHEMA = "ly_simulator_visual_asset_qa_v1"


@dataclass(frozen=True)
class Rect:
    x: int
    y: int
    width: int
    height: int


def import_pygame() -> Any:
    try:
        import pygame
    except ImportError as exc:
        raise RuntimeError("pygame is required for visual asset QA") from exc
    return pygame


def fit_rect(src_size: tuple[int, int], dst_rect: Rect) -> Rect:
    src_w, src_h = src_size
    scale = min(dst_rect.width / max(1, src_w), dst_rect.height / max(1, src_h))
    width = int(src_w * scale)
    height = int(src_h * scale)
    return Rect(
        dst_rect.x + (dst_rect.width - width) // 2,
        dst_rect.y + (dst_rect.height - height) // 2,
        width,
        height,
    )


def map_area_rect(config: dict[str, Any], screenshot_size: tuple[int, int]) -> Rect:
    window = as_dict(config.get("window"))
    width, height = screenshot_size
    panel_w = int(window.get("panel_width", 390))
    timeline_h = int(window.get("timeline_height", 92))
    return Rect(18, 18, width - panel_w - 36, height - timeline_h - 32)


def map_image_rect(config: dict[str, Any], screenshot_size: tuple[int, int], map_size: tuple[int, int]) -> Rect:
    area = map_area_rect(config, screenshot_size)
    inflated = Rect(area.x + 9, area.y + 9, area.width - 18, area.height - 18)
    return fit_rect(map_size, inflated)


def get_rgb(surface: Any, x: int, y: int) -> tuple[int, int, int]:
    color = surface.get_at((x, y))
    return (int(color.r), int(color.g), int(color.b))


def get_rgba(surface: Any, x: int, y: int) -> tuple[int, int, int, int]:
    color = surface.get_at((x, y))
    return (int(color.r), int(color.g), int(color.b), int(color.a))


def color_distance(a: tuple[int, int, int], b: tuple[int, int, int]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])


def informative_sprite_pixel(rgb: tuple[int, int, int]) -> bool:
    brightness = rgb[0] + rgb[1] + rgb[2]
    chroma = max(rgb) - min(rgb)
    return brightness >= 140 or chroma >= 28


def scaled_sprite(pygame: Any, path: Path, size_px: int) -> Any:
    raw = pygame.image.load(str(path))
    raw_rect = raw.get_rect()
    scale = min(size_px / max(1, raw_rect.width), size_px / max(1, raw_rect.height))
    target_size = (
        max(1, int(round(raw_rect.width * scale))),
        max(1, int(round(raw_rect.height * scale))),
    )
    return pygame.transform.smoothscale(raw, target_size)


def sprite_match_score(
    screenshot: Any,
    sprite: Any,
    center: tuple[int, int],
    *,
    alpha_threshold: int = 160,
    color_tolerance: int = 70,
) -> dict[str, Any]:
    sprite_w, sprite_h = sprite.get_size()
    left = int(round(center[0] - sprite_w / 2))
    top = int(round(center[1] - sprite_h / 2))
    screen_w, screen_h = screenshot.get_size()
    informative = 0
    matched = 0
    sampled = 0
    step = max(1, min(sprite_w, sprite_h) // 20)
    for sy in range(0, sprite_h, step):
        screen_y = top + sy
        if screen_y < 0 or screen_y >= screen_h:
            continue
        for sx in range(0, sprite_w, step):
            screen_x = left + sx
            if screen_x < 0 or screen_x >= screen_w:
                continue
            expected = get_rgba(sprite, sx, sy)
            if expected[3] < alpha_threshold:
                continue
            if not informative_sprite_pixel(expected[:3]):
                continue
            informative += 1
            sampled += 1
            actual = get_rgb(screenshot, screen_x, screen_y)
            if color_distance(actual, expected[:3]) <= color_tolerance:
                matched += 1
    ratio = 0.0 if informative == 0 else matched / informative
    return {
        "opaque_samples": informative,
        "sampled": sampled,
        "matched": matched,
        "match_ratio": round(ratio, 4),
        "sprite_rect": {"x": left, "y": top, "width": sprite_w, "height": sprite_h},
    }


def build_visual_asset_report(
    screenshot_path: Path,
    unit_scene_path: Path,
    *,
    config_path: Path | None = None,
    map_path: Path | None = None,
    team: str = "red",
    min_match_ratio: float = 0.12,
    min_opaque_samples: int = 20,
) -> dict[str, Any]:
    pygame = import_pygame()
    pygame.init()
    try:
        config = load_config(config_path)
        paths = as_dict(config.get("paths"))
        resolved_map = resolve_path(map_path or paths.get("default_map", "tools/maps/basemaps/buff_map_field.png"))
        screenshot = pygame.image.load(str(screenshot_path))
        map_image = pygame.image.load(str(resolved_map))
        screenshot_size = screenshot.get_size()
        image_rect = map_image_rect(config, screenshot_size, map_image.get_size())
        field = FieldGeometry.from_config(config.get("field_cm"))
        catalog = load_unit_asset_catalog(as_dict(config.get("assets")))
        units = load_unit_scene_file(unit_scene_path)
        unit_size_px = int(catalog.unit_size_px)

        checked: list[dict[str, Any]] = []
        issues: list[dict[str, Any]] = []
        for unit in units:
            side = str(unit.get("side", ""))
            type_id = int(unit.get("type_id", 0))
            type_name = str(unit.get("type", f"Unit{type_id}"))
            field_side = relative_side_to_field_side(side, team)
            sprite_path = catalog.path_for(field_side, type_name)
            position = (float(unit.get("x", 0.0)), float(unit.get("y", 0.0)))
            center = field_to_screen(position, image_rect, field)
            item = {
                "side": side,
                "field_side": field_side,
                "type_id": type_id,
                "type": type_name,
                "position_cm": {"x": position[0], "y": position[1]},
                "screen_center": {"x": center[0], "y": center[1]},
                "sprite": sprite_path.as_posix() if sprite_path is not None else None,
            }
            if sprite_path is None:
                issues.append(
                    {
                        "severity": "error",
                        "code": "visual_asset.missing_sprite",
                        "unit": f"{side}:{type_name}",
                        "message": "No sprite path resolved for unit.",
                    }
                )
                checked.append({**item, "visible": False})
                continue

            sprite = scaled_sprite(pygame, sprite_path, unit_size_px)
            score = sprite_match_score(screenshot, sprite, center)
            visible = (
                score["opaque_samples"] >= int(min_opaque_samples)
                and score["match_ratio"] >= float(min_match_ratio)
            )
            checked.append({**item, **score, "visible": visible})
            if not visible:
                issues.append(
                    {
                        "severity": "error",
                        "code": "visual_asset.sprite_not_visible",
                        "unit": f"{side}:{type_name}",
                        "message": (
                            f"Sprite match ratio {score['match_ratio']:.3f} with "
                            f"{score['opaque_samples']} opaque samples is below threshold."
                        ),
                    }
                )

        error_count = sum(1 for issue in issues if issue["severity"] == "error")
        return {
            "schema": SCHEMA,
            "screenshot": screenshot_path.as_posix(),
            "scene": unit_scene_path.as_posix(),
            "team": team,
            "map_rect": {
                "x": image_rect.x,
                "y": image_rect.y,
                "width": image_rect.width,
                "height": image_rect.height,
            },
            "thresholds": {
                "min_match_ratio": float(min_match_ratio),
                "min_opaque_samples": int(min_opaque_samples),
            },
            "summary": {
                "status": "FAIL" if error_count else "PASS",
                "units": len(checked),
                "visible_units": sum(1 for item in checked if item.get("visible")),
                "errors": error_count,
            },
            "units": checked,
            "issues": issues,
        }
    finally:
        pygame.quit()


def print_text_report(report: dict[str, Any]) -> int:
    summary = as_dict(report.get("summary"))
    print("Visual Asset QA")
    print(f"Status: {summary.get('status', 'FAIL')}")
    print(f"Screenshot: {Path(str(report.get('screenshot', ''))).name}")
    print(f"Units: visible={summary.get('visible_units', 0)} / {summary.get('units', 0)}")
    for unit in report.get("units", []):
        item = as_dict(unit)
        status = "PASS" if item.get("visible") else "FAIL"
        print(
            f"  {status} {item.get('side')}:{item.get('type')} "
            f"ratio={item.get('match_ratio', '-')} center={as_dict(item.get('screen_center'))}"
        )
    for issue in report.get("issues", []):
        item = as_dict(issue)
        print(f"  ERROR {item.get('code')} {item.get('unit')}: {item.get('message')}")
    return 0 if summary.get("status") == "PASS" else 1


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check that simulator unit sprites are visible in a smoke screenshot.")
    parser.add_argument("--screenshot", required=True, help="Rendered simulator smoke screenshot PNG.")
    parser.add_argument("--unit-scene", required=True, help="Unit scene expected to be visible in the screenshot.")
    parser.add_argument("--config", default="", help="Optional simulator config YAML.")
    parser.add_argument("--map", dest="map_path", default="", help="Optional basemap path.")
    parser.add_argument("--team", choices=("red", "blue"), default="red")
    parser.add_argument("--min-match-ratio", type=float, default=0.12)
    parser.add_argument("--min-opaque-samples", type=int, default=20)
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    screenshot_path = resolve_path(args.screenshot)
    scene_path = resolve_path(args.unit_scene)
    config_path = resolve_path(args.config) if str(args.config).strip() else None
    map_path = resolve_path(args.map_path) if str(args.map_path).strip() else None
    try:
        report = build_visual_asset_report(
            screenshot_path,
            scene_path,
            config_path=config_path,
            map_path=map_path,
            team=args.team,
            min_match_ratio=args.min_match_ratio,
            min_opaque_samples=args.min_opaque_samples,
        )
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"visual asset QA failed: {exc}")
        return 2
    if args.json:
        print(json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True))
        return 0 if report.get("summary", {}).get("status") == "PASS" else 1
    return print_text_report(report)


if __name__ == "__main__":
    raise SystemExit(main())
