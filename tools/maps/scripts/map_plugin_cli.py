#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_AREA_HEADER = REPO_ROOT / "src" / "behavior_tree" / "module" / "Area.hpp"
DEFAULT_BASIC_TYPES = REPO_ROOT / "src" / "behavior_tree" / "module" / "BasicTypes.hpp"

FALLBACK_POINT_ID_NAME = {
    0: "Home",
    1: "Base",
    2: "Recovery",
    3: "BuffShoot",
    4: "LeftHighLand",
    5: "CastleLeft1",
    6: "Castle",
    7: "CastleRight1",
    8: "CastleRight2",
    9: "FlyRoad",
    10: "OutpostArea",
    11: "MidShoot",
    12: "LeftShoot",
    13: "OutpostShoot",
    14: "BuffAround1",
    15: "BuffAround2",
    16: "RightShoot",
    17: "HoleRoad",
    18: "OccupyArea",
    19: "Highland",
    20: "CastleLeft2",
    21: "BaseToCentral",
    22: "CentralToBase",
}


@dataclass
class PointEntry:
    id_value: int
    name: str
    red: tuple[int, int]
    blue: tuple[int, int]


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def parse_teamed_location_ids(text: str) -> dict[int, str]:
    pattern = re.compile(
        r"static\s+constexpr\s+TeamedLocation\s+([A-Za-z_]\w*)\s*\{\s*(\d+)\s*\}\s*;"
    )
    result: dict[int, str] = {}
    for match in pattern.finditer(text):
        result[int(match.group(2))] = match.group(1)
    return result


def parse_area_location_names(text: str) -> list[str]:
    pattern = re.compile(
        r"(?:static\s+const\s+)?(?:auto|Location\s*<[^>]+>)\s+([A-Za-z_]\w*)\s*"
        r"\{\s*\{\s*-?\d+\s*,\s*-?\d+\s*\}\s*,\s*\{\s*-?\d+\s*,\s*-?\d+\s*\}\s*\}\s*;?"
    )
    return [match.group(1) for match in pattern.finditer(text)]


def parse_area_locations(text: str) -> dict[str, tuple[tuple[int, int], tuple[int, int]]]:
    pattern = re.compile(
        r"(?:static\s+const\s+)?(?:auto|Location\s*<[^>]+>)\s+([A-Za-z_]\w*)\s*"
        r"\{\s*\{\s*(-?\d+)\s*,\s*(-?\d+)\s*\}\s*,\s*\{\s*(-?\d+)\s*,\s*(-?\d+)\s*\}\s*\}\s*;?"
    )
    locations: dict[str, tuple[tuple[int, int], tuple[int, int]]] = {}
    for match in pattern.finditer(text):
        locations[match.group(1)] = (
            (int(match.group(2)), int(match.group(3))),
            (int(match.group(4)), int(match.group(5))),
        )
    return locations


def read_text_if_exists(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return ""


def load_point_catalog(area_header: str | None = None, basic_types: str | None = None) -> dict[int, str]:
    basic_types_path = Path(basic_types).expanduser().resolve() if basic_types else DEFAULT_BASIC_TYPES
    area_header_path = Path(area_header).expanduser().resolve() if area_header else DEFAULT_AREA_HEADER

    point_id_name = parse_teamed_location_ids(read_text_if_exists(basic_types_path))
    if not point_id_name:
        point_id_name = dict(FALLBACK_POINT_ID_NAME)

    used_names = set(point_id_name.values())
    next_id = max(point_id_name, default=-1) + 1
    for name in parse_area_location_names(read_text_if_exists(area_header_path)):
        if name in used_names:
            continue
        while next_id in point_id_name:
            next_id += 1
        point_id_name[next_id] = name
        used_names.add(name)
        next_id += 1
    return dict(sorted(point_id_name.items()))


def load_area_locations(area_header: str | None = None) -> dict[str, tuple[tuple[int, int], tuple[int, int]]]:
    area_header_path = Path(area_header).expanduser().resolve() if area_header else DEFAULT_AREA_HEADER
    return parse_area_locations(read_text_if_exists(area_header_path))


def normalize_point(raw: dict[str, Any], point_id_name: dict[int, str]) -> PointEntry:
    id_value = int(raw["id"])
    name = str(raw.get("name") or point_id_name.get(id_value, f"Point{id_value}"))
    red = tuple(int(round(float(value))) for value in raw["red"])
    blue = tuple(int(round(float(value))) for value in raw["blue"])
    return PointEntry(id_value=id_value, name=name, red=(red[0], red[1]), blue=(blue[0], blue[1]))


def validate_plugin(data: dict[str, Any], point_id_name: dict[int, str]) -> tuple[list[PointEntry], list[str]]:
    errors: list[str] = []
    points_raw = data.get("points")
    if not isinstance(points_raw, list):
        return [], ["'points' must be a list"]

    entries: list[PointEntry] = []
    seen_ids: set[int] = set()

    for index, raw in enumerate(points_raw):
        if not isinstance(raw, dict):
            errors.append(f"points[{index}] must be an object")
            continue
        for key in ("id", "red", "blue"):
            if key not in raw:
                errors.append(f"points[{index}] missing key '{key}'")
        if "id" not in raw or "red" not in raw or "blue" not in raw:
            continue
        try:
            entry = normalize_point(raw, point_id_name)
        except Exception as error:
            errors.append(f"points[{index}] parse error: {error}")
            continue
        if entry.id_value not in point_id_name:
            errors.append(f"points[{index}] id={entry.id_value} is not defined by Area.hpp/BasicTypes.hpp")
        if entry.id_value in seen_ids:
            errors.append(f"duplicate point id={entry.id_value}")
        seen_ids.add(entry.id_value)
        entries.append(entry)

    for id_value, name in point_id_name.items():
        if id_value not in seen_ids:
            errors.append(f"missing point id={id_value} ({name})")

    map_size = data.get("map_size_cm", {})
    if not isinstance(map_size, dict):
        errors.append("'map_size_cm' must be an object")
    else:
        width = map_size.get("width")
        height = map_size.get("height")
        if width is None or height is None:
            errors.append("'map_size_cm.width' and 'map_size_cm.height' are required")
        else:
            try:
                width_value = float(width)
                height_value = float(height)
                if width_value <= 0 or height_value <= 0:
                    errors.append("map_size_cm must be > 0")
            except Exception:
                errors.append("map_size_cm width/height must be numeric")

    return entries, errors


def sync_plugin_data(
    data: dict[str, Any],
    point_id_name: dict[int, str],
    area_locations: dict[str, tuple[tuple[int, int], tuple[int, int]]],
    update_coords: bool,
) -> dict[str, Any]:
    points_raw = data.get("points", [])
    if not isinstance(points_raw, list):
        points_raw = []

    existing_by_id: dict[int, dict[str, Any]] = {}
    for raw in points_raw:
        if not isinstance(raw, dict) or "id" not in raw:
            continue
        try:
            existing_by_id[int(raw["id"])] = raw
        except Exception:
            continue

    synced = dict(data)
    synced_points: list[dict[str, Any]] = []
    for id_value, name in point_id_name.items():
        existing = existing_by_id.get(id_value, {})
        point = {
            "id": id_value,
            "name": name,
            "red": list(existing.get("red", [0, 0])),
            "blue": list(existing.get("blue", [0, 0])),
        }
        if update_coords and name in area_locations:
            red, blue = area_locations[name]
            point["red"] = [red[0], red[1]]
            point["blue"] = [blue[0], blue[1]]
        synced_points.append(point)

    synced["points"] = synced_points
    return synced


def cmd_init(args: argparse.Namespace) -> None:
    output = Path(args.output).expanduser().resolve()
    point_id_name = load_point_catalog(args.area_header, args.basic_types)
    data = {
        "map_name": args.map_name,
        "frame": "left_bottom_origin_cm",
        "map_size_cm": {"width": 2800, "height": 1500},
        "points": [
            {
                "id": id_value,
                "name": name,
                "red": [0, 0],
                "blue": [0, 0],
            }
            for id_value, name in point_id_name.items()
        ],
    }
    write_json(output, data)
    print(f"created: {output}")


def cmd_validate(args: argparse.Namespace) -> None:
    input_path = Path(args.input).expanduser().resolve()
    data = read_json(input_path)
    point_id_name = load_point_catalog(args.area_header, args.basic_types)
    _, errors = validate_plugin(data, point_id_name)
    if errors:
        print("validation failed:")
        for error in errors:
            print(f"- {error}")
        raise SystemExit(2)
    print("validation ok")


def cmd_sync(args: argparse.Namespace) -> None:
    input_path = Path(args.input).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve() if args.output else input_path
    data = read_json(input_path)
    point_id_name = load_point_catalog(args.area_header, args.basic_types)
    area_locations = load_area_locations(args.area_header)
    synced = sync_plugin_data(
        data,
        point_id_name,
        area_locations,
        update_coords=not args.preserve_existing_coords,
    )
    _, errors = validate_plugin(synced, point_id_name)
    if errors:
        print("sync produced invalid plugin:")
        for error in errors:
            print(f"- {error}")
        raise SystemExit(2)
    write_json(output_path, synced)
    print(f"synced: {output_path}")


def cmd_emit_area(args: argparse.Namespace) -> None:
    input_path = Path(args.input).expanduser().resolve()
    data = read_json(input_path)
    point_id_name = load_point_catalog(args.area_header, args.basic_types)
    entries, errors = validate_plugin(data, point_id_name)
    if errors:
        print("validation failed:")
        for error in errors:
            print(f"- {error}")
        raise SystemExit(2)

    entries_by_id = {entry.id_value: entry for entry in entries}
    for id_value in sorted(point_id_name):
        entry = entries_by_id[id_value]
        print(
            f"static const Location<std::uint16_t> {entry.name}"
            f"{{ {{{entry.red[0]}, {entry.red[1]}}}, {{{entry.blue[0]}, {entry.blue[1]}}} }};"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Map plugin helper for BT map points.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    def add_catalog_args(subparser: argparse.ArgumentParser) -> None:
        subparser.add_argument(
            "--area-header",
            default=str(DEFAULT_AREA_HEADER),
            help="Area.hpp path used to discover Location entries",
        )
        subparser.add_argument(
            "--basic-types",
            default=str(DEFAULT_BASIC_TYPES),
            help="BasicTypes.hpp path used to discover TeamedLocation IDs",
        )

    parser_init = subparsers.add_parser("init", help="Create plugin JSON template")
    parser_init.add_argument("--output", required=True, help="Output JSON path")
    parser_init.add_argument("--map-name", default="RMUC2026_custom", help="Map name")
    add_catalog_args(parser_init)
    parser_init.set_defaults(func=cmd_init)

    parser_validate = subparsers.add_parser("validate", help="Validate plugin JSON")
    parser_validate.add_argument("--input", required=True, help="Plugin JSON path")
    add_catalog_args(parser_validate)
    parser_validate.set_defaults(func=cmd_validate)

    parser_sync = subparsers.add_parser(
        "sync",
        help="Sync plugin JSON point fields from BasicTypes.hpp and Area.hpp",
    )
    parser_sync.add_argument("--input", required=True, help="Plugin JSON path")
    parser_sync.add_argument("--output", default="", help="Output JSON path; defaults to overwriting input")
    parser_sync.add_argument(
        "--preserve-existing-coords",
        action="store_true",
        help="Only add/rename point fields; do not refresh coordinates from Area.hpp",
    )
    add_catalog_args(parser_sync)
    parser_sync.set_defaults(func=cmd_sync)

    parser_emit = subparsers.add_parser("emit-area", help="Emit Area.hpp Location lines")
    parser_emit.add_argument("--input", required=True, help="Plugin JSON path")
    add_catalog_args(parser_emit)
    parser_emit.set_defaults(func=cmd_emit_area)
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
