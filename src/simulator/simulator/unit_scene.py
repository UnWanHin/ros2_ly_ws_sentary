from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .assets import load_unit_asset_catalog
from .config import load_config, package_share, resolve_path
from .field import FieldGeometry
from .interactive_inputs import SimulatorInputState, load_unit_scene_file


SOURCE_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SAMPLE_SCENE_GLOBS = ("*.json", "*.yaml", "*.yml")


def as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


@dataclass(frozen=True)
class UnitSceneSample:
    name: str
    path: Path
    description: str
    unit_count: int


def sample_scene_roots() -> list[Path]:
    roots = [
        SOURCE_PACKAGE_ROOT / "sample",
        SOURCE_PACKAGE_ROOT / "sample" / "unit_scenes",
    ]
    share = package_share()
    if share is not None:
        roots.extend([share / "sample", share / "sample" / "unit_scenes"])
    seen: set[Path] = set()
    out: list[Path] = []
    for root in roots:
        resolved = root.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        out.append(root)
    return out


def iter_sample_scene_paths() -> list[Path]:
    paths: list[Path] = []
    for root in sample_scene_roots():
        if not root.exists():
            continue
        for pattern in SAMPLE_SCENE_GLOBS:
            paths.extend(root.glob(pattern))
    return sorted({path.resolve() for path in paths if path.is_file() and path.name != "manifest.json"})


def read_scene_description(path: Path) -> str:
    try:
        with path.open("r", encoding="utf-8") as stream:
            if path.suffix.lower() in {".yaml", ".yml"}:
                import yaml

                raw = yaml.safe_load(stream) or {}
            else:
                raw = json.load(stream)
    except Exception:
        return ""
    if isinstance(raw, dict):
        return str(raw.get("description", "")).strip()
    return ""


def load_unit_scene_sample(path: Path) -> UnitSceneSample:
    scene_path = path.expanduser().resolve()
    units = load_unit_scene_file(scene_path)
    return UnitSceneSample(
        name=scene_path.name,
        path=scene_path,
        description=read_scene_description(scene_path),
        unit_count=len(units),
    )


def iter_unit_scene_samples() -> list[UnitSceneSample]:
    return [load_unit_scene_sample(path) for path in iter_sample_scene_paths()]


def build_scene_summary(scene_path: Path, team: str = "red") -> dict[str, Any]:
    config = load_config(None)
    field = FieldGeometry.from_config(config.get("field_cm"))
    simulator_inputs_cfg = dict(as_dict(config.get("simulator_inputs")))
    state = SimulatorInputState.from_config(simulator_inputs_cfg, field=field)
    units = load_unit_scene_file(scene_path)
    state.apply_unit_scene(units, clear=True)
    snapshot = state.snapshot(team=team, goals={})
    catalog = load_unit_asset_catalog(as_dict(config.get("assets")))

    enriched_units: list[dict[str, Any]] = []
    for unit in snapshot.get("units", []):
        item = dict(as_dict(unit))
        field_side = str(item.get("field_side", ""))
        type_name = str(item.get("type", ""))
        sprite = catalog.path_for(field_side, type_name)
        item["asset"] = {
            "field_side": field_side,
            "type": type_name,
            "sprite": sprite.as_posix() if sprite is not None else None,
            "sprite_available": sprite is not None,
        }
        enriched_units.append(item)

    health_fields = {
        "friend": state.health_fields("friend", state.self_health),
        "enemy": state.health_fields("enemy", 400),
    }
    return {
        "schema": "ly_simulator_unit_scene_summary_v1",
        "scene": scene_path.as_posix(),
        "team": snapshot.get("team", team),
        "summary": snapshot.get("summary", {}),
        "runtime": snapshot.get("runtime", {}),
        "health_fields": health_fields,
        "units": enriched_units,
    }


def print_unit_scene_samples() -> int:
    samples = iter_unit_scene_samples()
    if not samples:
        print("No unit scene samples found.")
        return 0
    print("Unit scene samples:")
    for sample in samples:
        print(f"  {sample.name}: {sample.unit_count} units")
        print(f"    path: {sample.path}")
        if sample.description:
            print(f"    description: {sample.description}")
    return 0


def print_scene_summary(summary: dict[str, Any]) -> int:
    scene_name = Path(str(summary.get("scene", ""))).name
    counts = as_dict(summary.get("summary"))
    print(f"Unit scene: {scene_name}")
    print(f"Team mapping: friend={summary.get('team')} enemy={'blue' if summary.get('team') == 'red' else 'red'}")
    print(
        "Units: "
        f"friend={counts.get('friend_units', 0)} "
        f"enemy={counts.get('enemy_units', 0)} "
        f"low_hp={len(counts.get('low_hp_units', []))}"
    )
    for unit in summary.get("units", []):
        item = as_dict(unit)
        pos = as_dict(item.get("position_cm"))
        position_data = as_dict(item.get("position_data"))
        asset = as_dict(item.get("asset"))
        health_field = item.get("health_field")
        health_text = str(health_field) if health_field is not None else "visual-only"
        decision_text = str(item.get("decision_summary") or "").strip() or "visual"
        sprite_text = "yes" if asset.get("sprite_available") else "fallback"
        print(
            "  "
            f"{item.get('side')}->{item.get('field_side')} "
            f"{item.get('type')}#{item.get('type_id')} "
            f"hp={item.get('hp')}/{item.get('max_hp')} "
            f"field=({pos.get('x')},{pos.get('y')}) "
            f"PositionData(car_id={position_data.get('car_id')},raw_y={position_data.get('raw_y')}) "
            f"Health={health_text} "
            f"decision={decision_text} "
            f"sprite={sprite_text}"
        )
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Inspect simulator unit-scene samples and offline mock input effects.")
    parser.add_argument("scene", nargs="?", help="JSON/YAML unit scene to inspect.")
    parser.add_argument("--team", choices=("red", "blue"), default="red", help="Friend team used for field-side mapping.")
    parser.add_argument("--list-samples", action="store_true", help="List bundled unit scene examples and exit.")
    parser.add_argument("--json", action="store_true", help="Print machine-readable scene summary JSON.")
    args = parser.parse_args(argv)
    if not args.list_samples and not str(args.scene or "").strip():
        parser.error("scene is required unless --list-samples is used")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.list_samples:
        try:
            return print_unit_scene_samples()
        except (OSError, RuntimeError, ValueError) as exc:
            print(f"failed to list unit scene samples: {exc}")
            return 2

    scene_path = resolve_path(args.scene).resolve()
    try:
        summary = build_scene_summary(scene_path, team=args.team)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"failed to load unit scene {scene_path}: {exc}")
        return 2
    if args.json:
        print(json.dumps(summary, ensure_ascii=True, indent=2, sort_keys=True))
        return 0
    return print_scene_summary(summary)


if __name__ == "__main__":
    raise SystemExit(main())
