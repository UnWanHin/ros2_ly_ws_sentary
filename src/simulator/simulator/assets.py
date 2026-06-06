from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .config import resolve_path


PNG_SUFFIXES = {".png"}


def as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def normalized_side(value: Any) -> str | None:
    text = str(value).strip().lower()
    return text if text in {"red", "blue"} else None


def normalized_type_key(value: Any) -> str:
    return str(value).strip().lower()


def bounded_int(value: Any, default: int, low: int, high: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return default
    return max(low, min(high, parsed))


@dataclass(frozen=True)
class UnitAssetCatalog:
    enabled: bool
    manifest_path: Path | None
    provenance: dict[str, Any]
    unit_paths: dict[tuple[str, str], Path]
    armor_paths: dict[str, Path]
    aliases: dict[str, str]
    unit_size_px: int
    trace_unit_size_px: int
    drag_unit_size_px: int

    def canonical_type_key(self, type_name: str) -> str:
        key = normalized_type_key(type_name)
        return normalized_type_key(self.aliases.get(key, key))

    def path_for(self, field_side: str, type_name: str) -> Path | None:
        if not self.enabled:
            return None
        side = normalized_side(field_side)
        if side is None:
            return None
        direct_key = (side, normalized_type_key(type_name))
        if direct_key in self.unit_paths:
            return self.unit_paths[direct_key]
        return self.unit_paths.get((side, self.canonical_type_key(type_name)))

    def armor_path_for(self, name: str) -> Path | None:
        if not self.enabled:
            return None
        return self.armor_paths.get(normalized_type_key(name))


def empty_catalog(config: dict[str, Any] | None = None) -> UnitAssetCatalog:
    cfg = as_dict(config)
    return UnitAssetCatalog(
        enabled=False,
        manifest_path=None,
        provenance={},
        unit_paths={},
        armor_paths={},
        aliases={},
        unit_size_px=bounded_int(cfg.get("unit_size_px"), 42, 12, 96),
        trace_unit_size_px=bounded_int(cfg.get("trace_unit_size_px"), 34, 10, 80),
        drag_unit_size_px=bounded_int(cfg.get("drag_unit_size_px"), 46, 12, 112),
    )


def load_unit_asset_catalog(config: dict[str, Any] | None = None) -> UnitAssetCatalog:
    cfg = as_dict(config)
    if not bool(cfg.get("enabled", True)):
        return empty_catalog(cfg)

    manifest_value = str(cfg.get("manifest", "assets/manifest.yaml")).strip()
    manifest_path = resolve_path(manifest_value)
    if not manifest_path.exists():
        return empty_catalog(cfg)

    with manifest_path.open("r", encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream) or {}
    if not isinstance(manifest, dict):
        raise ValueError(f"asset manifest root must be a mapping: {manifest_path}")
    if str(manifest.get("schema", "")) != "ly_simulator_asset_manifest_v1":
        raise ValueError(f"unsupported asset manifest schema: {manifest_path}")

    root = manifest_path.parent
    provenance = as_dict(manifest.get("source"))
    aliases = parse_aliases(as_dict(manifest.get("unit_aliases")))
    unit_paths = parse_unit_paths(root, as_dict(manifest.get("unit_assets")))
    armor_paths = parse_named_png_paths(root, as_dict(manifest.get("armor_assets")))
    return UnitAssetCatalog(
        enabled=bool(unit_paths or armor_paths),
        manifest_path=manifest_path,
        provenance=provenance,
        unit_paths=unit_paths,
        armor_paths=armor_paths,
        aliases=aliases,
        unit_size_px=bounded_int(cfg.get("unit_size_px"), 42, 12, 96),
        trace_unit_size_px=bounded_int(cfg.get("trace_unit_size_px"), 34, 10, 80),
        drag_unit_size_px=bounded_int(cfg.get("drag_unit_size_px"), 46, 12, 112),
    )


def parse_aliases(raw: dict[str, Any]) -> dict[str, str]:
    aliases: dict[str, str] = {}
    for alias, target in raw.items():
        alias_key = normalized_type_key(alias)
        target_key = normalized_type_key(target)
        if alias_key and target_key:
            aliases[alias_key] = target_key
    return aliases


def parse_unit_paths(root: Path, raw: dict[str, Any]) -> dict[tuple[str, str], Path]:
    unit_paths: dict[tuple[str, str], Path] = {}
    for raw_side, raw_units in raw.items():
        side = normalized_side(raw_side)
        if side is None:
            continue
        for raw_type, raw_path in as_dict(raw_units).items():
            type_key = normalized_type_key(raw_type)
            relative = Path(str(raw_path).strip())
            if not type_key or relative.is_absolute() or relative.suffix.lower() not in PNG_SUFFIXES:
                continue
            path = (root / relative).resolve()
            if path.exists() and path.is_file():
                unit_paths[(side, type_key)] = path
    return unit_paths


def parse_named_png_paths(root: Path, raw: dict[str, Any]) -> dict[str, Path]:
    paths: dict[str, Path] = {}
    for raw_name, raw_path in raw.items():
        key = normalized_type_key(raw_name)
        relative = Path(str(raw_path).strip())
        if not key or relative.is_absolute() or relative.suffix.lower() not in PNG_SUFFIXES:
            continue
        path = (root / relative).resolve()
        if path.exists() and path.is_file():
            paths[key] = path
    return paths
