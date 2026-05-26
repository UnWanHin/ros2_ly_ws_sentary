from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


SOURCE_PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def package_share() -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return None
    try:
        return Path(get_package_share_directory("simulator"))
    except Exception:
        return None


def default_config_path() -> Path:
    source_config = SOURCE_PACKAGE_ROOT / "config" / "default.yaml"
    if source_config.exists():
        return source_config
    share = package_share()
    if share is not None:
        return share / "config" / "default.yaml"
    return source_config


DEFAULT_CONFIG = default_config_path()


def repo_root() -> Path:
    candidates = [Path.cwd(), *Path(__file__).resolve().parents]
    for candidate in candidates:
        if (candidate / "tools" / "maps").exists() and (candidate / "src").exists():
            return candidate
    return Path.cwd()


def deep_update(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    out = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_update(out[key], value)
        else:
            out[key] = value
    return out


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def load_config(path: Path | None) -> dict[str, Any]:
    config = load_yaml(DEFAULT_CONFIG)
    if path is not None:
        override_path = resolve_path(path)
        config = deep_update(config, load_yaml(override_path))
    return config


def resolve_path(path: str | Path) -> Path:
    raw = Path(path).expanduser()
    if raw.is_absolute():
        return raw
    root = repo_root()
    repo_candidate = (root / raw).resolve()
    if repo_candidate.exists():
        return repo_candidate
    package_candidate = (SOURCE_PACKAGE_ROOT / raw).resolve()
    if package_candidate.exists():
        return package_candidate
    share = package_share()
    if share is not None:
        share_candidate = (share / raw).resolve()
        if share_candidate.exists():
            return share_candidate
    return repo_candidate


def point_from(value: Any) -> tuple[float, float] | None:
    if isinstance(value, dict):
        x = value.get("x")
        y = value.get("y")
    elif isinstance(value, (list, tuple)) and len(value) >= 2:
        x, y = value[0], value[1]
    else:
        return None
    try:
        return (float(x), float(y))
    except (TypeError, ValueError):
        return None


def goals_by_id(config: dict[str, Any]) -> dict[int, dict[str, Any]]:
    goals: dict[int, dict[str, Any]] = {}
    for item in config.get("goals", []):
        if not isinstance(item, dict):
            continue
        try:
            goal_id = int(item["id"])
        except (KeyError, TypeError, ValueError):
            continue
        goals[goal_id] = {
            "name": str(item.get("name", f"Goal{goal_id}")),
            "red": point_from(item.get("red")),
            "blue": point_from(item.get("blue")),
        }
    return goals


def load_plugin_points(path: Path) -> dict[int, dict[str, Any]]:
    if not path.exists():
        return {}
    try:
        data = load_yaml(path) if path.suffix in {".yaml", ".yml"} else _load_json(path)
    except (OSError, ValueError):
        return {}
    out: dict[int, dict[str, Any]] = {}
    for item in data.get("points", []):
        if not isinstance(item, dict):
            continue
        try:
            point_id = int(item["id"])
        except (KeyError, TypeError, ValueError):
            continue
        red = point_from(item.get("red"))
        blue = point_from(item.get("blue"))
        if red == (0, 0) and blue == (0, 0):
            continue
        out[point_id] = {
            "name": str(item.get("name", f"Goal{point_id}")),
            "red": red,
            "blue": blue,
        }
    return out


def _load_json(path: Path) -> dict[str, Any]:
    import json

    with path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"JSON root must be a mapping: {path}")
    return data
