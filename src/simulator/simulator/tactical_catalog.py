from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping

import yaml

from .config import SOURCE_PACKAGE_ROOT, package_share


CATALOG_SCHEMA = "ly_simulator_tactical_catalog_v1"
PointCm = tuple[float, float]


@dataclass(frozen=True)
class FieldSpec:
    width_cm: int
    height_cm: int
    frame: str


@dataclass(frozen=True)
class VisualTokens:
    asset_manifest: str
    unit_scale_px: int
    trace_unit_scale_px: int
    drag_unit_scale_px: int
    health_bar_height_px: int
    label_rule: str
    team_colors: Mapping[str, str]


@dataclass(frozen=True)
class PositionDataProjection:
    """The formal relative-side car-ID encoding used by /ly/position/data."""

    friend_car_id_offset: int
    enemy_car_id_offset: int

    def car_id_for_side(self, base_car_id: int, side: str) -> int:
        relative_side = _relative_side(side, "position data side")
        offset = self.friend_car_id_offset if relative_side == "friend" else self.enemy_car_id_offset
        return base_car_id + offset


@dataclass(frozen=True)
class UnitArchetype:
    key: str
    label: str
    asset_key: str
    default_hp: int
    max_hp: int
    health_field: str | None
    position_car_id: int | None
    project_to_ros: bool
    decision_consumed: bool

    @property
    def health_published(self) -> bool:
        return self.project_to_ros and self.health_field is not None

    @property
    def position_published(self) -> bool:
        return self.project_to_ros and self.position_car_id is not None


@dataclass(frozen=True)
class StructureArchetype:
    key: str
    label: str
    side: str
    kind: str
    default_hp: int
    max_hp: int
    step: int
    health_topic: str
    red_position_cm: PointCm
    blue_position_cm: PointCm

    @property
    def structure(self) -> str:
        """Compatibility name for the existing simulator structure vocabulary."""
        return self.kind

    def position_for_field_side(self, field_side: str) -> PointCm:
        side = _field_side(field_side, f"structure {self.key} field side")
        return self.red_position_cm if side == "red" else self.blue_position_cm


@dataclass(frozen=True)
class GoalMarker:
    goal_id: int
    key: str
    label: str
    category: str
    marker_style: str
    red_position_cm: PointCm
    blue_position_cm: PointCm

    def position_for_field_side(self, field_side: str) -> PointCm:
        side = _field_side(field_side, f"goal {self.key} field side")
        return self.red_position_cm if side == "red" else self.blue_position_cm


@dataclass(frozen=True)
class SceneCatalog:
    schema: str
    field: FieldSpec
    visual: VisualTokens
    position_data: PositionDataProjection
    units: tuple[UnitArchetype, ...]
    structures: tuple[StructureArchetype, ...]
    goals: tuple[GoalMarker, ...]
    _units_by_key: Mapping[str, UnitArchetype] = field(repr=False, compare=False)
    _structures_by_key: Mapping[str, StructureArchetype] = field(repr=False, compare=False)
    _goals_by_id: Mapping[int, GoalMarker] = field(repr=False, compare=False)
    _goals_by_key: Mapping[str, GoalMarker] = field(repr=False, compare=False)

    @classmethod
    def load(cls, path: Path) -> "SceneCatalog":
        catalog_path = Path(path).expanduser()
        try:
            with catalog_path.open("r", encoding="utf-8") as stream:
                raw = yaml.safe_load(stream)
        except OSError as exc:
            raise ValueError(f"unable to read tactical catalog {catalog_path}: {exc}") from exc
        except yaml.YAMLError as exc:
            raise ValueError(f"invalid tactical catalog YAML {catalog_path}: {exc}") from exc
        return cls.from_mapping(_mapping(raw, "tactical catalog root"))

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any]) -> "SceneCatalog":
        root = _mapping(raw, "tactical catalog root")
        schema = _required_text(root, "schema", "tactical catalog")
        if schema != CATALOG_SCHEMA:
            raise ValueError(f"unsupported tactical catalog schema: {schema}")

        field_spec = _parse_field(_mapping(root.get("field"), "catalog field"))
        visual = _parse_visual(_mapping(root.get("visual"), "catalog visual"))
        position_data = _parse_position_data_projection(
            _mapping(root.get("ros_projection"), "catalog ros_projection")
        )
        units = _parse_units(_items(root.get("units"), "catalog units"))
        structures = _parse_structures(_items(root.get("structures"), "catalog structures"), field_spec)
        goals = _parse_goals(_items(root.get("goals"), "catalog goals"), field_spec)

        return cls(
            schema=schema,
            field=field_spec,
            visual=visual,
            position_data=position_data,
            units=units,
            structures=structures,
            goals=goals,
            _units_by_key=_frozen_index(units, lambda item: item.key),
            _structures_by_key=_frozen_index(structures, lambda item: item.key),
            _goals_by_id=_frozen_index(goals, lambda item: item.goal_id),
            _goals_by_key=_frozen_index(goals, lambda item: item.key),
        )

    def unit_by_key(self, key: str) -> UnitArchetype:
        return _lookup(self._units_by_key, _canonical_key(key, "unit key"), "unit")

    def structure_by_key(self, key: str) -> StructureArchetype:
        return _lookup(self._structures_by_key, _canonical_key(key, "structure key"), "structure")

    def position_car_id_for_side(self, unit_key: str, side: str) -> int | None:
        unit = self.unit_by_key(unit_key)
        if not unit.position_published:
            return None
        assert unit.position_car_id is not None
        return self.position_data.car_id_for_side(unit.position_car_id, side)

    def goal_by_id(self, goal_id: int) -> GoalMarker:
        if isinstance(goal_id, bool) or not isinstance(goal_id, int):
            raise KeyError(f"unknown goal id: {goal_id!r}")
        return _lookup(self._goals_by_id, goal_id, "goal")

    def goal_by_key(self, key: str) -> GoalMarker:
        return _lookup(self._goals_by_key, _canonical_key(key, "goal key"), "goal")


def default_catalog_path() -> Path:
    source_catalog = SOURCE_PACKAGE_ROOT / "config" / "tactical_catalog.yaml"
    if source_catalog.exists():
        return source_catalog
    share = package_share()
    if share is not None:
        return share / "config" / "tactical_catalog.yaml"
    return source_catalog


def validate_catalog_payload(raw: Mapping[str, Any]) -> None:
    """Raise ValueError unless ``raw`` is a complete, internally consistent catalog."""
    SceneCatalog.from_mapping(raw)


def _parse_field(raw: Mapping[str, Any]) -> FieldSpec:
    return FieldSpec(
        width_cm=_positive_int(raw.get("width_cm"), "catalog field width_cm"),
        height_cm=_positive_int(raw.get("height_cm"), "catalog field height_cm"),
        frame=_required_text(raw, "frame", "catalog field"),
    )


def _parse_visual(raw: Mapping[str, Any]) -> VisualTokens:
    team_colors_raw = _mapping(raw.get("team_colors"), "catalog visual team_colors")
    team_colors: dict[str, str] = {}
    for side in ("red", "blue"):
        team_colors[side] = _required_text(team_colors_raw, side, "catalog visual team_colors")
    return VisualTokens(
        asset_manifest=_relative_path(_required_text(raw, "asset_manifest", "catalog visual"), "asset_manifest"),
        unit_scale_px=_positive_int(raw.get("unit_scale_px"), "catalog visual unit_scale_px"),
        trace_unit_scale_px=_positive_int(raw.get("trace_unit_scale_px"), "catalog visual trace_unit_scale_px"),
        drag_unit_scale_px=_positive_int(raw.get("drag_unit_scale_px"), "catalog visual drag_unit_scale_px"),
        health_bar_height_px=_positive_int(raw.get("health_bar_height_px"), "catalog visual health_bar_height_px"),
        label_rule=_required_text(raw, "label_rule", "catalog visual"),
        team_colors=MappingProxyType(team_colors),
    )


def _parse_position_data_projection(raw: Mapping[str, Any]) -> PositionDataProjection:
    position_data = _mapping(raw.get("position_data"), "catalog ros_projection position_data")
    friend_offset = _nonnegative_int(
        position_data.get("friend_car_id_offset"),
        "catalog ros_projection position_data friend_car_id_offset",
    )
    enemy_offset = _nonnegative_int(
        position_data.get("enemy_car_id_offset"),
        "catalog ros_projection position_data enemy_car_id_offset",
    )
    if friend_offset != 0:
        raise ValueError("catalog ros_projection position_data friend_car_id_offset must be 0")
    if enemy_offset != 100:
        raise ValueError("catalog ros_projection position_data enemy_car_id_offset must be 100")
    return PositionDataProjection(
        friend_car_id_offset=friend_offset,
        enemy_car_id_offset=enemy_offset,
    )


def _parse_units(raw_items: tuple[Mapping[str, Any], ...]) -> tuple[UnitArchetype, ...]:
    if not raw_items:
        raise ValueError("catalog units must not be empty")

    units: list[UnitArchetype] = []
    seen_keys: set[str] = set()
    seen_car_ids: set[int] = set()
    seen_health_fields: set[str] = set()
    for raw in raw_items:
        key = _canonical_key(_required_text(raw, "key", "unit"), "unit key")
        if key in seen_keys:
            raise ValueError(f"duplicate unit key: {key}")
        seen_keys.add(key)

        max_hp = _positive_int(raw.get("max_hp"), f"unit {key} max_hp")
        default_hp = _nonnegative_int(raw.get("default_hp"), f"unit {key} default_hp")
        if default_hp > max_hp:
            raise ValueError(f"unit {key} default_hp must not exceed max_hp")

        health_field = _optional_text(raw, "health_field", f"unit {key}")
        position_car_id = _optional_car_id(raw, "position_car_id", f"unit {key}")
        project_to_ros = _required_bool(raw, "project_to_ros", f"unit {key}")
        decision_consumed = _required_bool(raw, "decision_consumed", f"unit {key}")
        if project_to_ros and health_field is None and position_car_id is None:
            raise ValueError(f"unit {key} projects to ROS without a formal mapping")
        if decision_consumed and not project_to_ros:
            raise ValueError(f"unit {key} is decision-consumed but not projectable to ROS")
        if position_car_id is not None:
            if position_car_id in seen_car_ids:
                raise ValueError(f"duplicate position_car_id: {position_car_id}")
            seen_car_ids.add(position_car_id)
        if health_field is not None:
            if health_field in seen_health_fields:
                raise ValueError(f"duplicate health_field: {health_field}")
            seen_health_fields.add(health_field)

        units.append(
            UnitArchetype(
                key=key,
                label=_required_text(raw, "label", f"unit {key}"),
                asset_key=_required_text(raw, "asset_key", f"unit {key}"),
                default_hp=default_hp,
                max_hp=max_hp,
                health_field=health_field,
                position_car_id=position_car_id,
                project_to_ros=project_to_ros,
                decision_consumed=decision_consumed,
            )
        )
    return tuple(units)


def _parse_structures(
    raw_items: tuple[Mapping[str, Any], ...],
    field_spec: FieldSpec,
) -> tuple[StructureArchetype, ...]:
    if not raw_items:
        raise ValueError("catalog structures must not be empty")

    structures: list[StructureArchetype] = []
    seen_keys: set[str] = set()
    seen_topics: set[str] = set()
    for raw in raw_items:
        key = _canonical_key(_required_text(raw, "key", "structure"), "structure key")
        if key in seen_keys:
            raise ValueError(f"duplicate structure key: {key}")
        seen_keys.add(key)
        side = _relative_side(_required_text(raw, "side", f"structure {key}"), f"structure {key} side")
        kind = _structure_kind(_required_text(raw, "kind", f"structure {key}"), f"structure {key} kind")
        max_hp = _positive_int(raw.get("max_hp"), f"structure {key} max_hp")
        default_hp = _nonnegative_int(raw.get("default_hp"), f"structure {key} default_hp")
        if default_hp > max_hp:
            raise ValueError(f"structure {key} default_hp must not exceed max_hp")
        topic = _topic(_required_text(raw, "health_topic", f"structure {key}"), f"structure {key} health_topic")
        if topic in seen_topics:
            raise ValueError(f"duplicate structure health_topic: {topic}")
        seen_topics.add(topic)
        positions = _team_positions(raw, f"structure {key}", field_spec)
        structures.append(
            StructureArchetype(
                key=key,
                label=_required_text(raw, "label", f"structure {key}"),
                side=side,
                kind=kind,
                default_hp=default_hp,
                max_hp=max_hp,
                step=_positive_int(raw.get("step"), f"structure {key} step"),
                health_topic=topic,
                red_position_cm=positions["red"],
                blue_position_cm=positions["blue"],
            )
        )
    return tuple(structures)


def _parse_goals(raw_items: tuple[Mapping[str, Any], ...], field_spec: FieldSpec) -> tuple[GoalMarker, ...]:
    if not raw_items:
        raise ValueError("catalog goals must not be empty")

    goals: list[GoalMarker] = []
    seen_ids: set[int] = set()
    seen_keys: set[str] = set()
    for raw in raw_items:
        goal_id = _nonnegative_int(raw.get("id"), "goal id")
        if goal_id in seen_ids:
            raise ValueError(f"duplicate goal id: {goal_id}")
        seen_ids.add(goal_id)
        key = _canonical_key(_required_text(raw, "key", f"goal {goal_id}"), "goal key")
        if key in seen_keys:
            raise ValueError(f"duplicate goal key: {key}")
        seen_keys.add(key)
        positions = _team_positions(raw, f"goal {key}", field_spec)
        goals.append(
            GoalMarker(
                goal_id=goal_id,
                key=key,
                label=_required_text(raw, "label", f"goal {key}"),
                category=_required_text(raw, "category", f"goal {key}"),
                marker_style=_required_text(raw, "marker_style", f"goal {key}"),
                red_position_cm=positions["red"],
                blue_position_cm=positions["blue"],
            )
        )
    return tuple(goals)


def _team_positions(raw: Mapping[str, Any], context: str, field_spec: FieldSpec) -> dict[str, PointCm]:
    positions = _mapping(raw.get("positions_cm"), f"{context} positions_cm")
    return {
        "red": _coordinate(positions.get("red"), f"{context} red coordinate", field_spec),
        "blue": _coordinate(positions.get("blue"), f"{context} blue coordinate", field_spec),
    }


def _coordinate(value: Any, context: str, field_spec: FieldSpec) -> PointCm:
    if isinstance(value, Mapping):
        raw_x = value.get("x")
        raw_y = value.get("y")
    elif isinstance(value, (list, tuple)) and len(value) == 2:
        raw_x, raw_y = value
    else:
        raise ValueError(f"{context} must be an [x, y] coordinate")
    x = _finite_number(raw_x, context)
    y = _finite_number(raw_y, context)
    if not 0.0 <= x <= float(field_spec.width_cm) or not 0.0 <= y <= float(field_spec.height_cm):
        raise ValueError(
            f"{context} is outside field bounds {field_spec.width_cm}x{field_spec.height_cm}"
        )
    return (x, y)


def _mapping(value: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{context} must be a mapping")
    return value


def _items(value: Any, context: str) -> tuple[Mapping[str, Any], ...]:
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"{context} must be a list")
    return tuple(_mapping(item, f"{context} entry") for item in value)


def _required_text(raw: Mapping[str, Any], name: str, context: str) -> str:
    value = raw.get(name)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{context} {name} must be a non-empty string")
    return value.strip()


def _optional_text(raw: Mapping[str, Any], name: str, context: str) -> str | None:
    if name not in raw or raw.get(name) is None:
        return None
    return _required_text(raw, name, context)


def _required_bool(raw: Mapping[str, Any], name: str, context: str) -> bool:
    value = raw.get(name)
    if not isinstance(value, bool):
        raise ValueError(f"{context} {name} must be a boolean")
    return value


def _positive_int(value: Any, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{context} must be a positive integer")
    return value


def _nonnegative_int(value: Any, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{context} must be a non-negative integer")
    return value


def _optional_car_id(raw: Mapping[str, Any], name: str, context: str) -> int | None:
    if name not in raw or raw.get(name) is None:
        return None
    value = _positive_int(raw.get(name), f"{context} {name}")
    if value > 99:
        raise ValueError(f"{context} {name} must be at most 99")
    return value


def _finite_number(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{context} must contain finite numeric x/y values")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{context} must contain finite numeric x/y values")
    return parsed


def _canonical_key(value: str, context: str) -> str:
    key = str(value).strip().lower()
    if not key:
        raise ValueError(f"{context} must be non-empty")
    return key


def _field_side(value: str, context: str) -> str:
    side = str(value).strip().lower()
    if side not in {"red", "blue"}:
        raise ValueError(f"{context} must be red or blue")
    return side


def _relative_side(value: str, context: str) -> str:
    side = str(value).strip().lower()
    if side not in {"friend", "enemy"}:
        raise ValueError(f"{context} must be friend or enemy")
    return side


def _structure_kind(value: str, context: str) -> str:
    kind = str(value).strip().lower()
    if kind not in {"base", "outpost"}:
        raise ValueError(f"{context} must be base or outpost")
    return kind


def _topic(value: str, context: str) -> str:
    topic = str(value).strip()
    if not topic.startswith("/ly/"):
        raise ValueError(f"{context} must be a /ly/ ROS topic")
    return topic


def _relative_path(value: str, context: str) -> str:
    path = Path(value)
    if path.is_absolute() or not value.strip():
        raise ValueError(f"{context} must be a non-empty relative path")
    return path.as_posix()


def _frozen_index(items: tuple[Any, ...], key_of: Any) -> Mapping[Any, Any]:
    return MappingProxyType({key_of(item): item for item in items})


def _lookup(index: Mapping[Any, Any], key: Any, label: str) -> Any:
    try:
        return index[key]
    except KeyError as exc:
        raise KeyError(f"unknown {label}: {key!r}") from exc
