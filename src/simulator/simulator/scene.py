from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Mapping

from .field import FieldGeometry, relative_side_to_field_side
from .tactical_catalog import SceneCatalog, StructureArchetype, UnitArchetype


OWNERSHIP_MODES = frozenset({"mock", "manual_ros"})


@dataclass(frozen=True)
class SceneUnit:
    """One placed piece whose stable identity is separate from its ROS role."""

    entity_id: str
    side: str
    unit_key: str
    hp: int
    x: int
    y: int


@dataclass(frozen=True)
class SceneCommand:
    """Canonical scene mutation. Legacy command aliases belong in the facade."""

    command: str
    entity_id: str | None = None
    side: str | None = None
    unit_key: str | None = None
    hp: int | None = None
    x: float | int | None = None
    y: float | int | None = None

    @classmethod
    def place_unit(
        cls,
        entity_id: str,
        side: str,
        unit_key: str,
        x: float | int,
        y: float | int,
        hp: int | None = None,
    ) -> "SceneCommand":
        return cls("place_unit", entity_id, side, unit_key, hp, x, y)

    @classmethod
    def set_unit_hp(cls, entity_id: str, hp: int) -> "SceneCommand":
        return cls("set_unit_hp", entity_id=entity_id, hp=hp)

    @classmethod
    def remove_unit(cls, entity_id: str) -> "SceneCommand":
        return cls("remove_unit", entity_id=entity_id)

    @classmethod
    def clear_units(cls) -> "SceneCommand":
        return cls("clear_units")

    @classmethod
    def set_structure_hp(cls, entity_id: str, hp: int) -> "SceneCommand":
        return cls("set_structure_hp", entity_id=entity_id, hp=hp)


@dataclass(frozen=True)
class SceneApplyResult:
    changed: bool
    reason: str | None = None
    entity_id: str | None = None


@dataclass(frozen=True)
class ProjectionConflict:
    kind: str
    side: str
    formal_key: str | int
    entity_ids: tuple[str, ...]


@dataclass(frozen=True)
class RosProjection:
    health: dict[str, dict[str, int]]
    positions: dict[str, dict[int, tuple[int, int]]]
    structures: dict[str, dict[str, int]]
    conflicts: tuple[ProjectionConflict, ...]

    def snapshot(self) -> dict[str, Any]:
        return {
            "health": {side: dict(values) for side, values in self.health.items()},
            "positions": {
                side: {
                    str(car_id): {"x": point[0], "y": point[1]}
                    for car_id, point in values.items()
                }
                for side, values in self.positions.items()
            },
            "structures": {side: dict(values) for side, values in self.structures.items()},
            "conflicts": [
                {
                    "kind": conflict.kind,
                    "side": conflict.side,
                    "formal_key": conflict.formal_key,
                    "entity_ids": list(conflict.entity_ids),
                }
                for conflict in self.conflicts
            ],
        }


class SceneState:
    """Catalog-backed tactical facts and the only owner of scene mutations."""

    def __init__(
        self,
        catalog: SceneCatalog,
        team: str,
        *,
        ownership_mode: str = "mock",
        field: FieldGeometry | None = None,
    ) -> None:
        self.catalog = catalog
        self.team = "blue" if str(team).strip().lower() == "blue" else "red"
        self.ownership_mode = _ownership_mode(ownership_mode)
        self.field = field or FieldGeometry(catalog.field.width_cm, catalog.field.height_cm)
        self.units: dict[str, SceneUnit] = {}
        self.structure_health = {item.key: int(item.default_hp) for item in catalog.structures}
        self.selected_entity_id: str | None = None

    @classmethod
    def from_catalog(
        cls,
        catalog: SceneCatalog,
        team: str,
        *,
        ownership_mode: str = "mock",
        field: FieldGeometry | None = None,
    ) -> "SceneState":
        return cls(catalog, team, ownership_mode=ownership_mode, field=field)

    def apply(self, command: SceneCommand) -> SceneApplyResult:
        if not isinstance(command, SceneCommand):
            return SceneApplyResult(False, "invalid_scene_command")
        if self.ownership_mode == "manual_ros":
            return SceneApplyResult(False, "manual_ros_observer_mode", command.entity_id)
        if command.command == "place_unit":
            return self._place_unit(command)
        if command.command == "set_unit_hp":
            return self._set_unit_hp(command)
        if command.command == "remove_unit":
            return self._remove_unit(command)
        if command.command == "clear_units":
            changed = bool(self.units) or self.selected_entity_id is not None
            self.units.clear()
            self.selected_entity_id = None
            return SceneApplyResult(changed, None if changed else "unchanged")
        if command.command == "set_structure_hp":
            return self._set_structure_hp(command)
        return SceneApplyResult(False, "unsupported_scene_command", command.entity_id)

    def project_ros_inputs(self) -> RosProjection:
        health_groups: dict[tuple[str, str], list[SceneUnit]] = {}
        position_groups: dict[tuple[str, int], list[SceneUnit]] = {}
        for unit in self.units.values():
            archetype = _unit(self.catalog, unit.unit_key)
            if archetype is None or not archetype.project_to_ros:
                continue
            if archetype.health_published and archetype.health_field is not None:
                health_groups.setdefault((unit.side, archetype.health_field), []).append(unit)
            car_id = self.catalog.position_car_id_for_side(archetype.key, unit.side)
            if car_id is not None:
                position_groups.setdefault((unit.side, car_id), []).append(unit)

        health = {"friend": {}, "enemy": {}}
        positions: dict[str, dict[int, tuple[int, int]]] = {"friend": {}, "enemy": {}}
        conflicts: list[ProjectionConflict] = []
        for (side, field_name), candidates in sorted(health_groups.items()):
            ordered = tuple(sorted(candidates, key=lambda item: item.entity_id))
            if len(ordered) == 1:
                health[side][field_name] = ordered[0].hp
            else:
                conflicts.append(
                    ProjectionConflict("health", side, field_name, tuple(item.entity_id for item in ordered))
                )
        for (side, car_id), candidates in sorted(position_groups.items()):
            ordered = tuple(sorted(candidates, key=lambda item: item.entity_id))
            if len(ordered) == 1:
                positions[side][car_id] = (
                    self.field.clamp_x(ordered[0].x),
                    self.field.position_data_raw_y(ordered[0].y),
                )
            else:
                conflicts.append(
                    ProjectionConflict("position", side, car_id, tuple(item.entity_id for item in ordered))
                )

        structures = {"friend": {}, "enemy": {}}
        for structure in self.catalog.structures:
            structures[structure.side][structure.kind] = self.structure_health[structure.key]
        return RosProjection(health, positions, structures, tuple(conflicts))

    def snapshot(self, team: str | None = None) -> dict[str, Any]:
        resolved_team = "blue" if str(team or self.team).strip().lower() == "blue" else "red"
        return {
            "team": resolved_team,
            "ownership_mode": self.ownership_mode,
            "field": {
                "width_cm": self.field.width,
                "height_cm": self.field.height,
                "frame": self.catalog.field.frame,
            },
            "selected_entity_id": self.selected_entity_id,
            "units": [
                {
                    "entity_id": unit.entity_id,
                    "side": unit.side,
                    "field_side": relative_side_to_field_side(unit.side, resolved_team),
                    "unit_key": unit.unit_key,
                    "hp": unit.hp,
                    "position_cm": {"x": unit.x, "y": unit.y},
                }
                for unit in sorted(self.units.values(), key=lambda item: item.entity_id)
            ],
            "structures": [
                {
                    "key": structure.key,
                    "side": structure.side,
                    "kind": structure.kind,
                    "hp": self.structure_health[structure.key],
                    "max_hp": structure.max_hp,
                }
                for structure in self.catalog.structures
            ],
            "projection": self.project_ros_inputs().snapshot(),
        }

    def _place_unit(self, command: SceneCommand) -> SceneApplyResult:
        entity_id = _entity_id(command.entity_id)
        side = _side(command.side)
        archetype = _unit(self.catalog, command.unit_key)
        if entity_id is None or side is None or archetype is None:
            return SceneApplyResult(False, "invalid_unit_command", command.entity_id)
        placed = SceneUnit(
            entity_id=entity_id,
            side=side,
            unit_key=archetype.key,
            hp=_clamp(command.hp, 0, archetype.max_hp, archetype.default_hp),
            x=self.field.clamp_x(command.x),
            y=self.field.clamp_y(command.y),
        )
        changed = self.units.get(entity_id) != placed or self.selected_entity_id != entity_id
        self.units[entity_id] = placed
        self.selected_entity_id = entity_id
        return SceneApplyResult(changed, None if changed else "unchanged", entity_id)

    def _set_unit_hp(self, command: SceneCommand) -> SceneApplyResult:
        entity_id = _entity_id(command.entity_id)
        if entity_id is None or entity_id not in self.units:
            return SceneApplyResult(False, "entity_not_found", command.entity_id)
        old = self.units[entity_id]
        archetype = _unit(self.catalog, old.unit_key)
        if archetype is None:
            return SceneApplyResult(False, "invalid_unit_command", entity_id)
        hp = _clamp(command.hp, 0, archetype.max_hp, old.hp)
        if hp == old.hp:
            return SceneApplyResult(False, "unchanged", entity_id)
        self.units[entity_id] = SceneUnit(old.entity_id, old.side, old.unit_key, hp, old.x, old.y)
        return SceneApplyResult(True, entity_id=entity_id)

    def _remove_unit(self, command: SceneCommand) -> SceneApplyResult:
        entity_id = _entity_id(command.entity_id)
        if entity_id is None or entity_id not in self.units:
            return SceneApplyResult(False, "entity_not_found", command.entity_id)
        self.units.pop(entity_id)
        if self.selected_entity_id == entity_id:
            self.selected_entity_id = None
        return SceneApplyResult(True, entity_id=entity_id)

    def _set_structure_hp(self, command: SceneCommand) -> SceneApplyResult:
        structure = _structure(self.catalog, command.entity_id)
        if structure is None:
            return SceneApplyResult(False, "unknown_structure", command.entity_id)
        hp = _clamp(command.hp, 0, structure.max_hp, self.structure_health[structure.key])
        changed = hp != self.structure_health[structure.key]
        self.structure_health[structure.key] = hp
        return SceneApplyResult(changed, None if changed else "unchanged", structure.key)


def normalize_scene_command(data: Mapping[str, Any], catalog: SceneCatalog) -> SceneCommand:
    """Validate a canonical command at the boundary before it reaches the reducer."""

    if not isinstance(data, Mapping):
        raise ValueError("scene command must be a mapping")
    command = str(data.get("command", "")).strip().lower()
    if command == "place_unit":
        entity_id = _required_entity_id(data)
        side = _side(data.get("side"))
        archetype = _unit(catalog, data.get("unit_key"))
        if side is None or archetype is None:
            raise ValueError("place_unit requires a known side and unit_key")
        x = _finite(data.get("x"))
        y = _finite(data.get("y"))
        if x is None or y is None:
            raise ValueError("place_unit requires finite x and y")
        return SceneCommand.place_unit(entity_id, side, archetype.key, x, y, _integer(data.get("hp")))
    if command == "set_unit_hp":
        return SceneCommand.set_unit_hp(_required_entity_id(data), _required_integer(data, "hp"))
    if command == "remove_unit":
        return SceneCommand.remove_unit(_required_entity_id(data))
    if command == "clear_units":
        return SceneCommand.clear_units()
    if command == "set_structure_hp":
        structure = _structure(catalog, data.get("entity_id"))
        if structure is None:
            raise ValueError("set_structure_hp requires a known entity_id")
        return SceneCommand.set_structure_hp(structure.key, _required_integer(data, "hp"))
    raise ValueError(f"unsupported canonical scene command: {command}")


def _side(value: Any) -> str | None:
    side = str(value).strip().lower()
    return side if side in {"friend", "enemy"} else None


def _unit(catalog: SceneCatalog, value: Any) -> UnitArchetype | None:
    if not isinstance(value, str):
        return None
    try:
        return catalog.unit_by_key(value)
    except (KeyError, ValueError):
        return None


def _structure(catalog: SceneCatalog, value: Any) -> StructureArchetype | None:
    entity_id = _entity_id(value)
    if entity_id is None:
        return None
    normalized = entity_id.lower().replace(":", "_")
    for structure in catalog.structures:
        if structure.key == normalized:
            return structure
    return None


def _entity_id(value: Any) -> str | None:
    return value.strip() if isinstance(value, str) and value.strip() else None


def _required_entity_id(data: Mapping[str, Any]) -> str:
    entity_id = _entity_id(data.get("entity_id"))
    if entity_id is None:
        raise ValueError("entity_id must be a non-empty string")
    return entity_id


def _finite(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _integer(value: Any) -> int | None:
    number = _finite(value)
    return None if number is None else int(round(number))


def _required_integer(data: Mapping[str, Any], key: str) -> int:
    value = _integer(data.get(key))
    if value is None:
        raise ValueError(f"{key} must be a number")
    return value


def _clamp(value: int | None, low: int, high: int, default: int) -> int:
    return max(low, min(high, default if value is None else int(value)))


def _ownership_mode(value: Any) -> str:
    mode = str(value).strip().lower()
    if mode not in OWNERSHIP_MODES:
        raise ValueError(f"ownership_mode must be one of {sorted(OWNERSHIP_MODES)}")
    return mode
