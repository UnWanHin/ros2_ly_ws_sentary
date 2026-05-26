from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .control_bus import command_name
from .field import FieldGeometry, relative_side_to_field_side


PointCm = tuple[float, float]

UNIT_TYPES: dict[int, tuple[str, int]] = {
    1: ("Hero", 200),
    2: ("Engineer", 250),
    3: ("Infantry1", 200),
    4: ("Infantry2", 200),
    5: ("Infantry3", 200),
    7: ("Sentry", 400),
}

UNIT_NAME_TO_ID = {name.lower(): unit_id for unit_id, (name, _) in UNIT_TYPES.items()}
HEALTH_FIELDS = {
    1: "hero",
    2: "engineer",
    3: "infantry1",
    4: "infantry2",
    5: "reserve",
    7: "sentry",
}
ALL_HEALTH_FIELDS = ("hero", "engineer", "infantry1", "infantry2", "reserve", "sentry")

SIM_UNIT_DEFAULTS = [
    {"side": "friend", "type_id": 1, "type": "Hero", "hp": 200, "max_hp": 200},
    {"side": "friend", "type_id": 2, "type": "Engineer", "hp": 250, "max_hp": 250},
    {"side": "friend", "type_id": 3, "type": "Infantry1", "hp": 200, "max_hp": 200},
    {"side": "friend", "type_id": 4, "type": "Infantry2", "hp": 200, "max_hp": 200},
    {"side": "friend", "type_id": 7, "type": "Sentry", "hp": 400, "max_hp": 400},
    {"side": "enemy", "type_id": 1, "type": "Hero", "hp": 200, "max_hp": 200},
    {"side": "enemy", "type_id": 2, "type": "Engineer", "hp": 250, "max_hp": 250},
    {"side": "enemy", "type_id": 3, "type": "Infantry1", "hp": 200, "max_hp": 200},
    {"side": "enemy", "type_id": 4, "type": "Infantry2", "hp": 200, "max_hp": 200},
    {"side": "enemy", "type_id": 7, "type": "Sentry", "hp": 400, "max_hp": 400},
]

SIM_STRUCTURE_DEFAULTS = [
    {"key": "enemy_outpost", "label": "Enemy Outpost", "side": "enemy", "structure": "outpost", "hp": 60, "max_hp": 60, "step": 10},
    {"key": "enemy_base", "label": "Enemy Base", "side": "enemy", "structure": "base", "hp": 5000, "max_hp": 5000, "step": 500},
    {"key": "friend_outpost", "label": "Friend Outpost", "side": "friend", "structure": "outpost", "hp": 60, "max_hp": 60, "step": 10},
    {"key": "friend_base", "label": "Friend Base", "side": "friend", "structure": "base", "hp": 5000, "max_hp": 5000, "step": 500},
]

DEFAULT_STRUCTURE_POSITIONS: dict[str, dict[str, PointCm]] = {
    "outpost": {
        "red": (1090.0, 370.0),
        "blue": (1707.0, 1141.0),
    },
    "base": {
        "red": (245.0, 750.0),
        "blue": (2555.0, 750.0),
    },
}


def as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def as_list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def parse_position(value: Any) -> PointCm | None:
    if isinstance(value, dict):
        raw_x = value.get("x")
        raw_y = value.get("y")
    elif isinstance(value, (list, tuple)) and len(value) >= 2:
        raw_x = value[0]
        raw_y = value[1]
    else:
        return None
    try:
        return (float(raw_x), float(raw_y))
    except (TypeError, ValueError):
        return None


def clamp_int(value: Any, low: int, high: int, default: int) -> int:
    try:
        parsed = int(round(float(value)))
    except (TypeError, ValueError):
        return default
    return max(low, min(high, parsed))


def normalize_side(value: Any) -> str | None:
    text = str(value).strip().lower()
    if text in {"friend", "self", "ally", "our", "me"}:
        return "friend"
    if text in {"enemy", "opponent"}:
        return "enemy"
    return None


def normalize_structure(value: Any) -> str | None:
    text = str(value).strip().lower()
    if text in {"outpost", "op", "op_hp"}:
        return "outpost"
    if text in {"base", "base_hp"}:
        return "base"
    return None


def unit_type_id(payload: dict[str, Any]) -> int | None:
    raw_type_id = payload.get("type_id", payload.get("unit_type_id", payload.get("id")))
    if raw_type_id is not None:
        try:
            type_id = int(raw_type_id)
        except (TypeError, ValueError):
            type_id = 0
        type_id %= 100
        if type_id in UNIT_TYPES:
            return type_id
    raw_name = str(payload.get("type", payload.get("unit", payload.get("name", "")))).strip().lower()
    return UNIT_NAME_TO_ID.get(raw_name)


def default_unit_hp(type_id: int) -> int:
    return UNIT_TYPES.get(type_id, ("Unknown", 200))[1]


def default_unit_name(type_id: int) -> str:
    return UNIT_TYPES.get(type_id, (f"Unit{type_id}", 200))[0]


@dataclass(frozen=True)
class StructureSpec:
    key: str
    label: str
    side: str
    structure: str
    hp: int
    max_hp: int
    step: int


@dataclass(frozen=True)
class UnitSpec:
    side: str
    type_id: int
    type_name: str
    hp: int
    max_hp: int

    def to_command_payload(self, x: int, y: int, hp: int | None = None) -> dict[str, Any]:
        return {
            "side": self.side,
            "type_id": self.type_id,
            "type": self.type_name,
            "hp": self.hp if hp is None else hp,
            "max_hp": self.max_hp,
            "x": x,
            "y": y,
        }


@dataclass(frozen=True)
class UnitState(UnitSpec):
    x: int
    y: int

    @property
    def key(self) -> tuple[str, int]:
        return (self.side, self.type_id)

    def to_command_payload(
        self,
        x: int | None = None,
        y: int | None = None,
        hp: int | None = None,
    ) -> dict[str, Any]:
        return {
            "side": self.side,
            "type_id": self.type_id,
            "type": self.type_name,
            "hp": self.hp if hp is None else hp,
            "max_hp": self.max_hp,
            "x": self.x if x is None else x,
            "y": self.y if y is None else y,
        }


@dataclass(frozen=True)
class PositionDataRow:
    side: str
    type_id: int
    raw_x: int
    raw_y: int

    @property
    def friend_car_id(self) -> int:
        return self.type_id if self.side == "friend" else 0

    @property
    def enemy_car_id(self) -> int:
        return 100 + self.type_id if self.side == "enemy" else 0


class SimulatorInputState:
    def __init__(
        self,
        structures: list[StructureSpec],
        unit_palette: list[UnitSpec],
        field: FieldGeometry | None = None,
        structure_positions: dict[str, dict[str, PointCm]] | None = None,
        structure_health_overrides: dict[tuple[str, str], int] | None = None,
    ) -> None:
        self.field = field or FieldGeometry()
        self.structures = structures
        self.unit_palette = unit_palette
        self.structure_positions = structure_positions or DEFAULT_STRUCTURE_POSITIONS
        self.structure_health: dict[str, int] = {}
        overrides = structure_health_overrides or {}
        for item in self.structures:
            override = overrides.get((item.side, item.structure))
            hp = item.hp if override is None else override
            self.structure_health[item.key] = clamp_int(hp, 0, item.max_hp, item.hp)
        self.units: dict[tuple[str, int], UnitState] = {}

    @classmethod
    def from_config(
        cls,
        simulator_inputs: dict[str, Any],
        field: FieldGeometry | None = None,
        structure_health_overrides: dict[tuple[str, str], int] | None = None,
    ) -> "SimulatorInputState":
        config = as_dict(simulator_inputs)
        return cls(
            structures=parse_structure_specs(config),
            unit_palette=parse_unit_palette(config),
            field=field,
            structure_positions=parse_structure_positions(config),
            structure_health_overrides=structure_health_overrides,
        )

    @classmethod
    def with_defaults(
        cls,
        field: FieldGeometry | None = None,
        structure_health_overrides: dict[tuple[str, str], int] | None = None,
    ) -> "SimulatorInputState":
        return cls.from_config({}, field=field, structure_health_overrides=structure_health_overrides)

    def find_structure(self, side: str, structure: str) -> StructureSpec | None:
        for item in self.structures:
            if item.side == side and item.structure == structure:
                return item
        return None

    def structure_hp(self, side: str, structure: str) -> int:
        item = self.find_structure(side, structure)
        if item is None:
            return 0
        return int(self.structure_health.get(item.key, item.hp))

    def structure_position(
        self,
        item: StructureSpec,
        team: str,
        goals: dict[int, dict[str, Any]],
    ) -> PointCm | None:
        field_side = relative_side_to_field_side(item.side, team)
        configured = self.structure_positions.get(item.structure, {}).get(field_side)
        if configured is not None:
            return configured
        if item.structure == "base":
            return parse_position(as_dict(goals.get(1)).get(field_side))
        return DEFAULT_STRUCTURE_POSITIONS.get(item.structure, {}).get(field_side)

    def apply_control_payload(self, payload: dict[str, Any]) -> bool:
        return self.apply_command(command_name(payload), payload)

    def apply_command(self, command: str, payload: dict[str, Any]) -> bool:
        cmd = str(command).strip().lower()
        body = payload or {}
        if cmd in {"set_structure_health", "set_structure_hp"}:
            side = normalize_side(body.get("side"))
            structure = normalize_structure(body.get("structure", body.get("kind")))
            if side is None or structure is None:
                return False
            item = self.find_structure(side, structure)
            if item is None:
                return False
            current = self.structure_health.get(item.key, item.hp)
            hp = clamp_int(body.get("hp", body.get("health")), 0, item.max_hp, current)
            self.structure_health[item.key] = hp
            return True

        if cmd == "set_unit":
            side = normalize_side(body.get("side"))
            type_id = unit_type_id(body)
            if side is None or type_id is None:
                return False
            max_hp = clamp_int(body.get("max_hp"), 1, 65535, default_unit_hp(type_id))
            hp = clamp_int(body.get("hp", body.get("health")), 0, max_hp, max_hp)
            x = self.field.clamp_x(body.get("x"))
            y = self.field.clamp_y(body.get("y"))
            self.units[(side, type_id)] = UnitState(
                side=side,
                type_id=type_id,
                type_name=str(body.get("type", default_unit_name(type_id))),
                hp=hp,
                max_hp=max_hp,
                x=x,
                y=y,
            )
            return True

        if cmd == "set_unit_hp":
            side = normalize_side(body.get("side"))
            type_id = unit_type_id(body)
            if side is None or type_id is None:
                return False
            unit = self.units.get((side, type_id))
            if unit is None:
                return False
            hp = clamp_int(body.get("hp", body.get("health")), 0, unit.max_hp, unit.hp)
            self.units[(side, type_id)] = UnitState(
                side=unit.side,
                type_id=unit.type_id,
                type_name=unit.type_name,
                hp=hp,
                max_hp=unit.max_hp,
                x=unit.x,
                y=unit.y,
            )
            return True

        if cmd == "remove_unit":
            side = normalize_side(body.get("side"))
            type_id = unit_type_id(body)
            if side is None or type_id is None:
                return False
            self.units.pop((side, type_id), None)
            return True

        if cmd == "clear_units":
            self.units.clear()
            return True

        return False

    def health_fields(self, side: str, base_hp: int) -> dict[str, int]:
        fields = {field: base_hp for field in ALL_HEALTH_FIELDS}
        for unit in self.units.values():
            if unit.side != side:
                continue
            field = HEALTH_FIELDS.get(unit.type_id)
            if field is not None:
                fields[field] = unit.hp
        return fields

    def position_rows(self) -> list[PositionDataRow]:
        rows: list[PositionDataRow] = []
        for unit in self.units.values():
            rows.append(
                PositionDataRow(
                    side=unit.side,
                    type_id=unit.type_id,
                    raw_x=self.field.clamp_x(unit.x),
                    raw_y=self.field.position_data_raw_y(unit.y),
                )
            )
        return rows


def parse_structure_specs(config: dict[str, Any]) -> list[StructureSpec]:
    raw_items = as_list(config.get("structures"))
    if not raw_items:
        raw_items = SIM_STRUCTURE_DEFAULTS
    out: list[StructureSpec] = []
    for raw in raw_items:
        item = as_dict(raw)
        side = normalize_side(item.get("side"))
        structure = normalize_structure(item.get("structure", item.get("kind")))
        if side is None or structure is None:
            continue
        max_hp = max(1, clamp_int(item.get("max_hp"), 1, 65535, 1))
        hp = clamp_int(item.get("hp"), 0, max_hp, max_hp)
        step = max(1, clamp_int(item.get("step"), 1, max_hp, max(1, max_hp // 6)))
        key = str(item.get("key", f"{side}_{structure}")).strip() or f"{side}_{structure}"
        out.append(
            StructureSpec(
                key=key,
                label=str(item.get("label", key)),
                side=side,
                structure=structure,
                hp=hp,
                max_hp=max_hp,
                step=step,
            )
        )
    return out


def parse_unit_palette(config: dict[str, Any]) -> list[UnitSpec]:
    raw_items = as_list(config.get("unit_palette"))
    if not raw_items:
        raw_items = SIM_UNIT_DEFAULTS
    out: list[UnitSpec] = []
    for raw in raw_items:
        item = as_dict(raw)
        side = normalize_side(item.get("side"))
        if side is None:
            continue
        type_id = unit_type_id(item)
        if type_id is None:
            continue
        max_hp = max(1, clamp_int(item.get("max_hp"), 1, 65535, default_unit_hp(type_id)))
        hp = clamp_int(item.get("hp"), 1, max_hp, max_hp)
        out.append(
            UnitSpec(
                side=side,
                type_id=type_id,
                type_name=str(item.get("type", item.get("name", default_unit_name(type_id)))),
                hp=hp,
                max_hp=max_hp,
            )
        )
    return out


def parse_structure_positions(config: dict[str, Any]) -> dict[str, dict[str, PointCm]]:
    out = {kind: dict(sides) for kind, sides in DEFAULT_STRUCTURE_POSITIONS.items()}
    raw_root = as_dict(config.get("structure_positions"))
    for structure, raw_sides in raw_root.items():
        normalized_structure = normalize_structure(structure)
        if normalized_structure is None:
            continue
        side_positions = out.setdefault(normalized_structure, {})
        for side, raw_pos in as_dict(raw_sides).items():
            side_text = str(side).strip().lower()
            if side_text not in ("red", "blue"):
                continue
            pos = parse_position(raw_pos)
            if pos is not None:
                side_positions[side_text] = pos
    return out
