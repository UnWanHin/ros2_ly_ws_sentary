from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
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
    6: ("Drone", 150),
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
BT_HEALTH_TYPE_IDS = frozenset({1, 2, 3, 4, 7})
BT_UNIT_INFO_TYPE_IDS = frozenset({1, 2, 3, 4, 7})

SIM_UNIT_DEFAULTS = [
    {"side": "friend", "type_id": 1, "type": "Hero", "hp": 200, "max_hp": 200},
    {"side": "friend", "type_id": 2, "type": "Engineer", "hp": 250, "max_hp": 250},
    {"side": "friend", "type_id": 3, "type": "Infantry1", "hp": 200, "max_hp": 200},
    {"side": "friend", "type_id": 4, "type": "Infantry2", "hp": 200, "max_hp": 200},
    {"side": "friend", "type_id": 5, "type": "Infantry3", "hp": 200, "max_hp": 200},
    {"side": "friend", "type_id": 6, "type": "Drone", "hp": 150, "max_hp": 150},
    {"side": "friend", "type_id": 7, "type": "Sentry", "hp": 400, "max_hp": 400},
    {"side": "enemy", "type_id": 1, "type": "Hero", "hp": 200, "max_hp": 200},
    {"side": "enemy", "type_id": 2, "type": "Engineer", "hp": 250, "max_hp": 250},
    {"side": "enemy", "type_id": 3, "type": "Infantry1", "hp": 200, "max_hp": 200},
    {"side": "enemy", "type_id": 4, "type": "Infantry2", "hp": 200, "max_hp": 200},
    {"side": "enemy", "type_id": 5, "type": "Infantry3", "hp": 200, "max_hp": 200},
    {"side": "enemy", "type_id": 6, "type": "Drone", "hp": 150, "max_hp": 150},
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


def point_payload(position: PointCm | None) -> dict[str, float] | None:
    if position is None:
        return None
    return {"x": float(position[0]), "y": float(position[1])}


def hp_ratio(hp: int, max_hp: int) -> float:
    return round(max(0.0, min(1.0, int(hp) / max(1, int(max_hp)))), 4)


def unit_decision_channels(side: str, type_id: int) -> dict[str, Any]:
    normalized = normalize_side(side) or str(side).strip().lower()
    health_field = HEALTH_FIELDS.get(type_id)
    health_consumed = type_id in BT_HEALTH_TYPE_IDS
    unit_info = type_id in BT_UNIT_INFO_TYPE_IDS
    position_published = type_id in UNIT_TYPES
    return {
        "health_topic": f"/ly/{normalized}/hp" if normalized in {"friend", "enemy"} else "",
        "health_field": health_field,
        "health_published": health_field is not None,
        "health_consumed_by_bt": health_consumed,
        "position_topic": "/ly/position/data" if position_published else "",
        "position_data_published": position_published,
        "position_consumed_by_bt": unit_info,
        "unit_info_emitted_by_bt": unit_info,
        "target_selection_consumed_by_bt": normalized == "enemy" and health_consumed,
        "regional_defense_position_used_by_bt": normalized == "enemy" and unit_info,
        "visual_piece": True,
    }


def unit_decision_badges(side: str, type_id: int) -> list[str]:
    channels = unit_decision_channels(side, type_id)
    badges: list[str] = []
    if channels["health_consumed_by_bt"]:
        badges.append("BT-HP")
    elif channels["health_published"]:
        badges.append("HP-pub")
    if channels["position_consumed_by_bt"]:
        badges.append("BT-POS")
    elif channels["position_data_published"]:
        badges.append("POS-pub")
    badges.append("UnitInfo" if channels["unit_info_emitted_by_bt"] else "NoUnitInfo")
    return badges


def unit_decision_summary(side: str, type_id: int) -> str:
    channels = unit_decision_channels(side, type_id)
    bt_parts: list[str] = []
    pub_parts: list[str] = []
    if channels["health_consumed_by_bt"]:
        bt_parts.append("HP")
    elif channels["health_published"]:
        pub_parts.append("HP")
    if channels["position_consumed_by_bt"]:
        bt_parts.append("POS")
    elif channels["position_data_published"]:
        pub_parts.append("POS")
    if channels["unit_info_emitted_by_bt"]:
        bt_parts.append("UI")
    parts: list[str] = []
    if bt_parts:
        parts.append("BT:" + ",".join(bt_parts))
    if pub_parts:
        parts.append("PUB:" + ",".join(pub_parts))
    if not channels["unit_info_emitted_by_bt"]:
        parts.append("noUI")
    return " ".join(parts) if parts else "visual"


def unit_type_id(payload: dict[str, Any]) -> int | None:
    raw_type_id = payload.get("type_id", payload.get("unit_type_id", payload.get("id", payload.get("car_id"))))
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


def normalize_bool(value: Any, default: bool) -> bool:
    if value is None:
        return default
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y", "on"}:
        return True
    if text in {"0", "false", "no", "n", "off"}:
        return False
    return default


def unit_scene_payload(raw: dict[str, Any], fallback_side: str | None = None) -> dict[str, Any] | None:
    side = normalize_side(raw.get("side", fallback_side))
    type_id = unit_type_id(raw)
    position = (
        parse_position(raw.get("position_cm"))
        or parse_position(raw.get("position"))
        or parse_position(raw.get("pos"))
        or parse_position(raw)
    )
    if side is None or type_id is None or position is None:
        return None

    max_hp = max(1, clamp_int(raw.get("max_hp"), 1, 65535, default_unit_hp(type_id)))
    hp = clamp_int(raw.get("hp", raw.get("health")), 0, max_hp, max_hp)
    return {
        "side": side,
        "type_id": type_id,
        "type": str(raw.get("type", raw.get("name", default_unit_name(type_id)))),
        "hp": hp,
        "max_hp": max_hp,
        "x": position[0],
        "y": position[1],
    }


def unit_scene_items(scene: Any) -> list[dict[str, Any]]:
    root = scene.get("units", scene) if isinstance(scene, dict) else scene
    raw_items: list[tuple[Any, str | None]] = []
    if isinstance(root, dict):
        for side in ("friend", "enemy"):
            for item in as_list(root.get(side)):
                raw_items.append((item, side))
        if any(key in root for key in ("side", "type", "type_id", "unit_type_id", "id", "car_id")):
            raw_items.append((root, None))
    else:
        for item in as_list(root):
            raw_items.append((item, None))

    out: list[dict[str, Any]] = []
    for raw, fallback_side in raw_items:
        item = as_dict(raw)
        if not item:
            continue
        payload = unit_scene_payload(item, fallback_side=fallback_side)
        if payload is not None:
            out.append(payload)
    return out


def load_unit_scene_file(path: str | Path) -> list[dict[str, Any]]:
    scene_path = Path(path).expanduser()
    with scene_path.open("r", encoding="utf-8") as stream:
        if scene_path.suffix.lower() in {".yaml", ".yml"}:
            try:
                import yaml
            except ImportError as exc:
                raise RuntimeError("YAML unit scenes require PyYAML") from exc

            try:
                scene = yaml.safe_load(stream) or {}
            except yaml.YAMLError as exc:
                raise ValueError(f"invalid YAML unit scene: {exc}") from exc
        else:
            try:
                scene = json.load(stream)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSON unit scene: {exc}") from exc
    return unit_scene_items(scene)


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
        self.self_health = 400
        self.ammo_left = 200
        self.posture = 1
        self.self_position_cm: PointCm | None = None

    @classmethod
    def from_config(
        cls,
        simulator_inputs: dict[str, Any],
        field: FieldGeometry | None = None,
        structure_health_overrides: dict[tuple[str, str], int] | None = None,
    ) -> "SimulatorInputState":
        config = as_dict(simulator_inputs)
        state = cls(
            structures=parse_structure_specs(config),
            unit_palette=parse_unit_palette(config),
            field=field,
            structure_positions=parse_structure_positions(config),
            structure_health_overrides=structure_health_overrides,
        )
        if "initial_units" in config:
            state.apply_unit_scene(config.get("initial_units"), clear=True)
        return state

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
        if cmd == "set_self_health":
            self.self_health = clamp_int(
                body.get("hp", body.get("health", body.get("self_health"))),
                0,
                65535,
                self.self_health,
            )
            return True

        if cmd == "set_ammo":
            self.ammo_left = clamp_int(
                body.get("ammo", body.get("ammo_left", body.get("count"))),
                0,
                65535,
                self.ammo_left,
            )
            return True

        if cmd == "set_posture":
            self.posture = clamp_int(
                body.get("posture", body.get("id", body.get("value"))),
                0,
                255,
                self.posture,
            )
            return True

        if cmd == "set_self_position":
            position = (
                parse_position(body.get("position_cm"))
                or parse_position(body.get("position"))
                or parse_position(body.get("pos"))
                or parse_position(body)
            )
            if position is None:
                return False
            self.self_position_cm = (
                max(1, self.field.clamp_x(position[0])),
                max(1, self.field.clamp_y(position[1])),
            )
            return True

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

        if cmd == "set_units":
            clear = normalize_bool(body.get("clear"), True)
            self.apply_unit_scene(body.get("units", body), clear=clear)
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

    def apply_unit_scene(self, scene: Any, clear: bool = True) -> int:
        items = unit_scene_items(scene)
        if clear:
            self.units.clear()
        count = 0
        for item in items:
            if self.apply_command("set_unit", item):
                count += 1
        return count

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

    def snapshot(self, team: str = "red", goals: dict[int, dict[str, Any]] | None = None) -> dict[str, Any]:
        goal_map = goals or {}
        structures: list[dict[str, Any]] = []
        destroyed_structures: list[str] = []
        for item in self.structures:
            hp = int(self.structure_health.get(item.key, item.hp))
            position = self.structure_position(item, team, goal_map)
            if hp <= 0:
                destroyed_structures.append(item.key)
            structures.append(
                {
                    "key": item.key,
                    "label": item.label,
                    "side": item.side,
                    "field_side": relative_side_to_field_side(item.side, team),
                    "structure": item.structure,
                    "hp": hp,
                    "max_hp": int(item.max_hp),
                    "hp_ratio": hp_ratio(hp, item.max_hp),
                    "position_cm": point_payload(position),
                }
            )

        unit_items: list[dict[str, Any]] = []
        counts = {"friend": 0, "enemy": 0}
        low_hp_units: list[str] = []
        for unit in sorted(self.units.values(), key=lambda item: (item.side, item.type_id)):
            counts[unit.side] = counts.get(unit.side, 0) + 1
            health_field = HEALTH_FIELDS.get(unit.type_id)
            decision_channels = unit_decision_channels(unit.side, unit.type_id)
            raw_y = self.field.position_data_raw_y(unit.y)
            car_id = unit.type_id if unit.side == "friend" else 100 + unit.type_id
            ratio = hp_ratio(unit.hp, unit.max_hp)
            if ratio <= 0.35:
                low_hp_units.append(f"{unit.side}:{unit.type_name}")
            unit_items.append(
                {
                    "side": unit.side,
                    "field_side": relative_side_to_field_side(unit.side, team),
                    "type_id": int(unit.type_id),
                    "type": unit.type_name,
                    "hp": int(unit.hp),
                    "max_hp": int(unit.max_hp),
                    "hp_ratio": ratio,
                    "health_field": health_field,
                    "health_published": health_field is not None,
                    "decision_channels": decision_channels,
                    "decision_badges": unit_decision_badges(unit.side, unit.type_id),
                    "decision_summary": unit_decision_summary(unit.side, unit.type_id),
                    "position_cm": {"x": int(unit.x), "y": int(unit.y)},
                    "position_data": {"car_id": car_id, "raw_x": int(unit.x), "raw_y": int(raw_y)},
                }
            )

        palette = [
            {
                "side": item.side,
                "type_id": int(item.type_id),
                "type": item.type_name,
                "hp": int(item.hp),
                "max_hp": int(item.max_hp),
                "health_field": HEALTH_FIELDS.get(item.type_id),
            }
            for item in self.unit_palette
        ]
        return {
            "team": team if team in {"red", "blue"} else "red",
            "summary": {
                "unit_count": len(unit_items),
                "friend_units": counts.get("friend", 0),
                "enemy_units": counts.get("enemy", 0),
                "structure_count": len(structures),
                "destroyed_structures": destroyed_structures,
                "low_hp_units": low_hp_units,
            },
            "runtime": {
                "self_health": int(self.self_health),
                "ammo_left": int(self.ammo_left),
                "posture": int(self.posture),
                "self_position_cm": point_payload(self.self_position_cm),
            },
            "structures": structures,
            "units": unit_items,
            "palette": palette,
        }


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
