from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from .model import (
    DecisionIntent,
    DecisionOutput,
    EventSnapshot,
    GimbalState,
    RelativeTarget,
    RuntimeGuard,
    TargetState,
    TraceRecord,
    UnitRecord,
)


LOCATION_COUNT = 50


def as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def as_list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def number(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        out = float(value)
        return out if math.isfinite(out) else default
    except (TypeError, ValueError):
        return default


def integer(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def optional_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in {"true", "1", "yes", "on"}:
            return True
        if text in {"false", "0", "no", "off"}:
            return False
    return None


def boolean(value: Any, default: bool = False) -> bool:
    parsed = optional_bool(value)
    return default if parsed is None else parsed


def optional_number(value: Any) -> float | None:
    out = number(value, float("nan"))
    return out if math.isfinite(out) else None


def compact_label(value: Any, default: str = "-") -> str:
    if isinstance(value, dict):
        name = value.get("name")
        ident = value.get("id")
        if name is not None and ident is not None:
            return f"{name} ({ident})"
        if name is not None:
            return str(name)
        if ident is not None:
            return str(ident)
    if value is None:
        return default
    return str(value)


def compact_label_list(value: Any) -> tuple[str, ...]:
    out: list[str] = []
    for item in as_list(value):
        out.append(compact_label(item))
    return tuple(out)


def parse_position(value: Any) -> tuple[float, float] | None:
    if isinstance(value, dict):
        x = value.get("x")
        y = value.get("y")
    elif isinstance(value, (list, tuple)) and len(value) >= 2:
        x, y = value[0], value[1]
    else:
        return None
    px = number(x, float("nan"))
    py = number(y, float("nan"))
    if not math.isfinite(px) or not math.isfinite(py):
        return None
    return (px, py)


def goal_base_id(goal_id: int) -> int:
    return goal_id - LOCATION_COUNT if goal_id >= LOCATION_COUNT else goal_id


def goal_side(goal_id: int, fallback_team: str) -> str:
    if goal_id >= LOCATION_COUNT:
        return "blue"
    if 0 <= goal_id < LOCATION_COUNT:
        return "red"
    return "blue" if fallback_team == "blue" else "red"


def normalize_unit(raw: dict[str, Any], fallback_side: str) -> UnitRecord:
    pos = parse_position(raw.get("position_cm"))
    if pos == (0, 0):
        pos = None
    return UnitRecord(
        side=str(raw.get("side", fallback_side)),
        type_name=str(raw.get("type", raw.get("name", "Unknown"))),
        type_id=integer(raw.get("type_id", raw.get("id")), 0),
        hp=integer(raw.get("hp"), 0),
        max_hp=integer(raw.get("max_hp"), 0),
        distance_m=number(raw.get("distance_m"), 0.0),
        position_cm=pos,
    )


def normalize_units(raw: dict[str, Any]) -> tuple[UnitRecord, ...]:
    units_root = as_dict(raw.get("units"))
    units: list[UnitRecord] = []
    for side in ("friend", "enemy"):
        for item in as_list(units_root.get(side)):
            if isinstance(item, dict):
                units.append(normalize_unit(item, side))
    return tuple(units)


def normalize_output(raw: dict[str, Any], navi: dict[str, Any], team: str, goal_names: dict[int, str]) -> DecisionOutput:
    output = as_dict(raw.get("decision_output"))
    flat_goal = None if isinstance(raw.get("navi_goal"), dict) else raw.get("navi_goal")
    goal_id = integer(output.get("goal_id", navi.get("id", flat_goal)), 0)
    base_id = integer(output.get("goal_base_id", navi.get("base_id")), goal_base_id(goal_id))
    side = str(output.get("goal_side", navi.get("side", goal_side(goal_id, team)))).lower()
    if side not in ("red", "blue"):
        side = goal_side(goal_id, team)

    pos = parse_position(output.get("goal_pos_cm"))
    if pos is None:
        pos = parse_position(navi.get("position_cm"))
    if pos == (0, 0):
        pos = None

    kind = str(output.get("kind", "goal_pos" if pos is not None else "goal_id"))
    uses_goal_pos = optional_bool(output.get("uses_goal_pos"))
    if uses_goal_pos is None:
        uses_goal_pos = pos is not None and kind != "goal_id"

    topic = str(output.get("output_topic", ""))
    if not topic and uses_goal_pos:
        topic = "/ly/navi/goal_pos"
    if not topic and kind == "goal_id":
        topic = "/ly/navi/goal"

    final_topic = str(output.get("final_goal_pos_topic", ""))
    if not final_topic and uses_goal_pos:
        if kind.endswith("bridge") or topic in {"/ly/navi/target_rel", "/ly/navi/goal_pos_raw"}:
            final_topic = "/goal_pose"
        else:
            final_topic = "/ly/navi/goal_pos"

    return DecisionOutput(
        kind=kind,
        goal_id=goal_id,
        goal_base_id=base_id,
        goal_name=str(output.get("goal_name", navi.get("name", goal_names.get(base_id, f"Goal{base_id}")))),
        goal_side=side,
        goal_pos_cm=pos,
        speed_level=integer(output.get("speed_level", navi.get("speed_level", raw.get("speed_level"))), 0),
        publish_enabled=optional_bool(output.get("publish_enabled", navi.get("publish_enabled"))),
        publish_allowed=optional_bool(output.get("publish_allowed", navi.get("publish_allowed"))),
        output_topic=topic,
        output_frame=str(output.get("output_frame", output.get("goal_pos_frame", "map"))),
        final_goal_pos_topic=final_topic,
        uses_goal_pos=uses_goal_pos,
        uses_to_navi=bool(output.get("uses_to_navi", output.get("uses_tf_goal_bridge", False))),
        relative_target_valid=bool(output.get("relative_target_valid", False)),
        source=str(output.get("source", "navi_goal")),
    )


def normalize_decision_intent(raw: dict[str, Any], output: DecisionOutput) -> DecisionIntent:
    intent = as_dict(raw.get("decision_intent"))
    return DecisionIntent(
        layer=str(intent.get("layer", "unknown")),
        reason=str(intent.get("reason", "unknown")),
        base_goal_id=integer(intent.get("base_goal_id"), output.goal_base_id),
        resolved_goal_id=integer(intent.get("resolved_goal_id"), output.goal_id),
        goal_team=str(intent.get("goal_team", output.goal_side)),
        apply_team_offset=optional_bool(intent.get("apply_team_offset")),
        priority=integer(intent.get("priority"), 0),
        detail=str(intent.get("detail", intent.get("reason", "-"))),
    )


def normalize_target_state(raw: dict[str, Any]) -> TargetState:
    state = as_dict(raw.get("target_state"))
    return TargetState(
        has_recent_target=boolean(state.get("has_recent_target")),
        fresh_auto_aim=boolean(state.get("fresh_auto_aim")),
        fresh_buff=boolean(state.get("fresh_buff")),
        fresh_outpost=boolean(state.get("fresh_outpost")),
        hitable_targets=compact_label_list(state.get("hitable_targets")),
        reliable_enemy_positions=compact_label_list(state.get("reliable_enemy_positions")),
    )


def normalize_events(raw: dict[str, Any]) -> EventSnapshot:
    events = as_dict(raw.get("events"))
    return EventSnapshot(
        event_data_fresh=boolean(events.get("event_data_fresh")),
        sentry_info_fresh=boolean(events.get("sentry_info_fresh")),
        buff_task_enabled=boolean(events.get("buff_task_enabled")),
        buff_can_activate=boolean(events.get("buff_can_activate")),
        buff_activating=boolean(events.get("buff_activating")),
        buff_activated=boolean(events.get("buff_activated")),
        outpost_task_enabled=boolean(events.get("outpost_task_enabled")),
        enemy_outpost_alive=boolean(events.get("enemy_outpost_alive")),
        outpost_attack_window_open=boolean(events.get("outpost_attack_window_open")),
        self_low_hp=boolean(events.get("self_low_hp")),
        self_low_ammo=boolean(events.get("self_low_ammo")),
        recent_damage_over_30=boolean(events.get("recent_damage_over_30")),
        armor_target_visible=boolean(events.get("armor_target_visible")),
        buff_target_locked=boolean(events.get("buff_target_locked")),
        outpost_target_locked=boolean(events.get("outpost_target_locked")),
        goal_reached=boolean(events.get("goal_reached")),
        goal_unreachable=boolean(events.get("goal_unreachable")),
        regional_defense_active=boolean(events.get("regional_defense_active")),
        self_fortress_gain_point_status=integer(events.get("self_fortress_gain_point_status"), 0),
        self_outpost_gain_point_status=integer(events.get("self_outpost_gain_point_status"), 0),
        self_base_gain_point_status=boolean(events.get("self_base_gain_point_status")),
    )


def normalize_relative_target(raw: dict[str, Any]) -> RelativeTarget:
    target = as_dict(raw.get("navi_relative_target"))
    return RelativeTarget(
        valid=boolean(target.get("valid")),
        x=optional_number(target.get("x")),
        y=optional_number(target.get("y")),
        z=optional_number(target.get("z")),
        distance=optional_number(target.get("distance")),
        yaw_error_deg=optional_number(target.get("yaw_error_deg")),
        pitch_error_deg=optional_number(target.get("pitch_error_deg")),
        armor_type=integer(target.get("armor_type"), 0),
        aim_mode=integer(target.get("aim_mode"), 0),
        official_target_valid=boolean(target.get("official_target_valid")),
        official_armor_type=integer(target.get("official_armor_type"), 0),
    )


def normalize_gimbal(raw: dict[str, Any]) -> GimbalState:
    gimbal = as_dict(raw.get("gimbal"))
    fire_code = as_dict(gimbal.get("fire_code"))
    return GimbalState(
        yaw_deg=optional_number(gimbal.get("yaw_deg")),
        pitch_deg=optional_number(gimbal.get("pitch_deg")),
        yaw_vel_deg_per_sec=optional_number(gimbal.get("yaw_vel_deg_per_sec")),
        yaw_angle_deg=optional_number(gimbal.get("yaw_angle_deg")),
        cap_v=integer(gimbal.get("cap_v"), 0),
        navi_lower_head=integer(gimbal.get("navi_lower_head"), 0),
        fire_status=integer(fire_code.get("fire_status"), 0),
        cap_state=integer(fire_code.get("cap_state"), 0),
        follow_mode=integer(fire_code.get("follow_mode"), 0),
        aim_mode=integer(fire_code.get("aim_mode"), 0),
        rotate=integer(fire_code.get("rotate"), 0),
    )


def normalize_runtime_guard(raw: dict[str, Any]) -> RuntimeGuard:
    guard = as_dict(raw.get("runtime_guard"))
    return RuntimeGuard(
        fault=str(guard.get("fault", "-")),
        recovery_requested=boolean(guard.get("recovery_requested")),
        recovering=boolean(guard.get("recovering")),
    )


def normalize_record(raw: dict[str, Any], index: int, goal_names: dict[int, str]) -> TraceRecord:
    navi = as_dict(raw.get("navi_goal"))
    posture = as_dict(raw.get("posture"))
    posture_runtime = as_dict(posture.get("runtime"))
    referee = as_dict(raw.get("referee"))
    target = as_dict(raw.get("target_armor"))

    team = str(raw.get("team", "red")).lower()
    if team not in ("red", "blue"):
        team = "red"

    output = normalize_output(raw, navi, team, goal_names)
    decision_intent = normalize_decision_intent(raw, output)
    target_state = normalize_target_state(raw)
    events = normalize_events(raw)
    navi_relative_target = normalize_relative_target(raw)
    gimbal = normalize_gimbal(raw)
    runtime_guard = normalize_runtime_guard(raw)

    return TraceRecord(
        raw=raw,
        index=index,
        t=number(raw.get("t", raw.get("elapsed_sec")), float(index)),
        event=str(raw.get("event", "tick")),
        tick=integer(raw.get("tick"), index),
        team=team,
        strategy=str(raw.get("strategy_mode", "-")),
        aim=str(raw.get("aim_mode", "-")),
        target=compact_label(target, str(raw.get("target_armor", "-"))),
        target_state=target_state,
        output=output,
        decision_intent=decision_intent,
        events=events,
        navi_relative_target=navi_relative_target,
        goal_id=output.goal_id,
        goal_base_id=output.goal_base_id,
        goal_name=output.goal_name,
        goal_side=output.goal_side,
        goal_position=output.goal_pos_cm,
        speed_level=output.speed_level,
        posture_command=compact_label(posture.get("command", raw.get("posture_cmd"))),
        posture_state=compact_label(posture.get("state", raw.get("posture_state"))),
        posture_current=compact_label(posture_runtime.get("current")),
        posture_desired=compact_label(posture_runtime.get("desired")),
        posture_pending=compact_label(posture_runtime.get("pending")),
        posture_reason=str(posture.get("last_reason", "-")),
        hp=integer(referee.get("self_hp", raw.get("self_hp")), 0),
        ammo=integer(referee.get("ammo", raw.get("ammo")), 0),
        time_left=integer(referee.get("time_left", raw.get("time_left")), 0),
        units=normalize_units(raw),
        gimbal=gimbal,
        runtime_guard=runtime_guard,
    )


def load_trace(path: Path, goal_names: dict[int, str]) -> tuple[list[TraceRecord], int]:
    records: list[TraceRecord] = []
    bad_lines = 0
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            text = line.strip()
            if not text or text.startswith("#"):
                continue
            try:
                item = json.loads(text)
            except json.JSONDecodeError:
                bad_lines += 1
                continue
            if not isinstance(item, dict):
                bad_lines += 1
                continue
            records.append(normalize_record(item, len(records), goal_names))
    if not records:
        raise ValueError(f"no trace records loaded from {path}")
    return records, bad_lines


def load_trace_incremental(
    path: Path,
    goal_names: dict[int, str],
    start_offset: int,
    start_index: int,
) -> tuple[list[TraceRecord], int, int]:
    records: list[TraceRecord] = []
    bad_lines = 0
    with path.open("r", encoding="utf-8") as stream:
        stream.seek(max(0, start_offset))
        while True:
            line_start = stream.tell()
            line = stream.readline()
            if not line:
                break
            # Writer may still be appending the current line; keep it for next poll.
            if not line.endswith("\n"):
                stream.seek(line_start)
                break
            text = line.strip()
            if not text or text.startswith("#"):
                continue
            try:
                item = json.loads(text)
            except json.JSONDecodeError:
                bad_lines += 1
                continue
            if not isinstance(item, dict):
                bad_lines += 1
                continue
            records.append(normalize_record(item, start_index + len(records), goal_names))
        new_offset = stream.tell()
    return records, bad_lines, new_offset


def build_changes(records: list[TraceRecord]) -> list[dict[str, Any]]:
    changes: list[dict[str, Any]] = []
    last: dict[str, Any] | None = None
    for record in records:
        state = {
            "strategy": record.strategy,
            "aim": record.aim,
            "route": record.output.route_key,
            "kind": record.output.kind,
            "target": record.target,
            "posture": (record.posture_command, record.posture_state),
            "intent": (
                record.decision_intent.layer,
                record.decision_intent.reason,
                record.decision_intent.base_goal_id,
                record.decision_intent.resolved_goal_id,
            ),
            "events": record.events.change_key(),
            "guard": record.runtime_guard.fault,
        }
        if last is None or state != last or record.event != "tick":
            parts = []
            if record.event != "tick":
                parts.append(record.event)
            if last is None or state["strategy"] != last["strategy"]:
                parts.append(f"strategy={record.strategy}")
            if last is None or state["aim"] != last["aim"]:
                parts.append(f"aim={record.aim}")
            if last is None or state["route"] != last["route"] or state["kind"] != last["kind"]:
                parts.append(f"output={record.output.kind}:{record.output.goal_name}:{record.output.goal_id}")
            if last is None or state["target"] != last["target"]:
                parts.append(f"target={record.target}")
            if last is None or state["intent"] != last["intent"]:
                parts.append(f"intent={record.decision_intent.layer}/{record.decision_intent.reason}")
            if last is None or state["events"] != last["events"]:
                parts.append(f"events={record.events.compact_text()}")
            if last is None or state["posture"] != last["posture"]:
                parts.append(f"posture={record.posture_command}/{record.posture_state}")
            if last is None or state["guard"] != last["guard"]:
                parts.append(f"guard={record.runtime_guard.fault}")
            changes.append({"index": record.index, "t": record.t, "text": ", ".join(parts)})
            last = state
    return changes
