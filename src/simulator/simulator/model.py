from __future__ import annotations

from dataclasses import dataclass
from typing import Any


PointCm = tuple[float, float]


@dataclass(frozen=True)
class DecisionOutput:
    kind: str
    goal_id: int
    goal_base_id: int
    goal_name: str
    goal_side: str
    goal_pos_cm: PointCm | None
    speed_level: int
    publish_enabled: bool | None
    publish_allowed: bool | None
    output_topic: str
    output_frame: str
    final_goal_pos_topic: str
    uses_goal_pos: bool
    uses_to_navi: bool
    relative_target_valid: bool
    source: str

    @property
    def route_key(self) -> tuple[int, str, PointCm | None]:
        return (self.goal_id, self.goal_side, self.goal_pos_cm)

    def publish_text(self) -> str:
        if self.publish_allowed is None and self.publish_enabled is None:
            return "-"
        return f"allowed={self.publish_allowed} enabled={self.publish_enabled}"

    def topic_text(self) -> str:
        if not self.output_topic and not self.final_goal_pos_topic:
            return "-"
        if self.output_topic == self.final_goal_pos_topic or not self.final_goal_pos_topic:
            return self.output_topic
        return f"{self.output_topic} -> {self.final_goal_pos_topic}"


@dataclass(frozen=True)
class UnitRecord:
    side: str
    type_name: str
    type_id: int
    hp: int
    max_hp: int
    distance_m: float
    position_cm: PointCm | None

    @property
    def health_ratio(self) -> float | None:
        if self.max_hp <= 0:
            return None
        return max(0.0, min(1.0, self.hp / self.max_hp))


@dataclass(frozen=True)
class DecisionIntent:
    layer: str
    reason: str
    base_goal_id: int
    resolved_goal_id: int
    goal_team: str
    apply_team_offset: bool | None
    priority: int
    detail: str


@dataclass(frozen=True)
class TargetState:
    has_recent_target: bool
    fresh_auto_aim: bool
    fresh_buff: bool
    fresh_outpost: bool
    hitable_targets: tuple[str, ...]
    reliable_enemy_positions: tuple[str, ...]

    def fresh_text(self) -> str:
        parts = [
            f"recent={int(self.has_recent_target)}",
            f"auto={int(self.fresh_auto_aim)}",
            f"buff={int(self.fresh_buff)}",
            f"outpost={int(self.fresh_outpost)}",
        ]
        return " ".join(parts)


@dataclass(frozen=True)
class EventSnapshot:
    event_data_fresh: bool
    sentry_info_fresh: bool
    buff_task_enabled: bool
    buff_can_activate: bool
    buff_activating: bool
    buff_activated: bool
    outpost_task_enabled: bool
    enemy_outpost_alive: bool
    outpost_attack_window_open: bool
    self_low_hp: bool
    self_low_ammo: bool
    recent_damage_over_30: bool
    armor_target_visible: bool
    buff_target_locked: bool
    outpost_target_locked: bool
    goal_reached: bool
    goal_unreachable: bool
    regional_defense_active: bool
    self_fortress_gain_point_status: int
    self_outpost_gain_point_status: int
    self_base_gain_point_status: bool

    def compact_text(self) -> str:
        parts: list[str] = []
        if self.regional_defense_active:
            parts.append("RD")
        if self.buff_activated:
            parts.append("Buff=done")
        elif self.buff_activating:
            parts.append("Buff=active")
        elif self.buff_can_activate:
            parts.append("Buff=can")
        elif self.buff_task_enabled:
            parts.append("Buff=task")
        if self.enemy_outpost_alive:
            parts.append("Outpost=alive")
        if self.outpost_attack_window_open:
            parts.append("Window=open")
        risks = []
        if self.self_low_hp:
            risks.append("hp")
        if self.self_low_ammo:
            risks.append("ammo")
        if self.recent_damage_over_30:
            risks.append("damage")
        if risks:
            parts.append("Low=" + ",".join(risks))
        if self.goal_reached:
            parts.append("Goal=reached")
        if self.goal_unreachable:
            parts.append("Goal=blocked")
        return " ".join(parts) if parts else "-"

    def change_key(self) -> tuple[object, ...]:
        return (
            self.regional_defense_active,
            self.buff_can_activate,
            self.buff_activating,
            self.buff_activated,
            self.outpost_attack_window_open,
            self.self_low_hp,
            self.self_low_ammo,
            self.recent_damage_over_30,
            self.goal_reached,
            self.goal_unreachable,
        )


@dataclass(frozen=True)
class RelativeTarget:
    valid: bool
    x: float | None
    y: float | None
    z: float | None
    distance: float | None
    yaw_error_deg: float | None
    pitch_error_deg: float | None
    armor_type: int
    aim_mode: int
    official_target_valid: bool
    official_armor_type: int


@dataclass(frozen=True)
class GimbalState:
    yaw_deg: float | None
    pitch_deg: float | None
    yaw_vel_deg_per_sec: float | None
    yaw_angle_deg: float | None
    cap_v: int
    navi_lower_head: int
    fire_status: int
    cap_state: int
    follow_mode: int
    aim_mode: int
    rotate: int


@dataclass(frozen=True)
class RuntimeGuard:
    fault: str
    recovery_requested: bool
    recovering: bool


@dataclass(frozen=True)
class TraceRecord:
    raw: dict[str, Any]
    index: int
    t: float
    event: str
    tick: int
    team: str
    strategy: str
    aim: str
    target: str
    target_state: TargetState
    output: DecisionOutput
    decision_intent: DecisionIntent
    events: EventSnapshot
    navi_relative_target: RelativeTarget
    goal_id: int
    goal_base_id: int
    goal_name: str
    goal_side: str
    goal_position: PointCm | None
    speed_level: int
    posture_command: str
    posture_state: str
    posture_current: str
    posture_desired: str
    posture_pending: str
    posture_reason: str
    hp: int
    ammo: int
    time_left: int
    units: tuple[UnitRecord, ...]
    gimbal: GimbalState
    runtime_guard: RuntimeGuard
