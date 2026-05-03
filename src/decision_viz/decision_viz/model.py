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
    output: DecisionOutput
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
