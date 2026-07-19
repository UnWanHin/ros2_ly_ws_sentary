#!/usr/bin/env python3

"""Pure state machine for the standalone gimbal-driver debug control bridge."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ControlSnapshot:
    velocity: tuple[int, int] | None
    angles: tuple[float, float] | None
    trajectory: tuple[float, float, float, float, float, float] | None
    follow_mode: bool
    rotate: int
    aim_mode: bool
    fire_toggle: bool


class DebugControlState:
    def __init__(
        self,
        *,
        stale_timeout_ns: int,
        rotate_level: int,
        follow_mode_when_false: bool,
    ) -> None:
        self.stale_timeout_ns = max(1, stale_timeout_ns)
        self.rotate_level = max(0, min(3, rotate_level))
        self.follow_mode_when_false = follow_mode_when_false
        self.latest_velocity = (0, 0)
        self.last_navigation_ns: int | None = None
        self.should_rotate = True
        self.latest_aim: tuple[float, float, float, float, float, float] | None = None
        self.last_aim_ns: int | None = None
        self.pending_fire_toggle = False

    def update_navigation(self, raw_x: int, raw_y: int, received_ns: int) -> None:
        self.latest_velocity = (raw_x, raw_y)
        self.last_navigation_ns = received_ns

    def update_should_rotate(self, should_rotate: bool) -> None:
        self.should_rotate = should_rotate

    def update_aim(
        self,
        *,
        follow: bool,
        fire: bool,
        yaw: float,
        pitch: float,
        received_ns: int,
        yaw_omega: float = 0.0,
        pitch_omega: float = 0.0,
        yaw_alpha: float = 0.0,
        pitch_alpha: float = 0.0,
    ) -> None:
        trajectory = (yaw, pitch, yaw_omega, pitch_omega, yaw_alpha, pitch_alpha)
        valid = follow and all(math.isfinite(value) for value in trajectory)
        self.latest_aim = trajectory if valid else None
        self.last_aim_ns = received_ns if valid else None
        self.pending_fire_toggle = valid and fire

    def snapshot(self, now_ns: int, *, navi_mode: bool, aim_mode: bool) -> ControlSnapshot:
        navigation_fresh = (
            self.last_navigation_ns is not None
            and now_ns - self.last_navigation_ns <= self.stale_timeout_ns
        )
        velocity = None
        if navi_mode:
            velocity = self.latest_velocity if navigation_fresh else (0, 0)

        aim_fresh = (
            aim_mode
            and self.latest_aim is not None
            and self.last_aim_ns is not None
            and now_ns - self.last_aim_ns <= self.stale_timeout_ns
        )
        fire_toggle = aim_fresh and self.pending_fire_toggle
        self.pending_fire_toggle = False

        return ControlSnapshot(
            velocity=velocity,
            angles=(self.latest_aim[0], self.latest_aim[1]) if aim_fresh else None,
            trajectory=self.latest_aim if aim_fresh else None,
            follow_mode=navi_mode
            and self.follow_mode_when_false
            and not self.should_rotate,
            rotate=self.rotate_level if navi_mode and self.should_rotate else 0,
            aim_mode=aim_fresh,
            fire_toggle=fire_toggle,
        )
