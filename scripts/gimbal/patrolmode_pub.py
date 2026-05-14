#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import argparse
import math
from pathlib import Path
from typing import Any, Dict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import yaml

from gimbal_driver.msg import FireCode, GimbalAngles


def normalize_angle(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def load_patrol_config(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    params = data.get("behavior_tree", {}).get("ros__parameters", {})
    return params.get("PatrolScan", {})


def get_float(config: Dict[str, Any], section: str, key: str, default: float) -> float:
    value = config.get(section, {}).get(key, default)
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(value):
        return default
    return value


class PatrolModePublisher(Node):
    def __init__(
        self,
        mode: int,
        config: Dict[str, Any],
        yaw_start: float,
        hz: float,
        angles_topic: str,
        firecode_topic: str,
        publish_firecode: bool,
        safe_firecode: int,
        outpost_mode: bool,
    ) -> None:
        super().__init__(f"patrolmode{mode}_test")
        self.mode = mode
        self.config = config
        self.yaw = yaw_start
        self.mode2_center_yaw = yaw_start
        self.mode2_phase_rad = 0.0
        self.start_time = self.get_clock().now()
        self.safe_firecode = max(0, min(255, safe_firecode))
        self.outpost_mode = outpost_mode

        self.angles_pub = self.create_publisher(GimbalAngles, angles_topic, 10)
        self.firecode_pub = (
            self.create_publisher(FireCode, firecode_topic, 10)
            if publish_firecode
            else None
        )
        self.timer = self.create_timer(1.0 / max(1.0, hz), self.on_timer)

        self.get_logger().info(
            f"patrol mode {self.mode} started: yaw_start={yaw_start:.2f} "
            f"hz={hz:.1f} angles={angles_topic} "
            f"firecode={'on' if publish_firecode else 'off'} "
            f"outpost_bias={'on' if self.outpost_mode else 'off'}"
        )

    def on_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()
        msg = GimbalAngles()
        msg.header.stamp = stamp
        msg.yaw = float(self.yaw)
        msg.pitch = float(self.current_pitch())
        self.angles_pub.publish(msg)

        if self.firecode_pub is not None:
            fire_msg = FireCode()
            fire_msg.header.stamp = stamp
            fire_msg.field_mask = FireCode.FIELD_ALL
            fire_msg.fire_status = self.safe_firecode & 0x03
            fire_msg.cap_state = (self.safe_firecode >> 2) & 0x03
            fire_msg.follow_mode = ((self.safe_firecode >> 4) & 0x01) != 0
            fire_msg.aim_mode = ((self.safe_firecode >> 5) & 0x01) != 0
            fire_msg.rotate = (self.safe_firecode >> 6) & 0x03
            fire_msg.raw = self.safe_firecode
            self.firecode_pub.publish(fire_msg)

        self.advance_yaw()

    def elapsed_ms(self) -> float:
        now = self.get_clock().now()
        return (now.nanoseconds - self.start_time.nanoseconds) / 1_000_000.0

    def current_pitch(self) -> float:
        if self.mode == 2:
            center = get_float(self.config, "Mode2", "PitchCenterDeg", 0.0)
            half_range = get_float(self.config, "Mode2", "PitchHalfRangeDeg", 13.0)
            period_ms = get_float(self.config, "Mode2", "PitchPeriodMs", 500.0)
        elif self.mode == 3:
            center = get_float(self.config, "Mode3", "PitchOffsetDeg", 15.0)
            half_range = get_float(self.config, "Mode3", "PitchHalfRangeDeg", 3.0)
            period_ms = get_float(self.config, "Mode3", "PitchPeriodMs", 500.0)
        else:
            center = get_float(self.config, "Mode1", "PitchCenterDeg", 0.0)
            half_range = get_float(self.config, "Mode1", "PitchHalfRangeDeg", 13.0)
            period_ms = get_float(self.config, "Mode1", "PitchPeriodMs", 500.0)
        period_ms = max(period_ms, 1.0)
        pitch = center + half_range * math.sin(self.elapsed_ms() * 2.0 * math.pi / period_ms)
        if self.outpost_mode:
            pitch += 15.0
        return pitch

    def advance_yaw(self) -> None:
        if self.mode == 2:
            step = abs(get_float(self.config, "Mode2", "YawStepDegPerTick", 1.0))
            half_range = max(abs(get_float(self.config, "Mode2", "YawHalfRangeDeg", 30.0)), 1.0)
            drift = get_float(self.config, "Mode2", "CenterDriftPerCycleDeg", -70.0)
            phase_step = step / half_range
            self.mode2_center_yaw = normalize_angle(
                self.mode2_center_yaw + drift * phase_step / (2.0 * math.pi)
            )
            self.mode2_phase_rad = (self.mode2_phase_rad + phase_step) % (2.0 * math.pi)
            self.yaw = normalize_angle(
                self.mode2_center_yaw + half_range * math.sin(self.mode2_phase_rad)
            )
            return

        section = "Mode3" if self.mode == 3 else "Mode1"
        step = abs(get_float(self.config, section, "YawStepDegPerTick", 1.0))
        self.yaw = normalize_angle(self.yaw + step)


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish PatrolScan-style gimbal angles")
    parser.add_argument("--mode", type=int, choices=(1, 2, 3), required=True)
    parser.add_argument("--patrol-config", type=Path, required=True)
    parser.add_argument("--yaw-start", type=float, default=0.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--angles-topic", default="/ly/control/angles")
    parser.add_argument("--firecode-topic", default="/ly/control/firecode")
    parser.add_argument("--no-firecode", action="store_true")
    parser.add_argument("--safe-firecode", type=int, default=0)
    parser.add_argument("--outpost", action="store_true")
    args = parser.parse_args()

    config = load_patrol_config(args.patrol_config)

    rclpy.init()
    node = PatrolModePublisher(
        mode=args.mode,
        config=config,
        yaw_start=args.yaw_start,
        hz=args.hz,
        angles_topic=args.angles_topic,
        firecode_topic=args.firecode_topic,
        publish_firecode=not args.no_firecode,
        safe_firecode=args.safe_firecode,
        outpost_mode=args.outpost,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("patrol mode test interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
