#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# Dry-run subscriber for /ly/control/* commands. It never writes to lower hardware.

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from gimbal_driver.msg import ControlVelocity, FireCode, GimbalAngles, SentryCmd


@dataclass
class LastCommand:
    angles: Optional[GimbalAngles] = None
    firecode: Optional[FireCode] = None
    vel: Optional[ControlVelocity] = None
    posture: Optional[SentryCmd] = None
    sentry_cmd: Optional[SentryCmd] = None
    angles_count: int = 0
    firecode_count: int = 0
    vel_count: int = 0
    posture_count: int = 0
    sentry_cmd_count: int = 0


class ControlSink(Node):
    def __init__(self) -> None:
        super().__init__("control_sink")
        self.angles_topic = self.declare_parameter("angles_topic", "/ly/control/angles").value
        self.firecode_topic = self.declare_parameter("firecode_topic", "/ly/control/firecode").value
        self.vel_topic = self.declare_parameter("vel_topic", "/ly/control/vel").value
        self.posture_topic = self.declare_parameter("posture_topic", "/ly/control/posture").value
        self.sentry_cmd_topic = self.declare_parameter(
            "sentry_cmd_topic", "/ly/control/sentry_cmd").value
        self.include_vel = bool(self.declare_parameter("include_vel", True).value)
        self.include_posture = bool(self.declare_parameter("include_posture", True).value)
        self.verbose = bool(self.declare_parameter("verbose", False).value)
        self.log_every_sec = max(0.1, float(self.declare_parameter("log_every_sec", 1.0).value))

        self.last = LastCommand()
        self.create_subscription(GimbalAngles, self.angles_topic, self.on_angles, 50)
        self.create_subscription(FireCode, self.firecode_topic, self.on_firecode, 50)
        if self.include_vel:
            self.create_subscription(ControlVelocity, self.vel_topic, self.on_vel, 50)
        if self.include_posture:
            self.create_subscription(SentryCmd, self.posture_topic, self.on_posture, 20)
            self.create_subscription(SentryCmd, self.sentry_cmd_topic, self.on_sentry_cmd, 20)
        self.create_timer(self.log_every_sec, self.log_summary)
        self.get_logger().info(
            "control sink started: subscribing to control topics only; no gimbal_driver/hardware output")

    def on_angles(self, msg: GimbalAngles) -> None:
        self.last.angles = msg
        self.last.angles_count += 1
        if self.verbose:
            self.get_logger().info(f"angles yaw={msg.yaw:.2f} pitch={msg.pitch:.2f}")

    def on_firecode(self, msg: FireCode) -> None:
        self.last.firecode = msg
        self.last.firecode_count += 1
        if self.verbose:
            self.get_logger().info(
                f"firecode mask={msg.field_mask} fire={msg.fire_status} cap={msg.cap_state} "
                f"follow={msg.follow_mode} aim={msg.aim_mode} rotate={msg.rotate} raw={msg.raw}")

    def on_vel(self, msg: ControlVelocity) -> None:
        self.last.vel = msg
        self.last.vel_count += 1
        if self.verbose:
            self.get_logger().info(
                f"vel raw=({msg.raw_x},{msg.raw_y}) mps=({msg.x_mps:.3f},{msg.y_mps:.3f}) "
                f"use_raw={msg.use_raw}")

    def on_posture(self, msg: SentryCmd) -> None:
        self.last.posture = msg
        self.last.posture_count += 1
        if self.verbose:
            self.get_logger().info(
                f"posture mask={msg.field_mask} posture={msg.posture} raw={msg.raw}")

    def on_sentry_cmd(self, msg: SentryCmd) -> None:
        self.last.sentry_cmd = msg
        self.last.sentry_cmd_count += 1
        if self.verbose:
            self.get_logger().info(
                f"sentry_cmd mask={msg.field_mask} posture={msg.posture} raw={msg.raw}")

    def log_summary(self) -> None:
        parts = [
            f"rx angles={self.last.angles_count}",
            f"firecode={self.last.firecode_count}",
        ]
        if self.include_vel:
            parts.append(f"vel={self.last.vel_count}")
        if self.include_posture:
            parts.append(f"posture={self.last.posture_count}")
            parts.append(f"sentry_cmd={self.last.sentry_cmd_count}")

        if self.last.angles is not None:
            parts.append(
                f"last_angles=({self.last.angles.yaw:.2f},{self.last.angles.pitch:.2f})")
        if self.last.firecode is not None:
            parts.append(
                "last_firecode="
                f"(mask={self.last.firecode.field_mask},fire={self.last.firecode.fire_status},"
                f"follow={self.last.firecode.follow_mode},aim={self.last.firecode.aim_mode},"
                f"rotate={self.last.firecode.rotate})")
        if self.include_vel and self.last.vel is not None:
            parts.append(f"last_vel_raw=({self.last.vel.raw_x},{self.last.vel.raw_y})")
        self.get_logger().info(" ".join(parts))


def main() -> None:
    rclpy.init()
    node = ControlSink()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
