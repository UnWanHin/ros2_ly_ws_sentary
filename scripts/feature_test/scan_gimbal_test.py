#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import argparse
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from gimbal_driver.msg import FireCode, GimbalAngles


class ScanGimbalNode(Node):
    def __init__(
        self,
        yaw_min: float,
        yaw_max: float,
        pitch: float,
        step_deg: float,
        hz: float,
        angles_topic: str,
        firecode_topic: str,
        safe_firecode: int,
        scan_mode: int,
        publish_firecode: bool,
    ) -> None:
        super().__init__("scan_gimbal_test")
        if yaw_min > yaw_max:
            yaw_min, yaw_max = yaw_max, yaw_min

        self.scan_mode = scan_mode if scan_mode in (1, 2, 3) else 1
        self.yaw_min = yaw_min
        self.yaw_max = yaw_max
        self.pitch = pitch
        self.step_deg = abs(step_deg) if abs(step_deg) > 0.01 else 0.5
        self.yaw = yaw_min
        self.direction = 1.0
        self.center_yaw = (yaw_min + yaw_max) * 0.5
        self.half_range = max((yaw_max - yaw_min) * 0.5, 1.0)
        self.phase = -math.pi * 0.5
        self.safe_firecode = max(0, min(255, safe_firecode))
        self.publish_firecode = publish_firecode

        self.angles_pub = self.create_publisher(GimbalAngles, angles_topic, 10)
        self.firecode_pub = self.create_publisher(FireCode, firecode_topic, 10) if publish_firecode else None
        period = 1.0 / max(1.0, hz)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"scan started: mode={self.scan_mode} yaw=[{self.yaw_min:.2f},{self.yaw_max:.2f}] "
            f"pitch={self.pitch:.2f} step={self.step_deg:.2f} "
            f"angles_topic={angles_topic} publish_firecode={self.publish_firecode}"
        )

    def _on_timer(self) -> None:
        msg = GimbalAngles()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.yaw = float(self.yaw)
        msg.pitch = float(self.pitch)
        self.angles_pub.publish(msg)

        if self.firecode_pub is not None:
            fire_msg = FireCode()
            fire_msg.header.stamp = msg.header.stamp
            fire_msg.field_mask = FireCode.FIELD_ALL
            fire_msg.fire_status = self.safe_firecode & 0x03
            fire_msg.cap_state = (self.safe_firecode >> 2) & 0x03
            fire_msg.follow_mode = ((self.safe_firecode >> 4) & 0x01) != 0
            fire_msg.aim_mode = ((self.safe_firecode >> 5) & 0x01) != 0
            fire_msg.rotate = (self.safe_firecode >> 6) & 0x03
            fire_msg.raw = self.safe_firecode
            self.firecode_pub.publish(fire_msg)

        if self.scan_mode == 2:
            self.phase += self.step_deg / self.half_range
            if self.phase > math.pi * 1.5:
                self.phase -= math.pi * 2.0
            self.yaw = self.center_yaw + self.half_range * math.sin(self.phase)
            return
        if self.scan_mode == 3:
            self.yaw = normalize_angle(self.yaw + self.step_deg)
            return

        self.yaw += self.direction * self.step_deg
        if self.yaw >= self.yaw_max:
            self.yaw = self.yaw_max
            self.direction = -1.0
        elif self.yaw <= self.yaw_min:
            self.yaw = self.yaw_min
            self.direction = 1.0


def main() -> None:
    parser = argparse.ArgumentParser(description="External gimbal scan publisher")
    parser.add_argument("--yaw-min", type=float, default=-15.0)
    parser.add_argument("--yaw-max", type=float, default=15.0)
    parser.add_argument("--pitch", type=float, default=8.0)
    parser.add_argument("--step-deg", type=float, default=1.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--angles-topic", type=str, default="/ly/control/angles")
    parser.add_argument("--firecode-topic", type=str, default="/ly/control/firecode")
    parser.add_argument("--safe-firecode", type=int, default=0)
    parser.add_argument("--scan-mode", type=int, choices=(1, 2, 3), default=1)
    parser.add_argument("--no-firecode", action="store_true")
    cli_args = parser.parse_args()

    rclpy.init()
    node = ScanGimbalNode(
        yaw_min=cli_args.yaw_min,
        yaw_max=cli_args.yaw_max,
        pitch=cli_args.pitch,
        step_deg=cli_args.step_deg,
        hz=cli_args.hz,
        angles_topic=cli_args.angles_topic,
        firecode_topic=cli_args.firecode_topic,
        safe_firecode=cli_args.safe_firecode,
        scan_mode=cli_args.scan_mode,
        publish_firecode=not cli_args.no_firecode,
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("scan_gimbal_test interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def normalize_angle(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


if __name__ == "__main__":
    main()
