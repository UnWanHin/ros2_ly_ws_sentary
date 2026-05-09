#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# Minimal navigation velocity bridge for hardware chain tests.
# It does not run behavior_tree or issue aim/rotate commands.

import math
from typing import Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from gimbal_driver.msg import ControlVelocity, FireCode, GimbalAngles, Vel


def clamp_int8(value: float) -> int:
    if not math.isfinite(value):
        return 0
    return max(-128, min(127, int(round(value))))


class NaviVelControlBridge(Node):
    def __init__(self) -> None:
        super().__init__("navi_vel_control_bridge")

        self.input_topic = self.declare_parameter("input_topic", "/ly/navi/vel").value
        self.control_vel_topic = self.declare_parameter("control_vel_topic", "/ly/control/vel").value
        self.gimbal_angles_topic = self.declare_parameter("gimbal_angles_topic", "/ly/gimbal/angles").value
        self.control_angles_topic = self.declare_parameter("control_angles_topic", "/ly/control/angles").value
        self.control_firecode_topic = self.declare_parameter(
            "control_firecode_topic", "/ly/control/firecode").value
        self.wait_for_gimbal_angles = bool(
            self.declare_parameter("wait_for_gimbal_angles", True).value)
        self.publish_hold_angles = bool(
            self.declare_parameter("publish_hold_angles", True).value)
        self.publish_safe_firecode = bool(
            self.declare_parameter("publish_safe_firecode", True).value)
        self.publish_initial_zero = bool(
            self.declare_parameter("publish_initial_zero", True).value)
        self.stale_timeout_sec = max(
            0.0, float(self.declare_parameter("stale_timeout_sec", 0.5).value))
        self.velocity_raw_to_mps = float(
            self.declare_parameter("velocity_raw_to_mps", 0.025).value)

        self.last_angles: Optional[Tuple[float, float]] = None
        self.last_vel_time_ns: Optional[int] = None
        self.last_command_raw: Tuple[int, int] = (0, 0)
        self.initial_zero_sent = False
        self.last_wait_warn_ns = 0

        self.pub_control_vel = self.create_publisher(ControlVelocity, self.control_vel_topic, 10)
        self.pub_control_angles = self.create_publisher(GimbalAngles, self.control_angles_topic, 10)
        self.pub_control_firecode = self.create_publisher(FireCode, self.control_firecode_topic, 10)
        self.create_subscription(Vel, self.input_topic, self.on_navi_vel, 20)
        self.create_subscription(GimbalAngles, self.gimbal_angles_topic, self.on_gimbal_angles, 20)
        self.create_timer(0.05, self.on_timer)

        self.get_logger().info(
            f"navi velocity bridge: {self.input_topic} -> {self.control_vel_topic}, "
            f"hold_angles={self.publish_hold_angles} wait_gimbal={self.wait_for_gimbal_angles} "
            f"safe_firecode={self.publish_safe_firecode} stale_timeout={self.stale_timeout_sec:.3f}s"
        )

    def on_gimbal_angles(self, msg: GimbalAngles) -> None:
        self.last_angles = (float(msg.yaw), float(msg.pitch))

    def on_navi_vel(self, msg: Vel) -> None:
        if not self.can_publish_control():
            return
        raw_x = clamp_int8(msg.x)
        raw_y = clamp_int8(msg.y)
        self.publish_control(raw_x, raw_y, "navi_vel")
        self.last_vel_time_ns = self.get_clock().now().nanoseconds

    def on_timer(self) -> None:
        if not self.can_publish_control(log_wait=False):
            return

        if self.publish_initial_zero and not self.initial_zero_sent:
            self.publish_control(0, 0, "initial_zero")
            self.initial_zero_sent = True
            return

        if self.stale_timeout_sec <= 0.0 or self.last_vel_time_ns is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        stale_ns = int(self.stale_timeout_sec * 1e9)
        if now_ns - self.last_vel_time_ns > stale_ns and self.last_command_raw != (0, 0):
            self.publish_control(0, 0, "stale_zero")

    def can_publish_control(self, log_wait: bool = True) -> bool:
        if not self.wait_for_gimbal_angles or self.last_angles is not None:
            return True
        if log_wait:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_wait_warn_ns > int(2e9):
                self.get_logger().warn(
                    f"waiting for {self.gimbal_angles_topic} before forwarding velocity; "
                    "no control frame published")
                self.last_wait_warn_ns = now_ns
        return False

    def publish_control(self, raw_x: int, raw_y: int, reason: str) -> None:
        stamp = self.get_clock().now().to_msg()

        if self.publish_hold_angles and self.last_angles is not None:
            angle_msg = GimbalAngles()
            angle_msg.header.stamp = stamp
            angle_msg.yaw = float(self.last_angles[0])
            angle_msg.pitch = float(self.last_angles[1])
            self.pub_control_angles.publish(angle_msg)

        if self.publish_safe_firecode:
            fire_msg = FireCode()
            fire_msg.header.stamp = stamp
            fire_msg.field_mask = FireCode.FIELD_ALL
            fire_msg.fire_status = 0
            fire_msg.cap_state = 0
            fire_msg.follow_mode = False
            fire_msg.aim_mode = False
            fire_msg.rotate = 0
            fire_msg.raw = 0
            self.pub_control_firecode.publish(fire_msg)

        vel_msg = ControlVelocity()
        vel_msg.header.stamp = stamp
        vel_msg.raw_x = raw_x
        vel_msg.raw_y = raw_y
        vel_msg.x_mps = float(raw_x) * self.velocity_raw_to_mps
        vel_msg.y_mps = float(raw_y) * self.velocity_raw_to_mps
        vel_msg.use_raw = True
        self.pub_control_vel.publish(vel_msg)
        self.last_command_raw = (raw_x, raw_y)

        self.get_logger().debug(f"publish {reason} velocity raw=({raw_x},{raw_y})")


def main() -> None:
    rclpy.init()
    node = NaviVelControlBridge()
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
