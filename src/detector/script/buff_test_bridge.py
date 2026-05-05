#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from auto_aim_common.msg import Target
from gimbal_driver.msg import ControlVelocity, FireCode, GimbalAngles
from std_msgs.msg import UInt8


@dataclass
class TargetState:
    yaw: float
    pitch: float
    status: bool
    rx_ns: int


class BuffTestBridge(Node):
    def __init__(self) -> None:
        super().__init__("buff_test_bridge")

        self.declare_parameter("target_topic", "/ly/buff/target")
        self.declare_parameter("vision_mode_topic", "/ly/vision/mode")
        self.declare_parameter("bt_target_topic", "/ly/bt/target")
        self.declare_parameter("control_angles_topic", "/ly/control/angles")
        self.declare_parameter("control_firecode_topic", "/ly/control/firecode")
        self.declare_parameter("control_vel_topic", "/ly/control/vel")
        self.declare_parameter("gate_vision_mode", 2)
        self.declare_parameter("gate_publish_bt_target", False)
        self.declare_parameter("gate_bt_target", 1)
        self.declare_parameter("timeout_sec", 1.0)
        self.declare_parameter("publish_hz", 80.0)
        self.declare_parameter("gate_hz", 2.0)
        self.declare_parameter("fire_hz", 20.0)
        self.declare_parameter("enable_fire", True)
        self.declare_parameter("use_target_status", True)
        self.declare_parameter("zero_velocity", True)

        self.target_topic = str(self.get_parameter("target_topic").value)
        self.vision_mode_topic = str(self.get_parameter("vision_mode_topic").value)
        self.bt_target_topic = str(self.get_parameter("bt_target_topic").value)
        self.control_angles_topic = str(self.get_parameter("control_angles_topic").value)
        self.control_firecode_topic = str(self.get_parameter("control_firecode_topic").value)
        self.control_vel_topic = str(self.get_parameter("control_vel_topic").value)

        self.gate_vision_mode = int(self.get_parameter("gate_vision_mode").value) & 0xFF
        self.gate_publish_bt_target = bool(self.get_parameter("gate_publish_bt_target").value)
        self.gate_bt_target = int(self.get_parameter("gate_bt_target").value) & 0xFF

        timeout_sec = float(self.get_parameter("timeout_sec").value)
        publish_hz = float(self.get_parameter("publish_hz").value)
        gate_hz = float(self.get_parameter("gate_hz").value)
        fire_hz = float(self.get_parameter("fire_hz").value)
        self.enable_fire = bool(self.get_parameter("enable_fire").value)
        self.use_target_status = bool(self.get_parameter("use_target_status").value)
        self.zero_velocity = bool(self.get_parameter("zero_velocity").value)

        self.timeout_ns = int(max(timeout_sec, 0.05) * 1e9)
        self.fire_interval_ns = int(1e9 / max(fire_hz, 1.0))

        self.pub_vision_mode = self.create_publisher(UInt8, self.vision_mode_topic, 10)
        self.pub_bt_target = self.create_publisher(UInt8, self.bt_target_topic, 10)
        self.pub_angles = self.create_publisher(GimbalAngles, self.control_angles_topic, 50)
        self.pub_firecode = self.create_publisher(FireCode, self.control_firecode_topic, 50)
        self.pub_vel = self.create_publisher(ControlVelocity, self.control_vel_topic, 20)

        self.sub_target = self.create_subscription(Target, self.target_topic, self._on_target, 20)

        self.latest_target: Optional[TargetState] = None
        self.last_fire_toggle_ns = 0
        self.fire_status = 0
        self.last_timeout_log_ns = 0

        self.control_timer = self.create_timer(1.0 / max(publish_hz, 1.0), self._on_control_timer)
        self.gate_timer = self.create_timer(1.0 / max(gate_hz, 0.5), self._on_gate_timer)

        self._on_gate_timer()
        self.get_logger().info(
            "buff_test_bridge started: "
            f"target={self.target_topic} -> angles={self.control_angles_topic}, firecode={self.control_firecode_topic}, "
            f"vision_mode={self.gate_vision_mode}@{self.vision_mode_topic}, "
            f"bt_target={'off' if not self.gate_publish_bt_target else self.gate_bt_target}@{self.bt_target_topic}, "
            f"enable_fire={int(self.enable_fire)} use_target_status={int(self.use_target_status)} fire_hz={fire_hz:.1f}"
        )

    def _on_target(self, msg: Target) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self.latest_target = TargetState(
            yaw=float(msg.yaw),
            pitch=float(msg.pitch),
            status=bool(msg.status),
            rx_ns=now_ns,
        )

    def _on_gate_timer(self) -> None:
        mode_msg = UInt8()
        mode_msg.data = self.gate_vision_mode
        self.pub_vision_mode.publish(mode_msg)

        if self.gate_publish_bt_target:
            target_msg = UInt8()
            target_msg.data = self.gate_bt_target
            self.pub_bt_target.publish(target_msg)

    def _publish_zero_velocity(self) -> None:
        if not self.zero_velocity:
            return
        vel_msg = ControlVelocity()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.x_mps = 0.0
        vel_msg.y_mps = 0.0
        vel_msg.raw_x = 0
        vel_msg.raw_y = 0
        vel_msg.use_raw = True
        self.pub_vel.publish(vel_msg)

    def _publish_firecode(self, fire_status: int, aim_mode: bool) -> None:
        msg = FireCode()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.field_mask = FireCode.FIELD_FIRE_STATUS | FireCode.FIELD_AIM_MODE
        msg.fire_status = int(fire_status) & 0x03
        msg.aim_mode = bool(aim_mode)
        self.pub_firecode.publish(msg)

    def _on_control_timer(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        target_fresh = (
            self.latest_target is not None and
            (now_ns - self.latest_target.rx_ns) <= self.timeout_ns
        )

        if not target_fresh:
            self.fire_status = 0
            self._publish_firecode(self.fire_status, False)
            self._publish_zero_velocity()
            if now_ns - self.last_timeout_log_ns > int(2e9):
                self.get_logger().warn("buff target timeout, publish safe firecode.")
                self.last_timeout_log_ns = now_ns
            return

        angles = GimbalAngles()
        angles.header.stamp = now.to_msg()
        angles.yaw = float(self.latest_target.yaw)
        angles.pitch = float(self.latest_target.pitch)
        self.pub_angles.publish(angles)

        if self.use_target_status:
            should_fire = self.enable_fire and bool(self.latest_target.status)
        else:
            should_fire = self.enable_fire
        if should_fire and (now_ns - self.last_fire_toggle_ns) >= self.fire_interval_ns:
            self.fire_status = 0b11 if self.fire_status == 0 else 0
            self.last_fire_toggle_ns = now_ns
        if not should_fire:
            self.fire_status = 0

        self._publish_firecode(self.fire_status, True)
        self._publish_zero_velocity()


def main() -> None:
    rclpy.init()
    node = BuffTestBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("buff_test_bridge interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
