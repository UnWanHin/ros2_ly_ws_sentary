#!/usr/bin/env python3

"""Standalone debug bridge from navigation velocity to the formal control topic."""

import math

import rclpy
from rclpy.node import Node

from gimbal_driver.msg import ControlVelocity, Vel


NAV_VEL_TOPIC = "/ly/navi/vel"
CONTROL_VEL_TOPIC = "/ly/control/vel"
VELOCITY_RAW_TO_MPS = 0.025


def encode_navigation_raw(value: float) -> int:
    if not math.isfinite(value):
        return 0
    rounded = math.floor(value + 0.5) if value >= 0.0 else math.ceil(value - 0.5)
    return max(-128, min(127, int(rounded)))


class NaviVelToControlVel(Node):
    def __init__(self) -> None:
        super().__init__("navi_vel_to_control_vel")
        self.declare_parameter("stale_timeout_ms", 500)
        self.declare_parameter("publish_hz", 100.0)
        self.stale_timeout_ns = max(
            1, int(self.get_parameter("stale_timeout_ms").value) * 1_000_000
        )
        self.publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.latest_raw_x = 0
        self.latest_raw_y = 0
        self.last_rx_ns: int | None = None

        self.publisher = self.create_publisher(ControlVelocity, CONTROL_VEL_TOPIC, 10)
        self.subscription = self.create_subscription(Vel, NAV_VEL_TOPIC, self.on_navi_vel, 10)
        self.timer = self.create_timer(1.0 / self.publish_hz, self.publish_control_vel)

    def on_navi_vel(self, message: Vel) -> None:
        self.latest_raw_x = encode_navigation_raw(message.x)
        self.latest_raw_y = encode_navigation_raw(message.y)
        self.last_rx_ns = self.get_clock().now().nanoseconds

    def publish_control_vel(self) -> None:
        now = self.get_clock().now()
        if self.last_rx_ns is None or now.nanoseconds - self.last_rx_ns > self.stale_timeout_ns:
            raw_x = 0
            raw_y = 0
        else:
            raw_x = self.latest_raw_x
            raw_y = self.latest_raw_y

        message = ControlVelocity()
        message.header.stamp = now.to_msg()
        message.raw_x = raw_x
        message.raw_y = raw_y
        message.x_mps = raw_x * VELOCITY_RAW_TO_MPS
        message.y_mps = raw_y * VELOCITY_RAW_TO_MPS
        message.use_raw = True
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = NaviVelToControlVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
