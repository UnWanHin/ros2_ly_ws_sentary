#!/usr/bin/env python3

import rclpy
from gimbal_driver.msg import GimbalAngles
from rclpy.node import Node
from std_msgs.msg import Float32


class MockGimbalStateNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_gimbal_state_node")

        self.declare_parameter("gimbal_angles_topic", "/ly/gimbal/angles")
        self.declare_parameter("gimbal_big_yaw_topic", "/ly/gimbal/big_yaw_angles")
        self.declare_parameter("yaw_deg", 0.0)
        self.declare_parameter("pitch_deg", 0.0)
        self.declare_parameter("big_yaw_deg", 0.0)
        self.declare_parameter("publish_big_yaw", True)
        self.declare_parameter("publish_hz", 30.0)

        gimbal_angles_topic = str(self.get_parameter("gimbal_angles_topic").value)
        gimbal_big_yaw_topic = str(self.get_parameter("gimbal_big_yaw_topic").value)
        self.yaw_deg = float(self.get_parameter("yaw_deg").value)
        self.pitch_deg = float(self.get_parameter("pitch_deg").value)
        self.big_yaw_deg = float(self.get_parameter("big_yaw_deg").value)
        self.publish_big_yaw = bool(self.get_parameter("publish_big_yaw").value)
        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))

        self.pub_angles = self.create_publisher(GimbalAngles, gimbal_angles_topic, 10)
        self.pub_big_yaw = (
            self.create_publisher(Float32, gimbal_big_yaw_topic, 10)
            if self.publish_big_yaw
            else None
        )
        self.timer = self.create_timer(1.0 / publish_hz, self._on_timer)
        self.last_info_ns = 0

        self.get_logger().info(
            "mock gimbal state started: "
            f"angles=({self.yaw_deg:.2f}, {self.pitch_deg:.2f})deg -> {gimbal_angles_topic}, "
            f"big_yaw={self.big_yaw_deg:.2f}deg -> "
            f"{gimbal_big_yaw_topic if self.publish_big_yaw else '<off>'}"
        )

    def _on_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()

        angles = GimbalAngles()
        angles.header.stamp = stamp
        angles.yaw = float(self.yaw_deg)
        angles.pitch = float(self.pitch_deg)
        self.pub_angles.publish(angles)

        if self.pub_big_yaw is not None:
            big_yaw = Float32()
            big_yaw.data = float(self.big_yaw_deg)
            self.pub_big_yaw.publish(big_yaw)

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_info_ns > int(5e9):
            self.get_logger().info(
                "mock gimbal state: "
                f"yaw={self.yaw_deg:.2f} pitch={self.pitch_deg:.2f} "
                f"big_yaw={self.big_yaw_deg:.2f}"
            )
            self.last_info_ns = now_ns


def main() -> None:
    rclpy.init()
    node = MockGimbalStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("mock_gimbal_state_node interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
