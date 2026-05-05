#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


class ControlModePublisher(Node):
    def __init__(
        self,
        vision_mode: int,
        bt_target: int,
        hz: float,
    ) -> None:
        super().__init__("standalone_control_mode_publisher")
        self.vision_mode = max(0, min(255, int(vision_mode)))
        self.bt_target = max(0, min(255, int(bt_target)))

        self.pub_vision_mode = self.create_publisher(UInt8, "/ly/vision/mode", 10)
        self.pub_target = self.create_publisher(UInt8, "/ly/bt/target", 10)

        period = 1.0 / max(1.0, float(hz))
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            "control mode publisher started: "
            f"vision_mode={self.vision_mode} bt_target={self.bt_target} hz={1.0 / period:.1f}"
        )

    def _on_timer(self) -> None:
        mode_msg = UInt8()
        mode_msg.data = self.vision_mode
        self.pub_vision_mode.publish(mode_msg)

        target_msg = UInt8()
        target_msg.data = self.bt_target
        self.pub_target.publish(target_msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish standalone mode switch topics")
    parser.add_argument("--vision-mode", type=int, default=1, help="0=disabled, 1=armor, 2=buff, 3=outpost")
    parser.add_argument("--bt-target", type=int, default=6)
    parser.add_argument("--hz", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = ControlModePublisher(
        vision_mode=args.vision_mode,
        bt_target=args.bt_target,
        hz=args.hz,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("control mode publisher interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
