#!/usr/bin/env python3
import argparse
from typing import List, Set

import rclpy
from rclpy.node import Node

from auto_aim_common.msg import Target
from gimbal_driver.msg import GimbalAngles
from std_msgs.msg import Bool, UInt8


def str2bool(value: str) -> bool:
    if isinstance(value, bool):
        return value
    lowered = value.lower()
    if lowered in ("true", "1", "yes", "y", "on"):
        return True
    if lowered in ("false", "0", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value: {value}")


class TargetToGimbalNode(Node):
    def __init__(self, is_red=True, target_id=6, enable_fire=True, auto_fire=True, diag_period=1.0):
        super().__init__("target_to_gimbal_mapper")

        self.is_red = is_red
        self.target_id = target_id
        self.enable_fire = enable_fire
        self.auto_fire = auto_fire
        self.diag_period = max(0.2, float(diag_period))

        self.fire_status = 0
        self.last_fire_command = False

        self.rx_target_count = 0
        self.tx_angle_count = 0
        self.tx_fire_count = 0
        self.last_yaw = 0.0
        self.last_pitch = 0.0
        self.last_status = False
        self.last_diag_time = self.get_clock().now()

        self.control_angles_topic = self.resolve_topic_name("/ly/control/angles")
        self.control_firecode_topic = self.resolve_topic_name("/ly/control/firecode")

        self.last_warned_angle_pubs = 0
        self.last_warned_fire_pubs = 0

        self.target_sub = self.create_subscription(
            Target,
            "/ly/predictor/target",
            self.target_callback,
            10,
        )
        self.gimbal_pub = self.create_publisher(GimbalAngles, "/ly/control/angles", 10)
        self.firecode_pub = self.create_publisher(UInt8, "/ly/control/firecode", 10)
        self.team_pub = self.create_publisher(Bool, "/ly/me/is_team_red", 10)
        self.bt_target_pub = self.create_publisher(UInt8, "/ly/bt/target", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        if self.enable_fire:
            self.publish_firecode(self.fire_status)
            self.get_logger().info(f"初始火控状态: 0b{self.fire_status:08b}={self.fire_status}")

        self.get_logger().info(
            f"节点启动: 队伍={'红方' if is_red else '蓝方'}, "
            f"target_id={target_id}, "
            f"火控={'启用' if enable_fire else '禁用'}, "
            f"自动开火={'是' if auto_fire else '否'}, "
            f"diag_period={self.diag_period:.1f}s"
        )
        self.get_logger().info(
            f"控制话题: angles={self.control_angles_topic}, firecode={self.control_firecode_topic}"
        )

    @staticmethod
    def format_pub_nodes(pub_infos) -> List[str]:
        nodes: Set[str] = set()
        for info in pub_infos:
            namespace = info.node_namespace or "/"
            node_name = info.node_name or "unknown"
            full_name = f"{namespace.rstrip('/')}/{node_name}".replace("//", "/")
            nodes.add(full_name)
        return sorted(nodes)

    def publish_firecode(self, fire_status: int) -> None:
        fire_msg = UInt8()
        fire_msg.data = fire_status
        self.firecode_pub.publish(fire_msg)
        self.tx_fire_count += 1

    def target_callback(self, msg: Target):
        self.rx_target_count += 1
        self.last_yaw = msg.yaw
        self.last_pitch = msg.pitch
        self.last_status = bool(msg.status)

        gimbal_msg = GimbalAngles()
        gimbal_msg.header = msg.header
        gimbal_msg.yaw = msg.yaw
        gimbal_msg.pitch = msg.pitch
        self.gimbal_pub.publish(gimbal_msg)
        self.tx_angle_count += 1

        if self.enable_fire and self.auto_fire:
            self.last_fire_command = msg.status

    def emit_diag(self):
        angle_pub_infos = self.get_publishers_info_by_topic(self.control_angles_topic)
        fire_pub_infos = self.get_publishers_info_by_topic(self.control_firecode_topic)
        angle_pub_nodes = self.format_pub_nodes(angle_pub_infos)
        fire_pub_nodes = self.format_pub_nodes(fire_pub_infos)

        if len(angle_pub_nodes) > 1 and len(angle_pub_nodes) != self.last_warned_angle_pubs:
            self.get_logger().warn(
                f"{self.control_angles_topic} 存在多发布者({len(angle_pub_nodes)}): {', '.join(angle_pub_nodes)}"
            )
            self.last_warned_angle_pubs = len(angle_pub_nodes)
        elif len(angle_pub_nodes) <= 1:
            self.last_warned_angle_pubs = 0

        if len(fire_pub_nodes) > 1 and len(fire_pub_nodes) != self.last_warned_fire_pubs:
            self.get_logger().warn(
                f"{self.control_firecode_topic} 存在多发布者({len(fire_pub_nodes)}): {', '.join(fire_pub_nodes)}"
            )
            self.last_warned_fire_pubs = len(fire_pub_nodes)
        elif len(fire_pub_nodes) <= 1:
            self.last_warned_fire_pubs = 0

        self.get_logger().info(
            f"diag: rx_target={self.rx_target_count} tx_angles={self.tx_angle_count} "
            f"tx_fire={self.tx_fire_count} last(yaw={self.last_yaw:.2f}, pitch={self.last_pitch:.2f}, "
            f"status={int(self.last_status)})"
        )

    def timer_callback(self):
        team_msg = Bool()
        team_msg.data = self.is_red
        self.team_pub.publish(team_msg)

        target_msg = UInt8()
        target_msg.data = self.target_id
        self.bt_target_pub.publish(target_msg)

        if self.enable_fire and self.auto_fire:
            if self.last_fire_command:
                self.fire_status = 0b11 if self.fire_status == 0 else 0b00
            self.publish_firecode(self.fire_status)
        elif self.enable_fire:
            self.publish_firecode(self.fire_status)

        now = self.get_clock().now()
        if (now - self.last_diag_time).nanoseconds >= int(self.diag_period * 1e9):
            self.last_diag_time = now
            self.emit_diag()


def main(args=None):
    parser = argparse.ArgumentParser(description="Target到云台角度映射节点")
    parser.add_argument(
        "--red",
        type=str2bool,
        default=True,
        help="是否为红方 (默认: true)",
    )
    parser.add_argument(
        "--target-id",
        type=int,
        default=6,
        help="BT目标ID (默认: 6)",
    )
    parser.add_argument(
        "--enable-fire",
        type=str2bool,
        default=True,
        help="是否启用火控功能 (默认: true)",
    )
    parser.add_argument(
        "--auto-fire",
        type=str2bool,
        default=True,
        help="是否自动开火 (默认: true)",
    )
    parser.add_argument(
        "--diag-period",
        type=float,
        default=1.0,
        help="诊断日志周期（秒，默认: 1.0）",
    )

    cli_args, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = TargetToGimbalNode(
        is_red=cli_args.red,
        target_id=cli_args.target_id,
        enable_fire=cli_args.enable_fire,
        auto_fire=cli_args.auto_fire,
        diag_period=cli_args.diag_period,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到Ctrl+C，节点退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
