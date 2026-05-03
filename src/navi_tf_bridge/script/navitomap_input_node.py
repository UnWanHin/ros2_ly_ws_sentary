#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

from copy import deepcopy
import re
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray


class NaviToMapInputNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "navitomap_input_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.raw_topic = str(self.get_param_compat("raw_topic", "/ly/navi/goal_pos_raw"))
        self.goal_topic = str(self.get_param_compat("goal_topic", "/goal_pose"))
        self.confirmed_goal_topic = str(
            self.get_param_compat("confirmed_goal_topic", "/goal_pose")
        )
        self.confirm_before_publish = bool(
            self.get_param_compat("confirm_before_publish", False)
        )
        self.conversion_timeout_sec = max(
            0.1,
            float(self.get_param_compat("conversion_timeout_sec", 2.0)),
        )
        self.match_wait_timeout_sec = max(
            0.0,
            float(self.get_param_compat("match_wait_timeout_sec", 8.0)),
        )
        self.input_unit = str(self.get_param_compat("input_unit", "cm")).strip().lower()
        if self.input_unit not in {"cm", "m"}:
            self.get_logger().warn(
                f"Unsupported input_unit={self.input_unit}, fallback to 'cm'."
            )
            self.input_unit = "cm"
        self.echo_goal = bool(self.get_param_compat("echo_goal", True))

        self.pub_raw = self.create_publisher(UInt16MultiArray, self.raw_topic, 10)
        self.pub_confirmed = (
            self.create_publisher(PoseStamped, self.confirmed_goal_topic, 10)
            if self.confirm_before_publish
            else None
        )
        self.sub_goal = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.on_goal_pose,
            10,
        )

        self.stop_event = threading.Event()
        self.seq = 0
        self.goal_condition = threading.Condition()
        self.goal_seq = 0
        self.last_goal_pose: PoseStamped | None = None

        self.get_logger().info(
            f"navitomap input ready: raw_topic={self.raw_topic} "
            f"goal_pose_topic={self.goal_topic} input_unit={self.input_unit} "
            f"confirm_before_publish={self.confirm_before_publish} "
            f"confirmed_goal_pose_topic={self.confirmed_goal_topic}"
        )
        self.get_logger().info(
            "Input format: x y. Example: '1200 650' (cm) or '12.0 6.5' (m). Type 'q' to quit."
        )
        if self.confirm_before_publish and self.goal_topic == self.confirmed_goal_topic:
            self.get_logger().warn(
                "goal_topic equals confirmed_goal_topic; converted poses may already be on the final topic."
            )

        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def get_param_compat(self, name: str, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def wait_for_bridge_matches(self) -> None:
        deadline = time.monotonic() + self.match_wait_timeout_sec
        last_log_time = 0.0
        while rclpy.ok() and not self.stop_event.is_set():
            raw_subscribers = self.count_subscribers(self.raw_topic)
            goal_publishers = self.count_publishers(self.goal_topic)
            raw_ready = raw_subscribers > 0
            goal_ready = (not self.confirm_before_publish) or goal_publishers > 0
            if raw_ready and goal_ready:
                self.get_logger().info(
                    f"bridge topics matched: raw_subscribers={raw_subscribers} "
                    f"goal_pose_publishers={goal_publishers}"
                )
                return

            now = time.monotonic()
            if now >= deadline:
                self.get_logger().warn(
                    f"Timed out waiting for bridge topic matches: "
                    f"{self.raw_topic} subscribers={raw_subscribers}, "
                    f"{self.goal_topic} publishers={goal_publishers}. "
                    "First raw goal may be dropped if ROS discovery is still pending."
                )
                return
            if now - last_log_time >= 1.0:
                self.get_logger().info(
                    f"waiting for bridge topic matches: "
                    f"{self.raw_topic} subscribers={raw_subscribers}, "
                    f"{self.goal_topic} publishers={goal_publishers}"
                )
                last_log_time = now
            time.sleep(0.1)

    def to_cm(self, value_text: str) -> Optional[int]:
        try:
            number = float(value_text)
        except ValueError:
            return None
        if self.input_unit == "m":
            number *= 100.0
        return int(round(number))

    @staticmethod
    def pose_xy_text(pose: PoseStamped) -> str:
        x_m = float(pose.pose.position.x)
        y_m = float(pose.pose.position.y)
        return f"x={x_m:.3f} m y={y_m:.3f} m ({x_m * 100.0:.0f}, {y_m * 100.0:.0f} cm)"

    def on_goal_pose(self, msg: PoseStamped) -> None:
        if self.confirm_before_publish:
            with self.goal_condition:
                self.last_goal_pose = deepcopy(msg)
                self.goal_seq += 1
                self.goal_condition.notify_all()
            return
        if not self.echo_goal:
            return
        self.get_logger().info(
            f"goal_pose <= {self.pose_xy_text(msg)} frame={msg.header.frame_id or '<empty>'}"
        )

    def wait_for_converted_goal(self, start_seq: int) -> PoseStamped | None:
        deadline = time.monotonic() + self.conversion_timeout_sec
        with self.goal_condition:
            while rclpy.ok() and self.goal_seq <= start_seq:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self.goal_condition.wait(timeout=remaining)
            return deepcopy(self.last_goal_pose) if self.last_goal_pose is not None else None

    def ask_confirm_publish(self, goal_pose: PoseStamped) -> bool:
        prompt = (
            f"converted goal_pose {self.pose_xy_text(goal_pose)} "
            f"frame={goal_pose.header.frame_id or '<empty>'}. "
            f"Publish to {self.confirmed_goal_topic}? [y/N] > "
        )
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                answer = input(prompt).strip().lower()
            except EOFError:
                self.get_logger().info("stdin closed, skip confirmed publish.")
                return False
            except KeyboardInterrupt:
                self.stop_event.set()
                return False
            if answer in {"y", "yes"}:
                return True
            if answer in {"", "n", "no"}:
                return False
            print("Input y or n.")
        return False

    def publish_confirmed_goal(self, goal_pose: PoseStamped) -> None:
        if self.pub_confirmed is None:
            return
        out = deepcopy(goal_pose)
        out.header.stamp = self.get_clock().now().to_msg()
        self.pub_confirmed.publish(out)
        self.get_logger().info(
            f"confirmed goal_pose => {self.pose_xy_text(out)} "
            f"frame={out.header.frame_id or '<empty>'} topic={self.confirmed_goal_topic}"
        )

    def input_loop(self) -> None:
        self.wait_for_bridge_matches()
        prompt = "[navitomap] x y > "
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                line = input(prompt)
            except EOFError:
                self.get_logger().info("stdin closed, exit.")
                break
            except KeyboardInterrupt:
                break

            text = line.strip()
            if not text:
                continue
            if text.lower() in {"q", "quit", "exit"}:
                break

            parts = [p for p in re.split(r"[\s,]+", text) if p]
            if len(parts) < 2:
                self.get_logger().warn("Invalid input. Need two numbers: x y")
                continue

            x_cm = self.to_cm(parts[0])
            y_cm = self.to_cm(parts[1])
            if x_cm is None or y_cm is None:
                self.get_logger().warn("Invalid number. Example: 1200 650")
                continue
            if x_cm < 0 or y_cm < 0 or x_cm > 65535 or y_cm > 65535:
                self.get_logger().warn(
                    f"Out of range. Need 0~65535 cm, got x={x_cm} y={y_cm}"
                )
                continue

            msg = UInt16MultiArray()
            msg.data = [x_cm, y_cm]
            with self.goal_condition:
                start_goal_seq = self.goal_seq
            self.pub_raw.publish(msg)
            self.seq += 1
            self.get_logger().info(
                f"published raw_goal[{self.seq}] => x={x_cm} cm y={y_cm} cm "
                f"topic={self.raw_topic}"
            )
            if self.confirm_before_publish:
                goal_pose = self.wait_for_converted_goal(start_goal_seq)
                if goal_pose is None:
                    self.get_logger().warn(
                        f"Timed out waiting for converted goal_pose on {self.goal_topic}."
                    )
                    continue
                if self.ask_confirm_publish(goal_pose):
                    self.publish_confirmed_goal(goal_pose)
                else:
                    self.get_logger().info("skip confirmed goal_pose publish.")

        self.stop_event.set()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NaviToMapInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
