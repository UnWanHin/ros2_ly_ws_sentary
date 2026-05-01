from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path
from typing import Any


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Write selected live ROS topic values for decision_viz.")
    parser.add_argument("--state-file", default="/tmp/decision_viz_ros_topics.json")
    parser.add_argument("--hz", type=float, default=20.0)
    args = parser.parse_args(argv)
    if args.hz <= 0:
        parser.error("--hz must be > 0")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from std_msgs.msg import Bool, UInt8, UInt16, UInt16MultiArray
    except ImportError as exc:
        print("ROS Python deps are missing. Source ROS/workspace first.", flush=True)
        print(str(exc), flush=True)
        return 2

    state_file = Path(args.state_file).expanduser().resolve()
    state_file.parent.mkdir(parents=True, exist_ok=True)

    class DecisionVizRosTopicMonitor(Node):
        def __init__(self) -> None:
            super().__init__("decision_viz_ros_topic_monitor")
            self.values: dict[str, dict[str, Any]] = {}
            self.create_subscription(PoseStamped, "/goal_pose", self.on_goal_pose, 10)
            self.create_subscription(UInt16MultiArray, "/ly/navi/goal_pos", self.on_goal_pos, 10)
            self.create_subscription(UInt16MultiArray, "/ly/navi/goal_pos_raw", self.on_goal_pos_raw, 10)
            self.create_subscription(UInt8, "/ly/navi/goal", self.on_goal, 10)
            self.create_subscription(UInt8, "/ly/navi/speed_level", self.on_speed_level, 10)
            self.create_subscription(Bool, "/ly/game/is_start", self.on_game_start, 10)
            self.create_subscription(UInt16, "/ly/game/time_left", self.on_time_left, 10)
            self.timer = self.create_timer(1.0 / float(args.hz), self.write_state)
            self.get_logger().info(f"writing live ROS topic state to {state_file}")

        def put(self, topic: str, value: Any) -> None:
            self.values[topic] = {
                "value": value,
                "wall_time": time.time(),
            }

        def on_goal_pose(self, msg: Any) -> None:
            self.put(
                "/goal_pose",
                [
                    float(msg.pose.position.x) * 100.0,
                    float(msg.pose.position.y) * 100.0,
                ],
            )

        def on_goal_pos(self, msg: Any) -> None:
            self.put("/ly/navi/goal_pos", [int(item) for item in msg.data[:2]])

        def on_goal_pos_raw(self, msg: Any) -> None:
            self.put("/ly/navi/goal_pos_raw", [int(item) for item in msg.data[:2]])

        def on_goal(self, msg: Any) -> None:
            self.put("/ly/navi/goal", int(msg.data))

        def on_speed_level(self, msg: Any) -> None:
            self.put("/ly/navi/speed_level", int(msg.data))

        def on_game_start(self, msg: Any) -> None:
            self.put("/ly/game/is_start", bool(msg.data))

        def on_time_left(self, msg: Any) -> None:
            self.put("/ly/game/time_left", int(msg.data))

        def write_state(self) -> None:
            payload = {
                "schema": "ly_decision_viz_ros_topics_v1",
                "wall_time": time.time(),
                "topics": self.values,
            }
            tmp_path = state_file.with_suffix(state_file.suffix + ".tmp")
            with tmp_path.open("w", encoding="utf-8") as stream:
                json.dump(payload, stream, ensure_ascii=True, separators=(",", ":"))
                stream.write("\n")
            os.replace(tmp_path, state_file)

    rclpy.init(args=None)
    node = DecisionVizRosTopicMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if exc.__class__.__name__ != "ExternalShutdownException":
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
