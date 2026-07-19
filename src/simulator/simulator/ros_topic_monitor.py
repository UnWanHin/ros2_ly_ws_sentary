from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path
from typing import Any


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Write selected live ROS topic values for simulator.")
    parser.add_argument("--state-file", default="/tmp/simulator_ros_topics.json")
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

    try:
        from gimbal_driver.msg import ControlVelocity, FireCode, GimbalAngles, GimbalTrajectory
    except ImportError:
        ControlVelocity = None
        FireCode = None
        GimbalAngles = None
        GimbalTrajectory = None

    state_file = Path(args.state_file).expanduser().resolve()
    state_file.parent.mkdir(parents=True, exist_ok=True)

    class SimulatorRosTopicMonitor(Node):
        def __init__(self) -> None:
            super().__init__("simulator_ros_topic_monitor")
            self.values: dict[str, dict[str, Any]] = {}
            self.create_subscription(PoseStamped, "/goal_pose", self.on_goal_pose, 10)
            self.create_subscription(UInt16MultiArray, "/ly/navi/goal_pos", self.on_goal_pos, 10)
            self.create_subscription(UInt16MultiArray, "/ly/navi/goal_pos_raw", self.on_goal_pos_raw, 10)
            self.create_subscription(UInt8, "/ly/navi/goal", self.on_goal, 10)
            self.create_subscription(UInt8, "/ly/navi/speed_level", self.on_speed_level, 10)
            self.create_subscription(Bool, "/ly/navi/should_rotate", self.on_should_rotate, 10)
            self.create_subscription(Bool, "/ly/game/is_start", self.on_game_start, 10)
            self.create_subscription(UInt16, "/ly/game/time_left", self.on_time_left, 10)
            optional_types_missing: list[str] = []
            if GimbalAngles is None:
                optional_types_missing.append("gimbal_driver/msg/GimbalAngles")
            else:
                self.create_subscription(GimbalAngles, "/ly/control/angles", self.on_control_angles, 10)
            if FireCode is None:
                optional_types_missing.append("gimbal_driver/msg/FireCode")
            else:
                self.create_subscription(FireCode, "/ly/control/firecode", self.on_control_firecode, 10)
            if GimbalTrajectory is None:
                optional_types_missing.append("gimbal_driver/msg/GimbalTrajectory")
            else:
                self.create_subscription(
                    GimbalTrajectory,
                    "/ly/control/trajectory",
                    self.on_control_trajectory,
                    10,
                )
            if ControlVelocity is None:
                optional_types_missing.append("gimbal_driver/msg/ControlVelocity")
            else:
                self.create_subscription(ControlVelocity, "/ly/control/vel", self.on_control_vel, 10)
            self.timer = self.create_timer(1.0 / float(args.hz), self.write_state)
            self.get_logger().info(f"writing live ROS topic state to {state_file}")
            if optional_types_missing:
                self.get_logger().warn(
                    "optional control topic monitoring unavailable: " + ", ".join(optional_types_missing)
                )

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

        def on_should_rotate(self, msg: Any) -> None:
            self.put("/ly/navi/should_rotate", bool(msg.data))

        def on_control_angles(self, msg: Any) -> None:
            self.put(
                "/ly/control/angles",
                {
                    "yaw": float(msg.yaw),
                    "pitch": float(msg.pitch),
                },
            )

        def on_control_firecode(self, msg: Any) -> None:
            self.put(
                "/ly/control/firecode",
                {
                    "field_mask": int(msg.field_mask),
                    "fire_status": int(msg.fire_status),
                    "cap_state": int(msg.cap_state),
                    "follow_mode": bool(msg.follow_mode),
                    "aim_mode": bool(msg.aim_mode),
                    "rotate": int(msg.rotate),
                    "raw": int(msg.raw),
                },
            )

        def on_control_trajectory(self, msg: Any) -> None:
            self.put(
                "/ly/control/trajectory",
                {
                    "yaw": float(msg.yaw),
                    "yaw_omega": float(msg.yaw_omega),
                    "yaw_alpha": float(msg.yaw_alpha),
                    "pitch": float(msg.pitch),
                    "pitch_omega": float(msg.pitch_omega),
                    "pitch_alpha": float(msg.pitch_alpha),
                },
            )

        def on_control_vel(self, msg: Any) -> None:
            self.put(
                "/ly/control/vel",
                {
                    "x_mps": float(msg.x_mps),
                    "y_mps": float(msg.y_mps),
                    "raw_x": int(msg.raw_x),
                    "raw_y": int(msg.raw_y),
                    "use_raw": bool(msg.use_raw),
                },
            )

        def on_game_start(self, msg: Any) -> None:
            self.put("/ly/game/is_start", bool(msg.data))

        def on_time_left(self, msg: Any) -> None:
            self.put("/ly/game/time_left", int(msg.data))

        def write_state(self) -> None:
            payload = {
                "schema": "ly_simulator_ros_topics_v1",
                "wall_time": time.time(),
                "topics": self.values,
            }
            tmp_path = state_file.with_suffix(state_file.suffix + ".tmp")
            with tmp_path.open("w", encoding="utf-8") as stream:
                json.dump(payload, stream, ensure_ascii=True, separators=(",", ":"))
                stream.write("\n")
            os.replace(tmp_path, state_file)

    rclpy.init(args=None)
    node = SimulatorRosTopicMonitor()
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
