#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from gimbal_driver.msg import GimbalAngles
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


class MapAimPointNode(Node):
    def __init__(self) -> None:
        super().__init__("map_aim_point_node")

        self.declare_parameter("target_x_cm", 1400.0)
        self.declare_parameter("target_y_cm", 750.0)
        self.declare_parameter("target_z_cm", 100.0)
        self.declare_parameter("target_frame", "official_map")
        self.declare_parameter("aim_frame", "gimbal_barrel_joint")
        self.declare_parameter("gimbal_angles_topic", "/ly/gimbal/angles")
        self.declare_parameter("control_angles_topic", "/ly/control/angles")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("tf_timeout_sec", 0.05)
        self.declare_parameter("min_distance_m", 0.10)
        self.declare_parameter("yaw_sign", 1.0)
        self.declare_parameter("pitch_sign", 1.0)
        self.declare_parameter("yaw_bias_deg", 0.0)
        self.declare_parameter("pitch_bias_deg", 0.0)
        self.declare_parameter("max_yaw_step_deg", 0.0)
        self.declare_parameter("max_pitch_step_deg", 0.0)

        self.target_x_m = float(self.get_parameter("target_x_cm").value) * 0.01
        self.target_y_m = float(self.get_parameter("target_y_cm").value) * 0.01
        self.target_z_m = float(self.get_parameter("target_z_cm").value) * 0.01
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.aim_frame = str(self.get_parameter("aim_frame").value)
        gimbal_topic = str(self.get_parameter("gimbal_angles_topic").value)
        control_topic = str(self.get_parameter("control_angles_topic").value)
        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.tf_timeout = Duration(seconds=max(0.01, float(self.get_parameter("tf_timeout_sec").value)))
        self.min_distance_m = max(0.01, float(self.get_parameter("min_distance_m").value))
        self.yaw_sign = float(self.get_parameter("yaw_sign").value)
        self.pitch_sign = float(self.get_parameter("pitch_sign").value)
        self.yaw_bias_deg = float(self.get_parameter("yaw_bias_deg").value)
        self.pitch_bias_deg = float(self.get_parameter("pitch_bias_deg").value)
        self.max_yaw_step_deg = max(0.0, float(self.get_parameter("max_yaw_step_deg").value))
        self.max_pitch_step_deg = max(0.0, float(self.get_parameter("max_pitch_step_deg").value))

        self.current_angles: Optional[GimbalAngles] = None
        self.last_warn_ns = 0
        self.last_info_ns = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub_angles = self.create_publisher(GimbalAngles, control_topic, 10)
        self.sub_angles = self.create_subscription(GimbalAngles, gimbal_topic, self._on_gimbal_angles, 20)
        self.timer = self.create_timer(1.0 / publish_hz, self._on_timer)

        self.get_logger().info(
            "map aim point started: "
            f"target=({self.target_x_m:.3f}, {self.target_y_m:.3f}, {self.target_z_m:.3f})m@{self.target_frame} "
            f"aim_frame={self.aim_frame} -> {control_topic}, gimbal={gimbal_topic}"
        )

    def _on_gimbal_angles(self, msg: GimbalAngles) -> None:
        self.current_angles = msg

    @staticmethod
    def _rotate_vector(qx: float, qy: float, qz: float, qw: float, vector: tuple[float, float, float]) -> tuple[float, float, float]:
        vx, vy, vz = vector
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        rx = vx + qw * tx + (qy * tz - qz * ty)
        ry = vy + qw * ty + (qz * tx - qx * tz)
        rz = vz + qw * tz + (qx * ty - qy * tx)
        return rx, ry, rz

    @classmethod
    def _transform_point(cls, tf_msg: TransformStamped, point: tuple[float, float, float]) -> tuple[float, float, float]:
        q = tf_msg.transform.rotation
        t = tf_msg.transform.translation
        rx, ry, rz = cls._rotate_vector(q.x, q.y, q.z, q.w, point)
        return rx + t.x, ry + t.y, rz + t.z

    @staticmethod
    def _normalize_near(target_deg: float, reference_deg: float) -> float:
        return reference_deg + math.remainder(target_deg - reference_deg, 360.0)

    @staticmethod
    def _limit_step(target_deg: float, current_deg: float, max_step_deg: float) -> float:
        if max_step_deg <= 0.0:
            return target_deg
        delta = math.remainder(target_deg - current_deg, 360.0)
        delta = max(-max_step_deg, min(max_step_deg, delta))
        return current_deg + delta

    def _warn_throttled(self, text: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_warn_ns > int(2e9):
            self.get_logger().warn(text)
            self.last_warn_ns = now_ns

    def _on_timer(self) -> None:
        if self.current_angles is None:
            self._warn_throttled("waiting for /ly/gimbal/angles before publishing map aim command")
            return

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.aim_frame,
                self.target_frame,
                Time(),
                timeout=self.tf_timeout,
            )
        except Exception as exc:
            self._warn_throttled(f"TF not ready: {self.aim_frame} <- {self.target_frame}: {exc}")
            return

        x, y, z = self._transform_point(
            tf_msg,
            (self.target_x_m, self.target_y_m, self.target_z_m),
        )
        horizontal = math.hypot(x, y)
        distance = math.hypot(horizontal, z)
        if distance < self.min_distance_m:
            self._warn_throttled(
                f"target too close in {self.aim_frame}: ({x:.3f}, {y:.3f}, {z:.3f})m"
            )
            return

        yaw_error_deg = self.yaw_sign * math.degrees(math.atan2(y, x)) + self.yaw_bias_deg
        pitch_cmd_deg = self.pitch_sign * math.degrees(math.atan2(z, horizontal)) + self.pitch_bias_deg
        yaw_cmd_deg = float(self.current_angles.yaw) + yaw_error_deg
        yaw_cmd_deg = self._normalize_near(yaw_cmd_deg, float(self.current_angles.yaw))
        yaw_cmd_deg = self._limit_step(yaw_cmd_deg, float(self.current_angles.yaw), self.max_yaw_step_deg)
        pitch_cmd_deg = self._limit_step(pitch_cmd_deg, float(self.current_angles.pitch), self.max_pitch_step_deg)

        msg = GimbalAngles()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.yaw = float(yaw_cmd_deg)
        msg.pitch = float(pitch_cmd_deg)
        self.pub_angles.publish(msg)

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_info_ns > int(2e9):
            self.get_logger().info(
                "map aim command: "
                f"target_in_{self.aim_frame}=({x:.2f},{y:.2f},{z:.2f})m "
                f"yaw={msg.yaw:.2f} pitch={msg.pitch:.2f} "
                f"err_yaw={yaw_error_deg:.2f}"
            )
            self.last_info_ns = now_ns


def main() -> None:
    rclpy.init()
    node = MapAimPointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("map_aim_point_node interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
