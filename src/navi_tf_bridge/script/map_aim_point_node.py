#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from gimbal_driver.msg import FireCode, GimbalAngles
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros

try:
    import yaml
except Exception:
    yaml = None


class MapAimPointNode(Node):
    def __init__(self) -> None:
        super().__init__("map_aim_point_node")

        self.declare_parameter("target_x_cm", 1400.0)
        self.declare_parameter("target_y_cm", 750.0)
        self.declare_parameter("target_z_cm", 100.0)
        self.declare_parameter("target_frame", "official_map")
        self.declare_parameter("aim_frame", "gimbal_world")
        self.declare_parameter("camera_frame", "gx_camera")
        self.declare_parameter("solve_mode", "camera_projection")
        self.declare_parameter("solve_frame", "base_link")
        self.declare_parameter("gimbal_angles_topic", "/ly/gimbal/angles")
        self.declare_parameter("control_angles_topic", "/ly/control/angles")
        self.declare_parameter("control_firecode_topic", "/ly/control/firecode")
        self.declare_parameter("publish_firecode", True)
        self.declare_parameter("aim_mode", True)
        self.declare_parameter("bridge_config_file", "")
        self.declare_parameter("use_raw_goal_static_calibration", False)
        self.declare_parameter("raw_goal_target_frame", "")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("tf_timeout_sec", 0.05)
        self.declare_parameter("use_gimbal_stamp_for_tf", False)
        self.declare_parameter("max_gimbal_stamp_age_sec", 0.50)
        self.declare_parameter("min_distance_m", 0.10)
        self.declare_parameter("max_target_distance_m", 100.0)
        self.declare_parameter("command_filter_alpha", 1.0)
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
        self.camera_frame = str(self.get_parameter("camera_frame").value).strip() or "gx_camera"
        self.solve_mode = str(self.get_parameter("solve_mode").value).strip() or "camera_projection"
        self.solve_frame = str(self.get_parameter("solve_frame").value).strip() or "base_link"
        gimbal_topic = str(self.get_parameter("gimbal_angles_topic").value)
        control_topic = str(self.get_parameter("control_angles_topic").value)
        firecode_topic = str(self.get_parameter("control_firecode_topic").value)
        self.publish_firecode = bool(self.get_parameter("publish_firecode").value)
        self.aim_mode = bool(self.get_parameter("aim_mode").value)
        bridge_config_file = str(self.get_parameter("bridge_config_file").value)
        use_static_calibration = bool(self.get_parameter("use_raw_goal_static_calibration").value)
        self.raw_goal_target_frame_override = str(self.get_parameter("raw_goal_target_frame").value).strip()
        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.tf_timeout = Duration(seconds=max(0.01, float(self.get_parameter("tf_timeout_sec").value)))
        self.use_gimbal_stamp_for_tf = bool(self.get_parameter("use_gimbal_stamp_for_tf").value)
        self.max_gimbal_stamp_age_sec = max(
            0.0,
            float(self.get_parameter("max_gimbal_stamp_age_sec").value),
        )
        self.min_distance_m = max(0.01, float(self.get_parameter("min_distance_m").value))
        self.max_target_distance_m = max(0.0, float(self.get_parameter("max_target_distance_m").value))
        self.command_filter_alpha = max(
            0.0,
            min(1.0, float(self.get_parameter("command_filter_alpha").value)),
        )
        self.yaw_sign = float(self.get_parameter("yaw_sign").value)
        self.pitch_sign = float(self.get_parameter("pitch_sign").value)
        self.yaw_bias_deg = float(self.get_parameter("yaw_bias_deg").value)
        self.pitch_bias_deg = float(self.get_parameter("pitch_bias_deg").value)
        self.max_yaw_step_deg = max(0.0, float(self.get_parameter("max_yaw_step_deg").value))
        self.max_pitch_step_deg = max(0.0, float(self.get_parameter("max_pitch_step_deg").value))
        self.active_target_frame = self.target_frame
        self.active_target_point = (self.target_x_m, self.target_y_m, self.target_z_m)
        self.raw_target_point = (self.target_x_m, self.target_y_m, self.target_z_m)
        self.static_calibration_ready = False

        self.current_angles: Optional[GimbalAngles] = None
        self.last_yaw_cmd_deg: Optional[float] = None
        self.last_pitch_cmd_deg: Optional[float] = None
        self.last_warn_ns = 0
        self.last_info_ns = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub_angles = self.create_publisher(GimbalAngles, control_topic, 10)
        self.pub_firecode = (
            self.create_publisher(FireCode, firecode_topic, 10)
            if self.publish_firecode
            else None
        )
        self.sub_angles = self.create_subscription(GimbalAngles, gimbal_topic, self._on_gimbal_angles, 20)
        self.timer = self.create_timer(1.0 / publish_hz, self._on_timer)

        if use_static_calibration:
            self._load_raw_goal_static_calibration(bridge_config_file)

        self.get_logger().info(
            "map aim point started: "
            f"raw_target=({self.target_x_m:.3f}, {self.target_y_m:.3f}, {self.target_z_m:.3f})m@{self.target_frame} "
            f"active_target=({self.active_target_point[0]:.3f}, {self.active_target_point[1]:.3f}, "
            f"{self.active_target_point[2]:.3f})m@{self.active_target_frame} "
            f"solve_mode={self.solve_mode}, solve_frame={self.solve_frame}, "
            f"camera_frame={self.camera_frame} -> {control_topic}, gimbal={gimbal_topic}, "
            f"firecode={'on' if self.publish_firecode else 'off'}, "
            f"use_gimbal_stamp_for_tf={self.use_gimbal_stamp_for_tf}, "
            f"command_filter_alpha={self.command_filter_alpha:.2f}"
        )

    def _on_gimbal_angles(self, msg: GimbalAngles) -> None:
        self.current_angles = msg

    def _load_raw_goal_static_calibration(self, bridge_config_file: str) -> None:
        if yaml is None:
            self.get_logger().warn("PyYAML is unavailable; raw-goal static calibration disabled")
            return
        if not bridge_config_file:
            self.get_logger().warn("bridge_config_file is empty; raw-goal static calibration disabled")
            return
        try:
            with open(bridge_config_file, encoding="utf-8") as fh:
                root = yaml.safe_load(fh) or {}
        except Exception as exc:
            self.get_logger().warn(
                f"failed to load bridge_config_file '{bridge_config_file}': {exc}; "
                "raw-goal static calibration disabled"
            )
            return

        params = root.get("target_rel_to_goal_pos_node", {}).get("ros__parameters", {})
        if not isinstance(params, dict):
            self.get_logger().warn(
                f"missing target_rel_to_goal_pos_node.ros__parameters in '{bridge_config_file}'; "
                "raw-goal static calibration disabled"
            )
            return

        model = str(params.get("raw_goal_calibration_model", "matrix"))
        if model not in ("matrix", "MATRIX", "matrix_4x4"):
            self.get_logger().warn(
                f"map aim point only supports raw_goal_calibration_model=matrix, got '{model}'; "
                "raw-goal static calibration disabled"
            )
            return

        matrix = params.get("raw_goal_transform_matrix", [])
        if not isinstance(matrix, list) or len(matrix) != 16:
            self.get_logger().warn(
                f"raw_goal_transform_matrix in '{bridge_config_file}' must contain 16 values; "
                "raw-goal static calibration disabled"
            )
            return

        unit = str(params.get("raw_goal_calibration_unit", "m"))
        if unit in ("m", "M"):
            unit_scale = 1.0
        elif unit in ("cm", "CM"):
            unit_scale = 0.01
        else:
            self.get_logger().warn(
                f"raw_goal_calibration_unit '{unit}' is invalid; raw-goal static calibration disabled"
            )
            return

        try:
            m = [float(v) for v in matrix]
        except (TypeError, ValueError):
            self.get_logger().warn(
                f"raw_goal_transform_matrix in '{bridge_config_file}' contains non-numeric values; "
                "raw-goal static calibration disabled"
            )
            return

        x, y, z = self.raw_target_point
        # Match target_rel_to_goal_pos_node raw-goal bridge for official-map X/Y.
        # Z is already a map-frame aim height, so keep it unchanged.
        self.active_target_point = (
            m[0] * x + m[1] * y + m[3] * unit_scale,
            m[4] * x + m[5] * y + m[7] * unit_scale,
            z,
        )
        configured_target_frame = str(params.get("raw_goal_target_frame", "map"))
        self.active_target_frame = self.raw_goal_target_frame_override or configured_target_frame
        self.static_calibration_ready = True
        self.get_logger().info(
            "raw-goal static calibration loaded for map aim point: "
            f"official_xy=({x:.3f}, {y:.3f})m z_map={z:.3f}m -> {self.active_target_frame}, "
            f"target=({self.active_target_point[0]:.3f}, {self.active_target_point[1]:.3f}, "
            f"{self.active_target_point[2]:.3f})m"
        )

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

    @staticmethod
    def _stamp_is_zero(msg: GimbalAngles) -> bool:
        return msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0

    def _select_tf_lookup_time(self, msg: GimbalAngles) -> Time:
        if not self.use_gimbal_stamp_for_tf or self._stamp_is_zero(msg):
            return Time()

        stamp = Time.from_msg(msg.header.stamp)
        if self.max_gimbal_stamp_age_sec > 0.0:
            age_sec = (self.get_clock().now().nanoseconds - stamp.nanoseconds) * 1e-9
            if age_sec > self.max_gimbal_stamp_age_sec:
                self._warn_throttled(
                    f"gimbal angle stamp is stale ({age_sec:.3f}s old); skip map aim command"
                )
                raise RuntimeError("stale gimbal stamp")
        return stamp

    @staticmethod
    def _lowpass_angle(target_deg: float, previous_deg: float, alpha: float) -> float:
        target_near = previous_deg + math.remainder(target_deg - previous_deg, 360.0)
        if alpha <= 0.0 or alpha >= 1.0:
            return target_near
        return previous_deg + alpha * (target_near - previous_deg)

    def _warn_throttled(self, text: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_warn_ns > int(2e9):
            self.get_logger().warn(text)
            self.last_warn_ns = now_ns

    def _lookup_target_in_frame(
        self,
        frame: str,
        lookup_time: Time,
        missing_hint: str,
    ) -> Optional[tuple[float, float, float]]:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                frame,
                self.active_target_frame,
                lookup_time,
                timeout=self.tf_timeout,
            )
        except Exception as exc:
            exc_text = str(exc)
            hint = ""
            if "does not exist" in exc_text and self.active_target_frame in exc_text:
                hint = (
                    f"; frame '{self.active_target_frame}' is absent. Start navigation/localization "
                    "that publishes map -> base_link, or use use_mock_map_to_base:=true for bench tests"
                )
            elif "does not exist" in exc_text and frame in exc_text:
                hint = missing_hint
            self._warn_throttled(
                f"TF not ready: {frame} <- {self.active_target_frame}: {exc}{hint}"
            )
            return None

        return self._transform_point(tf_msg, self.active_target_point)

    def _solve_base_link_angles(
        self,
        lookup_time: Time,
        current_yaw_deg: float,
        current_pitch_deg: float,
    ) -> Optional[tuple[float, float, tuple[float, float, float], str]]:
        target = self._lookup_target_in_frame(
            self.solve_frame,
            lookup_time,
            (
                f"; frame '{self.solve_frame}' is absent. Start localization/TF and make sure "
                "base_link is connected to map"
            ),
        )
        if target is None:
            return None

        x, y, z = target
        horizontal = math.hypot(x, y)
        distance = math.hypot(horizontal, z)
        if distance < self.min_distance_m:
            self._warn_throttled(
                f"target too close in {self.solve_frame}: ({x:.3f}, {y:.3f}, {z:.3f})m"
            )
            return None
        if self.max_target_distance_m > 0.0 and distance > self.max_target_distance_m:
            self._warn_throttled(
                f"target distance in {self.solve_frame} is unreasonable ({distance:.3f}m); "
                "skip map aim command. Check odom/localization TF."
            )
            return None

        target_yaw_deg = self.yaw_sign * math.degrees(math.atan2(y, x)) + self.yaw_bias_deg
        pitch_cmd_deg = self.pitch_sign * math.degrees(math.atan2(z, horizontal)) + self.pitch_bias_deg
        yaw_cmd_deg = self._normalize_near(target_yaw_deg, current_yaw_deg)
        detail = (
            f"target_in_{self.solve_frame}=({x:.2f},{y:.2f},{z:.2f})m "
            f"target_yaw={target_yaw_deg:.2f}"
        )
        return yaw_cmd_deg, pitch_cmd_deg, target, detail

    def _solve_camera_projection_angles(
        self,
        lookup_time: Time,
        current_yaw_deg: float,
        current_pitch_deg: float,
    ) -> Optional[tuple[float, float, tuple[float, float, float], str]]:
        target_camera = self._lookup_target_in_frame(
            self.camera_frame,
            lookup_time,
            (
                f"; frame '{self.camera_frame}' is absent. Start sentry_tf/tf_tree and make sure "
                "gimbal_barrel -> gx_camera is published"
            ),
        )
        if target_camera is None:
            return None

        cx, cy, cz = target_camera
        distance = math.sqrt(cx * cx + cy * cy + cz * cz)
        if distance < self.min_distance_m:
            self._warn_throttled(
                f"target too close in {self.camera_frame}: ({cx:.3f}, {cy:.3f}, {cz:.3f})m"
            )
            return None
        if self.max_target_distance_m > 0.0 and distance > self.max_target_distance_m:
            self._warn_throttled(
                f"target distance in {self.camera_frame} is unreasonable ({distance:.3f}m); "
                "skip map aim command. Check odom/localization TF."
            )
            return None
        if cz <= 0.0:
            self._warn_throttled(
                f"target is behind {self.camera_frame}: ({cx:.3f}, {cy:.3f}, {cz:.3f})m"
            )
            return None

        yaw_error_deg = self.yaw_sign * math.degrees(math.atan2(cx, cz)) + self.yaw_bias_deg
        pitch_error_deg = self.pitch_sign * math.degrees(math.atan2(-cy, math.hypot(cx, cz))) + self.pitch_bias_deg
        yaw_cmd_deg = self._normalize_near(current_yaw_deg + yaw_error_deg, current_yaw_deg)
        pitch_cmd_deg = current_pitch_deg + pitch_error_deg

        target_solve = self._lookup_target_in_frame(
            self.solve_frame,
            lookup_time,
            "",
        )
        if target_solve is None:
            target_solve = (float("nan"), float("nan"), float("nan"))
        sx, sy, sz = target_solve
        detail = (
            f"target_in_{self.solve_frame}=({sx:.2f},{sy:.2f},{sz:.2f})m "
            f"target_in_{self.camera_frame}=({cx:.2f},{cy:.2f},{cz:.2f})m "
            f"err_yaw={yaw_error_deg:.2f} err_pitch={pitch_error_deg:.2f}"
        )
        return yaw_cmd_deg, pitch_cmd_deg, target_camera, detail

    def _on_timer(self) -> None:
        if self.current_angles is None:
            self._warn_throttled("waiting for /ly/gimbal/angles before publishing map aim command")
            return

        current_angles = self.current_angles
        try:
            lookup_time = self._select_tf_lookup_time(current_angles)
        except RuntimeError:
            return

        current_yaw_deg = float(current_angles.yaw)
        current_pitch_deg = float(current_angles.pitch)
        if self.solve_mode in ("camera", "camera_projection", "gx_camera"):
            solved = self._solve_camera_projection_angles(
                lookup_time,
                current_yaw_deg,
                current_pitch_deg,
            )
        elif self.solve_mode in ("base", "base_link", "absolute"):
            solved = self._solve_base_link_angles(
                lookup_time,
                current_yaw_deg,
                current_pitch_deg,
            )
        else:
            self._warn_throttled(
                f"unknown solve_mode '{self.solve_mode}'; use camera_projection or base_link"
            )
            return
        if solved is None:
            return
        yaw_cmd_deg, pitch_cmd_deg, _, detail = solved

        if self.last_yaw_cmd_deg is not None and 0.0 < self.command_filter_alpha < 1.0:
            yaw_cmd_deg = self._lowpass_angle(
                yaw_cmd_deg,
                self.last_yaw_cmd_deg,
                self.command_filter_alpha,
            )
        if self.last_pitch_cmd_deg is not None and 0.0 < self.command_filter_alpha < 1.0:
            pitch_cmd_deg = (
                self.last_pitch_cmd_deg
                + self.command_filter_alpha * (pitch_cmd_deg - self.last_pitch_cmd_deg)
            )

        yaw_step_ref = self.last_yaw_cmd_deg if self.last_yaw_cmd_deg is not None else current_yaw_deg
        pitch_step_ref = (
            self.last_pitch_cmd_deg if self.last_pitch_cmd_deg is not None else current_pitch_deg
        )
        yaw_cmd_deg = self._limit_step(yaw_cmd_deg, yaw_step_ref, self.max_yaw_step_deg)
        pitch_cmd_deg = self._limit_step(pitch_cmd_deg, pitch_step_ref, self.max_pitch_step_deg)

        stamp = self.get_clock().now().to_msg()
        angle_msg = GimbalAngles()
        angle_msg.header.stamp = stamp
        angle_msg.yaw = float(yaw_cmd_deg)
        angle_msg.pitch = float(pitch_cmd_deg)
        self.pub_angles.publish(angle_msg)
        self.last_yaw_cmd_deg = float(yaw_cmd_deg)
        self.last_pitch_cmd_deg = float(pitch_cmd_deg)

        if self.pub_firecode is not None:
            fire_msg = FireCode()
            fire_msg.header.stamp = stamp
            fire_msg.field_mask = FireCode.FIELD_FIRE_STATUS | FireCode.FIELD_AIM_MODE
            fire_msg.fire_status = 0
            fire_msg.aim_mode = self.aim_mode
            self.pub_firecode.publish(fire_msg)

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_info_ns > int(2e9):
            self.get_logger().info(
                "map aim command: "
                f"target_{self.active_target_frame}=({self.active_target_point[0]:.2f},"
                f"{self.active_target_point[1]:.2f},{self.active_target_point[2]:.2f})m "
                f"{detail} "
                f"yaw={angle_msg.yaw:.2f} pitch={angle_msg.pitch:.2f} "
                f"current_yaw={current_yaw_deg:.2f} current_pitch={current_pitch_deg:.2f}"
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
