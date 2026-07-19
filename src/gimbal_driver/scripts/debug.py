#!/usr/bin/env python3

"""Standalone navigation/aim debug bridge to the formal control topics."""

import math
import yaml

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

from gimbal_driver.msg import ControlVelocity, FireCode, GimbalAngles, Vel
from sentry_msgs.msg import AimResult

from debug_control_state import DebugControlState


NAV_VEL_TOPIC = "/ly/navi/vel"
CONTROL_VEL_TOPIC = "/ly/control/vel"
CONTROL_FIRECODE_TOPIC = "/ly/control/firecode"
CONTROL_ANGLES_TOPIC = "/ly/control/angles"
GIMBAL_ANGLES_TOPIC = "/ly/gimbal/angles"
VELOCITY_RAW_TO_MPS = 0.025


def encode_navigation_raw(value: float) -> int:
    if not math.isfinite(value):
        return 0
    rounded = math.floor(value + 0.5) if value >= 0.0 else math.ceil(value - 0.5)
    return max(-128, min(127, int(rounded)))


class NaviVelToControlVel(Node):
    def __init__(self) -> None:
        super().__init__("debug_control_bridge")
        self.declare_parameter("navi_mode", True)
        self.declare_parameter("aim_mode", False)
        self.declare_parameter("stale_timeout_ms", 500)
        self.declare_parameter("publish_hz", 100.0)
        self.declare_parameter("rotate_level", 1)
        self.declare_parameter("follow_mode_when_false", True)
        self.declare_parameter("patrol", False)
        self.declare_parameter("patrol_config_file", "")
        self.stale_timeout_ns = max(
            1, int(self.get_parameter("stale_timeout_ms").value) * 1_000_000
        )
        self.publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.rotate_level = max(0, min(3, int(self.get_parameter("rotate_level").value)))
        self.follow_mode_when_false = bool(
            self.get_parameter("follow_mode_when_false").value
        )
        self.navi_mode = bool(self.get_parameter("navi_mode").value)
        self.aim_mode = bool(self.get_parameter("aim_mode").value)
        self.patrol = bool(self.get_parameter("patrol").value)
        self.control_state = DebugControlState(
            stale_timeout_ns=self.stale_timeout_ns,
            rotate_level=self.rotate_level,
            follow_mode_when_false=self.follow_mode_when_false,
        )
        self.fire_status = 0
        self.cap_state = 0
        self.feedback_angles: GimbalAngles | None = None
        self.patrol_start_ns: int | None = None
        self.patrol_phase = 0.0
        self.patrol_center_yaw = 0.0
        self.patrol_config = self.load_patrol_config(
            str(self.get_parameter("patrol_config_file").value)
        )

        self.velocity_publisher = self.create_publisher(ControlVelocity, CONTROL_VEL_TOPIC, 10)
        self.firecode_publisher = self.create_publisher(FireCode, CONTROL_FIRECODE_TOPIC, 10)
        self.angles_publisher = self.create_publisher(GimbalAngles, CONTROL_ANGLES_TOPIC, 10)
        self.angle_subscription = self.create_subscription(
            GimbalAngles, GIMBAL_ANGLES_TOPIC, self.on_gimbal_angles, 10
        )
        self.firecode_subscription = self.create_subscription(
            FireCode, "/ly/gimbal/firecode", self.on_gimbal_firecode, 10
        )
        if self.navi_mode:
            self.velocity_subscription = self.create_subscription(
                Vel, NAV_VEL_TOPIC, self.on_navi_vel, 10
            )
            self.rotate_subscription = self.create_subscription(
                Bool, "/ly/navi/should_rotate", self.on_should_rotate, 10
            )
        if self.aim_mode:
            self.aim_subscription = self.create_subscription(
                AimResult, "/ly/aim/result", self.on_aim_result, 10
            )
        self.timer = self.create_timer(1.0 / self.publish_hz, self.publish_control)

    def on_navi_vel(self, message: Vel) -> None:
        self.control_state.update_navigation(
            encode_navigation_raw(message.x),
            encode_navigation_raw(message.y),
            self.get_clock().now().nanoseconds,
        )

    def on_should_rotate(self, message: Bool) -> None:
        self.control_state.update_should_rotate(message.data)

    def on_aim_result(self, message: AimResult) -> None:
        self.control_state.update_aim(
            follow=message.follow,
            fire=message.fire,
            yaw=message.yaw,
            pitch=message.pitch,
            received_ns=self.get_clock().now().nanoseconds,
        )

    def on_gimbal_angles(self, message: GimbalAngles) -> None:
        self.feedback_angles = message
        if self.patrol_start_ns is None:
            self.patrol_start_ns = self.get_clock().now().nanoseconds
            self.patrol_center_yaw = message.yaw

    def on_gimbal_firecode(self, message: FireCode) -> None:
        self.fire_status = message.fire_status & 0b11
        self.cap_state = message.cap_state & 0b11

    @staticmethod
    def load_patrol_config(path: str) -> dict:
        if not path:
            return {}
        try:
            with open(path, encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
            return data.get("behavior_tree", {}).get("ros__parameters", {}).get("PatrolScan", {})
        except (OSError, yaml.YAMLError):
            return {}

    def patrol_angles(self, now_ns: int) -> tuple[float, float]:
        assert self.feedback_angles is not None
        config = self.patrol_config
        mode = int(config.get("Mode", 1))
        elapsed_ms = (now_ns - (self.patrol_start_ns or now_ns)) / 1_000_000.0
        mode_config = config.get(f"Mode{mode}", {})
        if mode == 2:
            half = float(mode_config.get("YawHalfRangeDeg", 30.0))
            step = float(mode_config.get("YawStepDegPerTick", 1.0))
            self.patrol_phase = (self.patrol_phase + step / max(half, 1.0)) % (2.0 * math.pi)
            self.patrol_center_yaw += float(mode_config.get("CenterDriftPerCycleDeg", -70.0)) * step / max(half, 1.0) / (2.0 * math.pi)
            yaw = self.patrol_center_yaw + half * math.sin(self.patrol_phase)
            pitch_center = float(mode_config.get("PitchCenterDeg", 0.0))
        else:
            yaw = self.feedback_angles.yaw + float(mode_config.get("YawStepDegPerTick", 9.0 if mode == 1 else 6.0))
            pitch_center = float(mode_config.get("PitchCenterDeg", mode_config.get("PitchOffsetDeg", 0.0)))
        pitch = pitch_center + float(mode_config.get("PitchHalfRangeDeg", 0.0)) * math.sin(
            elapsed_ms * 2.0 * math.pi / max(float(mode_config.get("PitchPeriodMs", 2000.0)), 1.0)
        )
        return yaw, pitch

    def publish_control(self) -> None:
        now = self.get_clock().now()
        snapshot = self.control_state.snapshot(
            now.nanoseconds,
            navi_mode=self.navi_mode,
            aim_mode=self.aim_mode,
        )

        if snapshot.velocity is not None:
            raw_x, raw_y = snapshot.velocity
            velocity = ControlVelocity()
            velocity.header.stamp = now.to_msg()
            velocity.raw_x = raw_x
            velocity.raw_y = raw_y
            velocity.x_mps = raw_x * VELOCITY_RAW_TO_MPS
            velocity.y_mps = raw_y * VELOCITY_RAW_TO_MPS
            velocity.use_raw = True
            self.velocity_publisher.publish(velocity)

        if snapshot.fire_toggle:
            self.fire_status ^= 0b11

        firecode = FireCode()
        firecode.header.stamp = now.to_msg()
        firecode.field_mask = FireCode.FIELD_ALL
        firecode.fire_status = self.fire_status
        firecode.cap_state = self.cap_state
        firecode.follow_mode = snapshot.follow_mode
        firecode.aim_mode = snapshot.aim_mode
        firecode.rotate = snapshot.rotate
        self.firecode_publisher.publish(firecode)

        angles_to_publish = snapshot.angles
        if angles_to_publish is None and self.feedback_angles is not None:
            if self.patrol:
                angles_to_publish = self.patrol_angles(now.nanoseconds)
            else:
                angles_to_publish = (self.feedback_angles.yaw, self.feedback_angles.pitch)
        if angles_to_publish is not None:
            angles = GimbalAngles()
            angles.header.stamp = now.to_msg()
            angles.yaw, angles.pitch = angles_to_publish
            self.angles_publisher.publish(angles)


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
