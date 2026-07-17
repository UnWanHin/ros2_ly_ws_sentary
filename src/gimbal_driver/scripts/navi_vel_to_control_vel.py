#!/usr/bin/env python3

"""Standalone debug bridge from navigation velocity to the formal control topic."""

import math
import yaml

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

from gimbal_driver.msg import ControlVelocity, FireCode, GimbalAngles, Vel


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
        super().__init__("navi_vel_to_control_vel")
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
        self.latest_raw_x = 0
        self.latest_raw_y = 0
        self.last_rx_ns: int | None = None
        self.should_rotate = True
        self.patrol = bool(self.get_parameter("patrol").value)
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
        self.velocity_subscription = self.create_subscription(
            Vel, NAV_VEL_TOPIC, self.on_navi_vel, 10
        )
        self.rotate_subscription = self.create_subscription(
            Bool, "/ly/navi/should_rotate", self.on_should_rotate, 10
        )
        self.angle_subscription = self.create_subscription(
            GimbalAngles, GIMBAL_ANGLES_TOPIC, self.on_gimbal_angles, 10
        )
        self.timer = self.create_timer(1.0 / self.publish_hz, self.publish_control)

    def on_navi_vel(self, message: Vel) -> None:
        self.latest_raw_x = encode_navigation_raw(message.x)
        self.latest_raw_y = encode_navigation_raw(message.y)
        self.last_rx_ns = self.get_clock().now().nanoseconds

    def on_should_rotate(self, message: Bool) -> None:
        self.should_rotate = message.data

    def on_gimbal_angles(self, message: GimbalAngles) -> None:
        self.feedback_angles = message
        if self.patrol_start_ns is None:
            self.patrol_start_ns = self.get_clock().now().nanoseconds
            self.patrol_center_yaw = message.yaw

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
        if self.feedback_angles is None:
            return
        if self.last_rx_ns is None or now.nanoseconds - self.last_rx_ns > self.stale_timeout_ns:
            raw_x = 0
            raw_y = 0
        else:
            raw_x = self.latest_raw_x
            raw_y = self.latest_raw_y

        velocity = ControlVelocity()
        velocity.header.stamp = now.to_msg()
        velocity.raw_x = raw_x
        velocity.raw_y = raw_y
        velocity.x_mps = raw_x * VELOCITY_RAW_TO_MPS
        velocity.y_mps = raw_y * VELOCITY_RAW_TO_MPS
        velocity.use_raw = True
        self.velocity_publisher.publish(velocity)

        firecode = FireCode()
        firecode.header.stamp = now.to_msg()
        firecode.field_mask = FireCode.FIELD_FOLLOW_MODE | FireCode.FIELD_ROTATE
        firecode.follow_mode = self.follow_mode_when_false and not self.should_rotate
        firecode.rotate = self.rotate_level if self.should_rotate else 0
        self.firecode_publisher.publish(firecode)

        angles = GimbalAngles()
        angles.header.stamp = now.to_msg()
        if self.patrol:
            angles.yaw, angles.pitch = self.patrol_angles(now.nanoseconds)
        else:
            angles.yaw = self.feedback_angles.yaw
            angles.pitch = self.feedback_angles.pitch
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
