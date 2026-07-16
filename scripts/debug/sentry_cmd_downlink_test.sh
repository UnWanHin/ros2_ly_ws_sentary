#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

MODE="cycle"
POSTURE="2"
TX_TOPIC="/ly/control/sentry_cmd"
POSTURE_TOPIC="/ly/control/posture"
RAW_TX_TOPIC="/ly/log/gimbal_raw_tx"
RAW_RX_TOPIC="/ly/log/gimbal_raw_rx"
LAUNCH_GIMBAL=1
WAIT_SEC=3
WATCH_SEC=5
LOOP=0
INTERVAL_SEC=5
USE_VIRTUAL_DEVICE="false"
OUTPUT_MODE="screen"
CONFIG_FILE="${ROOT_DIR}/src/gimbal_driver/config/gimbal_driver_config.yaml"
RAW_ECHO=1
UPLINK_ECHO=0
UPLINK_TOPICS=("/ly/gimbal/posture" "/ly/game/rfid" "/ly/game/sentry/info")
GIMBAL_PID=""
RAW_ECHO_PID=""
UPLINK_ECHO_PIDS=()
BRIDGE_PID=""
STACK_NODE_REGEX="/(gimbal_driver_node)([[:space:]]|$)"
STACK_LAUNCH_REGEX="ros2 launch gimbal_driver gimbal_driver.launch.py"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/gimbal_test_lifecycle.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [cycle|noop|posture|bridge-posture|sentry-posture|energy-pulse|watch|uplink] [options]

Modes:
  cycle           Default. Launch gimbal_driver, relay ${POSTURE_TOPIC} -> ${TX_TOPIC},
                  publish posture 1 -> 2 -> 3 every ${INTERVAL_SEC}s, and monitor downlink/uplink.
  noop            Publish a zero SentryCmd snapshot to ${TX_TOPIC}. No motion command.
  posture         Publish posture through ${POSTURE_TOPIC}; verifies posture -> sentry_cmd bit21-22.
  bridge-posture  Start a temporary ${POSTURE_TOPIC} -> ${TX_TOPIC} relay, then publish posture.
  sentry-posture  Publish posture through ${TX_TOPIC}; verifies full SentryCmd topic path.
  energy-pulse    Publish confirm_energy_activate true then false through ${TX_TOPIC}.
  watch           Launch/attach and only echo ${RAW_TX_TOPIC}.
  uplink          Launch/attach and echo posture/RFID/sentry-info plus ${RAW_RX_TOPIC}; no downlink publish.

Options:
  --posture VALUE       Posture for posture modes: 1=Attack, 2=Defense, 3=Move. Default: ${POSTURE}
  --tx-topic TOPIC      Full SentryCmd command topic. Default: ${TX_TOPIC}
  --posture-topic TOPIC Posture command topic. Default: ${POSTURE_TOPIC}
  --raw-topic TOPIC     Raw downlink debug topic. Default: ${RAW_TX_TOPIC}
  --raw-rx-topic TOPIC  Raw uplink debug topic. Default: ${RAW_RX_TOPIC}
  --watch-sec SEC       Seconds to keep raw echo after publish. Default: ${WATCH_SEC}
  --loop                Publish repeatedly until Ctrl+C.
  --interval SEC        Publish interval for cycle/--loop. Default: ${INTERVAL_SEC}
  --raw-echo / --no-raw-echo
                       Whether to echo raw TX frames during publish. Default: echo.
  --uplink-echo / --no-uplink-echo
                       Whether to echo uplink topics during publish. Default: no uplink echo.
  --launch-gimbal / --no-launch-gimbal
                       Whether to start gimbal_driver automatically. Default: launch.
  --wait SEC            Wait time after starting gimbal_driver. Default: ${WAIT_SEC}
  --config-file FILE    Params YAML for gimbal_driver. Default: ${CONFIG_FILE}
  --use-virtual-device true|false
                       Whether to force virtual device in gimbal_driver. Default: ${USE_VIRTUAL_DEVICE}
  --output screen|log   gimbal_driver launch output. Default: ${OUTPUT_MODE}
  -h, --help            Show help.

Expected sentry_cmd_raw values:
  noop            0
  posture=1       2097152
  posture=2       4194304
  posture=3       6291456
  energy-pulse    8388608 then 0
EOF
}

cleanup() {
  if [[ -n "${RAW_ECHO_PID:-}" ]] && kill -0 "${RAW_ECHO_PID}" 2>/dev/null; then
    kill -TERM "${RAW_ECHO_PID}" 2>/dev/null || true
    wait "${RAW_ECHO_PID}" 2>/dev/null || true
  fi
  local pid
  for pid in "${UPLINK_ECHO_PIDS[@]:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
  if [[ -n "${BRIDGE_PID:-}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    kill -TERM "${BRIDGE_PID}" 2>/dev/null || true
    wait "${BRIDGE_PID}" 2>/dev/null || true
  fi
  gimbal_test_stop_driver
}

trap cleanup EXIT INT TERM

while [[ $# -gt 0 ]]; do
  case "$1" in
    cycle|noop|posture|bridge-posture|bridge_posture|sentry-posture|sentry_posture|energy-pulse|energy_pulse|watch|uplink)
      MODE="${1//_/-}"
      shift
      ;;
    --posture)
      POSTURE="$2"
      shift 2
      ;;
    --tx-topic)
      TX_TOPIC="$2"
      shift 2
      ;;
    --posture-topic)
      POSTURE_TOPIC="$2"
      shift 2
      ;;
    --raw-topic)
      RAW_TX_TOPIC="$2"
      shift 2
      ;;
    --raw-rx-topic)
      RAW_RX_TOPIC="$2"
      shift 2
      ;;
    --watch-sec)
      WATCH_SEC="$2"
      shift 2
      ;;
    --loop)
      LOOP=1
      shift
      ;;
    --once)
      LOOP=0
      shift
      ;;
    --interval)
      INTERVAL_SEC="$2"
      shift 2
      ;;
    --raw-echo)
      RAW_ECHO=1
      shift
      ;;
    --no-raw-echo)
      RAW_ECHO=0
      shift
      ;;
    --uplink-echo)
      UPLINK_ECHO=1
      shift
      ;;
    --no-uplink-echo)
      UPLINK_ECHO=0
      shift
      ;;
    --launch-gimbal)
      LAUNCH_GIMBAL=1
      shift
      ;;
    --no-launch-gimbal)
      LAUNCH_GIMBAL=0
      shift
      ;;
    --wait)
      WAIT_SEC="$2"
      shift 2
      ;;
    --config-file)
      CONFIG_FILE="$2"
      shift 2
      ;;
    --use-virtual-device)
      USE_VIRTUAL_DEVICE="$2"
      shift 2
      ;;
    --output)
      OUTPUT_MODE="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

source_ros_workspace "${ROOT_DIR}"

validate_posture() {
  if ! [[ "${POSTURE}" =~ ^[1-3]$ ]]; then
    echo "[ERROR] posture must be 1..3, got: ${POSTURE}" >&2
    exit 2
  fi
}

expected_posture_raw() {
  echo $((POSTURE << 21))
}

launch_gimbal_driver() {
  if (( LAUNCH_GIMBAL == 0 )); then
    return 0
  fi

  echo "[SENTRY-CMD-DOWNLINK][INFO] Launching gimbal_driver with raw debug topics enabled" >&2
  gimbal_test_launch_driver "SENTRY-CMD-DOWNLINK" "${CONFIG_FILE}" "${USE_VIRTUAL_DEVICE}" "${OUTPUT_MODE}" "${WAIT_SEC}" "${STACK_NODE_REGEX}" "${STACK_LAUNCH_REGEX}" \
    "raw_topic_enable:=true" \
    "raw_topic_downlink:=true" \
    "raw_topic_uplink:=true"
}

start_raw_echo() {
  if (( RAW_ECHO == 0 )); then
    return 0
  fi

  echo "[SENTRY-CMD-DOWNLINK][INFO] Echoing raw downlink frames from ${RAW_TX_TOPIC}" >&2
  if (( LOOP == 1 )); then
    ros2 topic echo "${RAW_TX_TOPIC}" &
  else
    timeout "${WATCH_SEC}" ros2 topic echo "${RAW_TX_TOPIC}" &
  fi
  RAW_ECHO_PID="$!"
  sleep 1
}

wait_raw_echo() {
  if [[ -z "${RAW_ECHO_PID:-}" ]]; then
    return 0
  fi

  wait "${RAW_ECHO_PID}" 2>/dev/null || true
  RAW_ECHO_PID=""
}

start_uplink_echo() {
  if (( UPLINK_ECHO == 0 )); then
    return 0
  fi

  local topic
  echo "[SENTRY-CMD-DOWNLINK][INFO] Echoing uplink topics for ${WATCH_SEC}s" >&2
  for topic in "${UPLINK_TOPICS[@]}"; do
    echo "[SENTRY-CMD-DOWNLINK][INFO]   ${topic}" >&2
    if (( LOOP == 1 )); then
      ros2 topic echo "${topic}" &
    else
      timeout "${WATCH_SEC}" ros2 topic echo "${topic}" &
    fi
    UPLINK_ECHO_PIDS+=("$!")
  done
  echo "[SENTRY-CMD-DOWNLINK][INFO]   ${RAW_RX_TOPIC}" >&2
  if (( LOOP == 1 )); then
    ros2 topic echo "${RAW_RX_TOPIC}" &
  else
    timeout "${WATCH_SEC}" ros2 topic echo "${RAW_RX_TOPIC}" &
  fi
  UPLINK_ECHO_PIDS+=("$!")
  sleep 1
}

wait_uplink_echo() {
  local pid
  for pid in "${UPLINK_ECHO_PIDS[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done
  UPLINK_ECHO_PIDS=()
}

start_posture_bridge() {
  echo "[SENTRY-CMD-DOWNLINK][INFO] Starting temporary relay: ${POSTURE_TOPIC} -> ${TX_TOPIC}" >&2
  python3 - "${POSTURE_TOPIC}" "${TX_TOPIC}" <<'PY' &
import sys

import rclpy
from rclpy.node import Node

from gimbal_driver.msg import SentryCmd


class PostureBridge(Node):
    def __init__(self, posture_topic: str, sentry_cmd_topic: str) -> None:
        super().__init__("posture_to_sentry_cmd_debug_bridge")
        self.publisher = self.create_publisher(SentryCmd, sentry_cmd_topic, 10)
        self.subscription = self.create_subscription(SentryCmd, posture_topic, self.on_posture, 10)
        self.get_logger().info(f"relay active: {posture_topic} -> {sentry_cmd_topic}")

    def on_posture(self, msg: SentryCmd) -> None:
        has_posture = msg.field_mask == 0 or (msg.field_mask & SentryCmd.FIELD_POSTURE) != 0
        if not has_posture:
            self.get_logger().warn("ignore posture message without FIELD_POSTURE")
            return
        if msg.posture > 3:
            self.get_logger().warn(f"ignore invalid posture={msg.posture}; expect 0..3")
            return

        out = SentryCmd()
        out.header = msg.header
        out.field_mask = SentryCmd.FIELD_POSTURE
        out.posture = msg.posture
        out.raw = int(msg.posture) << 21
        self.publisher.publish(out)
        self.get_logger().info(
            f"relayed posture={msg.posture} expected_sentry_cmd_raw={out.raw}"
        )


def main() -> None:
    posture_topic, sentry_cmd_topic = sys.argv[1:3]
    rclpy.init()
    node = PostureBridge(posture_topic, sentry_cmd_topic)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
PY
  BRIDGE_PID="$!"
  sleep 1

  if ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    echo "[ERROR] posture -> sentry_cmd bridge exited early." >&2
    exit 1
  fi
}

publish_noop() {
  echo "[SENTRY-CMD-DOWNLINK][TX] noop topic=${TX_TOPIC} expected_sentry_cmd_raw=0" >&2
  ros2 topic pub "${TX_TOPIC}" gimbal_driver/msg/SentryCmd \
    "{field_mask: 127}" -1 >/dev/null
}

publish_posture_topic() {
  validate_posture
  echo "[SENTRY-CMD-DOWNLINK][TX] posture topic=${POSTURE_TOPIC} posture=${POSTURE} expected_sentry_cmd_raw=$(expected_posture_raw)" >&2
  ros2 topic pub "${POSTURE_TOPIC}" gimbal_driver/msg/SentryCmd \
    "{field_mask: 32, posture: ${POSTURE}}" -1 >/dev/null
}

publish_sentry_posture() {
  validate_posture
  echo "[SENTRY-CMD-DOWNLINK][TX] sentry-posture topic=${TX_TOPIC} posture=${POSTURE} expected_sentry_cmd_raw=$(expected_posture_raw)" >&2
  ros2 topic pub "${TX_TOPIC}" gimbal_driver/msg/SentryCmd \
    "{field_mask: 32, posture: ${POSTURE}}" -1 >/dev/null
}

publish_energy_pulse() {
  echo "[SENTRY-CMD-DOWNLINK][TX] energy-pulse topic=${TX_TOPIC} expected_sentry_cmd_raw=8388608 then 0" >&2
  ros2 topic pub "${TX_TOPIC}" gimbal_driver/msg/SentryCmd \
    "{field_mask: 64, confirm_energy_activate: true}" -1 >/dev/null
  sleep 0.2
  ros2 topic pub "${TX_TOPIC}" gimbal_driver/msg/SentryCmd \
    "{field_mask: 64, confirm_energy_activate: false}" -1 >/dev/null
}

run_publish() {
  local publish_func="$1"

  if (( LOOP == 0 )); then
    "${publish_func}"
    return 0
  fi

  echo "[SENTRY-CMD-DOWNLINK][INFO] Continuous publish enabled; press Ctrl+C to stop. interval=${INTERVAL_SEC}s" >&2
  while true; do
    "${publish_func}"
    sleep "${INTERVAL_SEC}"
  done
}

run_cycle_monitor() {
  validate_posture
  echo "[SENTRY-CMD-DOWNLINK][INFO] Default cycle mode:" >&2
  echo "[SENTRY-CMD-DOWNLINK][INFO]   publish ${POSTURE_TOPIC}: posture 1 -> 2 -> 3 every ${INTERVAL_SEC}s" >&2
  echo "[SENTRY-CMD-DOWNLINK][INFO]   relay ${POSTURE_TOPIC} -> ${TX_TOPIC}" >&2
  echo "[SENTRY-CMD-DOWNLINK][INFO]   monitor ${TX_TOPIC}, ${RAW_TX_TOPIC}, ${RAW_RX_TOPIC}, RFID, posture feedback, sentry info" >&2
  echo "[SENTRY-CMD-DOWNLINK][INFO] Press Ctrl+C to stop." >&2

  python3 - "${POSTURE_TOPIC}" "${TX_TOPIC}" "${RAW_TX_TOPIC}" "${RAW_RX_TOPIC}" "${INTERVAL_SEC}" <<'PY'
import sys
import time

import rclpy
from rclpy.node import Node

from gimbal_driver.msg import GimbalRawFrame, RfidStatus, SentryCmd, SentryInfo
from std_msgs.msg import UInt8


POSTURE_NAMES = {
    0: "Reserved",
    1: "Attack",
    2: "Defense",
    3: "Move",
}

RAW_RX_TYPE_NAMES = {
    0: "GimbalData",
    1: "GameData",
    2: "HealthMyselfData",
    3: "HealthEnemyData",
    4: "RFIDAndBuffData",
    5: "PositionData",
    6: "ChassisData",
    7: "SentryData",
    8: "RfidStatus2",
}

RFID_FLAG_NAMES = [
    "friend_base",
    "friend_central",
    "enemy_central",
    "friend_highland",
    "enemy_highland",
    "friend_flyroad_front",
    "friend_flyroad_back",
    "enemy_flyroad_front",
    "enemy_flyroad_back",
    "friend_central_under",
    "friend_central_high",
    "enemy_central_under",
    "enemy_central_high",
    "friend_roadland_under",
    "friend_roadland_high",
    "enemy_roadland_under",
    "enemy_roadland_high",
    "friend_bastion",
    "friend_outpost",
    "friend_supply_noremix",
    "friend_supply_remix",
    "friend_armor",
    "enemy_armor",
    "central_rmul",
    "enemy_bastion",
    "enemy_outpost",
    "friend_tunnel_roadland_down",
    "friend_tunnel_roadland_mid",
    "friend_tunnel_roadland_up",
    "friend_tunnel_highland_low",
    "friend_tunnel_highland_mid",
    "friend_tunnel_highland_high",
    "enemy_tunnel_roadland_down",
    "enemy_tunnel_roadland_mid",
    "enemy_tunnel_roadland_up",
    "enemy_tunnel_highland_low",
    "enemy_tunnel_highland_mid",
    "enemy_tunnel_highland_high",
]


def stamp() -> str:
    return time.strftime("%H:%M:%S")


def posture_name(value: int) -> str:
    return POSTURE_NAMES.get(int(value), f"Unknown({int(value)})")


def data_hex(data) -> str:
    return bytes(data).hex(" ")


class SentryCmdCycleMonitor(Node):
    def __init__(self, posture_topic: str, sentry_cmd_topic: str, raw_tx_topic: str, raw_rx_topic: str, interval: float) -> None:
        super().__init__("sentry_cmd_downlink_cycle_monitor")
        self.posture_topic = posture_topic
        self.sentry_cmd_topic = sentry_cmd_topic
        self.values = [1, 2, 3]
        self.index = 0

        self.posture_pub = self.create_publisher(SentryCmd, posture_topic, 10)
        self.sentry_cmd_pub = self.create_publisher(SentryCmd, sentry_cmd_topic, 10)

        self.create_subscription(SentryCmd, posture_topic, self.on_posture_cmd, 10)
        self.create_subscription(SentryCmd, sentry_cmd_topic, self.on_sentry_cmd, 10)
        self.create_subscription(GimbalRawFrame, raw_tx_topic, self.on_raw_tx, 10)
        self.create_subscription(GimbalRawFrame, raw_rx_topic, self.on_raw_rx, 10)
        self.create_subscription(UInt8, "/ly/gimbal/posture", self.on_posture_feedback, 10)
        self.create_subscription(RfidStatus, "/ly/game/rfid", self.on_rfid, 10)
        self.create_subscription(SentryInfo, "/ly/game/sentry/info", self.on_sentry_info, 10)

        self.create_timer(max(interval, 0.1), self.publish_next_posture)
        self.log("READY", f"interval={interval:.3f}s")

    def log(self, tag: str, message: str) -> None:
        print(f"[{stamp()}][{tag}] {message}", flush=True)

    def publish_next_posture(self) -> None:
        value = self.values[self.index]
        self.index = (self.index + 1) % len(self.values)

        msg = SentryCmd()
        msg.field_mask = SentryCmd.FIELD_POSTURE
        msg.posture = value
        msg.raw = int(value) << 21
        self.posture_pub.publish(msg)
        self.log(
            "TX /ly/control/posture",
            f"posture={value}({posture_name(value)}) expected_sentry_cmd_raw={msg.raw}",
        )

    def on_posture_cmd(self, msg: SentryCmd) -> None:
        has_posture = msg.field_mask == 0 or (msg.field_mask & SentryCmd.FIELD_POSTURE) != 0
        if not has_posture:
            self.log("RELAY SKIP", f"field_mask={msg.field_mask} missing FIELD_POSTURE")
            return
        if msg.posture > 3:
            self.log("RELAY SKIP", f"invalid posture={msg.posture}")
            return

        out = SentryCmd()
        out.header = msg.header
        out.field_mask = SentryCmd.FIELD_POSTURE
        out.posture = msg.posture
        out.raw = int(msg.posture) << 21
        self.sentry_cmd_pub.publish(out)
        self.log(
            "RELAY posture->sentry_cmd",
            f"posture={msg.posture}({posture_name(msg.posture)}) raw={out.raw}",
        )

    def on_sentry_cmd(self, msg: SentryCmd) -> None:
        fields = []
        if msg.field_mask == 0 or (msg.field_mask & SentryCmd.FIELD_POSTURE) != 0:
            fields.append(f"posture={msg.posture}({posture_name(msg.posture)})")
        if msg.field_mask == 0 or (msg.field_mask & SentryCmd.FIELD_CONFIRM_ENERGY_ACTIVATE) != 0:
            fields.append(f"energy={int(msg.confirm_energy_activate)}")
        if not fields:
            fields.append(f"field_mask={msg.field_mask}")
        self.log("CMD /ly/control/sentry_cmd", " ".join(fields))

    def on_raw_tx(self, msg: GimbalRawFrame) -> None:
        posture = (int(msg.sentry_cmd_raw) >> 21) & 0x3
        energy = (int(msg.sentry_cmd_raw) >> 23) & 0x1
        self.log(
            "RAW TX",
            f"type_id={msg.type_id} sentry_cmd_raw={msg.sentry_cmd_raw} posture_bits={posture}({posture_name(posture)}) energy={energy} data=[{data_hex(msg.data)}]",
        )

    def on_raw_rx(self, msg: GimbalRawFrame) -> None:
        type_name = RAW_RX_TYPE_NAMES.get(int(msg.type_id), "Unknown")
        self.log("RAW RX", f"type_id={msg.type_id}({type_name}) data=[{data_hex(msg.data)}]")

    def on_posture_feedback(self, msg: UInt8) -> None:
        self.log("FB /ly/gimbal/posture", f"posture={msg.data}({posture_name(msg.data)})")

    def on_rfid(self, msg: RfidStatus) -> None:
        active = [name for name in RFID_FLAG_NAMES if getattr(msg, name, False)]
        if len(active) > 8:
            active_text = ",".join(active[:8]) + f",...(+{len(active) - 8})"
        else:
            active_text = ",".join(active) if active else "none"
        self.log(
            "RFID /ly/game/rfid",
            f"raw=0x{int(msg.raw):08x} active={active_text} rfid2={int(msg.rfid_status_2_raw)} has_rfid2={int(msg.has_rfid_status_2)}",
        )

    def on_sentry_info(self, msg: SentryInfo) -> None:
        self.log(
            "SENTRY /ly/game/sentry/info",
            " ".join([
                f"info=0x{int(msg.sentry_info_raw):08x}",
                f"info2=0x{int(msg.sentry_info_2_raw):04x}",
                f"posture={msg.posture}({posture_name(msg.posture)})",
                f"can_energy={int(msg.can_activate_energy_mechanism)}",
                f"out_of_combat={int(msg.out_of_combat)}",
                f"remote_projectile_count={int(msg.remote_projectile_exchange_count)}",
                f"remote_hp_count={int(msg.remote_hp_exchange_count)}",
            ]),
        )


def main() -> None:
    posture_topic, sentry_cmd_topic, raw_tx_topic, raw_rx_topic, interval_text = sys.argv[1:6]
    try:
        interval = float(interval_text)
    except ValueError:
        interval = 5.0

    rclpy.init()
    node = SentryCmdCycleMonitor(posture_topic, sentry_cmd_topic, raw_tx_topic, raw_rx_topic, interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
PY
}

launch_gimbal_driver

case "${MODE}" in
  cycle)
    run_cycle_monitor
    ;;
  watch)
    ros2 topic echo "${RAW_TX_TOPIC}"
    ;;
  uplink)
    UPLINK_ECHO=1
    start_uplink_echo
    wait_uplink_echo
    ;;
  noop)
    start_raw_echo
    start_uplink_echo
    run_publish publish_noop
    wait_raw_echo
    wait_uplink_echo
    ;;
  posture)
    start_raw_echo
    start_uplink_echo
    run_publish publish_posture_topic
    wait_raw_echo
    wait_uplink_echo
    ;;
  bridge-posture)
    start_posture_bridge
    start_raw_echo
    start_uplink_echo
    run_publish publish_posture_topic
    wait_raw_echo
    wait_uplink_echo
    ;;
  sentry-posture)
    start_raw_echo
    start_uplink_echo
    run_publish publish_sentry_posture
    wait_raw_echo
    wait_uplink_echo
    ;;
  energy-pulse)
    start_raw_echo
    start_uplink_echo
    run_publish publish_energy_pulse
    wait_raw_echo
    wait_uplink_echo
    ;;
esac
