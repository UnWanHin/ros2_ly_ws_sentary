#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

MODE="noop"
POSTURE="2"
TX_TOPIC="/ly/control/sentry_cmd"
POSTURE_TOPIC="/ly/control/posture"
RAW_TX_TOPIC="/ly/log/gimbal_raw_tx"
RAW_RX_TOPIC="/ly/log/gimbal_raw_rx"
LAUNCH_GIMBAL=1
WAIT_SEC=3
WATCH_SEC=5
USE_VIRTUAL_DEVICE="false"
OUTPUT_MODE="screen"
CONFIG_FILE="${ROOT_DIR}/config/base_config.yaml"
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

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [noop|posture|bridge-posture|sentry-posture|energy-pulse|watch|uplink] [options]

Modes:
  noop            Publish a zero SentryCmd snapshot to ${TX_TOPIC}. Default, no motion command.
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
  if [[ -n "${GIMBAL_PID:-}" ]] && kill -0 "${GIMBAL_PID}" 2>/dev/null; then
    kill -INT "${GIMBAL_PID}" 2>/dev/null || true
    sleep 1
    kill -TERM "${GIMBAL_PID}" 2>/dev/null || true
    wait "${GIMBAL_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

while [[ $# -gt 0 ]]; do
  case "$1" in
    noop|posture|bridge-posture|bridge_posture|sentry-posture|sentry_posture|energy-pulse|energy_pulse|watch|uplink)
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

  if [[ ! -f "${CONFIG_FILE}" ]]; then
    echo "[ERROR] Config file not found: ${CONFIG_FILE}" >&2
    exit 1
  fi

  cleanup_existing_stack "1" "${STACK_NODE_REGEX}" "${STACK_LAUNCH_REGEX}"

  echo "[SENTRY-CMD-DOWNLINK][INFO] Launching gimbal_driver with raw debug topics enabled" >&2
  echo "[SENTRY-CMD-DOWNLINK][INFO] config=${CONFIG_FILE} use_virtual_device=${USE_VIRTUAL_DEVICE}" >&2
  ros2 launch gimbal_driver gimbal_driver.launch.py \
    "config_file:=${CONFIG_FILE}" \
    "use_virtual_device:=${USE_VIRTUAL_DEVICE}" \
    "raw_topic_enable:=true" \
    "raw_topic_downlink:=true" \
    "raw_topic_uplink:=true" \
    "output:=${OUTPUT_MODE}" &
  GIMBAL_PID="$!"
  sleep "${WAIT_SEC}"

  if ! kill -0 "${GIMBAL_PID}" 2>/dev/null; then
    echo "[ERROR] gimbal_driver exited early." >&2
    exit 1
  fi
}

start_raw_echo() {
  if (( RAW_ECHO == 0 )); then
    return 0
  fi

  echo "[SENTRY-CMD-DOWNLINK][INFO] Echoing raw downlink frames from ${RAW_TX_TOPIC}" >&2
  timeout "${WATCH_SEC}" ros2 topic echo "${RAW_TX_TOPIC}" &
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
    timeout "${WATCH_SEC}" ros2 topic echo "${topic}" &
    UPLINK_ECHO_PIDS+=("$!")
  done
  echo "[SENTRY-CMD-DOWNLINK][INFO]   ${RAW_RX_TOPIC}" >&2
  timeout "${WATCH_SEC}" ros2 topic echo "${RAW_RX_TOPIC}" &
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

launch_gimbal_driver

case "${MODE}" in
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
    publish_noop
    wait_raw_echo
    wait_uplink_echo
    ;;
  posture)
    start_raw_echo
    start_uplink_echo
    publish_posture_topic
    wait_raw_echo
    wait_uplink_echo
    ;;
  bridge-posture)
    start_posture_bridge
    start_raw_echo
    start_uplink_echo
    publish_posture_topic
    wait_raw_echo
    wait_uplink_echo
    ;;
  sentry-posture)
    start_raw_echo
    start_uplink_echo
    publish_sentry_posture
    wait_raw_echo
    wait_uplink_echo
    ;;
  energy-pulse)
    start_raw_echo
    start_uplink_echo
    publish_energy_pulse
    wait_raw_echo
    wait_uplink_echo
    ;;
esac
