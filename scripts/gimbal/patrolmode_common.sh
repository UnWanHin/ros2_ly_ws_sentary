#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

MODE="${PATROL_MODE:-}"
YAW_START="0.0"
HZ="100.0"
ANGLES_TOPIC="/ly/control/angles"
FIRECODE_TOPIC="/ly/control/firecode"
PUBLISH_FIRECODE=1
SAFE_FIRECODE=0
OUTPOST_MODE=0
LAUNCH_GIMBAL=1
WAIT_SEC=3
USE_VIRTUAL_DEVICE="false"
OUTPUT_MODE="screen"
CONFIG_FILE="${ROOT_DIR}/src/gimbal_driver/config/gimbal_driver_config.yaml"
PATROL_CONFIG_FILE="${ROOT_DIR}/src/behavior_tree/config/Patrol.yaml"
GIMBAL_PID=""
STACK_NODE_REGEX="/(gimbal_driver_node)([[:space:]]|$)|patrolmode_pub\\.py"
STACK_LAUNCH_REGEX="ros2 launch gimbal_driver gimbal_driver.launch.py"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/gimbal_test_lifecycle.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [options]

Purpose:
  Launch gimbal_driver and publish PatrolScan-style /ly/control/angles
  for direct lower-machine downlink testing. Defaults use real serial
  hardware; pass --use-virtual-device true for offline testing.

Options:
  --yaw-start DEG          Initial yaw. Default: ${YAW_START}
  --hz HZ                  Publish rate. Default: ${HZ}
                           Actual yaw speed = YawStepDegPerTick * HZ.
  --patrol-config FILE     Patrol config YAML. Default: ${PATROL_CONFIG_FILE}
  --angles-topic TOPIC     Angles topic. Default: ${ANGLES_TOPIC}
  --firecode-topic TOPIC   FireCode topic. Default: ${FIRECODE_TOPIC}
  --no-firecode            Do not publish safe firecode.
  --safe-firecode VALUE    Raw firecode when firecode is published. Default: ${SAFE_FIRECODE}
  --outpost                Simulate formal Outpost patrol fallback pitch bias from PatrolScan.TaskOverrides.
  --launch-gimbal / --no-launch-gimbal
                           Whether to start gimbal_driver. Default: launch.
  --wait SEC               Wait after starting gimbal_driver. Default: ${WAIT_SEC}
  --config-file FILE       gimbal_driver params YAML. Default: ${CONFIG_FILE}
  --use-virtual-device true|false
                           gimbal_driver virtual device. Default: ${USE_VIRTUAL_DEVICE}
  --output screen|log      gimbal_driver launch output. Default: ${OUTPUT_MODE}
  -h, --help               Show help.

Examples:
  ./scripts/gimbal/patrolmode1.sh
  ./scripts/gimbal/patrolmode2.sh --hz 100
  ./scripts/gimbal/patrolmode3.sh --use-virtual-device true
EOF
}

cleanup() {
  gimbal_test_stop_driver
}

trap cleanup EXIT INT TERM

while [[ $# -gt 0 ]]; do
  case "$1" in
    --yaw-start)
      YAW_START="$2"
      shift 2
      ;;
    --hz)
      HZ="$2"
      shift 2
      ;;
    --patrol-config)
      PATROL_CONFIG_FILE="$2"
      shift 2
      ;;
    --angles-topic)
      ANGLES_TOPIC="$2"
      shift 2
      ;;
    --firecode-topic)
      FIRECODE_TOPIC="$2"
      shift 2
      ;;
    --no-firecode)
      PUBLISH_FIRECODE=0
      shift
      ;;
    --safe-firecode)
      SAFE_FIRECODE="$2"
      shift 2
      ;;
    --outpost)
      OUTPOST_MODE=1
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

if [[ "${MODE}" != "1" && "${MODE}" != "2" && "${MODE}" != "3" ]]; then
  echo "[ERROR] PATROL_MODE must be 1, 2, or 3." >&2
  exit 2
fi

source_ros_workspace "${ROOT_DIR}"

if [[ ! -f "${PATROL_CONFIG_FILE}" ]]; then
  echo "[ERROR] Patrol config file not found: ${PATROL_CONFIG_FILE}" >&2
  exit 1
fi

if (( LAUNCH_GIMBAL == 1 )); then
  gimbal_test_launch_driver "PATROLMODE${MODE}" "${CONFIG_FILE}" "${USE_VIRTUAL_DEVICE}" "${OUTPUT_MODE}" "${WAIT_SEC}" "${STACK_NODE_REGEX}" "${STACK_LAUNCH_REGEX}"
else
  cleanup_existing_stack "1" "patrolmode_pub\\.py" "a^"
fi

echo "[PATROLMODE${MODE}][INFO] Publishing /ly/control/angles from ${PATROL_CONFIG_FILE}; hz=${HZ}" >&2

PUBLISH_ARGS=(
  --mode "${MODE}"
  --patrol-config "${PATROL_CONFIG_FILE}"
  --yaw-start "${YAW_START}"
  --hz "${HZ}"
  --angles-topic "${ANGLES_TOPIC}"
  --firecode-topic "${FIRECODE_TOPIC}"
  --safe-firecode "${SAFE_FIRECODE}"
)

if (( OUTPOST_MODE == 1 )); then
  PUBLISH_ARGS+=(--outpost)
fi

if (( PUBLISH_FIRECODE == 0 )); then
  PUBLISH_ARGS+=(--no-firecode)
fi

exec python3 "${ROOT_DIR}/scripts/gimbal/patrolmode_pub.py" "${PUBLISH_ARGS[@]}"
