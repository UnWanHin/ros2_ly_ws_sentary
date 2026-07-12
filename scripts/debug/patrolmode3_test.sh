#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

YAW_START="0.0"
PITCH="15.0"
STEP_DEG="0.35"
HZ="20.0"
ANGLES_TOPIC="/ly/control/angles"
FIRECODE_TOPIC="/ly/control/firecode"
PUBLISH_FIRECODE=0
SAFE_FIRECODE=0
LAUNCH_GIMBAL=1
WAIT_SEC=3
USE_VIRTUAL_DEVICE="false"
OUTPUT_MODE="screen"
CONFIG_FILE="${ROOT_DIR}/src/gimbal_driver/config/gimbal_driver_config.yaml"
GIMBAL_PID=""
STACK_NODE_REGEX="/(gimbal_driver_node|scan_gimbal_test)([[:space:]]|$)"
STACK_LAUNCH_REGEX="ros2 launch gimbal_driver gimbal_driver.launch.py"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [options]

Purpose:
  Minimal PatrolScan mode 3 downlink test:
  launch gimbal_driver, then publish slow one-way yaw scan + fixed high pitch
  to ${ANGLES_TOPIC}. No behavior_tree, no navi, no vision.

Options:
  --yaw-start DEG          Initial yaw. Default: ${YAW_START}
  --pitch DEG              Fixed pitch. Default: ${PITCH}
  --step-deg DEG           Yaw step per publish. Default: ${STEP_DEG}
  --hz HZ                  Publish rate. Default: ${HZ}
  --angles-topic TOPIC     Angles topic. Default: ${ANGLES_TOPIC}
  --firecode-topic TOPIC   FireCode topic. Default: ${FIRECODE_TOPIC}
  --publish-firecode       Also publish safe firecode raw=${SAFE_FIRECODE}
  --safe-firecode VALUE    Raw firecode when --publish-firecode is set. Default: ${SAFE_FIRECODE}
  --launch-gimbal / --no-launch-gimbal
                           Whether to start gimbal_driver. Default: launch.
  --wait SEC               Wait after starting gimbal_driver. Default: ${WAIT_SEC}
  --config-file FILE       gimbal_driver params YAML. Default: ${CONFIG_FILE}
  --use-virtual-device true|false
                           Force gimbal virtual device. Default: ${USE_VIRTUAL_DEVICE}
  --output screen|log      gimbal_driver launch output. Default: ${OUTPUT_MODE}
  -h, --help               Show help.

Examples:
  ./scripts/debug/${SCRIPT_NAME}
  ./scripts/debug/${SCRIPT_NAME} --pitch 15 --step-deg 0.35 --hz 20
  ./scripts/debug/${SCRIPT_NAME} --no-launch-gimbal
EOF
}

cleanup() {
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
    --yaw-start)
      YAW_START="$2"
      shift 2
      ;;
    --pitch)
      PITCH="$2"
      shift 2
      ;;
    --step-deg)
      STEP_DEG="$2"
      shift 2
      ;;
    --hz)
      HZ="$2"
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
    --publish-firecode)
      PUBLISH_FIRECODE=1
      shift
      ;;
    --safe-firecode)
      SAFE_FIRECODE="$2"
      shift 2
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

if (( LAUNCH_GIMBAL == 1 )); then
  if [[ ! -f "${CONFIG_FILE}" ]]; then
    echo "[ERROR] Config file not found: ${CONFIG_FILE}" >&2
    exit 1
  fi

  cleanup_existing_stack "1" "${STACK_NODE_REGEX}" "${STACK_LAUNCH_REGEX}"

  echo "[PATROLMODE3-TEST][INFO] Launching gimbal_driver with config=${CONFIG_FILE} use_virtual_device=${USE_VIRTUAL_DEVICE}" >&2
  ros2 launch gimbal_driver gimbal_driver.launch.py \
    "config_file:=${CONFIG_FILE}" \
    "use_virtual_device:=${USE_VIRTUAL_DEVICE}" \
    "output:=${OUTPUT_MODE}" &
  GIMBAL_PID="$!"
  sleep "${WAIT_SEC}"

  if ! kill -0 "${GIMBAL_PID}" 2>/dev/null; then
    echo "[ERROR] gimbal_driver exited early." >&2
    exit 1
  fi
else
  cleanup_existing_stack "1" "/(scan_gimbal_test)([[:space:]]|$)" "a^"
fi

echo "[PATROLMODE3-TEST][INFO] Publishing mode3 scan: yaw_start=${YAW_START} pitch=${PITCH} step=${STEP_DEG} hz=${HZ}" >&2

SCAN_ARGS=(
  --scan-mode 3
  --yaw-min "${YAW_START}"
  --yaw-max "${YAW_START}"
  --pitch "${PITCH}"
  --step-deg "${STEP_DEG}"
  --hz "${HZ}"
  --angles-topic "${ANGLES_TOPIC}"
  --firecode-topic "${FIRECODE_TOPIC}"
  --safe-firecode "${SAFE_FIRECODE}"
)

if (( PUBLISH_FIRECODE == 0 )); then
  SCAN_ARGS+=(--no-firecode)
fi

exec python3 "${ROOT_DIR}/scripts/feature_test/scan_gimbal_test.py" "${SCAN_ARGS[@]}"
