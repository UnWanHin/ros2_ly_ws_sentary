#!/usr/bin/env bash

# Navigation-only velocity chain:
#   /ly/navi/vel -> navi_vel_control_bridge -> /ly/control/vel -> gimbal_driver
# This does not start behavior_tree, FaceMode, or rotate logic.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

CLEANUP_EXISTING=1
START_GIMBAL=1
ALLOW_WITH_BT=0
USE_VIRTUAL_DEVICE="${USE_VIRTUAL_DEVICE:-false}"
CONFIG_FILE="${CONFIG_FILE:-${ROOT_DIR}/config/base_config.yaml}"
OUTPUT="${OUTPUT:-screen}"
INPUT_TOPIC="${INPUT_TOPIC:-/ly/navi/vel}"
CONTROL_VEL_TOPIC="${CONTROL_VEL_TOPIC:-/ly/control/vel}"
STALE_TIMEOUT_SEC="${STALE_TIMEOUT_SEC:-0.5}"
WAIT_GIMBAL_ANGLES="${WAIT_GIMBAL_ANGLES:-true}"
PUBLISH_HOLD_ANGLES="${PUBLISH_HOLD_ANGLES:-true}"
PUBLISH_SAFE_FIRECODE="${PUBLISH_SAFE_FIRECODE:-true}"
PUBLISH_INITIAL_ZERO="${PUBLISH_INITIAL_ZERO:-true}"
VELOCITY_RAW_TO_MPS="${VELOCITY_RAW_TO_MPS:-0.025}"
EXTRA_BRIDGE_ARGS=()
PIDS=()

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [options] [-- <extra bridge ros args>]

Purpose:
  Start a navigation-only velocity chain for chassis testing.
  It waits for /ly/navi/vel and forwards it to /ly/control/vel.
  It does not start behavior_tree, vision, FaceMode, or rotate logic.

Options:
  --no-gimbal                 Do not launch gimbal_driver; only run the bridge.
  --virtual-device            Launch gimbal_driver with use_virtual_device:=true.
  --config-file <path>        gimbal_driver config file. Default: ${CONFIG_FILE}
  --input-topic <topic>       Default: ${INPUT_TOPIC}
  --control-vel-topic <topic> Default: ${CONTROL_VEL_TOPIC}
  --stale-timeout <sec>       Publish zero velocity after timeout. Default: ${STALE_TIMEOUT_SEC}
  --allow-no-gimbal-angle     Do not wait for /ly/gimbal/angles; velocity-only bridge.
  --no-initial-zero           Do not publish initial hold-angle zero velocity.
  --allow-with-bt             Allow running while /behavior_tree exists.
  --no-cleanup-existing       Do not clean old bridge/gimbal processes.

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --virtual-device
  ./${SCRIPT_NAME} --no-gimbal
  ./${SCRIPT_NAME} --allow-no-gimbal-angle
EOF
}

cleanup() {
  local pid
  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  sleep 0.3
  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
}

has_node() {
  local node_name="$1"
  ros2 node list 2>/dev/null | awk -v node="${node_name}" '$0 == node { found = 1 } END { exit found ? 0 : 1 }'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-gimbal)
      START_GIMBAL=0
      shift
      ;;
    --virtual-device)
      USE_VIRTUAL_DEVICE="true"
      shift
      ;;
    --config-file)
      if (( $# < 2 )); then
        echo "[ERROR] --config-file requires a path." >&2
        exit 2
      fi
      CONFIG_FILE="$2"
      shift 2
      ;;
    --input-topic)
      if (( $# < 2 )); then
        echo "[ERROR] --input-topic requires a topic." >&2
        exit 2
      fi
      INPUT_TOPIC="$2"
      shift 2
      ;;
    --control-vel-topic)
      if (( $# < 2 )); then
        echo "[ERROR] --control-vel-topic requires a topic." >&2
        exit 2
      fi
      CONTROL_VEL_TOPIC="$2"
      shift 2
      ;;
    --stale-timeout)
      if (( $# < 2 )); then
        echo "[ERROR] --stale-timeout requires seconds." >&2
        exit 2
      fi
      STALE_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --allow-no-gimbal-angle)
      WAIT_GIMBAL_ANGLES="false"
      PUBLISH_HOLD_ANGLES="false"
      shift
      ;;
    --no-initial-zero)
      PUBLISH_INITIAL_ZERO="false"
      shift
      ;;
    --allow-with-bt)
      ALLOW_WITH_BT=1
      shift
      ;;
    --cleanup-existing)
      CLEANUP_EXISTING=1
      shift
      ;;
    --no-cleanup-existing)
      CLEANUP_EXISTING=0
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      EXTRA_BRIDGE_ARGS=("$@")
      break
      ;;
    *)
      echo "[ERROR] Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

source_ros_workspace "${ROOT_DIR}"

if (( ALLOW_WITH_BT == 0 )) && has_node "/behavior_tree"; then
  echo "[ERROR] /behavior_tree is already running. This navigation-only chain would conflict with BT /ly/control/* publishers." >&2
  echo "        Stop sentry_all first, or pass --allow-with-bt if you really want to test graph conflicts." >&2
  exit 2
fi

BRIDGE_NODE_REGEX="/(navi_vel_control_bridge)([[:space:]]|$)|scripts/navi/navi_vel_chain\\.py"
GIMBAL_NODE_REGEX="/(gimbal_driver)([[:space:]]|$)|ros2 launch gimbal_driver gimbal_driver\\.launch\\.py"
if (( START_GIMBAL == 1 )); then
  cleanup_existing_stack "${CLEANUP_EXISTING}" "${BRIDGE_NODE_REGEX}|${GIMBAL_NODE_REGEX}" "ros2 launch gimbal_driver gimbal_driver\\.launch\\.py"
else
  cleanup_existing_stack "${CLEANUP_EXISTING}" "${BRIDGE_NODE_REGEX}" "scripts/navi/navi_vel_chain\\.py"
fi

trap cleanup EXIT INT TERM

if (( START_GIMBAL == 1 )); then
  echo "[INFO] starting gimbal_driver only; config=${CONFIG_FILE}, virtual=${USE_VIRTUAL_DEVICE}" >&2
  ros2 launch gimbal_driver gimbal_driver.launch.py \
    "config_file:=${CONFIG_FILE}" \
    "use_virtual_device:=${USE_VIRTUAL_DEVICE}" \
    "output:=${OUTPUT}" &
  PIDS+=("$!")
fi

echo "[INFO] starting navi velocity bridge: ${INPUT_TOPIC} -> ${CONTROL_VEL_TOPIC}" >&2
python3 "${ROOT_DIR}/scripts/navi/navi_vel_chain.py" \
  --ros-args \
  -p "input_topic:=${INPUT_TOPIC}" \
  -p "control_vel_topic:=${CONTROL_VEL_TOPIC}" \
  -p "stale_timeout_sec:=${STALE_TIMEOUT_SEC}" \
  -p "wait_for_gimbal_angles:=${WAIT_GIMBAL_ANGLES}" \
  -p "publish_hold_angles:=${PUBLISH_HOLD_ANGLES}" \
  -p "publish_safe_firecode:=${PUBLISH_SAFE_FIRECODE}" \
  -p "publish_initial_zero:=${PUBLISH_INITIAL_ZERO}" \
  -p "velocity_raw_to_mps:=${VELOCITY_RAW_TO_MPS}" \
  "${EXTRA_BRIDGE_ARGS[@]}"
