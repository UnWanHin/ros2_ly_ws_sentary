#!/usr/bin/env bash

# Navigation lower-machine control chain:
#   /ly/navi/vel -> navi_vel_control_bridge -> /ly/control/vel -> gimbal_driver
# Optional:
#   rotate=true -> publish /ly/control/firecode rotate field
#   scan=true   -> publish /ly/control/angles patrol scan

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
CONTROL_ANGLES_TOPIC="${CONTROL_ANGLES_TOPIC:-/ly/control/angles}"
CONTROL_FIRECODE_TOPIC="${CONTROL_FIRECODE_TOPIC:-/ly/control/firecode}"
STALE_TIMEOUT_SEC="${STALE_TIMEOUT_SEC:-0.5}"
WAIT_GIMBAL_ANGLES="${WAIT_GIMBAL_ANGLES:-true}"
PUBLISH_INITIAL_ZERO="${PUBLISH_INITIAL_ZERO:-true}"
VELOCITY_RAW_TO_MPS="${VELOCITY_RAW_TO_MPS:-0.025}"

ROTATE_ENABLED="${ROTATE_ENABLED:-true}"
ROTATE_LEVEL="${ROTATE_LEVEL:-1}"
ROTATE_HZ="${ROTATE_HZ:-20}"

SCAN_ENABLED="${SCAN_ENABLED:-true}"
SCAN_MODE="${SCAN_MODE:-2}"
SCAN_YAW_MIN="${SCAN_YAW_MIN:--15.0}"
SCAN_YAW_MAX="${SCAN_YAW_MAX:-15.0}"
SCAN_PITCH="${SCAN_PITCH:-8.0}"
SCAN_STEP_DEG="${SCAN_STEP_DEG:-1.0}"
SCAN_HZ="${SCAN_HZ:-20.0}"

EXTRA_BRIDGE_ARGS=()
PIDS=()

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [options] [-- <extra bridge ros args>]

Purpose:
  Start a navigation-only lower-machine chain for external navigation tests.
  It forwards /ly/navi/vel to /ly/control/vel and waits for navigation velocity.
  It does not start behavior_tree, detector, tracker, predictor, buff/outpost, or FaceMode.

Options:
  --rotate [true|false]       Enable chassis rotate publisher. Default: ${ROTATE_ENABLED}
  --scan [true|false]         Enable gimbal scan publisher. Default: ${SCAN_ENABLED}
  --rotate-level <0..3>       Rotate level when rotate=true. Default: ${ROTATE_LEVEL}
  --rotate-hz <hz>            Rotate firecode publish rate. Default: ${ROTATE_HZ}
  --scan-mode <1|2>           Gimbal scan mode. Default: ${SCAN_MODE}
  --scan-yaw-min <deg>        Scan yaw min. Default: ${SCAN_YAW_MIN}
  --scan-yaw-max <deg>        Scan yaw max. Default: ${SCAN_YAW_MAX}
  --scan-pitch <deg>          Scan pitch. Default: ${SCAN_PITCH}
  --scan-step-deg <deg>       Scan yaw step per tick. Default: ${SCAN_STEP_DEG}
  --scan-hz <hz>              Scan publish rate. Default: ${SCAN_HZ}

  --no-gimbal                 Do not launch gimbal_driver; only run publishers/bridge.
  --virtual-device            Launch gimbal_driver with use_virtual_device:=true.
  --config-file <path>        gimbal_driver config file. Default: ${CONFIG_FILE}
  --input-topic <topic>       Default: ${INPUT_TOPIC}
  --control-vel-topic <topic> Default: ${CONTROL_VEL_TOPIC}
  --stale-timeout <sec>       Publish zero velocity after timeout. Default: ${STALE_TIMEOUT_SEC}
  --allow-no-gimbal-angle     Do not wait for /ly/gimbal/angles before forwarding velocity.
  --no-initial-zero           Do not publish initial zero velocity.
  --allow-with-bt             Allow running while /behavior_tree exists.
  --no-cleanup-existing       Do not clean old bridge/gimbal/test processes.

Examples:
  ./scripts/navi/${SCRIPT_NAME}
  ./scripts/navi/${SCRIPT_NAME} --rotate true
  ./scripts/navi/${SCRIPT_NAME} --scan true --scan-mode 2
  ./scripts/navi/${SCRIPT_NAME} --rotate true --scan true --rotate-level 1
  ./scripts/navi/${SCRIPT_NAME} --rotate false --scan false
  ./scripts/navi/${SCRIPT_NAME} rotate=true scan=true
EOF
}

is_bool_token() {
  local value
  value="$(printf '%s' "${1:-}" | tr '[:upper:]' '[:lower:]')"
  case "${value}" in
    true|1|yes|y|on|false|0|no|n|off)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

parse_bool() {
  local value
  value="$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')"
  case "${value}" in
    true|1|yes|y|on)
      printf 'true'
      ;;
    false|0|no|n|off)
      printf 'false'
      ;;
    *)
      echo "[ERROR] invalid boolean value: $1" >&2
      exit 2
      ;;
  esac
}

take_optional_bool() {
  if (( $# >= 1 )) && is_bool_token "${1:-}"; then
    parse_bool "$1"
  else
    printf 'true'
  fi
}

optional_bool_shift() {
  if (( $# >= 1 )) && is_bool_token "${1:-}"; then
    printf '2'
  else
    printf '1'
  fi
}

validate_rotate_level() {
  if ! [[ "${ROTATE_LEVEL}" =~ ^[0-3]$ ]]; then
    echo "[ERROR] --rotate-level must be 0..3, got: ${ROTATE_LEVEL}" >&2
    exit 2
  fi
}

validate_scan_mode() {
  if ! [[ "${SCAN_MODE}" =~ ^[12]$ ]]; then
    echo "[ERROR] --scan-mode must be 1 or 2, got: ${SCAN_MODE}" >&2
    exit 2
  fi
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
    --rotate)
      ROTATE_ENABLED="$(take_optional_bool "${2:-}")"
      shift "$(optional_bool_shift "${2:-}")"
      ;;
    --rotate=*)
      ROTATE_ENABLED="$(parse_bool "${1#*=}")"
      shift
      ;;
    rotate=*)
      ROTATE_ENABLED="$(parse_bool "${1#*=}")"
      shift
      ;;
    --scan)
      SCAN_ENABLED="$(take_optional_bool "${2:-}")"
      shift "$(optional_bool_shift "${2:-}")"
      ;;
    --scan=*)
      SCAN_ENABLED="$(parse_bool "${1#*=}")"
      shift
      ;;
    scan=*)
      SCAN_ENABLED="$(parse_bool "${1#*=}")"
      shift
      ;;
    --scan-mode)
      if (( $# < 2 )); then
        echo "[ERROR] --scan-mode requires 1 or 2." >&2
        exit 2
      fi
      SCAN_MODE="$2"
      shift 2
      ;;
    --scan-mode=*)
      SCAN_MODE="${1#*=}"
      shift
      ;;
    scanmode=*|scan_mode=*)
      SCAN_MODE="${1#*=}"
      shift
      ;;
    --rotate-level)
      if (( $# < 2 )); then
        echo "[ERROR] --rotate-level requires a value." >&2
        exit 2
      fi
      ROTATE_LEVEL="$2"
      shift 2
      ;;
    --rotate-hz)
      if (( $# < 2 )); then
        echo "[ERROR] --rotate-hz requires a value." >&2
        exit 2
      fi
      ROTATE_HZ="$2"
      shift 2
      ;;
    --scan-yaw-min)
      if (( $# < 2 )); then
        echo "[ERROR] --scan-yaw-min requires a value." >&2
        exit 2
      fi
      SCAN_YAW_MIN="$2"
      shift 2
      ;;
    --scan-yaw-max)
      if (( $# < 2 )); then
        echo "[ERROR] --scan-yaw-max requires a value." >&2
        exit 2
      fi
      SCAN_YAW_MAX="$2"
      shift 2
      ;;
    --scan-pitch)
      if (( $# < 2 )); then
        echo "[ERROR] --scan-pitch requires a value." >&2
        exit 2
      fi
      SCAN_PITCH="$2"
      shift 2
      ;;
    --scan-step-deg)
      if (( $# < 2 )); then
        echo "[ERROR] --scan-step-deg requires a value." >&2
        exit 2
      fi
      SCAN_STEP_DEG="$2"
      shift 2
      ;;
    --scan-hz)
      if (( $# < 2 )); then
        echo "[ERROR] --scan-hz requires a value." >&2
        exit 2
      fi
      SCAN_HZ="$2"
      shift 2
      ;;
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

validate_rotate_level
validate_scan_mode
source_ros_workspace "${ROOT_DIR}"

if (( ALLOW_WITH_BT == 0 )) && has_node "/behavior_tree"; then
  echo "[ERROR] /behavior_tree is already running. This navigation-only chain would conflict with BT /ly/control/* publishers." >&2
  echo "        Stop sentry_all first, or pass --allow-with-bt if you really want to test graph conflicts." >&2
  exit 2
fi

BRIDGE_NODE_REGEX="/(navi_vel_control_bridge|scan_gimbal_test|chassis_spin_test)([[:space:]]|$)|scripts/navi/navi_vel_chain\\.py|scripts/feature_test/(scan_gimbal_test|chassis_spin_test)\\.py"
GIMBAL_NODE_REGEX="/(gimbal_driver)([[:space:]]|$)|ros2 launch gimbal_driver gimbal_driver\\.launch\\.py"
if (( START_GIMBAL == 1 )); then
  cleanup_existing_stack "${CLEANUP_EXISTING}" "${BRIDGE_NODE_REGEX}|${GIMBAL_NODE_REGEX}" "ros2 launch gimbal_driver gimbal_driver\\.launch\\.py"
else
  cleanup_existing_stack "${CLEANUP_EXISTING}" "${BRIDGE_NODE_REGEX}" "scripts/navi/navi_vel_chain\\.py"
fi

trap cleanup EXIT INT TERM

if [[ "${SCAN_ENABLED}" == "true" ]]; then
  PUBLISH_HOLD_ANGLES="false"
else
  PUBLISH_HOLD_ANGLES="true"
fi

if [[ "${SCAN_ENABLED}" == "true" || "${ROTATE_ENABLED}" == "true" ]]; then
  PUBLISH_SAFE_FIRECODE="false"
else
  PUBLISH_SAFE_FIRECODE="true"
fi

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
  -p "control_angles_topic:=${CONTROL_ANGLES_TOPIC}" \
  -p "control_firecode_topic:=${CONTROL_FIRECODE_TOPIC}" \
  -p "stale_timeout_sec:=${STALE_TIMEOUT_SEC}" \
  -p "wait_for_gimbal_angles:=${WAIT_GIMBAL_ANGLES}" \
  -p "publish_hold_angles:=${PUBLISH_HOLD_ANGLES}" \
  -p "publish_safe_firecode:=${PUBLISH_SAFE_FIRECODE}" \
  -p "publish_initial_zero:=${PUBLISH_INITIAL_ZERO}" \
  -p "velocity_raw_to_mps:=${VELOCITY_RAW_TO_MPS}" \
  "${EXTRA_BRIDGE_ARGS[@]}" &
PIDS+=("$!")

if [[ "${SCAN_ENABLED}" == "true" ]]; then
  SCAN_FIRECODE_ARGS=(--safe-firecode 0)
  if [[ "${ROTATE_ENABLED}" == "true" ]]; then
    SCAN_FIRECODE_ARGS=(--no-firecode)
  fi
  echo "[INFO] starting gimbal scan: mode=${SCAN_MODE} yaw=[${SCAN_YAW_MIN},${SCAN_YAW_MAX}] pitch=${SCAN_PITCH} rotate=${ROTATE_ENABLED} level=${ROTATE_LEVEL}" >&2
  python3 "${ROOT_DIR}/scripts/feature_test/scan_gimbal_test.py" \
    --scan-mode "${SCAN_MODE}" \
    --yaw-min "${SCAN_YAW_MIN}" \
    --yaw-max "${SCAN_YAW_MAX}" \
    --pitch "${SCAN_PITCH}" \
    --step-deg "${SCAN_STEP_DEG}" \
    --hz "${SCAN_HZ}" \
    --angles-topic "${CONTROL_ANGLES_TOPIC}" \
    --firecode-topic "${CONTROL_FIRECODE_TOPIC}" \
    "${SCAN_FIRECODE_ARGS[@]}" &
  PIDS+=("$!")
fi

if [[ "${ROTATE_ENABLED}" == "true" ]]; then
  echo "[INFO] starting chassis rotate publisher: level=${ROTATE_LEVEL} hz=${ROTATE_HZ}" >&2
  python3 "${ROOT_DIR}/scripts/feature_test/chassis_spin_test.py" \
    --rotate-level "${ROTATE_LEVEL}" \
    --hz "${ROTATE_HZ}" \
    --topic "${CONTROL_FIRECODE_TOPIC}" &
  PIDS+=("$!")
fi

if [[ "${SCAN_ENABLED}" != "true" && "${ROTATE_ENABLED}" != "true" ]]; then
  echo "[INFO] rotate=false scan=false; bridge publishes hold angles and safe firecode=0." >&2
fi

echo "[INFO] waiting for external navigation on ${INPUT_TOPIC}. Press Ctrl-C to stop." >&2

set +e
wait -n "${PIDS[@]}"
STATUS=$?
set -e

echo "[INFO] one child process exited; shutting down navigation control chain." >&2
exit "${STATUS}"
