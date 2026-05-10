#!/usr/bin/env bash

# Formal navigation control chain:
#   /ly/navi/vel -> behavior_tree -> /ly/control/vel -> gimbal_driver
# Uses the same BT path as armor_patrol_test: no firing, rotate enabled, PatrolScan enabled.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

USE_NOGATE=1
OFFLINE_MODE=0
CLEANUP_EXISTING=1
ROTATE_ENABLED="${ROTATE_ENABLED:-true}"
SCAN_ENABLED="${SCAN_ENABLED:-true}"
SCAN_MODE="${SCAN_MODE:-2}"
WITH_VISION=0
OUTPUT="${OUTPUT:-screen}"
LAUNCH_ARGS=()
TEMP_BT_CONFIG=""

DEFAULT_BASE_CONFIG_FILE="${ROOT_DIR}/config/base_config.yaml"
DEFAULT_DETECTOR_CONFIG_FILE="${ROOT_DIR}/src/detector/config/detector_config.yaml"
DEFAULT_PREDICTOR_CONFIG_FILE="${ROOT_DIR}/src/predictor/config/predictor_config.yaml"
DEFAULT_OVERRIDE_CONFIG_FILE="${ROOT_DIR}/config/override_config.yaml"
DEFAULT_BT_CONFIG_FILE="${ROOT_DIR}/src/behavior_tree/Scripts/ConfigJson/regional/debug/armor_patrol_test.json"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [options] [-- <launch_args...>]

Purpose:
  Start a formal behavior_tree navigation-control chain, like armor_patrol_test:
  - no firing
  - /ly/navi/vel is handled by behavior_tree and forwarded to /ly/control/vel
  - rotate and PatrolScan are controlled by behavior_tree, not external test publishers
  - detector/tracker/predictor are disabled by default so gimbal scan is not interrupted

Options:
  --rotate [true|false]       Enable BT rotate output. Default: ${ROTATE_ENABLED}
  --scan [true|false]         Enable BT gimbal patrol scan. Default: ${SCAN_ENABLED}
  --scan-mode <1|2>           PatrolScan.Mode. Default: ${SCAN_MODE}
  --with-vision               Also start detector/tracker/predictor, matching armor_test closer.
  --no-vision                 Do not start detector/tracker/predictor. Default.
  --nogate                    Bypass /ly/game/is_start. Default.
  --with-gate                 Wait for /ly/game/is_start.
  --online                    Use real gimbal device config. Default.
  --offline|--virtual-device  Force offline virtual-device launch behavior.
  --config-file <path>        Global override YAML. Default: ${DEFAULT_OVERRIDE_CONFIG_FILE}
  --base-config-file <path>   Base config YAML. Default: ${DEFAULT_BASE_CONFIG_FILE}
  --detector-config-file <p>  Detector config YAML. Default: ${DEFAULT_DETECTOR_CONFIG_FILE}
  --predictor-config-file <p> Predictor config YAML. Default: ${DEFAULT_PREDICTOR_CONFIG_FILE}
  --bt-config-file <path>     Source BT JSON. Default: armor_patrol_test.json
  --output screen|log         Launch output mode. Default: ${OUTPUT}
  --no-cleanup-existing       Do not clean old sentry/BT/driver processes.

Examples:
  ./scripts/navi/${SCRIPT_NAME}
  ./scripts/navi/${SCRIPT_NAME} --scan-mode 1
  ./scripts/navi/${SCRIPT_NAME} --rotate false --scan true
  ./scripts/navi/${SCRIPT_NAME} --with-vision
  ./scripts/navi/${SCRIPT_NAME} scanmode=2 rotate=true scan=true
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

has_launch_arg_key() {
  local key="$1"
  local arg
  for arg in "${LAUNCH_ARGS[@]}"; do
    if [[ "${arg}" == "${key}:="* ]]; then
      return 0
    fi
  done
  return 1
}

append_launch_arg_if_missing() {
  local key="$1"
  local value="$2"
  if ! has_launch_arg_key "${key}"; then
    LAUNCH_ARGS=("${key}:=${value}" "${LAUNCH_ARGS[@]}")
  fi
}

validate_scan_mode() {
  if ! [[ "${SCAN_MODE}" =~ ^[12]$ ]]; then
    echo "[ERROR] --scan-mode must be 1 or 2, got: ${SCAN_MODE}" >&2
    exit 2
  fi
}

make_bt_config() {
  TEMP_BT_CONFIG="$(mktemp /tmp/ly_navi_control_bt_XXXXXX.json)"
  python3 - "${DEFAULT_BT_CONFIG_FILE}" "${TEMP_BT_CONFIG}" "${ROTATE_ENABLED}" "${SCAN_ENABLED}" "${SCAN_MODE}" <<'PY'
import json
import sys

src, dst, rotate_raw, scan_raw, scan_mode_raw = sys.argv[1:6]

def as_bool(value: str) -> bool:
    return value.lower() in ("true", "1", "yes", "on")

with open(src, "r", encoding="utf-8") as f:
    data = json.load(f)

rotate_enabled = as_bool(rotate_raw)
scan_enabled = as_bool(scan_raw)
scan_mode = int(scan_mode_raw)

aim_debug = data.setdefault("AimDebug", {})
aim_debug["StopFire"] = True
aim_debug["StopRotate"] = not rotate_enabled
aim_debug["StopScan"] = not scan_enabled
aim_debug["HitCar"] = False
aim_debug["FireRequireTargetStatus"] = True

data.setdefault("PatrolScan", {})["Mode"] = scan_mode

navi = data.setdefault("NaviSetting", {})
navi["UseXY"] = False
navi["ToNavi"] = True

data.setdefault("Rate", {})["NaviCommandRate"] = 1

with open(dst, "w", encoding="utf-8") as f:
    json.dump(data, f, indent=4, ensure_ascii=False)
    f.write("\n")
PY
}

cleanup() {
  if [[ -n "${TEMP_BT_CONFIG}" && -f "${TEMP_BT_CONFIG}" ]]; then
    rm -f "${TEMP_BT_CONFIG}"
  fi
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
    --with-vision)
      WITH_VISION=1
      shift
      ;;
    --no-vision)
      WITH_VISION=0
      shift
      ;;
    --nogate)
      USE_NOGATE=1
      shift
      ;;
    --with-gate)
      USE_NOGATE=0
      shift
      ;;
    --online)
      OFFLINE_MODE=0
      shift
      ;;
    --offline|--virtual-device)
      OFFLINE_MODE=1
      shift
      ;;
    --config-file)
      if (( $# < 2 )); then
        echo "[ERROR] --config-file requires a path." >&2
        exit 2
      fi
      DEFAULT_OVERRIDE_CONFIG_FILE="$2"
      shift 2
      ;;
    --base-config-file)
      if (( $# < 2 )); then
        echo "[ERROR] --base-config-file requires a path." >&2
        exit 2
      fi
      DEFAULT_BASE_CONFIG_FILE="$2"
      shift 2
      ;;
    --detector-config-file)
      if (( $# < 2 )); then
        echo "[ERROR] --detector-config-file requires a path." >&2
        exit 2
      fi
      DEFAULT_DETECTOR_CONFIG_FILE="$2"
      shift 2
      ;;
    --predictor-config-file)
      if (( $# < 2 )); then
        echo "[ERROR] --predictor-config-file requires a path." >&2
        exit 2
      fi
      DEFAULT_PREDICTOR_CONFIG_FILE="$2"
      shift 2
      ;;
    --bt-config-file)
      if (( $# < 2 )); then
        echo "[ERROR] --bt-config-file requires a path." >&2
        exit 2
      fi
      DEFAULT_BT_CONFIG_FILE="$2"
      shift 2
      ;;
    --output)
      if (( $# < 2 )); then
        echo "[ERROR] --output requires screen or log." >&2
        exit 2
      fi
      OUTPUT="$2"
      shift 2
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
      LAUNCH_ARGS+=("$@")
      break
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

validate_scan_mode
trap cleanup EXIT

source_ros_workspace "${ROOT_DIR}"
cleanup_existing_stack \
  "${CLEANUP_EXISTING}" \
  "/(gimbal_driver_node|detector_node|tracker_solver_node|predictor_node|outpost_hitter_node|buff_hitter_node|behavior_tree_node|navi_vel_control_bridge|scan_gimbal_test|chassis_spin_test)([[:space:]]|$)|scripts/navi/navi_vel_chain\\.py|scripts/feature_test/(scan_gimbal_test|chassis_spin_test)\\.py" \
  "ros2 launch behavior_tree (armor_patrol_test|competition_autoaim|sentry_all|chase_only|showcase|navi_debug)\\.launch\\.py|ros2 launch gimbal_driver gimbal_driver\\.launch\\.py"

make_bt_config

append_launch_arg_if_missing "base_config_file" "${DEFAULT_BASE_CONFIG_FILE}"
append_launch_arg_if_missing "config_file" "${DEFAULT_OVERRIDE_CONFIG_FILE}"
append_launch_arg_if_missing "detector_config_file" "${DEFAULT_DETECTOR_CONFIG_FILE}"
append_launch_arg_if_missing "predictor_config_file" "${DEFAULT_PREDICTOR_CONFIG_FILE}"
append_launch_arg_if_missing "bt_config_file" "${TEMP_BT_CONFIG}"
append_launch_arg_if_missing "debug_bypass_is_start" "$([[ "${USE_NOGATE}" == "1" ]] && printf true || printf false)"
append_launch_arg_if_missing "wait_for_game_start_timeout_sec" "0"
append_launch_arg_if_missing "publish_navi_goal" "false"
append_launch_arg_if_missing "use_outpost" "false"
append_launch_arg_if_missing "use_buff" "false"
append_launch_arg_if_missing "use_detector" "$([[ "${WITH_VISION}" == "1" ]] && printf true || printf false)"
append_launch_arg_if_missing "use_tracker" "$([[ "${WITH_VISION}" == "1" ]] && printf true || printf false)"
append_launch_arg_if_missing "use_predictor" "$([[ "${WITH_VISION}" == "1" ]] && printf true || printf false)"
append_launch_arg_if_missing "output" "${OUTPUT}"

if (( OFFLINE_MODE == 1 )); then
  append_launch_arg_if_missing "offline" "true"
fi

echo "[INFO] formal navi control chain: /ly/navi/vel -> behavior_tree -> /ly/control/vel -> gimbal_driver" >&2
echo "[INFO] fire=false rotate=${ROTATE_ENABLED} scan=${SCAN_ENABLED} scan_mode=${SCAN_MODE} vision=${WITH_VISION}" >&2
echo "[INFO] generated bt_config=${TEMP_BT_CONFIG}" >&2

ros2 launch behavior_tree armor_patrol_test.launch.py "${LAUNCH_ARGS[@]}"
