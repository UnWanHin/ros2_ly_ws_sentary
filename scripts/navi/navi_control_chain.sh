#!/usr/bin/env bash

# Formal navigation control chain:
#   /ly/navi/vel -> behavior_tree -> /ly/control/vel -> gimbal_driver
# Based on regional area --pure presets, with /goal_pose disabled.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

AREA="${AREA:-central}"
USE_NOGATE=1
OFFLINE_MODE=0
ROTATE_ENABLED="${ROTATE_ENABLED:-true}"
SCAN_ENABLED="${SCAN_ENABLED:-true}"
SCAN_MODE="${SCAN_MODE:-2}"
OUTPUT="${OUTPUT:-screen}"
LAUNCH_ARGS=()
START_ARGS=(--mode regional --no-prompt)
TEMP_BT_CONFIG=""
TEMP_PATROL_CONFIG=""
BT_CONFIG_SOURCE=""

CONFIG_DIR="${ROOT_DIR}/src/behavior_tree/Scripts/ConfigJson/regional/test"
DEFAULT_BASE_CONFIG_FILE="${ROOT_DIR}/config/base_config.yaml"
DEFAULT_OVERRIDE_CONFIG_FILE="${ROOT_DIR}/config/override_config.yaml"
DEFAULT_PATROL_CONFIG_FILE="${ROOT_DIR}/src/behavior_tree/config/Patrol.yaml"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [options] [-- <launch_args...>]

Purpose:
  Start the same formal sentry_all/regional chain used by area_test --pure,
  but keep /goal_pose disabled. External navigation can still send /ly/navi/vel,
  and behavior_tree forwards it to /ly/control/vel.

Defaults:
  area=central, fire=false, rotate=true, scan=true, scan_mode=2.

Options:
  --area <base|highland|roadland|central>
  --rotate [true|false]       Enable BT rotate output. Default: ${ROTATE_ENABLED}
  --scan [true|false]         Enable BT gimbal patrol scan. Default: ${SCAN_ENABLED}
  --scan-mode <1|2|3>         PatrolScan.Mode injected through a temp Patrol.yaml. Default: ${SCAN_MODE}
  --nogate                    Bypass /ly/game/is_start. Default.
  --with-gate                 Wait for /ly/game/is_start.
  --online                    Use real gimbal device config. Default.
  --offline|--virtual-device  Pass offline:=true to sentry_all.
  --config-file <path>        Global override YAML. Default: ${DEFAULT_OVERRIDE_CONFIG_FILE}
  --base-config-file <path>   Base config YAML. Default: ${DEFAULT_BASE_CONFIG_FILE}
  --bt-config-file <path>     Source pure BT JSON instead of area preset.
  --output screen|log         Launch output mode. Default: ${OUTPUT}
  --cleanup-existing          Let start_sentry_all clean old stack. Default.
  --no-cleanup-existing       Keep old stack processes.

Examples:
  ./scripts/navi/${SCRIPT_NAME}
  ./scripts/navi/${SCRIPT_NAME} --area roadland --scan-mode 1
  ./scripts/navi/${SCRIPT_NAME} --rotate false --scan true
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

add_launch_arg_if_missing() {
  local key="$1"
  local value="$2"
  if ! has_launch_arg_key "${key}"; then
    LAUNCH_ARGS=("${key}:=${value}" "${LAUNCH_ARGS[@]}")
  fi
}

select_area_config() {
  case "${AREA}" in
    base|my_base)
      AREA="base"
      BT_CONFIG_SOURCE="${CONFIG_DIR}/regional_area_my_base_pure.json"
      ;;
    highland|my_highland)
      AREA="highland"
      BT_CONFIG_SOURCE="${CONFIG_DIR}/regional_area_my_highland_pure.json"
      ;;
    roadland|my_roadland)
      AREA="roadland"
      BT_CONFIG_SOURCE="${CONFIG_DIR}/regional_area_my_roadland_pure.json"
      ;;
    central|common_central)
      AREA="central"
      BT_CONFIG_SOURCE="${CONFIG_DIR}/regional_area_common_central_pure.json"
      ;;
    *)
      echo "[ERROR] Unknown area '${AREA}'. Expected base/highland/roadland/central." >&2
      exit 2
      ;;
  esac
}

validate_scan_mode() {
  if ! [[ "${SCAN_MODE}" =~ ^[123]$ ]]; then
    echo "[ERROR] --scan-mode must be 1, 2, or 3, got: ${SCAN_MODE}" >&2
    exit 2
  fi
}

make_bt_config() {
  TEMP_BT_CONFIG="$(mktemp /tmp/ly_navi_control_area_pure_XXXXXX.json)"
  python3 - "${BT_CONFIG_SOURCE}" "${TEMP_BT_CONFIG}" "${ROTATE_ENABLED}" "${SCAN_ENABLED}" <<'PY'
import json
import sys

src, dst, rotate_raw, scan_raw = sys.argv[1:5]

def as_bool(value: str) -> bool:
    return value.lower() in ("true", "1", "yes", "on")

with open(src, "r", encoding="utf-8") as f:
    data = json.load(f)

rotate_enabled = as_bool(rotate_raw)
scan_enabled = as_bool(scan_raw)

aim_debug = data.setdefault("AimDebug", {})
aim_debug["StopFire"] = True
aim_debug["StopRotate"] = not rotate_enabled
aim_debug["StopScan"] = not scan_enabled
aim_debug["HitCar"] = False
aim_debug["FireRequireTargetStatus"] = True

data.setdefault("RegionalAreaTask", {})["IgnoreRecovery"] = True
data.setdefault("Chase", {})["Enable"] = False
data.setdefault("Posture", {})["Enable"] = False
data.setdefault("Rate", {})["NaviCommandRate"] = 1

with open(dst, "w", encoding="utf-8") as f:
    json.dump(data, f, indent=4, ensure_ascii=False)
    f.write("\n")
PY
}

make_patrol_config() {
  TEMP_PATROL_CONFIG="$(mktemp /tmp/ly_navi_control_patrol_XXXXXX.yaml)"
  python3 - "${DEFAULT_PATROL_CONFIG_FILE}" "${TEMP_PATROL_CONFIG}" "${SCAN_MODE}" <<'PY'
import sys
import yaml

src, dst, scan_mode_raw = sys.argv[1:4]

with open(src, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}

params = data.setdefault("behavior_tree", {}).setdefault("ros__parameters", {})
patrol_scan = params.setdefault("PatrolScan", {})
patrol_scan["Mode"] = int(scan_mode_raw)

with open(dst, "w", encoding="utf-8") as f:
    yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
PY
}

cleanup() {
  if [[ -n "${TEMP_BT_CONFIG}" && -f "${TEMP_BT_CONFIG}" ]]; then
    rm -f "${TEMP_BT_CONFIG}"
  fi
  if [[ -n "${TEMP_PATROL_CONFIG}" && -f "${TEMP_PATROL_CONFIG}" ]]; then
    rm -f "${TEMP_PATROL_CONFIG}"
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --area)
      if (( $# < 2 )); then
        echo "[ERROR] --area requires base/highland/roadland/central." >&2
        exit 2
      fi
      AREA="$2"
      shift 2
      ;;
    --area=*)
      AREA="${1#*=}"
      shift
      ;;
    area=*)
      AREA="${1#*=}"
      shift
      ;;
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
        echo "[ERROR] --scan-mode requires 1, 2, or 3." >&2
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
    --bt-config-file)
      if (( $# < 2 )); then
        echo "[ERROR] --bt-config-file requires a path." >&2
        exit 2
      fi
      BT_CONFIG_SOURCE="$2"
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
    --cleanup-existing|--no-cleanup-existing)
      START_ARGS+=("$1")
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

if [[ -z "${BT_CONFIG_SOURCE}" ]]; then
  select_area_config
fi
validate_scan_mode

if [[ ! -f "${BT_CONFIG_SOURCE}" ]]; then
  echo "[ERROR] BT config not found: ${BT_CONFIG_SOURCE}" >&2
  exit 1
fi
if [[ ! -f "${DEFAULT_PATROL_CONFIG_FILE}" ]]; then
  echo "[ERROR] Patrol config not found: ${DEFAULT_PATROL_CONFIG_FILE}" >&2
  exit 1
fi

trap cleanup EXIT
make_bt_config
make_patrol_config

add_launch_arg_if_missing "bt_config_file" "${TEMP_BT_CONFIG}"
add_launch_arg_if_missing "patrol_config_file" "${TEMP_PATROL_CONFIG}"
add_launch_arg_if_missing "debug_bypass_is_start" "$([[ "${USE_NOGATE}" == "1" ]] && printf true || printf false)"
add_launch_arg_if_missing "wait_for_game_start_timeout_sec" "0"
add_launch_arg_if_missing "competition_profile" "regional"
add_launch_arg_if_missing "publish_navi_goal" "true"
add_launch_arg_if_missing "navi_publish_goal_pose" "false"
add_launch_arg_if_missing "base_config_file" "${DEFAULT_BASE_CONFIG_FILE}"
add_launch_arg_if_missing "config_file" "${DEFAULT_OVERRIDE_CONFIG_FILE}"
add_launch_arg_if_missing "output" "${OUTPUT}"

if (( OFFLINE_MODE == 1 )); then
  START_ARGS+=(--offline)
fi

echo "[INFO] navi control uses area_test --pure style: area=${AREA}" >&2
echo "[INFO] /goal_pose disabled: publish_navi_goal=true, navi_publish_goal_pose=false" >&2
echo "[INFO] fire=false rotate=${ROTATE_ENABLED} scan=${SCAN_ENABLED} scan_mode=${SCAN_MODE}" >&2
echo "[INFO] source bt_config=${BT_CONFIG_SOURCE}" >&2
echo "[INFO] generated bt_config=${TEMP_BT_CONFIG}" >&2
echo "[INFO] generated patrol_config=${TEMP_PATROL_CONFIG}" >&2

"${ROOT_DIR}/scripts/launch/start_sentry_all.sh" "${START_ARGS[@]}" -- "${LAUNCH_ARGS[@]}"
