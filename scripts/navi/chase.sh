#!/usr/bin/env bash

# Formal chase test chain:
#   external /ly/aim/armor_targets + /ly/aim/result -> behavior_tree
#   behavior_tree -> /ly/navi/target_rel -> navi_tf_bridge -> /goal_pose
# It keeps gimbal lock/patrol and chassis rotate enabled by default, and
# disables Chase.AreaLimit so this script only checks whether chase works.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

SOURCE_BT_CONFIG="${SOURCE_BT_CONFIG:-${ROOT_DIR}/src/behavior_tree/Scripts/ConfigJson/regional/debug/armor_patrol_test.json}"
DEFAULT_NAVI_ROTATE_CONFIG_FILE="${ROOT_DIR}/src/behavior_tree/config/NaviRotateControl.yaml"

USE_NOGATE=1
OFFLINE_MODE=0
CLEANUP_EXISTING=1
FIRE_ENABLED="${FIRE_ENABLED:-false}"
ROTATE_ENABLED="${ROTATE_ENABLED:-true}"
SCAN_ENABLED="${SCAN_ENABLED:-true}"
SCAN_MODE="${SCAN_MODE:-2}"
AREA_LIMIT_ENABLED="${AREA_LIMIT_ENABLED:-false}"
RESPECT_IS_ROTATE=0
OUTPUT="${OUTPUT:-screen}"
TEMP_BT_CONFIG=""
TEMP_NAVI_ROTATE_CONFIG=""
START_ARGS=(--mode regional --no-prompt)
LAUNCH_ARGS=()

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [options] [-- <launch_args...>]

Purpose:
  Start the formal sentry_all/regional chain for chase testing.
  It waits for external aim topics:
    /ly/aim/armor_targets   sentry_msgs/msg/AimTargetArray
    /ly/aim/result       sentry_msgs/msg/AimResult
  BT publishes:
    /ly/aim/select_target -> external aim
    /ly/navi/target_rel  -> navi_tf_bridge -> /goal_pose
    /ly/control/angles + /ly/control/firecode -> gimbal_driver

Defaults:
  fire=false, rotate=true, scan=true, scan_mode=2, area_limit=false,
  publish_navi_goal=false, ignore /ly/navi/should_rotate.

Options:
  --bt-config-file <path>          Source BT JSON to patch. Default:
                                  ${SOURCE_BT_CONFIG}
  --fire [true|false]              Allow fire from /ly/aim/result.fire. Default: ${FIRE_ENABLED}
  --no-fire                        Same as --fire false.
  --rotate [true|false]            Enable chassis rotate output. Default: ${ROTATE_ENABLED}
  --scan [true|false]              Enable gimbal patrol scan when no target. Default: ${SCAN_ENABLED}
  --scan-mode <1|2|3>              PatrolScan.Mode. Default: ${SCAN_MODE}
  --area-limit [true|false]        Enable Chase.AreaLimit. Default: ${AREA_LIMIT_ENABLED}
  --respect-is-rotate              Use normal NaviRotateControl.yaml and obey /ly/navi/should_rotate.
  --ignore-is-rotate               Disable NaviRotateControl for this test. Default.
  --nogate                         Bypass /ly/game/is_start. Default.
  --with-gate                      Do not bypass /ly/game/is_start.
  --online                         Use real gimbal device config. Default.
  --offline|--virtual-device       Pass offline:=true to sentry_all.
  --output screen|log              Launch output mode. Default: ${OUTPUT}
  --cleanup-existing               Clean old sentry/chase stack. Default.
  --no-cleanup-existing            Keep old processes.

Examples:
  ./scripts/navi/${SCRIPT_NAME}
  ./scripts/navi/${SCRIPT_NAME} --fire false --scan-mode 2
  ./scripts/navi/${SCRIPT_NAME} --respect-is-rotate --area-limit true
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

validate_scan_mode() {
  if ! [[ "${SCAN_MODE}" =~ ^[123]$ ]]; then
    echo "[ERROR] --scan-mode must be 1, 2, or 3, got: ${SCAN_MODE}" >&2
    exit 2
  fi
}

make_bt_config() {
  TEMP_BT_CONFIG="$(mktemp /tmp/ly_chase_formal_XXXXXX.json)"
  python3 - "${SOURCE_BT_CONFIG}" "${TEMP_BT_CONFIG}" \
    "${FIRE_ENABLED}" "${ROTATE_ENABLED}" "${SCAN_ENABLED}" "${SCAN_MODE}" "${AREA_LIMIT_ENABLED}" <<'PY'
import json
import sys

src, dst, fire_raw, rotate_raw, scan_raw, scan_mode_raw, area_limit_raw = sys.argv[1:8]

def as_bool(value: str) -> bool:
    return value.lower() in ("true", "1", "yes", "y", "on")

with open(src, "r", encoding="utf-8") as f:
    data = json.load(f)

fire_enabled = as_bool(fire_raw)
rotate_enabled = as_bool(rotate_raw)
scan_enabled = as_bool(scan_raw)
area_limit_enabled = as_bool(area_limit_raw)
scan_mode = int(scan_mode_raw)

data["CompetitionProfile"] = "regional"

aim_debug = data.setdefault("AimDebug", {})
aim_debug["StopFire"] = not fire_enabled
aim_debug["StopRotate"] = not rotate_enabled
aim_debug["StopScan"] = not scan_enabled
aim_debug["ForceBuff"] = False
aim_debug["ForceOutpost"] = False
aim_debug["HitCar"] = False
aim_debug["FireRequireTargetStatus"] = True

data.setdefault("PatrolScan", {})["Mode"] = scan_mode
data.setdefault("Rate", {})["NaviCommandRate"] = 1
data.setdefault("Task", {})["Buff"] = False
data.setdefault("Task", {})["Outpost"] = False
data.setdefault("NaviSetting", {})["UseXY"] = True
data.setdefault("NaviSetting", {})["ToNavi"] = True
data.setdefault("RegionalDefense", {})["Enable"] = False
data.setdefault("NaviProgressWatchdog", {})["Enable"] = False
data.setdefault("RegionalIdlePatrol", {})["Enable"] = False
data.setdefault("RegionalAreaTask", {})["IgnoreRecovery"] = True
data.setdefault("Posture", {})["Enable"] = False

external_aim = data.setdefault("ExternalAim", {})
external_aim["Enable"] = True
external_aim["UseTargetArrayAsArmorList"] = True
external_aim["PublishSelectTarget"] = True
external_aim.setdefault("ResultFreshTimeoutMs", 300)
external_aim.setdefault("TargetFreshTimeoutMs", 500)
external_aim.setdefault("TargetDefaultFrame", "gimbal_world")

chase = data.setdefault("Chase", {})
chase["Enable"] = True
chase["FollowAimTarget"] = True
chase["ToNavi"] = True
chase["UseOfficialPositionSource"] = False
chase["PreferOfficialPositionSource"] = False
chase["EnableInAutoAim"] = True
chase["EnableInRotateScan"] = True
chase["EnableInOutpostMode"] = False
chase["EnableInBuffMode"] = False
chase["StopWhenNoTarget"] = True
chase["LostTargetHoldMs"] = 200
chase["PreferredDistanceCm"] = 300
chase["DistanceDeadbandCm"] = 50
chase["MinValidDistanceCm"] = 80
chase["MaxValidDistanceCm"] = 1200
chase.setdefault("DistanceKp", 0.06)
chase.setdefault("MaxForwardSpeed", 35)
chase.setdefault("MaxBackwardSpeed", 25)
chase.setdefault("UseYawStrafe", True)
chase.setdefault("YawKp", 0.4)
chase.setdefault("YawDeadbandDeg", 3)
chase.setdefault("MaxStrafeSpeed", 22)
chase.setdefault("InvertStrafeDirection", False)

area_limit = chase.setdefault("AreaLimit", {})
area_limit["Enable"] = area_limit_enabled
area_limit.setdefault("BoundaryMarginCm", 30)
area_limit.setdefault("ChaseEnableCrossArea", False)
area_limit.setdefault("HoldWhenNoIntersection", True)

with open(dst, "w", encoding="utf-8") as f:
    json.dump(data, f, indent=4, ensure_ascii=False)
    f.write("\n")
PY
}

make_navi_rotate_config() {
  if (( RESPECT_IS_ROTATE == 1 )); then
    TEMP_NAVI_ROTATE_CONFIG="${DEFAULT_NAVI_ROTATE_CONFIG_FILE}"
    return
  fi

  TEMP_NAVI_ROTATE_CONFIG="$(mktemp /tmp/ly_chase_navi_rotate_XXXXXX.yaml)"
  cat > "${TEMP_NAVI_ROTATE_CONFIG}" <<'YAML'
behavior_tree:
  ros__parameters:
    NaviRotateControl:
      Enable: false
      FreshTimeoutMs: 500
      DefaultIsRotate: true
      ForceFollowModeWhenFalse: false
      ClearFollowModeWhenTrue: true
      ClearRegionalFaceModeWhenTrue: true
      StopRotateWhenFalse: false
YAML
}

cleanup() {
  if [[ -n "${TEMP_BT_CONFIG}" && -f "${TEMP_BT_CONFIG}" ]]; then
    rm -f "${TEMP_BT_CONFIG}"
  fi
  if (( RESPECT_IS_ROTATE == 0 )) &&
     [[ -n "${TEMP_NAVI_ROTATE_CONFIG}" && -f "${TEMP_NAVI_ROTATE_CONFIG}" ]]; then
    rm -f "${TEMP_NAVI_ROTATE_CONFIG}"
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bt-config-file)
      if (( $# < 2 )); then
        echo "[ERROR] --bt-config-file requires a path." >&2
        exit 2
      fi
      SOURCE_BT_CONFIG="$2"
      shift 2
      ;;
    --fire)
      FIRE_ENABLED="$(take_optional_bool "${2:-}")"
      shift "$(optional_bool_shift "${2:-}")"
      ;;
    --fire=*)
      FIRE_ENABLED="$(parse_bool "${1#*=}")"
      shift
      ;;
    fire=*)
      FIRE_ENABLED="$(parse_bool "${1#*=}")"
      shift
      ;;
    --no-fire)
      FIRE_ENABLED="false"
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
    --area-limit)
      AREA_LIMIT_ENABLED="$(take_optional_bool "${2:-}")"
      shift "$(optional_bool_shift "${2:-}")"
      ;;
    --area-limit=*)
      AREA_LIMIT_ENABLED="$(parse_bool "${1#*=}")"
      shift
      ;;
    area_limit=*)
      AREA_LIMIT_ENABLED="$(parse_bool "${1#*=}")"
      shift
      ;;
    --respect-is-rotate)
      RESPECT_IS_ROTATE=1
      shift
      ;;
    --ignore-is-rotate)
      RESPECT_IS_ROTATE=0
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
      START_ARGS+=("$1")
      shift
      ;;
    --no-cleanup-existing)
      CLEANUP_EXISTING=0
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

validate_scan_mode

if [[ ! -f "${SOURCE_BT_CONFIG}" ]]; then
  echo "[ERROR] BT config not found: ${SOURCE_BT_CONFIG}" >&2
  exit 1
fi

source_ros_workspace "${ROOT_DIR}"
cleanup_existing_stack "${CLEANUP_EXISTING}" \
  "/(gimbal_driver_node|behavior_tree_node|target_rel_to_goal_pos_node|map_aim_point_node)([[:space:]]|$)" \
  "ros2 launch (behavior_tree|navi_tf_bridge) (sentry_all|competition_autoaim|showcase|chase_only|decision_chase|target_rel_to_goal_pos)\\.launch.py"

trap cleanup EXIT
make_bt_config
make_navi_rotate_config

add_launch_arg_if_missing "bt_config_file" "${TEMP_BT_CONFIG}"
add_launch_arg_if_missing "debug_bypass_is_start" "$([[ "${USE_NOGATE}" == "1" ]] && printf true || printf false)"
add_launch_arg_if_missing "wait_for_game_start_timeout_sec" "0"
add_launch_arg_if_missing "competition_profile" "regional"
add_launch_arg_if_missing "publish_navi_goal" "false"
add_launch_arg_if_missing "navi_publish_goal_pose" "true"
add_launch_arg_if_missing "navi_rotate_config_file" "${TEMP_NAVI_ROTATE_CONFIG}"
add_launch_arg_if_missing "output" "${OUTPUT}"

if (( OFFLINE_MODE == 1 )); then
  START_ARGS+=(--offline)
fi

echo "[INFO] formal chase test via sentry_all/regional" >&2
echo "[INFO] external aim input: /ly/aim/armor_targets + /ly/aim/result" >&2
echo "[INFO] chase output: /ly/navi/target_rel -> /goal_pose" >&2
echo "[INFO] fire=${FIRE_ENABLED} rotate=${ROTATE_ENABLED} scan=${SCAN_ENABLED} scan_mode=${SCAN_MODE} area_limit=${AREA_LIMIT_ENABLED}" >&2
echo "[INFO] respect_should_rotate=${RESPECT_IS_ROTATE} source_bt_config=${SOURCE_BT_CONFIG}" >&2
echo "[INFO] generated bt_config=${TEMP_BT_CONFIG}" >&2
echo "[INFO] navi_rotate_config=${TEMP_NAVI_ROTATE_CONFIG}" >&2

"${ROOT_DIR}/scripts/launch/start_sentry_all.sh" "${START_ARGS[@]}" -- "${LAUNCH_ARGS[@]}"
