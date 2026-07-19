#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# FaceMode gimbal aim test. Edit OFFICIAL_MAP_X/OFFICIAL_MAP_Y/MAP_Z below for the point to face.
# Real map aiming needs navigation/localization to publish map -> base_link.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

# FaceMode 坐标写这里：默认单位 cm，可用 OFFICIAL_MAP_UNIT=m 或 --unit m 改为米。
# X/Y 是官方二维地图坐标，会按 navi_tf_bridge/config/tf_config.yaml 的 4x4
# 走和 /ly/navi/goal_pos_raw -> /goal_pose 一样的平面转换，默认把结果当 map 点使用。
# Z 已经是 map 系高度，只作为瞄准高度，不参与官方二维地图 X/Y 转换，单位跟 OFFICIAL_MAP_UNIT 一致。
OFFICIAL_MAP_X="${OFFICIAL_MAP_X:-}"
OFFICIAL_MAP_Y="${OFFICIAL_MAP_Y:-}"
MAP_Z="${MAP_Z:-}"
OFFICIAL_MAP_UNIT="${OFFICIAL_MAP_UNIT:-cm}"
TARGET_FRAME="${TARGET_FRAME:-official_map}"
USE_RAW_GOAL_STATIC_CALIBRATION="${USE_RAW_GOAL_STATIC_CALIBRATION:-true}"
RAW_GOAL_TARGET_FRAME="${RAW_GOAL_TARGET_FRAME:-map}"

# 默认按长焦 gx_camera_0 投影误差解 yaw/pitch；不可用时回退短焦 gx_camera_1。
AIM_FRAME="${AIM_FRAME:-gimbal_world}"
CAMERA_FRAME="${CAMERA_FRAME:-gx_camera_0}"
CAMERA_FALLBACK_FRAME="${CAMERA_FALLBACK_FRAME:-gx_camera_1}"
SOLVE_MODE="${SOLVE_MODE:-camera_projection}"
SOLVE_FRAME="${SOLVE_FRAME:-base_link}"
USE_GIMBAL="${USE_GIMBAL:-true}"
USE_VIRTUAL_DEVICE="${USE_VIRTUAL_DEVICE:-false}"
USE_MOCK_MAP_TO_BASE="${USE_MOCK_MAP_TO_BASE:-false}"
USE_MOCK_GIMBAL_STATE="${USE_MOCK_GIMBAL_STATE:-false}"
MOCK_MAP_TO_BASE_X="${MOCK_MAP_TO_BASE_X:-0.0}"
MOCK_MAP_TO_BASE_Y="${MOCK_MAP_TO_BASE_Y:-0.0}"
MOCK_MAP_TO_BASE_Z="${MOCK_MAP_TO_BASE_Z:-0.0}"
MOCK_MAP_TO_BASE_YAW="${MOCK_MAP_TO_BASE_YAW:-0.0}"
MOCK_MAP_TO_BASE_PITCH="${MOCK_MAP_TO_BASE_PITCH:-0.0}"
MOCK_MAP_TO_BASE_ROLL="${MOCK_MAP_TO_BASE_ROLL:-0.0}"
MOCK_GIMBAL_YAW_DEG="${MOCK_GIMBAL_YAW_DEG:-0.0}"
MOCK_GIMBAL_PITCH_DEG="${MOCK_GIMBAL_PITCH_DEG:-0.0}"
MOCK_GIMBAL_BIG_YAW_DEG="${MOCK_GIMBAL_BIG_YAW_DEG:-0.0}"
MOCK_GIMBAL_PUBLISH_BIG_YAW="${MOCK_GIMBAL_PUBLISH_BIG_YAW:-true}"
PUBLISH_FIRECODE="${PUBLISH_FIRECODE:-true}"
AIM_MODE="${AIM_MODE:-true}"
YAW_SIGN="${YAW_SIGN:--1.0}"
PITCH_SIGN="${PITCH_SIGN:-1.0}"
YAW_BIAS_DEG="${YAW_BIAS_DEG:-0.0}"
PITCH_BIAS_DEG="${PITCH_BIAS_DEG:-0.0}"
PUBLISH_HZ="${PUBLISH_HZ:-30.0}"
USE_GIMBAL_STAMP_FOR_TF="${USE_GIMBAL_STAMP_FOR_TF:-false}"
MAX_GIMBAL_STAMP_AGE_SEC="${MAX_GIMBAL_STAMP_AGE_SEC:-0.50}"
MAX_TARGET_DISTANCE_M="${MAX_TARGET_DISTANCE_M:-100.0}"
COMMAND_FILTER_ALPHA="${COMMAND_FILTER_ALPHA:-1.0}"
MAX_YAW_STEP_DEG="${MAX_YAW_STEP_DEG:-}"
MAX_PITCH_STEP_DEG="${MAX_PITCH_STEP_DEG:-}"
OUTPUT_MODE="${OUTPUT_MODE:-screen}"
LAUNCH_ARGS=()

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

COMMON_CONFIG_FILE="${ROOT_DIR}/config/common.yaml"
if [[ -z "${MAX_YAW_STEP_DEG}" ]]; then
  MAX_YAW_STEP_DEG="$(read_yaml_path_scalar "${COMMON_CONFIG_FILE}" "face_mode.max_yaw_step_deg" || printf '0.0\n')"
fi
if [[ -z "${MAX_PITCH_STEP_DEG}" ]]; then
  MAX_PITCH_STEP_DEG="$(read_yaml_path_scalar "${COMMON_CONFIG_FILE}" "face_mode.max_pitch_step_deg" || printf '0.0\n')"
fi

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--unit m|cm] [--bench] [--no-gimbal] [--mock-map-origin] [--mock-gimbal-state] [--no-firecode] [-- <launch_args...>]

Purpose:
  FaceMode: keep gimbal facing one fixed map/official-map point.
  Requires external sentry_tf to publish the gimbal TF chain.
  Publishes /ly/control/angles and, by default, /ly/control/firecode aim_mode=true.
  It does not publish chassis velocity or fire commands.
  A real map run requires an external navigation/localization TF chain that provides map -> base_link.
  For a bench-only map origin pose, use --mock-map-origin or use_mock_map_to_base:=true.
  For a no-hardware closed-loop smoke test, use --bench.

Required point parameters:
  OFFICIAL_MAP_X=${OFFICIAL_MAP_X}
  OFFICIAL_MAP_Y=${OFFICIAL_MAP_Y}
  MAP_Z=${MAP_Z}
  OFFICIAL_MAP_UNIT=${OFFICIAL_MAP_UNIT}  # m or cm; launch node still receives cm internally

Edit those variables near the top of this script or pass them as launch args/env vars.

Other defaults:
  TARGET_FRAME=${TARGET_FRAME}
  USE_RAW_GOAL_STATIC_CALIBRATION=${USE_RAW_GOAL_STATIC_CALIBRATION}
  RAW_GOAL_TARGET_FRAME=${RAW_GOAL_TARGET_FRAME}
  CAMERA_FRAME=${CAMERA_FRAME}
  CAMERA_FALLBACK_FRAME=${CAMERA_FALLBACK_FRAME}
  SOLVE_MODE=${SOLVE_MODE}
  SOLVE_FRAME=${SOLVE_FRAME}
  USE_MOCK_MAP_TO_BASE=${USE_MOCK_MAP_TO_BASE}
  USE_MOCK_GIMBAL_STATE=${USE_MOCK_GIMBAL_STATE}
  MOCK_MAP_TO_BASE_X=${MOCK_MAP_TO_BASE_X}
  MOCK_MAP_TO_BASE_Y=${MOCK_MAP_TO_BASE_Y}
  MOCK_MAP_TO_BASE_YAW=${MOCK_MAP_TO_BASE_YAW}
  MOCK_GIMBAL_YAW_DEG=${MOCK_GIMBAL_YAW_DEG}
  MOCK_GIMBAL_PITCH_DEG=${MOCK_GIMBAL_PITCH_DEG}
  MOCK_GIMBAL_BIG_YAW_DEG=${MOCK_GIMBAL_BIG_YAW_DEG}
  MOCK_GIMBAL_PUBLISH_BIG_YAW=${MOCK_GIMBAL_PUBLISH_BIG_YAW}
  USE_GIMBAL_STAMP_FOR_TF=${USE_GIMBAL_STAMP_FOR_TF}
  MAX_TARGET_DISTANCE_M=${MAX_TARGET_DISTANCE_M}
  MAX_GIMBAL_STAMP_AGE_SEC=${MAX_GIMBAL_STAMP_AGE_SEC}
  COMMAND_FILTER_ALPHA=${COMMAND_FILTER_ALPHA}
  MAX_YAW_STEP_DEG=${MAX_YAW_STEP_DEG}
  MAX_PITCH_STEP_DEG=${MAX_PITCH_STEP_DEG}

Examples:
  OFFICIAL_MAP_X=1093 OFFICIAL_MAP_Y=366 MAP_Z=100 ./${SCRIPT_NAME}
  OFFICIAL_MAP_UNIT=m OFFICIAL_MAP_X=10.93 OFFICIAL_MAP_Y=3.66 MAP_Z=1.00 ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --unit cm -- official_map_x:=1093 official_map_y:=366 map_z:=100
  OFFICIAL_MAP_X=1093 OFFICIAL_MAP_Y=366 MAP_Z=100 ./${SCRIPT_NAME} --bench
  OFFICIAL_MAP_X=1093 OFFICIAL_MAP_Y=366 MAP_Z=100 ./${SCRIPT_NAME} --mock-map-origin
  ./${SCRIPT_NAME} -- official_map_x:=1093 official_map_y:=366 map_z:=100
  OFFICIAL_MAP_X=1093 OFFICIAL_MAP_Y=366 MAP_Z=100 USE_MOCK_MAP_TO_BASE=true MOCK_MAP_TO_BASE_X=1.2 MOCK_MAP_TO_BASE_Y=-0.4 ./${SCRIPT_NAME}
  OFFICIAL_MAP_X=1093 OFFICIAL_MAP_Y=366 MAP_Z=100 COMMAND_FILTER_ALPHA=0.25 MAX_YAW_STEP_DEG=3.0 MAX_PITCH_STEP_DEG=1.5 ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} -- official_map_x:=1093 official_map_y:=366 map_z:=100 raw_goal_target_frame:=map yaw_sign:=-1.0 pitch_bias_deg:=2.0
EOF
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

launch_arg_value() {
  local key="$1"
  local default_value="$2"
  local arg
  for arg in "${LAUNCH_ARGS[@]}"; do
    if [[ "${arg}" == "${key}:="* ]]; then
      printf '%s\n' "${arg#*:=}"
      return 0
    fi
  done
  printf '%s\n' "${default_value}"
}

normalize_face_mode_unit() {
  local unit="$1"
  case "${unit}" in
    m|M)
      printf 'm\n'
      ;;
    cm|CM)
      printf 'cm\n'
      ;;
    *)
      echo "[ERROR] FaceMode official-map input unit must be m or cm, got '${unit}'." >&2
      exit 2
      ;;
  esac
}

to_centimeter_value() {
  local unit="$1"
  local value="$2"
  python3 - "${unit}" "${value}" <<'PY'
import math
import sys

unit, value_text = sys.argv[1:3]
try:
    value = float(value_text)
except ValueError as exc:
    print(f"[ERROR] invalid FaceMode coordinate '{value_text}': {exc}", file=sys.stderr)
    raise SystemExit(2)

if not math.isfinite(value):
    print(f"[ERROR] invalid FaceMode coordinate '{value_text}': not finite", file=sys.stderr)
    raise SystemExit(2)

if unit == "m":
    value *= 100.0
elif unit != "cm":
    print(f"[ERROR] invalid FaceMode unit '{unit}'", file=sys.stderr)
    raise SystemExit(2)

print(f"{value:.9f}".rstrip("0").rstrip("."))
PY
}

normalize_point_launch_args_to_cm() {
  local -a normalized=()
  local arg key value
  for arg in "${LAUNCH_ARGS[@]}"; do
    case "${arg}" in
      official_map_x:=*|official_map_y:=*|map_z:=*)
        key="${arg%%:=*}"
        value="${arg#*:=}"
        normalized+=("${key}:=$(to_centimeter_value "${OFFICIAL_MAP_UNIT}" "${value}")")
        ;;
      *)
        normalized+=("${arg}")
        ;;
    esac
  done
  LAUNCH_ARGS=("${normalized[@]}")
}

require_face_mode_point_args() {
  local -a missing=()
  if ! has_launch_arg_key "official_map_x" && [[ -z "${OFFICIAL_MAP_X}" ]]; then
    missing+=("official_map_x/OFFICIAL_MAP_X")
  fi
  if ! has_launch_arg_key "official_map_y" && [[ -z "${OFFICIAL_MAP_Y}" ]]; then
    missing+=("official_map_y/OFFICIAL_MAP_Y")
  fi
  if ! has_launch_arg_key "map_z" && [[ -z "${MAP_Z}" ]]; then
    missing+=("map_z/MAP_Z")
  fi
  if (( ${#missing[@]} > 0 )); then
    echo "[ERROR] FaceMode requires point parameters: ${missing[*]} (unit=${OFFICIAL_MAP_UNIT}, use --unit m|cm or OFFICIAL_MAP_UNIT=m|cm)" >&2
    echo "        Example: OFFICIAL_MAP_X=1093 OFFICIAL_MAP_Y=366 MAP_Z=100 ./${SCRIPT_NAME}" >&2
    echo "        Example: OFFICIAL_MAP_UNIT=m OFFICIAL_MAP_X=10.93 OFFICIAL_MAP_Y=3.66 MAP_Z=1.0 ./${SCRIPT_NAME}" >&2
    echo "        Or: ./${SCRIPT_NAME} --unit cm -- official_map_x:=1093 official_map_y:=366 map_z:=100" >&2
    exit 2
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --unit|--input-unit|--official-unit)
      if (( $# < 2 )); then
        echo "[ERROR] $1 requires m or cm." >&2
        exit 2
      fi
      OFFICIAL_MAP_UNIT="${2:-}"
      shift 2
      ;;
    --bench|--mock-all)
      USE_GIMBAL="false"
      USE_MOCK_MAP_TO_BASE="true"
      USE_MOCK_GIMBAL_STATE="true"
      shift
      ;;
    --no-gimbal)
      USE_GIMBAL="false"
      shift
      ;;
    --mock-map-origin|--mock-map-to-base)
      USE_MOCK_MAP_TO_BASE="true"
      shift
      ;;
    --mock-gimbal-state)
      USE_MOCK_GIMBAL_STATE="true"
      USE_GIMBAL="false"
      shift
      ;;
    --no-firecode)
      PUBLISH_FIRECODE="false"
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      LAUNCH_ARGS=("$@")
      break
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

OFFICIAL_MAP_UNIT="$(normalize_face_mode_unit "${OFFICIAL_MAP_UNIT}")"
require_face_mode_point_args
normalize_point_launch_args_to_cm
source_ros_workspace "${ROOT_DIR}"
cleanup_existing_launch_tree \
  "1" \
  "ros2 launch navi_tf_bridge map_aim_point\\.launch\\.py"

CLEANUP_NODE_REGEX="/(map_aim_point_node)([[:space:]]|$)"
if [[ "${USE_GIMBAL}" == "true" || "${USE_MOCK_GIMBAL_STATE}" == "true" ]]; then
  CLEANUP_NODE_REGEX="${CLEANUP_NODE_REGEX}|/(gimbal_driver_node)([[:space:]]|$)"
fi
if [[ "${USE_MOCK_MAP_TO_BASE}" == "true" ]]; then
  CLEANUP_NODE_REGEX="${CLEANUP_NODE_REGEX}|static_transform_publisher .* map base_link([[:space:]]|$)"
fi
if [[ "${USE_MOCK_GIMBAL_STATE}" == "true" ]]; then
  CLEANUP_NODE_REGEX="${CLEANUP_NODE_REGEX}|/(mock_gimbal_state_node)([[:space:]]|$)"
fi
cleanup_existing_stack \
  "1" \
  "${CLEANUP_NODE_REGEX}" \
  "ros2 launch navi_tf_bridge map_aim_point\\.launch\\.py"

if ! has_launch_arg_key "official_map_x"; then
  LAUNCH_ARGS=("official_map_x:=$(to_centimeter_value "${OFFICIAL_MAP_UNIT}" "${OFFICIAL_MAP_X}")" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "official_map_y"; then
  LAUNCH_ARGS=("official_map_y:=$(to_centimeter_value "${OFFICIAL_MAP_UNIT}" "${OFFICIAL_MAP_Y}")" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "map_z"; then
  LAUNCH_ARGS=("map_z:=$(to_centimeter_value "${OFFICIAL_MAP_UNIT}" "${MAP_Z}")" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "target_frame"; then
  LAUNCH_ARGS=("target_frame:=${TARGET_FRAME}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "use_raw_goal_static_calibration"; then
  LAUNCH_ARGS=("use_raw_goal_static_calibration:=${USE_RAW_GOAL_STATIC_CALIBRATION}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "raw_goal_target_frame"; then
  LAUNCH_ARGS=("raw_goal_target_frame:=${RAW_GOAL_TARGET_FRAME}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "aim_frame"; then
  LAUNCH_ARGS=("aim_frame:=${AIM_FRAME}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "camera_frame"; then
  LAUNCH_ARGS=("camera_frame:=${CAMERA_FRAME}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "camera_fallback_frame"; then
  LAUNCH_ARGS=("camera_fallback_frame:=${CAMERA_FALLBACK_FRAME}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "solve_mode"; then
  LAUNCH_ARGS=("solve_mode:=${SOLVE_MODE}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "solve_frame"; then
  LAUNCH_ARGS=("solve_frame:=${SOLVE_FRAME}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "use_gimbal"; then
  LAUNCH_ARGS=("use_gimbal:=${USE_GIMBAL}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "use_virtual_device"; then
  LAUNCH_ARGS=("use_virtual_device:=${USE_VIRTUAL_DEVICE}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "use_mock_map_to_base"; then
  LAUNCH_ARGS=("use_mock_map_to_base:=${USE_MOCK_MAP_TO_BASE}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "use_mock_gimbal_state"; then
  LAUNCH_ARGS=("use_mock_gimbal_state:=${USE_MOCK_GIMBAL_STATE}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "mock_map_to_base_x"; then
  LAUNCH_ARGS=("mock_map_to_base_x:=${MOCK_MAP_TO_BASE_X}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "mock_map_to_base_y"; then
  LAUNCH_ARGS=("mock_map_to_base_y:=${MOCK_MAP_TO_BASE_Y}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "mock_map_to_base_z"; then
  LAUNCH_ARGS=("mock_map_to_base_z:=${MOCK_MAP_TO_BASE_Z}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "mock_map_to_base_yaw"; then
  LAUNCH_ARGS=("mock_map_to_base_yaw:=${MOCK_MAP_TO_BASE_YAW}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "mock_map_to_base_pitch"; then
  LAUNCH_ARGS=("mock_map_to_base_pitch:=${MOCK_MAP_TO_BASE_PITCH}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "mock_map_to_base_roll"; then
  LAUNCH_ARGS=("mock_map_to_base_roll:=${MOCK_MAP_TO_BASE_ROLL}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "mock_gimbal_yaw_deg"; then
  LAUNCH_ARGS=("mock_gimbal_yaw_deg:=${MOCK_GIMBAL_YAW_DEG}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "mock_gimbal_pitch_deg"; then
  LAUNCH_ARGS=("mock_gimbal_pitch_deg:=${MOCK_GIMBAL_PITCH_DEG}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "mock_gimbal_big_yaw_deg"; then
  LAUNCH_ARGS=("mock_gimbal_big_yaw_deg:=${MOCK_GIMBAL_BIG_YAW_DEG}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "mock_gimbal_publish_big_yaw"; then
  LAUNCH_ARGS=("mock_gimbal_publish_big_yaw:=${MOCK_GIMBAL_PUBLISH_BIG_YAW}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "publish_firecode"; then
  LAUNCH_ARGS=("publish_firecode:=${PUBLISH_FIRECODE}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "aim_mode"; then
  LAUNCH_ARGS=("aim_mode:=${AIM_MODE}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "yaw_sign"; then
  LAUNCH_ARGS=("yaw_sign:=${YAW_SIGN}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "pitch_sign"; then
  LAUNCH_ARGS=("pitch_sign:=${PITCH_SIGN}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "yaw_bias_deg"; then
  LAUNCH_ARGS=("yaw_bias_deg:=${YAW_BIAS_DEG}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "pitch_bias_deg"; then
  LAUNCH_ARGS=("pitch_bias_deg:=${PITCH_BIAS_DEG}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "publish_hz"; then
  LAUNCH_ARGS=("publish_hz:=${PUBLISH_HZ}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "use_gimbal_stamp_for_tf"; then
  LAUNCH_ARGS=("use_gimbal_stamp_for_tf:=${USE_GIMBAL_STAMP_FOR_TF}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "max_gimbal_stamp_age_sec"; then
  LAUNCH_ARGS=("max_gimbal_stamp_age_sec:=${MAX_GIMBAL_STAMP_AGE_SEC}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "max_target_distance_m"; then
  LAUNCH_ARGS=("max_target_distance_m:=${MAX_TARGET_DISTANCE_M}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "command_filter_alpha"; then
  LAUNCH_ARGS=("command_filter_alpha:=${COMMAND_FILTER_ALPHA}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "max_yaw_step_deg"; then
  LAUNCH_ARGS=("max_yaw_step_deg:=${MAX_YAW_STEP_DEG}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "max_pitch_step_deg"; then
  LAUNCH_ARGS=("max_pitch_step_deg:=${MAX_PITCH_STEP_DEG}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "output"; then
  LAUNCH_ARGS=("output:=${OUTPUT_MODE}" "${LAUNCH_ARGS[@]}")
fi

DEFAULT_BRIDGE_CONFIG_FILE="${ROOT_DIR}/install/navi_tf_bridge/share/navi_tf_bridge/config/tf_config.yaml"
if [[ ! -f "${DEFAULT_BRIDGE_CONFIG_FILE}" ]]; then
  DEFAULT_BRIDGE_CONFIG_FILE="${ROOT_DIR}/src/navi_tf_bridge/config/tf_config.yaml"
fi
PREVIEW_BRIDGE_CONFIG_FILE="$(launch_arg_value "bridge_config_file" "${DEFAULT_BRIDGE_CONFIG_FILE}")"
PREVIEW_OFFICIAL_MAP_X="$(launch_arg_value "official_map_x" "${OFFICIAL_MAP_X}")"
PREVIEW_OFFICIAL_MAP_Y="$(launch_arg_value "official_map_y" "${OFFICIAL_MAP_Y}")"
PREVIEW_MAP_Z="$(launch_arg_value "map_z" "${MAP_Z}")"
PREVIEW_USE_RAW_GOAL_STATIC_CALIBRATION="$(launch_arg_value "use_raw_goal_static_calibration" "${USE_RAW_GOAL_STATIC_CALIBRATION}")"
PREVIEW_RAW_GOAL_TARGET_FRAME="$(launch_arg_value "raw_goal_target_frame" "${RAW_GOAL_TARGET_FRAME}")"
echo "[INFO] FaceMode point input unit=${OFFICIAL_MAP_UNIT}; node target params are cm: official_map=(${PREVIEW_OFFICIAL_MAP_X}, ${PREVIEW_OFFICIAL_MAP_Y}), map_z=${PREVIEW_MAP_Z}" >&2
print_raw_goal_map_preview \
  "${PREVIEW_BRIDGE_CONFIG_FILE}" \
  "${PREVIEW_OFFICIAL_MAP_X}" \
  "${PREVIEW_OFFICIAL_MAP_Y}" \
  "${PREVIEW_MAP_Z}" \
  "${PREVIEW_USE_RAW_GOAL_STATIC_CALIBRATION}" \
  "${PREVIEW_RAW_GOAL_TARGET_FRAME}"

exec ros2 launch navi_tf_bridge map_aim_point.launch.py "${LAUNCH_ARGS[@]}"
