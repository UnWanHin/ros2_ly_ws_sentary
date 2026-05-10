#!/usr/bin/env bash

# Attach to an already-running sentry stack and publish one fixed map aim command.
# This script does not launch or clean up gimbal, TF, map, camera, or navigation nodes.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

# FaceMode point, unit: cm. X/Y can be converted by tf_config.yaml.
OFFICIAL_MAP_X="${OFFICIAL_MAP_X:-}"
OFFICIAL_MAP_Y="${OFFICIAL_MAP_Y:-}"
MAP_Z="${MAP_Z:-}"
TARGET_FRAME="${TARGET_FRAME:-official_map}"
USE_RAW_GOAL_STATIC_CALIBRATION="${USE_RAW_GOAL_STATIC_CALIBRATION:-true}"
RAW_GOAL_TARGET_FRAME="${RAW_GOAL_TARGET_FRAME:-map}"
BRIDGE_CONFIG_FILE="${BRIDGE_CONFIG_FILE:-${ROOT_DIR}/src/navi_tf_bridge/config/tf_config.yaml}"

# Use the frames/topics already published by the running stack.
AIM_FRAME="${AIM_FRAME:-gimbal_world}"
CAMERA_FRAME="${CAMERA_FRAME:-gx_camera}"
SOLVE_MODE="${SOLVE_MODE:-camera_projection}"
SOLVE_FRAME="${SOLVE_FRAME:-base_link}"
GIMBAL_ANGLES_TOPIC="${GIMBAL_ANGLES_TOPIC:-/ly/gimbal/angles}"
CONTROL_ANGLES_TOPIC="${CONTROL_ANGLES_TOPIC:-/ly/control/angles}"
CONTROL_FIRECODE_TOPIC="${CONTROL_FIRECODE_TOPIC:-/ly/control/firecode}"
PUBLISH_FIRECODE="${PUBLISH_FIRECODE:-true}"
AIM_MODE="${AIM_MODE:-true}"

PUBLISH_HZ="${PUBLISH_HZ:-30.0}"
TF_TIMEOUT_SEC="${TF_TIMEOUT_SEC:-0.05}"
USE_GIMBAL_STAMP_FOR_TF="${USE_GIMBAL_STAMP_FOR_TF:-false}"
MAX_GIMBAL_STAMP_AGE_SEC="${MAX_GIMBAL_STAMP_AGE_SEC:-0.50}"
MAX_TARGET_DISTANCE_M="${MAX_TARGET_DISTANCE_M:-100.0}"
COMMAND_FILTER_ALPHA="${COMMAND_FILTER_ALPHA:-1.0}"
MAX_YAW_STEP_DEG="${MAX_YAW_STEP_DEG:-}"
MAX_PITCH_STEP_DEG="${MAX_PITCH_STEP_DEG:-}"
YAW_SIGN="${YAW_SIGN:--1.0}"
PITCH_SIGN="${PITCH_SIGN:-1.0}"
YAW_BIAS_DEG="${YAW_BIAS_DEG:-0.0}"
PITCH_BIAS_DEG="${PITCH_BIAS_DEG:-0.0}"

ECHO_AFTER_START="false"
ALLOW_DUPLICATE="false"
EXTRA_PARAMS=()

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
  ${SCRIPT_NAME} [--echo] [--smooth] [--strict-time] [--allow-duplicate] [-- <param:=value>...]

Purpose:
  Attach to an already-running stack and start only FaceMode/map_aim_point_node.
  This does not launch or clean up gimbal_driver, tf_tree, map_server, camera, or localization.

Common:
  OFFICIAL_MAP_X=1093 OFFICIAL_MAP_Y=366 MAP_Z=100 ./${SCRIPT_NAME}
  OFFICIAL_MAP_X=1093 OFFICIAL_MAP_Y=366 MAP_Z=100 ./${SCRIPT_NAME} --echo
  OFFICIAL_MAP_X=1093 OFFICIAL_MAP_Y=366 MAP_Z=100 ./${SCRIPT_NAME} --smooth --echo
  OFFICIAL_MAP_X=1093 OFFICIAL_MAP_Y=366 MAP_Z=100 ./${SCRIPT_NAME} --strict-time --echo
  OFFICIAL_MAP_X=1093 OFFICIAL_MAP_Y=366 MAP_Z=100 ./${SCRIPT_NAME} --allow-duplicate --echo
  OFFICIAL_MAP_X=1093 OFFICIAL_MAP_Y=366 MAP_Z=100 ./${SCRIPT_NAME} -- command_filter_alpha:=0.2 max_yaw_step_deg:=2.0 max_pitch_step_deg:=1.0
  ./${SCRIPT_NAME} -- official_map_x:=1093 official_map_y:=366 map_z:=100

Current defaults:
  FACE_POINT=(${OFFICIAL_MAP_X}, ${OFFICIAL_MAP_Y}, ${MAP_Z})cm@${TARGET_FRAME}
  RAW_GOAL_TARGET_FRAME=${RAW_GOAL_TARGET_FRAME}
  AIM_FRAME=${AIM_FRAME}
  CAMERA_FRAME=${CAMERA_FRAME}
  SOLVE_MODE=${SOLVE_MODE}
  SOLVE_FRAME=${SOLVE_FRAME}
  GIMBAL_ANGLES_TOPIC=${GIMBAL_ANGLES_TOPIC}
  CONTROL_ANGLES_TOPIC=${CONTROL_ANGLES_TOPIC}
  USE_GIMBAL_STAMP_FOR_TF=${USE_GIMBAL_STAMP_FOR_TF}
  MAX_TARGET_DISTANCE_M=${MAX_TARGET_DISTANCE_M}
  COMMAND_FILTER_ALPHA=${COMMAND_FILTER_ALPHA}
  MAX_YAW_STEP_DEG=${MAX_YAW_STEP_DEG}
  MAX_PITCH_STEP_DEG=${MAX_PITCH_STEP_DEG}

Notes:
  Default output is raw camera-projection correction using latest common TF time.
  In SOLVE_MODE=camera_projection, the fixed map point is transformed into CAMERA_FRAME and x/y projection error is driven to 0.
  --strict-time uses /ly/gimbal/angles stamps and requires the full map->gimbal TF chain to be time-synchronized.
  --smooth only limits command jumps for debugging jitter.
EOF
}

as_double() {
  local value="$1"
  if [[ "${value}" =~ ^[-+]?[0-9]+$ ]]; then
    printf '%s.0\n' "${value}"
  else
    printf '%s\n' "${value}"
  fi
}

normalize_override() {
  local param="$1"
  local key="${param%%:=*}"
  local value="${param#*:=}"

  case "${key}" in
    official_map_x|official_map_y|map_z|publish_hz|tf_timeout_sec|\
    max_gimbal_stamp_age_sec|max_target_distance_m|command_filter_alpha|max_yaw_step_deg|\
    max_pitch_step_deg|yaw_sign|pitch_sign|yaw_bias_deg|pitch_bias_deg)
      printf '%s:=%s\n' "${key}" "$(as_double "${value}")"
      ;;
    *)
      printf '%s\n' "${param}"
      ;;
  esac
}

is_face_mode_point_param() {
  local key="$1"
  [[ "${key}" == "official_map_x" || "${key}" == "official_map_y" || "${key}" == "map_z" ]]
}

override_value() {
  local key="$1"
  local default_value="$2"
  local param
  for param in "${EXTRA_PARAMS[@]}"; do
    if [[ "${param}" == "${key}:="* ]]; then
      printf '%s\n' "${param#*:=}"
      return 0
    fi
  done
  printf '%s\n' "${default_value}"
}

require_face_mode_point_args() {
  local x="$1"
  local y="$2"
  local z="$3"
  local -a missing=()
  [[ -n "${x}" ]] || missing+=("official_map_x/OFFICIAL_MAP_X")
  [[ -n "${y}" ]] || missing+=("official_map_y/OFFICIAL_MAP_Y")
  [[ -n "${z}" ]] || missing+=("map_z/MAP_Z")
  if (( ${#missing[@]} > 0 )); then
    echo "[ERROR] FaceMode requires point parameters in cm: ${missing[*]}" >&2
    echo "        Example: OFFICIAL_MAP_X=1093 OFFICIAL_MAP_Y=366 MAP_Z=100 ./${SCRIPT_NAME} --echo" >&2
    echo "        Or: ./${SCRIPT_NAME} -- official_map_x:=1093 official_map_y:=366 map_z:=100" >&2
    exit 2
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --echo)
      ECHO_AFTER_START="true"
      shift
      ;;
    --smooth)
      COMMAND_FILTER_ALPHA="0.25"
      MAX_YAW_STEP_DEG="3.0"
      MAX_PITCH_STEP_DEG="1.5"
      shift
      ;;
    --strict-time)
      USE_GIMBAL_STAMP_FOR_TF="true"
      shift
      ;;
    --allow-duplicate|--force)
      ALLOW_DUPLICATE="true"
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      EXTRA_PARAMS=("$@")
      break
      ;;
    *)
      EXTRA_PARAMS+=("$1")
      shift
      ;;
  esac
done

PREVIEW_BRIDGE_CONFIG_FILE="$(override_value "bridge_config_file" "${BRIDGE_CONFIG_FILE}")"
PREVIEW_OFFICIAL_MAP_X="$(override_value "official_map_x" "${OFFICIAL_MAP_X}")"
PREVIEW_OFFICIAL_MAP_Y="$(override_value "official_map_y" "${OFFICIAL_MAP_Y}")"
PREVIEW_MAP_Z="$(override_value "map_z" "${MAP_Z}")"
PREVIEW_USE_RAW_GOAL_STATIC_CALIBRATION="$(override_value "use_raw_goal_static_calibration" "${USE_RAW_GOAL_STATIC_CALIBRATION}")"
PREVIEW_RAW_GOAL_TARGET_FRAME="$(override_value "raw_goal_target_frame" "${RAW_GOAL_TARGET_FRAME}")"
require_face_mode_point_args "${PREVIEW_OFFICIAL_MAP_X}" "${PREVIEW_OFFICIAL_MAP_Y}" "${PREVIEW_MAP_Z}"

source_ros_workspace "${ROOT_DIR}"

if [[ "${ALLOW_DUPLICATE}" != "true" ]]; then
  if ros2 node list 2>/dev/null | awk '$0 == "/map_aim_point_node" { found = 1 } END { exit found ? 0 : 1 }'; then
    echo "[ERROR] /map_aim_point_node is already running; refusing to start a duplicate publisher." >&2
    echo "        Use 'ros2 topic echo ${CONTROL_ANGLES_TOPIC} gimbal_driver/msg/GimbalAngles' to watch it." >&2
    echo "        If you really want another one, pass --allow-duplicate." >&2
    exit 1
  fi
fi

print_raw_goal_map_preview \
  "${PREVIEW_BRIDGE_CONFIG_FILE}" \
  "${PREVIEW_OFFICIAL_MAP_X}" \
  "${PREVIEW_OFFICIAL_MAP_Y}" \
  "${PREVIEW_MAP_Z}" \
  "${PREVIEW_USE_RAW_GOAL_STATIC_CALIBRATION}" \
  "${PREVIEW_RAW_GOAL_TARGET_FRAME}"

ROS_ARGS=(
  --ros-args
  -p "official_map_x:=$(as_double "${PREVIEW_OFFICIAL_MAP_X}")"
  -p "official_map_y:=$(as_double "${PREVIEW_OFFICIAL_MAP_Y}")"
  -p "map_z:=$(as_double "${PREVIEW_MAP_Z}")"
  -p "target_frame:=${TARGET_FRAME}"
  -p "aim_frame:=${AIM_FRAME}"
  -p "camera_frame:=${CAMERA_FRAME}"
  -p "solve_mode:=${SOLVE_MODE}"
  -p "solve_frame:=${SOLVE_FRAME}"
  -p "gimbal_angles_topic:=${GIMBAL_ANGLES_TOPIC}"
  -p "control_angles_topic:=${CONTROL_ANGLES_TOPIC}"
  -p "control_firecode_topic:=${CONTROL_FIRECODE_TOPIC}"
  -p "publish_firecode:=${PUBLISH_FIRECODE}"
  -p "aim_mode:=${AIM_MODE}"
  -p "bridge_config_file:=${BRIDGE_CONFIG_FILE}"
  -p "use_raw_goal_static_calibration:=${USE_RAW_GOAL_STATIC_CALIBRATION}"
  -p "raw_goal_target_frame:=${RAW_GOAL_TARGET_FRAME}"
  -p "publish_hz:=$(as_double "${PUBLISH_HZ}")"
  -p "tf_timeout_sec:=$(as_double "${TF_TIMEOUT_SEC}")"
  -p "use_gimbal_stamp_for_tf:=${USE_GIMBAL_STAMP_FOR_TF}"
  -p "max_gimbal_stamp_age_sec:=$(as_double "${MAX_GIMBAL_STAMP_AGE_SEC}")"
  -p "max_target_distance_m:=$(as_double "${MAX_TARGET_DISTANCE_M}")"
  -p "command_filter_alpha:=$(as_double "${COMMAND_FILTER_ALPHA}")"
  -p "max_yaw_step_deg:=$(as_double "${MAX_YAW_STEP_DEG}")"
  -p "max_pitch_step_deg:=$(as_double "${MAX_PITCH_STEP_DEG}")"
  -p "yaw_sign:=$(as_double "${YAW_SIGN}")"
  -p "pitch_sign:=$(as_double "${PITCH_SIGN}")"
  -p "yaw_bias_deg:=$(as_double "${YAW_BIAS_DEG}")"
  -p "pitch_bias_deg:=$(as_double "${PITCH_BIAS_DEG}")"
)

for param in "${EXTRA_PARAMS[@]}"; do
  if [[ "${param}" == "--ros-args" || "${param}" == "-p" ]]; then
    echo "[ERROR] Pass overrides as key:=value, not raw --ros-args/-p." >&2
    exit 2
  fi
  if [[ "${param}" != *":="* ]]; then
    echo "[ERROR] Invalid override '${param}'. Expected key:=value." >&2
    exit 2
  fi
  param_key="${param%%:=*}"
  if is_face_mode_point_param "${param_key}"; then
    continue
  fi
  ROS_ARGS+=(-p "$(normalize_override "${param}")")
done

if [[ "${ECHO_AFTER_START}" != "true" ]]; then
  exec ros2 run navi_tf_bridge map_aim_point_node "${ROS_ARGS[@]}"
fi

ros2 run navi_tf_bridge map_aim_point_node "${ROS_ARGS[@]}" &
node_pid="$!"

cleanup() {
  kill -INT "${node_pid}" 2>/dev/null || true
  wait "${node_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 1
if ! kill -0 "${node_pid}" 2>/dev/null; then
  wait "${node_pid}" 2>/dev/null || true
  echo "[ERROR] map_aim_point_node exited before echo could start." >&2
  exit 1
fi
echo "[INFO] Echoing ${CONTROL_ANGLES_TOPIC}. Press Ctrl-C to stop this script and its map_aim_point_node." >&2
ros2 topic echo "${CONTROL_ANGLES_TOPIC}" gimbal_driver/msg/GimbalAngles
