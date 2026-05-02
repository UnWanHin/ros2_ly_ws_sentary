#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# Map-point gimbal aim test. Edit TARGET_* below for the point to face.
# Real map aiming needs navigation/localization to publish map -> base_link.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

# 坐标写这里：单位 cm。
# X/Y 是官方二维地图坐标，会按 navi_tf_bridge/config/tf_config.yaml 的 4x4
# 走和 /ly/navi/goal_pos_raw -> /goal_pose 一样的平面转换，默认把结果当 map 点使用。
# Z 已经是 map 系高度，只作为瞄准高度，不参与官方二维地图 X/Y 转换。
TARGET_X_CM="${TARGET_X_CM:-1093}"
TARGET_Y_CM="${TARGET_Y_CM:-366}"
TARGET_Z_CM="${TARGET_Z_CM:-100}"
TARGET_FRAME="${TARGET_FRAME:-official_map}"
USE_RAW_GOAL_STATIC_CALIBRATION="${USE_RAW_GOAL_STATIC_CALIBRATION:-true}"
RAW_GOAL_TARGET_FRAME="${RAW_GOAL_TARGET_FRAME:-map}"

# 默认按 gx_camera 投影误差解 yaw/pitch；solve_frame 只用于日志和 base_link 绝对角 fallback。
AIM_FRAME="${AIM_FRAME:-gimbal_world}"
CAMERA_FRAME="${CAMERA_FRAME:-gx_camera}"
SOLVE_MODE="${SOLVE_MODE:-camera_projection}"
SOLVE_FRAME="${SOLVE_FRAME:-base_link}"
USE_GIMBAL="${USE_GIMBAL:-true}"
USE_TF_TREE="${USE_TF_TREE:-false}"
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
YAW_SIGN="${YAW_SIGN:-1.0}"
PITCH_SIGN="${PITCH_SIGN:-1.0}"
YAW_BIAS_DEG="${YAW_BIAS_DEG:-0.0}"
PITCH_BIAS_DEG="${PITCH_BIAS_DEG:-0.0}"
PUBLISH_HZ="${PUBLISH_HZ:-30.0}"
USE_GIMBAL_STAMP_FOR_TF="${USE_GIMBAL_STAMP_FOR_TF:-false}"
MAX_GIMBAL_STAMP_AGE_SEC="${MAX_GIMBAL_STAMP_AGE_SEC:-0.50}"
MAX_TARGET_DISTANCE_M="${MAX_TARGET_DISTANCE_M:-100.0}"
COMMAND_FILTER_ALPHA="${COMMAND_FILTER_ALPHA:-1.0}"
MAX_YAW_STEP_DEG="${MAX_YAW_STEP_DEG:-0.0}"
MAX_PITCH_STEP_DEG="${MAX_PITCH_STEP_DEG:-0.0}"
OUTPUT_MODE="${OUTPUT_MODE:-screen}"
LAUNCH_ARGS=()

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--bench] [--no-gimbal] [--with-tf-tree] [--mock-map-origin] [--mock-gimbal-state] [--no-firecode] [-- <launch_args...>]

Purpose:
  Keep gimbal facing one fixed map/official-map point.
  Publishes /ly/control/angles and, by default, /ly/control/firecode aim_mode=true.
  It does not publish chassis velocity or fire commands.
  A real map run requires an external navigation/localization TF chain that provides map -> base_link.
  For a bench-only map origin pose, use --mock-map-origin or use_mock_map_to_base:=true.
  For a no-hardware closed-loop smoke test, use --bench.

Edit these variables near the top of this script:
  TARGET_X_CM=${TARGET_X_CM}
  TARGET_Y_CM=${TARGET_Y_CM}
  TARGET_Z_CM=${TARGET_Z_CM}
  TARGET_FRAME=${TARGET_FRAME}
  USE_RAW_GOAL_STATIC_CALIBRATION=${USE_RAW_GOAL_STATIC_CALIBRATION}
  RAW_GOAL_TARGET_FRAME=${RAW_GOAL_TARGET_FRAME}
  CAMERA_FRAME=${CAMERA_FRAME}
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
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --bench
  ./${SCRIPT_NAME} --mock-map-origin --with-tf-tree
  TARGET_X_CM=1400 TARGET_Y_CM=750 TARGET_Z_CM=100 ./${SCRIPT_NAME}
  USE_MOCK_MAP_TO_BASE=true MOCK_MAP_TO_BASE_X=1.2 MOCK_MAP_TO_BASE_Y=-0.4 ./${SCRIPT_NAME}
  COMMAND_FILTER_ALPHA=0.25 MAX_YAW_STEP_DEG=3.0 MAX_PITCH_STEP_DEG=1.5 ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} -- raw_goal_target_frame:=map yaw_sign:=-1.0 pitch_bias_deg:=2.0
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

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bench|--mock-all)
      USE_GIMBAL="false"
      USE_TF_TREE="true"
      USE_MOCK_MAP_TO_BASE="true"
      USE_MOCK_GIMBAL_STATE="true"
      shift
      ;;
    --no-gimbal)
      USE_GIMBAL="false"
      shift
      ;;
    --no-tf-tree)
      USE_TF_TREE="false"
      shift
      ;;
    --with-tf-tree)
      USE_TF_TREE="true"
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

source_ros_workspace "${ROOT_DIR}"
cleanup_existing_launch_tree \
  "1" \
  "ros2 launch navi_tf_bridge map_aim_point\\.launch\\.py"

CLEANUP_NODE_REGEX="/(map_aim_point_node)([[:space:]]|$)"
if [[ "${USE_GIMBAL}" == "true" || "${USE_MOCK_GIMBAL_STATE}" == "true" ]]; then
  CLEANUP_NODE_REGEX="${CLEANUP_NODE_REGEX}|/(gimbal_driver_node)([[:space:]]|$)"
fi
if [[ "${USE_TF_TREE}" == "true" ]]; then
  CLEANUP_NODE_REGEX="${CLEANUP_NODE_REGEX}|/(tf_tree/tf_node|sentry_tf/tf_node)([[:space:]]|$)|static_transform_publisher .* (base_link gimbal_big_yaw|gimbal_barrel gx_camera|gimbal_big_yaw usb_camera)([[:space:]]|$)"
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

if ! has_launch_arg_key "target_x_cm"; then
  LAUNCH_ARGS=("target_x_cm:=${TARGET_X_CM}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "target_y_cm"; then
  LAUNCH_ARGS=("target_y_cm:=${TARGET_Y_CM}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "target_z_cm"; then
  LAUNCH_ARGS=("target_z_cm:=${TARGET_Z_CM}" "${LAUNCH_ARGS[@]}")
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
if ! has_launch_arg_key "solve_mode"; then
  LAUNCH_ARGS=("solve_mode:=${SOLVE_MODE}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "solve_frame"; then
  LAUNCH_ARGS=("solve_frame:=${SOLVE_FRAME}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "use_gimbal"; then
  LAUNCH_ARGS=("use_gimbal:=${USE_GIMBAL}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "use_tf_tree"; then
  LAUNCH_ARGS=("use_tf_tree:=${USE_TF_TREE}" "${LAUNCH_ARGS[@]}")
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
PREVIEW_TARGET_X_CM="$(launch_arg_value "target_x_cm" "${TARGET_X_CM}")"
PREVIEW_TARGET_Y_CM="$(launch_arg_value "target_y_cm" "${TARGET_Y_CM}")"
PREVIEW_TARGET_Z_CM="$(launch_arg_value "target_z_cm" "${TARGET_Z_CM}")"
PREVIEW_USE_RAW_GOAL_STATIC_CALIBRATION="$(launch_arg_value "use_raw_goal_static_calibration" "${USE_RAW_GOAL_STATIC_CALIBRATION}")"
PREVIEW_RAW_GOAL_TARGET_FRAME="$(launch_arg_value "raw_goal_target_frame" "${RAW_GOAL_TARGET_FRAME}")"
print_raw_goal_map_preview \
  "${PREVIEW_BRIDGE_CONFIG_FILE}" \
  "${PREVIEW_TARGET_X_CM}" \
  "${PREVIEW_TARGET_Y_CM}" \
  "${PREVIEW_TARGET_Z_CM}" \
  "${PREVIEW_USE_RAW_GOAL_STATIC_CALIBRATION}" \
  "${PREVIEW_RAW_GOAL_TARGET_FRAME}"

exec ros2 launch navi_tf_bridge map_aim_point.launch.py "${LAUNCH_ARGS[@]}"
