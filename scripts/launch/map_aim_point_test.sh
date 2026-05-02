#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# Standalone map-point gimbal aim test. Edit TARGET_* below for the point to face.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

# 坐标写这里：单位 cm。
# X/Y 是官方二维地图坐标，会按 navi_tf_bridge/config/tf_config.yaml 的 4x4
# 走和 /ly/navi/goal_pos_raw -> /goal_pose 一样的平面转换到 map。
# Z 直接当 map 系瞄准高度，不参与官方二维地图转换。
TARGET_X_CM="${TARGET_X_CM:-1093}"
TARGET_Y_CM="${TARGET_Y_CM:-366}"
TARGET_Z_CM="${TARGET_Z_CM:-100}"
TARGET_FRAME="${TARGET_FRAME:-official_map}"
USE_RAW_GOAL_STATIC_CALIBRATION="${USE_RAW_GOAL_STATIC_CALIBRATION:-true}"

# gimbal_barrel_joint 是“当前 yaw 后、pitch 前”的云台坐标系；节点会算 yaw 误差和绝对 pitch。
AIM_FRAME="${AIM_FRAME:-gimbal_barrel_joint}"
USE_GIMBAL="${USE_GIMBAL:-true}"
USE_TF_TREE="${USE_TF_TREE:-true}"
USE_VIRTUAL_DEVICE="${USE_VIRTUAL_DEVICE:-false}"
YAW_SIGN="${YAW_SIGN:-1.0}"
PITCH_SIGN="${PITCH_SIGN:-1.0}"
YAW_BIAS_DEG="${YAW_BIAS_DEG:-0.0}"
PITCH_BIAS_DEG="${PITCH_BIAS_DEG:-0.0}"
PUBLISH_HZ="${PUBLISH_HZ:-30.0}"
OUTPUT_MODE="${OUTPUT_MODE:-screen}"
LAUNCH_ARGS=()

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--no-gimbal] [--no-tf-tree] [-- <launch_args...>]

Purpose:
  Keep gimbal facing one fixed map/official-map point.
  Only publishes /ly/control/angles; it does not publish chassis velocity or firecode.

Edit these variables near the top of this script:
  TARGET_X_CM=${TARGET_X_CM}
  TARGET_Y_CM=${TARGET_Y_CM}
  TARGET_Z_CM=${TARGET_Z_CM}
  TARGET_FRAME=${TARGET_FRAME}
  USE_RAW_GOAL_STATIC_CALIBRATION=${USE_RAW_GOAL_STATIC_CALIBRATION}

Examples:
  ./${SCRIPT_NAME}
  TARGET_X_CM=1400 TARGET_Y_CM=750 TARGET_Z_CM=100 ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} -- use_raw_goal_static_calibration:=false target_frame:=map yaw_sign:=-1.0 pitch_bias_deg:=2.0
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

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-gimbal)
      USE_GIMBAL="false"
      shift
      ;;
    --no-tf-tree)
      USE_TF_TREE="false"
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
cleanup_existing_stack \
  "1" \
  "/(map_aim_point_node|gimbal_driver_node|tf_node|static_transform_publisher)([[:space:]]|$)" \
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
if ! has_launch_arg_key "aim_frame"; then
  LAUNCH_ARGS=("aim_frame:=${AIM_FRAME}" "${LAUNCH_ARGS[@]}")
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
if ! has_launch_arg_key "output"; then
  LAUNCH_ARGS=("output:=${OUTPUT_MODE}" "${LAUNCH_ARGS[@]}")
fi

exec ros2 launch navi_tf_bridge map_aim_point.launch.py "${LAUNCH_ARGS[@]}"
