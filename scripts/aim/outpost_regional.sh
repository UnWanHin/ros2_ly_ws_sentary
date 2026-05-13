#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

USE_NOGATE=1
OFFLINE_MODE=0
FACE_MODE_SOLVER=1
MANUAL_SIGNED_OUTPOST_TEST=0
GOAL_MAP_X_M=""
GOAL_MAP_Y_M=""
FACE_MAP_X_M=""
FACE_MAP_Y_M=""
FACE_MAP_Z_M=""
POSITIONAL_MANUAL_M=()
LAUNCH_ARGS=()

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [GOAL_X_M GOAL_Y_M FACE_X_M FACE_Y_M FACE_Z_M]
                [--nogate|--with-gate] [--online|--offline] [--no-face-mode-solver]
                [-- <launch_args...>]

Purpose:
  Formal regional Outpost aim-mode test.
  Starts sentry_all through behavior_tree/outpost_regional_test.launch.py:
  - bypasses /ly/game/is_start by default
  - drives the normal regional Outpost task to BuffOutpost
  - selects /ly/aim/select_target=Outpost while Outpost aim/scout is active
  - uses /ly/aim/result for angles/fire when a target is present
  - returns to FaceMode after armor interruption releases
  - falls back to PatrolScan mode 3 when FaceMode has no /ly/face_mode/angles
  - positional args use signed map-frame meters:
      GOAL_X/Y -> /goal_pose, FACE_X/Y/Z -> FaceMode target

Examples:
  ./scripts/aim/${SCRIPT_NAME}
  ./scripts/aim/${SCRIPT_NAME} 6.17 -16.21 6.50 -18.00 1.00
  ./scripts/aim/${SCRIPT_NAME} --offline
  ./scripts/aim/${SCRIPT_NAME} --no-face-mode-solver
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

is_meter_value() {
  [[ "$1" =~ ^[-+]?[0-9]+([.][0-9]+)?$ || "$1" =~ ^[-+]?[.][0-9]+$ ]]
}

while [[ $# -gt 0 ]]; do
  case "$1" in
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
    --offline)
      OFFLINE_MODE=1
      shift
      ;;
    --no-face-mode-solver)
      FACE_MODE_SOLVER=0
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
      if [[ "$1" != *":="* ]] && is_meter_value "$1"; then
        if (( ${#POSITIONAL_MANUAL_M[@]} >= 5 )); then
          echo "Unexpected extra positional meter coordinate: $1. Pass launch args after --." >&2
          exit 2
        fi
        POSITIONAL_MANUAL_M+=("$1")
      else
        LAUNCH_ARGS+=("$1")
      fi
      shift
      ;;
  esac
done

if (( ${#POSITIONAL_MANUAL_M[@]} > 0 )); then
  if (( ${#POSITIONAL_MANUAL_M[@]} != 5 )); then
    echo "Manual outpost test requires exactly GOAL_X_M GOAL_Y_M FACE_X_M FACE_Y_M FACE_Z_M." >&2
    exit 2
  fi
  MANUAL_SIGNED_OUTPOST_TEST=1
  GOAL_MAP_X_M="${POSITIONAL_MANUAL_M[0]}"
  GOAL_MAP_Y_M="${POSITIONAL_MANUAL_M[1]}"
  FACE_MAP_X_M="${POSITIONAL_MANUAL_M[2]}"
  FACE_MAP_Y_M="${POSITIONAL_MANUAL_M[3]}"
  FACE_MAP_Z_M="${POSITIONAL_MANUAL_M[4]}"
fi

source_ros_workspace "${ROOT_DIR}"
require_sentry_msgs_for_behavior_tree
cleanup_existing_stack "1" "/(gimbal_driver_node|detector_node|tracker_solver_node|predictor_node|outpost_hitter_node|buff_hitter_node|behavior_tree_node|map_aim_point_node)([[:space:]]|$)" "ros2 launch (behavior_tree|detector|navi_tf_bridge) (competition_autoaim|sentry_all|outpost_regional_test|auto_aim|map_aim_point)\\.launch.py"

if (( USE_NOGATE == 1 )) && ! has_launch_arg_key "debug_bypass_is_start"; then
  LAUNCH_ARGS=("debug_bypass_is_start:=true" "${LAUNCH_ARGS[@]}")
fi
if (( USE_NOGATE == 0 )) && ! has_launch_arg_key "debug_bypass_is_start"; then
  LAUNCH_ARGS=("debug_bypass_is_start:=false" "${LAUNCH_ARGS[@]}")
fi
if (( OFFLINE_MODE == 1 )) && ! has_launch_arg_key "offline"; then
  LAUNCH_ARGS=("offline:=true" "${LAUNCH_ARGS[@]}")
fi
if (( FACE_MODE_SOLVER == 0 )) && ! has_launch_arg_key "use_face_mode_solver"; then
  LAUNCH_ARGS=("use_face_mode_solver:=false" "${LAUNCH_ARGS[@]}")
fi
if (( MANUAL_SIGNED_OUTPOST_TEST == 1 )); then
  if ! has_launch_arg_key "outpost_manual_goal_enable"; then
    LAUNCH_ARGS=("outpost_manual_goal_enable:=true" "${LAUNCH_ARGS[@]}")
  fi
  if ! has_launch_arg_key "outpost_manual_goal_x_m"; then
    LAUNCH_ARGS=("outpost_manual_goal_x_m:=${GOAL_MAP_X_M}" "${LAUNCH_ARGS[@]}")
  fi
  if ! has_launch_arg_key "outpost_manual_goal_y_m"; then
    LAUNCH_ARGS=("outpost_manual_goal_y_m:=${GOAL_MAP_Y_M}" "${LAUNCH_ARGS[@]}")
  fi
  if ! has_launch_arg_key "outpost_manual_goal_z_m"; then
    LAUNCH_ARGS=("outpost_manual_goal_z_m:=0.0" "${LAUNCH_ARGS[@]}")
  fi
  if ! has_launch_arg_key "face_mode_manual_target_enable"; then
    LAUNCH_ARGS=("face_mode_manual_target_enable:=true" "${LAUNCH_ARGS[@]}")
  fi
  if ! has_launch_arg_key "face_mode_manual_target_frame"; then
    LAUNCH_ARGS=("face_mode_manual_target_frame:=map" "${LAUNCH_ARGS[@]}")
  fi
  if ! has_launch_arg_key "face_mode_manual_target_x_m"; then
    LAUNCH_ARGS=("face_mode_manual_target_x_m:=${FACE_MAP_X_M}" "${LAUNCH_ARGS[@]}")
  fi
  if ! has_launch_arg_key "face_mode_manual_target_y_m"; then
    LAUNCH_ARGS=("face_mode_manual_target_y_m:=${FACE_MAP_Y_M}" "${LAUNCH_ARGS[@]}")
  fi
  if ! has_launch_arg_key "face_mode_manual_target_z_m"; then
    LAUNCH_ARGS=("face_mode_manual_target_z_m:=${FACE_MAP_Z_M}" "${LAUNCH_ARGS[@]}")
  fi
  if ! has_launch_arg_key "face_mode_target_frame"; then
    LAUNCH_ARGS=("face_mode_target_frame:=map" "${LAUNCH_ARGS[@]}")
  fi
  if ! has_launch_arg_key "face_mode_use_raw_goal_static_calibration"; then
    LAUNCH_ARGS=("face_mode_use_raw_goal_static_calibration:=false" "${LAUNCH_ARGS[@]}")
  fi
fi
if ! has_launch_arg_key "use_gimbal"; then
  LAUNCH_ARGS=("use_gimbal:=true" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "use_behavior_tree"; then
  LAUNCH_ARGS=("use_behavior_tree:=true" "${LAUNCH_ARGS[@]}")
fi

exec ros2 launch behavior_tree outpost_regional_test.launch.py "${LAUNCH_ARGS[@]}"
