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
MANUAL_OUTPOST_MAP=0
OUTPOST_MAP_X_CM=""
OUTPOST_MAP_Y_CM=""
OUTPOST_MAP_Z_CM=""
POSITIONAL_OUTPOST_MAP_M=()
LAUNCH_ARGS=()

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [X_M Y_M Z_M] [--nogate|--with-gate] [--online|--offline] [--no-face-mode-solver]
                [--outpost-map X_CM Y_CM Z_CM]
                [--outpost-map-x X_CM --outpost-map-y Y_CM --outpost-map-z Z_CM]
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
  - positional X_M Y_M Z_M uses already-calibrated map-frame meters directly for FaceMode
  - --outpost-map uses already-calibrated map-frame cm coordinates directly for FaceMode

Examples:
  ./scripts/aim/${SCRIPT_NAME}
  ./scripts/aim/${SCRIPT_NAME} 24.08 13.11 1.20
  ./scripts/aim/${SCRIPT_NAME} --offline
  ./scripts/aim/${SCRIPT_NAME} --no-face-mode-solver
  ./scripts/aim/${SCRIPT_NAME} --outpost-map 2408 1311 120
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

require_uint16_cm() {
  local name="$1"
  local value="$2"
  if [[ ! "${value}" =~ ^[0-9]+$ ]] || (( value > 65535 )); then
    echo "Invalid ${name}: ${value}. Expected integer cm in [0, 65535]." >&2
    exit 2
  fi
}

is_meter_value() {
  [[ "$1" =~ ^[+]?[0-9]+([.][0-9]+)?$ || "$1" =~ ^[+]?[.][0-9]+$ ]]
}

meter_to_uint16_cm() {
  local name="$1"
  local value="$2"
  awk -v name="${name}" -v value="${value}" '
    BEGIN {
      if (value !~ /^[+]?[0-9]+([.][0-9]+)?$/ && value !~ /^[+]?[.][0-9]+$/) {
        printf("Invalid %s: %s. Expected meter value in [0, 655.35].\n", name, value) > "/dev/stderr";
        exit 2;
      }
      cm = int(value * 100.0 + 0.5);
      if (cm < 0 || cm > 65535) {
        printf("Invalid %s: %s m -> %d cm. Expected [0, 655.35] m.\n", name, value, cm) > "/dev/stderr";
        exit 2;
      }
      printf("%d", cm);
    }
  '
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
    --outpost-map|--outpost-map-cm)
      if [[ $# -lt 4 ]]; then
        echo "$1 requires X_CM Y_CM Z_CM." >&2
        exit 2
      fi
      MANUAL_OUTPOST_MAP=1
      OUTPOST_MAP_X_CM="$2"
      OUTPOST_MAP_Y_CM="$3"
      OUTPOST_MAP_Z_CM="$4"
      shift 4
      ;;
    --outpost-map-x)
      if [[ $# -lt 2 ]]; then
        echo "$1 requires X_CM." >&2
        exit 2
      fi
      MANUAL_OUTPOST_MAP=1
      OUTPOST_MAP_X_CM="$2"
      shift 2
      ;;
    --outpost-map-y)
      if [[ $# -lt 2 ]]; then
        echo "$1 requires Y_CM." >&2
        exit 2
      fi
      MANUAL_OUTPOST_MAP=1
      OUTPOST_MAP_Y_CM="$2"
      shift 2
      ;;
    --outpost-map-z)
      if [[ $# -lt 2 ]]; then
        echo "$1 requires Z_CM." >&2
        exit 2
      fi
      MANUAL_OUTPOST_MAP=1
      OUTPOST_MAP_Z_CM="$2"
      shift 2
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
        if (( ${#POSITIONAL_OUTPOST_MAP_M[@]} >= 3 )); then
          echo "Unexpected extra positional meter coordinate: $1. Pass launch args after --." >&2
          exit 2
        fi
        POSITIONAL_OUTPOST_MAP_M+=("$1")
      else
        LAUNCH_ARGS+=("$1")
      fi
      shift
      ;;
  esac
done

if (( ${#POSITIONAL_OUTPOST_MAP_M[@]} > 0 )); then
  if (( MANUAL_OUTPOST_MAP == 1 )); then
    echo "Use either positional X_M Y_M Z_M or --outpost-map cm args, not both." >&2
    exit 2
  fi
  if (( ${#POSITIONAL_OUTPOST_MAP_M[@]} != 3 )); then
    echo "Manual outpost map target requires exactly X_M Y_M Z_M." >&2
    exit 2
  fi
  MANUAL_OUTPOST_MAP=1
  OUTPOST_MAP_X_CM="$(meter_to_uint16_cm "outpost map x" "${POSITIONAL_OUTPOST_MAP_M[0]}")"
  OUTPOST_MAP_Y_CM="$(meter_to_uint16_cm "outpost map y" "${POSITIONAL_OUTPOST_MAP_M[1]}")"
  OUTPOST_MAP_Z_CM="$(meter_to_uint16_cm "outpost map z" "${POSITIONAL_OUTPOST_MAP_M[2]}")"
fi

if (( MANUAL_OUTPOST_MAP == 1 )); then
  if [[ -z "${OUTPOST_MAP_X_CM}" || -z "${OUTPOST_MAP_Y_CM}" || -z "${OUTPOST_MAP_Z_CM}" ]]; then
    echo "Manual outpost map target requires X/Y/Z. Use --outpost-map X_CM Y_CM Z_CM." >&2
    exit 2
  fi
  require_uint16_cm "outpost map x" "${OUTPOST_MAP_X_CM}"
  require_uint16_cm "outpost map y" "${OUTPOST_MAP_Y_CM}"
  require_uint16_cm "outpost map z" "${OUTPOST_MAP_Z_CM}"
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
if (( MANUAL_OUTPOST_MAP == 1 )); then
  if ! has_launch_arg_key "face_mode_outpost_manual_target_enable"; then
    LAUNCH_ARGS=("face_mode_outpost_manual_target_enable:=true" "${LAUNCH_ARGS[@]}")
  fi
  if ! has_launch_arg_key "face_mode_outpost_manual_target_map_x_cm"; then
    LAUNCH_ARGS=("face_mode_outpost_manual_target_map_x_cm:=${OUTPOST_MAP_X_CM}" "${LAUNCH_ARGS[@]}")
  fi
  if ! has_launch_arg_key "face_mode_outpost_manual_target_map_y_cm"; then
    LAUNCH_ARGS=("face_mode_outpost_manual_target_map_y_cm:=${OUTPOST_MAP_Y_CM}" "${LAUNCH_ARGS[@]}")
  fi
  if ! has_launch_arg_key "face_mode_outpost_manual_target_map_z_cm"; then
    LAUNCH_ARGS=("face_mode_outpost_manual_target_map_z_cm:=${OUTPOST_MAP_Z_CM}" "${LAUNCH_ARGS[@]}")
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
