#!/usr/bin/env bash

# FaceMode fixed-point aiming wrapper.
# Positional point format: map_x map_y map_z, default unit m.
# This uses the navigation map frame directly and does not apply tf_config.yaml matrix.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
OFFICIAL_MAP_UNIT="${OFFICIAL_MAP_UNIT:-m}"
TARGET_FRAME="${TARGET_FRAME:-map}"
USE_RAW_GOAL_STATIC_CALIBRATION="${USE_RAW_GOAL_STATIC_CALIBRATION:-false}"
RAW_GOAL_TARGET_FRAME="${RAW_GOAL_TARGET_FRAME:-map}"
SOLVE_MODE="${SOLVE_MODE:-base_link}"
SOLVE_FRAME="${SOLVE_FRAME:-gimbal_small_yaw}"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--bt-output] <map_x> <map_y> <map_z> [map_aim_point_test args...]
  ${SCRIPT_NAME} -- official_map_x:=X official_map_y:=Y map_z:=Z [launch_args...]

Purpose:
  FaceMode map-frame direct input.
  X/Y/Z are used directly in target_frame=map; no tf_config.yaml matrix is applied.
  It uses TF geometry by default: map -> gimbal_small_yaw, no camera projection required.
  Default input unit is m, matching navigation/map coordinates. Pass --unit cm for centimeters.
  --bt-output publishes angles to /ly/face_mode/angles and disables FaceMode firecode output.

Examples:
  ./${SCRIPT_NAME} 1.20 3.40 1.00 --with-tf-tree
  ./${SCRIPT_NAME} --bt-output 1.20 3.40 1.00 --with-tf-tree
  ./${SCRIPT_NAME} --unit cm 120 340 100 --with-tf-tree
  ./${SCRIPT_NAME} -- official_map_x:=1.20 official_map_y:=3.40 map_z:=1.00 target_frame:=map use_raw_goal_static_calibration:=false
EOF
}

BT_OUTPUT=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bt-output)
      BT_OUTPUT=true
      shift
      ;;
    --unit|--input-unit|--official-unit)
      if (( $# < 2 )); then
        echo "[ERROR] $1 requires m or cm." >&2
        exit 2
      fi
      OFFICIAL_MAP_UNIT="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      export OFFICIAL_MAP_UNIT
      export TARGET_FRAME
      export USE_RAW_GOAL_STATIC_CALIBRATION
      export RAW_GOAL_TARGET_FRAME
      export SOLVE_MODE
      export SOLVE_FRAME
      exec "${ROOT_DIR}/scripts/navi/map_aim_point_test.sh" "$@"
      ;;
    --*)
      echo "[ERROR] Unknown wrapper option before coordinates: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      break
      ;;
  esac
done

if (( $# < 3 )); then
  usage >&2
  exit 2
fi

export OFFICIAL_MAP_UNIT
export TARGET_FRAME
export USE_RAW_GOAL_STATIC_CALIBRATION
export RAW_GOAL_TARGET_FRAME
export SOLVE_MODE
export SOLVE_FRAME
export OFFICIAL_MAP_X="$1"
export OFFICIAL_MAP_Y="$2"
export MAP_Z="$3"
shift 3

if [[ "${BT_OUTPUT}" == "true" ]]; then
  set -- "control_angles_topic:=/ly/face_mode/angles" "publish_firecode:=false" "$@"
fi

exec "${ROOT_DIR}/scripts/navi/map_aim_point_test.sh" "$@"
