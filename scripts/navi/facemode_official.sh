#!/usr/bin/env bash

# FaceMode fixed-point aiming wrapper.
# Positional point format: official_map_x official_map_y official_map_z, default unit cm.
# This uses the official_map frame directly and does not apply tf_config.yaml matrix.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
OFFICIAL_MAP_UNIT="${OFFICIAL_MAP_UNIT:-cm}"
TARGET_FRAME="${TARGET_FRAME:-official_map}"
USE_RAW_GOAL_STATIC_CALIBRATION="${USE_RAW_GOAL_STATIC_CALIBRATION:-false}"
RAW_GOAL_TARGET_FRAME="${RAW_GOAL_TARGET_FRAME:-official_map}"
SOLVE_MODE="${SOLVE_MODE:-relative_geometry}"
SOLVE_FRAME="${SOLVE_FRAME:-gimbal_barrel_joint}"
YAW_SIGN="${YAW_SIGN:-1.0}"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--bt-output] <official_map_x> <official_map_y> <official_map_z> [map_aim_point_test args...]
  ${SCRIPT_NAME} -- official_map_x:=X official_map_y:=Y map_z:=Z [launch_args...]

Purpose:
  FaceMode official-frame direct input.
  X/Y/Z are used directly in target_frame=official_map; no tf_config.yaml matrix is applied.
  The fixed point is faced by TF relative geometry: target -> gimbal_barrel_joint, no camera projection required.
  This requires the running TF tree to provide official_map -> base_link/gimbal frames.
  Default input unit is cm; pass --unit m for meters.
  --bt-output publishes angles to /ly/face_mode/angles and disables FaceMode firecode output.

Examples:
  ./${SCRIPT_NAME} 1093 366 100
  ./${SCRIPT_NAME} --bt-output 744 1263 100
  ./${SCRIPT_NAME} --unit m 10.93 3.66 1.0
  ./${SCRIPT_NAME} -- official_map_x:=1093 official_map_y:=366 map_z:=100 target_frame:=official_map
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
      export YAW_SIGN
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
export YAW_SIGN
export OFFICIAL_MAP_X="$1"
export OFFICIAL_MAP_Y="$2"
export MAP_Z="$3"
shift 3

if [[ "${BT_OUTPUT}" == "true" ]]; then
  set -- "control_angles_topic:=/ly/face_mode/angles" "publish_firecode:=false" "$@"
fi

exec "${ROOT_DIR}/scripts/navi/map_aim_point_test.sh" "$@"
