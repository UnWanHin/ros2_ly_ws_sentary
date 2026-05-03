#!/usr/bin/env bash

# FaceMode fixed-point gimbal aiming wrapper.
# Positional point format: official_map_x official_map_y map_z, unit cm.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} <official_map_x_cm> <official_map_y_cm> <map_z_cm> [map_aim_point_test args...]
  ${SCRIPT_NAME} -- official_map_x:=X official_map_y:=Y map_z:=Z [launch_args...]

Purpose:
  FaceMode: keep gimbal facing one fixed official-map/map-height point.
  X/Y are official map coordinates in cm and are converted by navi_tf_bridge tf_config.yaml.
  Z is map-frame height in cm. Default yaw_sign is -1.0 from map_aim_point.launch.py.

Examples:
  ./${SCRIPT_NAME} 1093 366 100 --with-tf-tree
  ./${SCRIPT_NAME} 1093 366 100 --no-gimbal
  ./${SCRIPT_NAME} -- official_map_x:=1093 official_map_y:=366 map_z:=100 yaw_sign:=-1.0
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

if [[ "${1:-}" == "--" ]]; then
  exec "${ROOT_DIR}/scripts/navi/map_aim_point_test.sh" "$@"
fi

if (( $# < 3 )); then
  usage >&2
  exit 2
fi

export OFFICIAL_MAP_X="$1"
export OFFICIAL_MAP_Y="$2"
export MAP_Z="$3"
shift 3

exec "${ROOT_DIR}/scripts/navi/map_aim_point_test.sh" "$@"
