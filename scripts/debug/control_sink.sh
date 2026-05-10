#!/usr/bin/env bash

# Dry-run receiver for /ly/control/* commands.
# It subscribes and logs commands, but does not start gimbal_driver or write to hardware.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
CLEANUP_EXISTING=1
ALLOW_WITH_GIMBAL=0
VERBOSE=false
INCLUDE_VEL=true
INCLUDE_POSTURE=true
LOG_EVERY_SEC="${LOG_EVERY_SEC:-1.0}"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [options]

Purpose:
  Receive /ly/control/angles and /ly/control/firecode without moving hardware.
  This script does not launch gimbal_driver; it only subscribes and logs.

Options:
  --verbose              Print every received command, not just summary.
  --log-every <sec>      Summary interval. Default: ${LOG_EVERY_SEC}
  --no-vel               Do not subscribe /ly/control/vel.
  --no-posture           Do not subscribe /ly/control/posture or /ly/control/sentry_cmd.
  --allow-with-gimbal    Allow running when /gimbal_driver is online.
  --no-cleanup-existing  Do not clean old control_sink process.

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --verbose
EOF
}

has_node() {
  local node_name="$1"
  ros2 node list 2>/dev/null | awk -v node="${node_name}" '$0 == node { found = 1 } END { exit found ? 0 : 1 }'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --verbose)
      VERBOSE=true
      shift
      ;;
    --log-every)
      if (( $# < 2 )); then
        echo "[ERROR] --log-every requires seconds." >&2
        exit 2
      fi
      LOG_EVERY_SEC="$2"
      shift 2
      ;;
    --no-vel)
      INCLUDE_VEL=false
      shift
      ;;
    --no-posture)
      INCLUDE_POSTURE=false
      shift
      ;;
    --allow-with-gimbal)
      ALLOW_WITH_GIMBAL=1
      shift
      ;;
    --cleanup-existing)
      CLEANUP_EXISTING=1
      shift
      ;;
    --no-cleanup-existing)
      CLEANUP_EXISTING=0
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

source_ros_workspace "${ROOT_DIR}"

if (( ALLOW_WITH_GIMBAL == 0 )) && has_node "/gimbal_driver"; then
  echo "[ERROR] /gimbal_driver is online. Commands on /ly/control/* may still move hardware." >&2
  echo "        Stop gimbal_driver first, or pass --allow-with-gimbal only for virtual-device tests." >&2
  exit 2
fi

cleanup_existing_stack \
  "${CLEANUP_EXISTING}" \
  "/(control_sink)([[:space:]]|$)|scripts/debug/control_sink\\.py" \
  "scripts/debug/control_sink\\.py"

echo "[INFO] starting control sink. It receives /ly/control/* but never writes to lower hardware." >&2
exec python3 "${ROOT_DIR}/scripts/debug/control_sink.py" \
  --ros-args \
  -p "verbose:=${VERBOSE}" \
  -p "include_vel:=${INCLUDE_VEL}" \
  -p "include_posture:=${INCLUDE_POSTURE}" \
  -p "log_every_sec:=${LOG_EVERY_SEC}"
