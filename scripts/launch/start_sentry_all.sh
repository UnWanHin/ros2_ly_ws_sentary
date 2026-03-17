#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
CLEANUP_EXISTING=1
OFFLINE_MODE=0
MODE_ARG=""
LAUNCH_ARGS=()

STACK_NODE_REGEX="/(gimbal_driver_node|detector_node|tracker_solver_node|predictor_node|outpost_hitter_node|buff_hitter_node|behavior_tree_node|mapper_node|fire_flip_test)([[:space:]]|$)"
STACK_LAUNCH_REGEX="ros2 launch behavior_tree sentry_all.launch.py"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--cleanup-existing|--no-cleanup-existing] [--offline] [--mode 1|2|3|league|regional|showcase] [-- <launch_args...>]

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --no-cleanup-existing
  ./${SCRIPT_NAME} --offline
  ./${SCRIPT_NAME} --mode 1
  ./${SCRIPT_NAME} --mode regional --no-prompt
  ./${SCRIPT_NAME} --mode 3 --no-prompt
  ./${SCRIPT_NAME} -- use_buff:=false use_outpost:=false
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cleanup-existing)
      CLEANUP_EXISTING=1
      shift
      ;;
    --no-cleanup-existing)
      CLEANUP_EXISTING=0
      shift
      ;;
    --offline)
      OFFLINE_MODE=1
      shift
      ;;
    --mode)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --mode requires a value: 1|2|3|league|regional|showcase" >&2
        exit 2
      fi
      MODE_ARG="$2"
      shift 2
      ;;
    --no-prompt)
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
cleanup_existing_stack "${CLEANUP_EXISTING}" "${STACK_NODE_REGEX}" "${STACK_LAUNCH_REGEX}"

if [[ -n "${MODE_ARG}" ]]; then
  LAUNCH_ARGS=("mode:=${MODE_ARG}" "${LAUNCH_ARGS[@]}")
fi
if (( OFFLINE_MODE == 1 )); then
  LAUNCH_ARGS=("offline:=true" "${LAUNCH_ARGS[@]}")
fi

exec ros2 launch behavior_tree sentry_all.launch.py "${LAUNCH_ARGS[@]}"
