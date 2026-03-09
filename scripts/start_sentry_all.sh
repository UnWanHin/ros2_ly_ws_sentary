#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
CLEANUP_EXISTING=1
LAUNCH_ARGS=()

STACK_NODE_REGEX="/(gimbal_driver_node|detector_node|tracker_solver_node|predictor_node|outpost_hitter_node|buff_hitter_node|behavior_tree_node|mapper_node|fire_flip_test)([[:space:]]|$)"
STACK_LAUNCH_REGEX="ros2 launch behavior_tree sentry_all.launch.py"

: "${ROS_LOG_DIR:=/tmp/ros2_logs}"
mkdir -p "${ROS_LOG_DIR}"
export ROS_LOG_DIR

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--cleanup-existing|--no-cleanup-existing] [-- <launch_args...>]

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --no-cleanup-existing
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

source_ros() {
  if [[ -n "${ROS_DISTRO:-}" ]] && [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    set +u
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
    return
  fi

  for distro in humble iron jazzy rolling; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      # shellcheck disable=SC1090
      set +u
      source "/opt/ros/${distro}/setup.bash"
      set -u
      return
    fi
  done

  echo "[ERROR] No ROS2 setup found under /opt/ros." >&2
  exit 1
}

source_workspace() {
  if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u
    source "${ROOT_DIR}/install/setup.bash"
    set -u
    return
  fi

  echo "[ERROR] ${ROOT_DIR}/install/setup.bash not found." >&2
  echo "        Run: colcon build" >&2
  exit 1
}

source_ros
source_workspace

mapfile -t EXISTING_STACK_PROCS < <(
  {
    pgrep -af "${STACK_NODE_REGEX}" || true
    pgrep -af "${STACK_LAUNCH_REGEX}" || true
  } | awk '!seen[$0]++'
)

if (( ${#EXISTING_STACK_PROCS[@]} > 0 )); then
  echo "[WARN] Detected existing stack-related processes:" >&2
  printf "  %s\n" "${EXISTING_STACK_PROCS[@]}" >&2
  if (( CLEANUP_EXISTING == 1 )); then
    echo "[INFO] Cleaning up stale processes before launch..." >&2
    pkill -f "${STACK_NODE_REGEX}" || true
    pkill -f "${STACK_LAUNCH_REGEX}" || true
    sleep 1
  else
    echo "[ERROR] Existing processes found. Re-run with --cleanup-existing." >&2
    exit 3
  fi
fi

exec ros2 launch behavior_tree sentry_all.launch.py "${LAUNCH_ARGS[@]}"
