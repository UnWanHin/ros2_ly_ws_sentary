#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

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

exec ros2 launch behavior_tree sentry_all.launch.py "$@"
