#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
USE_NOGATE=1
LAUNCH_ARGS=()
DEFAULT_BASE_CONFIG_FILE="${ROOT_DIR}/config/base_config.yaml"
DEFAULT_OVERRIDE_CONFIG_FILE="${ROOT_DIR}/config/override_config.yaml"
DEFAULT_COMMON_CONFIG_FILE="${ROOT_DIR}/config/common.yaml"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--nogate|--with-gate] [-- <launch_args...>]

Purpose:
  Thin wrapper for behavior_tree/showcase.launch.py.
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

read_common_bool_value() {
  local value="${1,,}"
  case "${value}" in
    true|1|yes|on) printf '1\n' ;;
    false|0|no|off) printf '0\n' ;;
    *) return 1 ;;
  esac
}

read_common_scalar() {
  local config_file="$1"
  local key="$2"
  python3 - "$config_file" "$key" <<'PY'
import sys
from pathlib import Path

path = Path(sys.argv[1])
key = sys.argv[2]
if not path.exists():
    sys.exit(1)

for raw in path.read_text().splitlines():
    line = raw.split("#", 1)[0].strip()
    if not line or ":" not in line:
        continue
    lhs, rhs = line.split(":", 1)
    if lhs.strip() == key:
        print(rhs.strip().strip('"').strip("'"))
        sys.exit(0)
sys.exit(1)
PY
}

read_common_rosbag() {
  local config_file="$1"
  [[ -f "${config_file}" ]] || return 1

  local value=""
  value="$(read_common_scalar "${config_file}" "rosbag")" || return 1
  read_common_bool_value "${value}"
}

read_common_rosbag_path() {
  local config_file="$1"
  [[ -f "${config_file}" ]] || return 1

  local value=""
  value="$(read_common_scalar "${config_file}" "rosbag_path")" || return 1
  [[ -n "${value}" ]] || return 1

  if [[ "${value}" == "~" ]]; then
    value="${HOME}"
  elif [[ "${value}" == "~/"* ]]; then
    value="${HOME}/${value#"~/"}"
  fi

  printf '%s\n' "${value}"
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
cleanup_existing_stack "1" "/(gimbal_driver_node|detector_node|tracker_solver_node|predictor_node|outpost_hitter_node|buff_hitter_node|behavior_tree_node)([[:space:]]|$)" "ros2 launch behavior_tree (showcase|sentry_all)\\.launch.py"

if [[ -f "${DEFAULT_BASE_CONFIG_FILE}" ]] && ! has_launch_arg_key "base_config_file"; then
  LAUNCH_ARGS=("base_config_file:=${DEFAULT_BASE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi
if [[ -f "${DEFAULT_OVERRIDE_CONFIG_FILE}" ]] && ! has_launch_arg_key "config_file"; then
  LAUNCH_ARGS=("config_file:=${DEFAULT_OVERRIDE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi

if ! has_launch_arg_key "rosbag"; then
  if ROSBAG_FROM_COMMON="$(read_common_rosbag "${DEFAULT_COMMON_CONFIG_FILE}")"; then
    if [[ "${ROSBAG_FROM_COMMON}" == "0" ]]; then
      LAUNCH_ARGS=("rosbag:=false" "${LAUNCH_ARGS[@]}")
      echo "[INFO] default rosbag=false (from ${DEFAULT_COMMON_CONFIG_FILE})"
    else
      LAUNCH_ARGS=("rosbag:=true" "${LAUNCH_ARGS[@]}")
      echo "[INFO] default rosbag=true (from ${DEFAULT_COMMON_CONFIG_FILE})"
    fi
  fi
fi

if ! has_launch_arg_key "rosbag_path"; then
  if ROSBAG_PATH_FROM_COMMON="$(read_common_rosbag_path "${DEFAULT_COMMON_CONFIG_FILE}")"; then
    LAUNCH_ARGS=("rosbag_path:=${ROSBAG_PATH_FROM_COMMON}" "${LAUNCH_ARGS[@]}")
    echo "[INFO] default rosbag_path=${ROSBAG_PATH_FROM_COMMON} (from ${DEFAULT_COMMON_CONFIG_FILE})"
  fi
fi

if (( USE_NOGATE == 1 )); then
  LAUNCH_ARGS=("debug_bypass_is_start:=true" "${LAUNCH_ARGS[@]}")
else
  LAUNCH_ARGS=("debug_bypass_is_start:=false" "${LAUNCH_ARGS[@]}")
fi

exec ros2 launch behavior_tree showcase.launch.py "${LAUNCH_ARGS[@]}"
