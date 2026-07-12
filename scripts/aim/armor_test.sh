#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
USE_NOGATE=1
MODE_ARG="regional"
DEFAULT_BASE_CONFIG_FILE="${ROOT_DIR}/config/base_config.yaml"
DEFAULT_OVERRIDE_CONFIG_FILE="${ROOT_DIR}/config/override_config.yaml"
LAUNCH_ARGS=()

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--nogate|--with-gate] [--mode league|regional|showcase] [-- <launch_args...>]

Purpose:
  Formal external-aim armor test wrapper.
  Starts behavior_tree/sentry_all via competition_autoaim.launch.py and waits for
  external /ly/aim/armor_targets + /ly/aim/result.
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
    --mode)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --mode requires a value" >&2
        exit 2
      fi
      MODE_ARG="$2"
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
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

source_ros_workspace "${ROOT_DIR}"
require_sentry_msgs_for_behavior_tree
cleanup_existing_stack "1" "/(gimbal_driver_node|behavior_tree_node|map_aim_point_node|map_path_to_game_path_node|target_rel_to_goal_pos_node)([[:space:]]|$)" "ros2 launch (behavior_tree|navi_tf_bridge) (competition_autoaim|sentry_all|map_aim_point)\\.launch.py"

if (( USE_NOGATE == 1 )); then
  LAUNCH_ARGS=("debug_bypass_is_start:=true" "${LAUNCH_ARGS[@]}")
else
  LAUNCH_ARGS=("debug_bypass_is_start:=false" "${LAUNCH_ARGS[@]}")
fi

if [[ -f "${DEFAULT_BASE_CONFIG_FILE}" ]] && ! has_launch_arg_key "base_config_file"; then
  LAUNCH_ARGS=("base_config_file:=${DEFAULT_BASE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi

if [[ -f "${DEFAULT_OVERRIDE_CONFIG_FILE}" ]] && ! has_launch_arg_key "config_file"; then
  LAUNCH_ARGS=("config_file:=${DEFAULT_OVERRIDE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi

if ! has_launch_arg_key "mode"; then
  LAUNCH_ARGS=("mode:=${MODE_ARG}" "${LAUNCH_ARGS[@]}")
fi

exec ros2 launch behavior_tree competition_autoaim.launch.py "${LAUNCH_ARGS[@]}"
