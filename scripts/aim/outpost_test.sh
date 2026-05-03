#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

OFFLINE_MODE=0
LAUNCH_ARGS=()

DEFAULT_BASE_CONFIG_FILE="${ROOT_DIR}/config/base_config.yaml"
DEFAULT_DETECTOR_CONFIG_FILE="${ROOT_DIR}/src/detector/config/detector_config.yaml"
DEFAULT_PREDICTOR_CONFIG_FILE="${ROOT_DIR}/src/predictor/config/predictor_config.yaml"
DEFAULT_OVERRIDE_CONFIG_FILE="${ROOT_DIR}/config/override_config.yaml"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--online|--offline]
                 [-- <launch_args...>]

Purpose:
  Armor-style recognition chain without behavior_tree (default):
  - no behavior_tree
  - start gimbal_driver + detector + tracker_solver + predictor
  - no bridge/outpost_hitter forwarding to /ly/control/*

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --offline
  ./${SCRIPT_NAME} -- detector_config.show:=true predictor_config.publish_only_on_new_tracker_frame:=true
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
    --online)
      OFFLINE_MODE=0
      shift
      ;;
    --offline)
      OFFLINE_MODE=1
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
cleanup_existing_stack "1" "/(gimbal_driver_node|detector_node|tracker_solver_node|predictor_node|outpost_hitter_node|outpost_test_bridge|behavior_tree_node|buff_test_bridge)([[:space:]]|$)" "ros2 launch (detector|behavior_tree) (auto_aim|outpost_test|outpost|buff_test|competition_autoaim|sentry_all|chase_only|showcase)\\.launch.py"

if [[ -f "${DEFAULT_BASE_CONFIG_FILE}" ]] && ! has_launch_arg_key "base_config_file"; then
  LAUNCH_ARGS=("base_config_file:=${DEFAULT_BASE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi
if [[ -f "${DEFAULT_DETECTOR_CONFIG_FILE}" ]] && ! has_launch_arg_key "detector_config_file"; then
  LAUNCH_ARGS=("detector_config_file:=${DEFAULT_DETECTOR_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi
if [[ -f "${DEFAULT_PREDICTOR_CONFIG_FILE}" ]] && ! has_launch_arg_key "predictor_config_file"; then
  LAUNCH_ARGS=("predictor_config_file:=${DEFAULT_PREDICTOR_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi
if [[ -f "${DEFAULT_OVERRIDE_CONFIG_FILE}" ]] && ! has_launch_arg_key "config_file"; then
  LAUNCH_ARGS=("config_file:=${DEFAULT_OVERRIDE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
fi

if ! has_launch_arg_key "use_gimbal"; then
  LAUNCH_ARGS=("use_gimbal:=true" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "use_detector"; then
  LAUNCH_ARGS=("use_detector:=true" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "use_tracker"; then
  LAUNCH_ARGS=("use_tracker:=true" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "use_predictor"; then
  LAUNCH_ARGS=("use_predictor:=true" "${LAUNCH_ARGS[@]}")
fi

if ! has_launch_arg_key "detector_config.show" && ! has_launch_arg_key "detector_config/show"; then
  LAUNCH_ARGS=("detector_config.show:=true" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "detector_config.draw" && ! has_launch_arg_key "detector_config/draw"; then
  LAUNCH_ARGS=("detector_config.draw:=true" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "detector_config.web_show" && ! has_launch_arg_key "detector_config/web_show"; then
  LAUNCH_ARGS=("detector_config.web_show:=true" "${LAUNCH_ARGS[@]}")
fi
if (( OFFLINE_MODE == 1 )) && ! has_launch_arg_key "offline"; then
  LAUNCH_ARGS=("offline:=true" "${LAUNCH_ARGS[@]}")
fi

exec ros2 launch detector auto_aim.launch.py "${LAUNCH_ARGS[@]}"
