#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

RAW_TOPIC="/ly/navi/goal_pos_raw"
GOAL_POSE_TOPIC="/goal_pose"
PREVIEW_TOPIC="/ly/navi/goal_pose_preview"
CONFIRM_PUBLISH=1
RAW_FRAME="map"
MAP_FRAME=""
BASE_FRAME=""
FALLBACK_BASE_FRAME=""
INPUT_UNIT="cm"
LAUNCH_ARGS=()

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [options] [-- <extra launch args...>]

Purpose:
  Navi-to-map static goal_pose conversion test, not chase.
  - Static calibration only: official/raw x y -> converted geometry_msgs/PoseStamped goal
  - Start the raw goal conversion bridge in background
  - Ask for x y continuously in terminal
  - Publish raw input to /ly/navi/goal_pos_raw
  - Read converted preview output in terminal from /ly/navi/goal_pose_preview
  - Ask y/n before publishing confirmed output to /goal_pose
  - Final navigation command topic is /goal_pose (geometry_msgs/msg/PoseStamped)
  - Static calibration points are read from navi_tf_bridge/config/tf_config.yaml
  - Chase is a separate flow: it listens to /ly/navi/target_rel and also ends at /goal_pose

Options:
  --raw-topic <topic>               (default: /ly/navi/goal_pos_raw)
  --goal-pose-topic <topic>         (default: /goal_pose)
  --goal-topic <topic>              alias for --goal-pose-topic
  --preview-topic <topic>           (default: /ly/navi/goal_pose_preview)
  --confirm-publish | --no-confirm-publish
                                     confirm before publishing to --goal-pose-topic (default: confirm)
  --raw-frame <frame>               (default: map)
  --map-frame <frame>               (default from tf_config.yaml)
  --base-frame <frame>              (default from tf_config.yaml)
  --fallback-base-frame <frame>     (default from tf_config.yaml)
  --input-unit cm|m                 (default: cm)
  --help

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --raw-frame world
  ./${SCRIPT_NAME} --input-unit m
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
    --raw-topic)
      RAW_TOPIC="${2:-}"
      shift 2
      ;;
    --goal-pose-topic|--goal-topic)
      GOAL_POSE_TOPIC="${2:-}"
      shift 2
      ;;
    --preview-topic)
      PREVIEW_TOPIC="${2:-}"
      shift 2
      ;;
    --confirm-publish)
      CONFIRM_PUBLISH=1
      shift
      ;;
    --no-confirm-publish)
      CONFIRM_PUBLISH=0
      shift
      ;;
    --raw-frame)
      RAW_FRAME="${2:-}"
      shift 2
      ;;
    --map-frame)
      MAP_FRAME="${2:-}"
      shift 2
      ;;
    --base-frame)
      BASE_FRAME="${2:-}"
      shift 2
      ;;
    --fallback-base-frame)
      FALLBACK_BASE_FRAME="${2:-}"
      shift 2
      ;;
    --input-unit)
      INPUT_UNIT="${2:-}"
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

if [[ -z "${RAW_TOPIC}" || -z "${GOAL_POSE_TOPIC}" || -z "${RAW_FRAME}" ]]; then
  echo "[ERROR] --raw-topic/--goal-pose-topic/--raw-frame cannot be empty." >&2
  exit 1
fi
if (( CONFIRM_PUBLISH == 1 )) && [[ -z "${PREVIEW_TOPIC}" ]]; then
  echo "[ERROR] --preview-topic cannot be empty when confirmation is enabled." >&2
  exit 1
fi
if [[ "${INPUT_UNIT}" != "cm" && "${INPUT_UNIT}" != "m" ]]; then
  echo "[ERROR] --input-unit must be 'cm' or 'm'." >&2
  exit 1
fi
if (( CONFIRM_PUBLISH == 1 )) && has_launch_arg_key "output_goal_pose_topic"; then
  echo "[ERROR] Do not pass output_goal_pose_topic:=... in confirm mode; use --preview-topic instead." >&2
  exit 1
fi

source_ros_workspace "${ROOT_DIR}"
cleanup_existing_stack "1" "/(target_rel_to_goal_pos_node|navitomap_input_node)([[:space:]]|$)" "ros2 launch navi_tf_bridge (navitomap|target_rel_to_goal_pos)\\.launch.py"

if ! has_launch_arg_key "input_goal_pos_raw_topic"; then
  LAUNCH_ARGS=("input_goal_pos_raw_topic:=${RAW_TOPIC}" "${LAUNCH_ARGS[@]}")
fi
BRIDGE_GOAL_POSE_TOPIC="${GOAL_POSE_TOPIC}"
if (( CONFIRM_PUBLISH == 1 )); then
  BRIDGE_GOAL_POSE_TOPIC="${PREVIEW_TOPIC}"
fi
CONFIRM_PUBLISH_PARAM="false"
if (( CONFIRM_PUBLISH == 1 )); then
  CONFIRM_PUBLISH_PARAM="true"
fi

if ! has_launch_arg_key "output_goal_pose_topic"; then
  LAUNCH_ARGS=("output_goal_pose_topic:=${BRIDGE_GOAL_POSE_TOPIC}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "publish_goal_pose"; then
  LAUNCH_ARGS=("publish_goal_pose:=true" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "publish_goal_pos"; then
  LAUNCH_ARGS=("publish_goal_pos:=false" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "goal_pos_raw_frame"; then
  LAUNCH_ARGS=("goal_pos_raw_frame:=${RAW_FRAME}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "use_raw_goal_static_calibration"; then
  LAUNCH_ARGS=("use_raw_goal_static_calibration:=true" "${LAUNCH_ARGS[@]}")
fi
if [[ -n "${MAP_FRAME}" ]] && ! has_launch_arg_key "map_frame"; then
  LAUNCH_ARGS=("map_frame:=${MAP_FRAME}" "${LAUNCH_ARGS[@]}")
fi
if [[ -n "${BASE_FRAME}" ]] && ! has_launch_arg_key "base_frame"; then
  LAUNCH_ARGS=("base_frame:=${BASE_FRAME}" "${LAUNCH_ARGS[@]}")
fi
if [[ -n "${FALLBACK_BASE_FRAME}" ]] && ! has_launch_arg_key "fallback_base_frame"; then
  LAUNCH_ARGS=("fallback_base_frame:=${FALLBACK_BASE_FRAME}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "debug_export_point_pairs"; then
  LAUNCH_ARGS=("debug_export_point_pairs:=false" "${LAUNCH_ARGS[@]}")
fi

echo "[INFO] Launch goal_pose conversion bridge with args: ${LAUNCH_ARGS[*]}"
ros2 launch navi_tf_bridge navitomap.launch.py "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!

cleanup() {
  local rc=$?
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    sleep 1
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
  fi
  wait "${LAUNCH_PID}" 2>/dev/null || true
  exit "${rc}"
}
trap cleanup EXIT INT TERM

sleep 1
echo "[INFO] Start navitomap input node. Type q to quit."
ros2 run navi_tf_bridge navitomap_input_node --ros-args \
  -p raw_topic:="${RAW_TOPIC}" \
  -p goal_topic:="${BRIDGE_GOAL_POSE_TOPIC}" \
  -p confirmed_goal_topic:="${GOAL_POSE_TOPIC}" \
  -p confirm_before_publish:="${CONFIRM_PUBLISH_PARAM}" \
  -p input_unit:="${INPUT_UNIT}"
