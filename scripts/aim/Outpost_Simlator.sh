#!/usr/bin/env bash

# Formal Outpost task simulator.
# It keeps the chassis still, simulates that the sentry is already at BuffOutpost,
# and lets the real BT Outpost -> FaceMode -> /ly/control/* chain drive gimbal output.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

TEAM="red"
USE_NOGATE=1
FACE_MODE_SOLVER=1
TF_TREE_MODE=""
MONITOR=0
SIM_HZ="10"
FACE_MAP_X_M=""
FACE_MAP_Y_M=""
FACE_MAP_Z_M=""
POSITIONAL_M=()
LAUNCH_ARGS=()
PUB_PIDS=()

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} FACE_X_M FACE_Y_M FACE_Z_M
                [--team red|blue] [--with-tf-tree|--no-tf-tree]
                [--nogate|--with-gate] [--no-face-mode-solver]
                [--monitor] [--sim-hz HZ] [-- <launch_args...>]

Purpose:
  Formal Outpost FaceMode simulator.
  - starts behavior_tree/outpost_regional_test.launch.py -> sentry_all.launch.py
  - keeps chassis navigation output disabled with publish_navi_goal:=false
  - disables navi_tf_bridge so this script owns /ly/navi/position during the test
  - simulates that the sentry is already at own BuffOutpost, inside the 300cm FaceMode gate
  - passes FACE_X/Y/Z directly as a map-frame FaceMode target in meters
  - does not publish /ly/control/angles or /ly/control/firecode directly

Examples:
  ./scripts/aim/${SCRIPT_NAME} 6.17 -16.21 1.00 --with-tf-tree
  ./scripts/aim/${SCRIPT_NAME} 6.17 -16.21 1.00 --team blue --monitor
  ./scripts/aim/${SCRIPT_NAME} 6.17 -16.21 1.00 -- use_gimbal:=false
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

is_meter_value() {
  [[ "$1" =~ ^[-+]?[0-9]+([.][0-9]+)?$ || "$1" =~ ^[-+]?[.][0-9]+$ ]]
}

team_is_red_bool() {
  case "${TEAM}" in
    red|Red|RED)
      printf 'true\n'
      ;;
    blue|Blue|BLUE)
      printf 'false\n'
      ;;
    *)
      echo "[ERROR] --team must be red or blue, got '${TEAM}'." >&2
      exit 2
      ;;
  esac
}

buff_outpost_position_cm() {
  case "${TEAM}" in
    red|Red|RED)
      printf '1196 1256\n'
      ;;
    blue|Blue|BLUE)
      printf '1604 244\n'
      ;;
    *)
      echo "[ERROR] --team must be red or blue, got '${TEAM}'." >&2
      exit 2
      ;;
  esac
}

start_pub() {
  "$@" &
  PUB_PIDS+=("$!")
}

cleanup() {
  trap - INT TERM EXIT
  local pid
  for pid in "${PUB_PIDS[@]:-}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}

monitor_formal_chain() {
  local launch_pid="$1"
  local topic
  local info
  local type
  local pubs
  local subs
  local topics=(
    "/ly/friend/is_team_red"
    "/ly/navi/position"
    "/ly/navi/reached"
    "/ly/navi/reachable"
    "/ly/aim/select_target"
    "/ly/aim/result"
    "/ly/face_mode/target_raw"
    "/ly/face_mode/angles"
    "/ly/gimbal/facemode"
    "/ly/control/angles"
    "/ly/control/firecode"
  )

  sleep 4
  while kill -0 "${launch_pid}" 2>/dev/null; do
    echo "[MONITOR][$(date '+%H:%M:%S')] formal Outpost simulator endpoints" >&2
    for topic in "${topics[@]}"; do
      if info="$(timeout 2 ros2 topic info "${topic}" 2>/dev/null)"; then
        type="$(awk -F': ' '/^Type:/{print $2}' <<<"${info}")"
        pubs="$(awk -F': ' '/^Publisher count:/{print $2}' <<<"${info}")"
        subs="$(awk -F': ' '/^Subscription count:/{print $2}' <<<"${info}")"
        echo "[MONITOR] ${topic} type=${type:-?} pubs=${pubs:-?} subs=${subs:-?}" >&2
      else
        echo "[MONITOR] ${topic} not available yet" >&2
      fi
    done
    sleep 2
  done
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --team)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --team requires red or blue." >&2
        exit 2
      fi
      TEAM="$2"
      shift 2
      ;;
    --nogate)
      USE_NOGATE=1
      shift
      ;;
    --with-gate)
      USE_NOGATE=0
      shift
      ;;
    --no-face-mode-solver)
      FACE_MODE_SOLVER=0
      shift
      ;;
    --with-tf-tree)
      TF_TREE_MODE=1
      shift
      ;;
    --no-tf-tree)
      TF_TREE_MODE=0
      shift
      ;;
    --monitor)
      MONITOR=1
      shift
      ;;
    --sim-hz)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --sim-hz requires a value." >&2
        exit 2
      fi
      SIM_HZ="$2"
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
      if [[ "$1" != *":="* ]] && is_meter_value "$1"; then
        if (( ${#POSITIONAL_M[@]} >= 3 )); then
          echo "Unexpected extra map coordinate: $1. Pass launch args after --." >&2
          exit 2
        fi
        POSITIONAL_M+=("$1")
      else
        LAUNCH_ARGS+=("$1")
      fi
      shift
      ;;
  esac
done

if (( ${#POSITIONAL_M[@]} != 3 )); then
  echo "[ERROR] ${SCRIPT_NAME} requires FACE_X_M FACE_Y_M FACE_Z_M in map frame." >&2
  usage >&2
  exit 2
fi

FACE_MAP_X_M="${POSITIONAL_M[0]}"
FACE_MAP_Y_M="${POSITIONAL_M[1]}"
FACE_MAP_Z_M="${POSITIONAL_M[2]}"
TEAM_IS_RED="$(team_is_red_bool)"
read -r SIM_POS_X_CM SIM_POS_Y_CM < <(buff_outpost_position_cm)

source_ros_workspace "${ROOT_DIR}"
require_sentry_msgs_for_behavior_tree
cleanup_existing_stack "1" "/(gimbal_driver_node|behavior_tree_node|map_aim_point_node|map_path_to_game_path_node|target_rel_to_goal_pos_node)([[:space:]]|$)" "ros2 launch (behavior_tree|navi_tf_bridge) (competition_autoaim|sentry_all|outpost_regional_test|map_aim_point)\\.launch.py"

if (( USE_NOGATE == 1 )) && ! has_launch_arg_key "debug_bypass_is_start"; then
  LAUNCH_ARGS=("debug_bypass_is_start:=true" "${LAUNCH_ARGS[@]}")
fi
if (( USE_NOGATE == 0 )) && ! has_launch_arg_key "debug_bypass_is_start"; then
  LAUNCH_ARGS=("debug_bypass_is_start:=false" "${LAUNCH_ARGS[@]}")
fi
if (( FACE_MODE_SOLVER == 0 )) && ! has_launch_arg_key "use_face_mode_solver"; then
  LAUNCH_ARGS=("use_face_mode_solver:=false" "${LAUNCH_ARGS[@]}")
fi
if [[ "${TF_TREE_MODE}" == "1" ]] && ! has_launch_arg_key "use_tf_tree"; then
  LAUNCH_ARGS=("use_tf_tree:=true" "${LAUNCH_ARGS[@]}")
fi
if [[ "${TF_TREE_MODE}" == "0" ]] && ! has_launch_arg_key "use_tf_tree"; then
  LAUNCH_ARGS=("use_tf_tree:=false" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "publish_navi_goal"; then
  LAUNCH_ARGS=("publish_navi_goal:=false" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "navi_publish_goal_pose"; then
  LAUNCH_ARGS=("navi_publish_goal_pose:=false" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "use_navi_tf_bridge"; then
  LAUNCH_ARGS=("use_navi_tf_bridge:=false" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "face_mode_manual_target_enable"; then
  LAUNCH_ARGS=("face_mode_manual_target_enable:=true" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "face_mode_manual_target_frame"; then
  LAUNCH_ARGS=("face_mode_manual_target_frame:=map" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "face_mode_manual_target_x_m"; then
  LAUNCH_ARGS=("face_mode_manual_target_x_m:=${FACE_MAP_X_M}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "face_mode_manual_target_y_m"; then
  LAUNCH_ARGS=("face_mode_manual_target_y_m:=${FACE_MAP_Y_M}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "face_mode_manual_target_z_m"; then
  LAUNCH_ARGS=("face_mode_manual_target_z_m:=${FACE_MAP_Z_M}" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "face_mode_target_frame"; then
  LAUNCH_ARGS=("face_mode_target_frame:=map" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "face_mode_use_raw_goal_static_calibration"; then
  LAUNCH_ARGS=("face_mode_use_raw_goal_static_calibration:=false" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "use_gimbal"; then
  LAUNCH_ARGS=("use_gimbal:=true" "${LAUNCH_ARGS[@]}")
fi
if ! has_launch_arg_key "use_behavior_tree"; then
  LAUNCH_ARGS=("use_behavior_tree:=true" "${LAUNCH_ARGS[@]}")
fi

echo "[INFO] Formal Outpost simulator launch: behavior_tree/outpost_regional_test.launch.py -> sentry_all.launch.py" >&2
echo "[INFO] Sim team=${TEAM}; /ly/navi/position official_map_cm=(${SIM_POS_X_CM}, ${SIM_POS_Y_CM}); reached=true; reachable=true" >&2
echo "[INFO] FaceMode manual map target=(${FACE_MAP_X_M}, ${FACE_MAP_Y_M}, ${FACE_MAP_Z_M}) m" >&2
echo "[INFO] Chassis navigation output disabled: publish_navi_goal:=false, use_navi_tf_bridge:=false" >&2

ros2 launch behavior_tree outpost_regional_test.launch.py "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!
trap cleanup INT TERM EXIT

sleep 2
start_pub ros2 topic pub -r "${SIM_HZ}" /ly/friend/is_team_red std_msgs/msg/Bool "{data: ${TEAM_IS_RED}}"
start_pub ros2 topic pub -r "${SIM_HZ}" /ly/navi/reached std_msgs/msg/Bool "{data: true}"
start_pub ros2 topic pub -r "${SIM_HZ}" /ly/navi/reachable std_msgs/msg/Bool "{data: true}"
start_pub ros2 topic pub -r "${SIM_HZ}" /ly/navi/position gimbal_driver/msg/StampedUInt16MultiArray "{header: {frame_id: 'official_map'}, data: [${SIM_POS_X_CM}, ${SIM_POS_Y_CM}], map_frame: 'map', source_frame: '${SCRIPT_NAME}'}"

if (( MONITOR == 1 )); then
  monitor_formal_chain "${LAUNCH_PID}" &
  PUB_PIDS+=("$!")
fi

set +e
wait "${LAUNCH_PID}"
STATUS=$?
set -e
cleanup
exit "${STATUS}"
