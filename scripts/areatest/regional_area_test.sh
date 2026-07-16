#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# Single-area regional BT runner. It keeps the real sentry_all/regional chain
# and only changes the DecisionAutonomy candidate config used by behavior_tree.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF2
Usage:
  ${SCRIPT_NAME} <base|highland|roadland|central> [options] [-- <launch_args...>]

Single-area wrappers:
  ./scripts/areatest/regional_base.sh
  ./scripts/areatest/regional_highland.sh
  ./scripts/areatest/regional_roadland.sh
  ./scripts/areatest/regional_central.sh

Options:
  --pure                 Only run the area's basic route/hold task. Disable firing, chase, posture, and aim perception nodes.
  --cleanup-existing     Clean previous sentry_all launch tree before starting (default from start_sentry_all.sh)
  --no-cleanup-existing  Keep existing sentry_all launch tree
  --offline              Pass offline:=true to sentry_all
  --fake-referee         Publish healthy /ly/game/all and /ly/friend/ammo_left for bench tests
  --hp VALUE             Fake referee self health, default 450
  --ammo VALUE           Fake referee ammo_left, default 200
  --red|--blue           With --fake-referee, publish /ly/friend/is_team_red
  --mock-gimbal-state    Publish mock /ly/gimbal/angles for bench route tests
  --no-mock-gimbal-state Do not publish mock gimbal angles; require real /ly/gimbal/angles
  --mock-gimbal-yaw DEG  Mock gimbal yaw, default 0.0
  --mock-gimbal-pitch DEG Mock gimbal pitch, default 0.0

Examples:
  ./scripts/areatest/regional_base.sh --pure
  ./scripts/areatest/regional_central.sh --pure --fake-referee
  ./scripts/areatest/regional_roadland.sh
EOF2
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

add_launch_arg_if_missing() {
  local key="$1"
  local value="$2"
  if ! has_launch_arg_key "${key}"; then
    LAUNCH_ARGS=("${key}:=${value}" "${LAUNCH_ARGS[@]}")
  fi
}

if [[ $# -lt 1 ]]; then
  usage
  exit 2
fi

AREA="$1"
shift

CONFIG_DIR="${ROOT_DIR}/src/behavior_tree/Scripts/ConfigJson/regional/test"
BT_CONFIG_TEMPLATE="${CONFIG_DIR}/regional_area_template.json"
BT_PROFILE_GENERATOR="${ROOT_DIR}/scripts/areatest/regional_area_profile.py"
case "${AREA}" in
  base|my_base)
    AREA_LABEL="my_base"
    ;;
  highland|my_highland)
    AREA_LABEL="my_highland"
    ;;
  roadland|my_roadland)
    AREA_LABEL="my_roadland"
    ;;
  central|common_central)
    AREA_LABEL="common_central"
    ;;
  --help|-h)
    usage
    exit 0
    ;;
  *)
    echo "[ERROR] Unknown area '${AREA}'. Expected base/highland/roadland/central." >&2
    usage >&2
    exit 2
    ;;
esac

START_ARGS=(--mode regional --no-prompt)
LAUNCH_ARGS=()
PURE_MODE=0
FAKE_REFEREE=0
FAKE_HP=450
FAKE_AMMO=200
FAKE_TEAM_RED=""
MOCK_GIMBAL_STATE="auto"
MOCK_GIMBAL_YAW=0.0
MOCK_GIMBAL_PITCH=0.0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pure)
      PURE_MODE=1
      shift
      ;;
    --cleanup-existing|--no-cleanup-existing|--offline)
      START_ARGS+=("$1")
      shift
      ;;
    --fake-referee)
      FAKE_REFEREE=1
      shift
      ;;
    --no-fake-referee)
      FAKE_REFEREE=0
      shift
      ;;
    --hp)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --hp requires a value." >&2
        exit 2
      fi
      FAKE_HP="$2"
      shift 2
      ;;
    --ammo)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --ammo requires a value." >&2
        exit 2
      fi
      FAKE_AMMO="$2"
      shift 2
      ;;
    --red)
      FAKE_TEAM_RED="true"
      shift
      ;;
    --blue)
      FAKE_TEAM_RED="false"
      shift
      ;;
    --mock-gimbal-state)
      MOCK_GIMBAL_STATE="true"
      shift
      ;;
    --no-mock-gimbal-state|--real-gimbal-state)
      MOCK_GIMBAL_STATE="false"
      shift
      ;;
    --mock-gimbal-yaw)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --mock-gimbal-yaw requires a value." >&2
        exit 2
      fi
      MOCK_GIMBAL_YAW="$2"
      shift 2
      ;;
    --mock-gimbal-pitch)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --mock-gimbal-pitch requires a value." >&2
        exit 2
      fi
      MOCK_GIMBAL_PITCH="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      LAUNCH_ARGS+=("$@")
      break
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

if ! [[ "${FAKE_HP}" =~ ^[0-9]+$ ]]; then
  echo "[ERROR] --hp must be a non-negative integer." >&2
  exit 2
fi
if ! [[ "${FAKE_AMMO}" =~ ^[0-9]+$ ]]; then
  echo "[ERROR] --ammo must be a non-negative integer." >&2
  exit 2
fi

if [[ ! -f "${BT_CONFIG_TEMPLATE}" ]]; then
  echo "[ERROR] BT config template not found: ${BT_CONFIG_TEMPLATE}" >&2
  exit 1
fi
if [[ ! -f "${BT_PROFILE_GENERATOR}" ]]; then
  echo "[ERROR] BT profile generator not found: ${BT_PROFILE_GENERATOR}" >&2
  exit 1
fi

CHILD_PIDS=()
LAUNCH_PID=""
BT_CONFIG_FILE=""

cleanup_children() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  fi
  if (( ${#CHILD_PIDS[@]} > 0 )); then
    local pid
    for pid in "${CHILD_PIDS[@]}"; do
      kill -TERM "${pid}" 2>/dev/null || true
    done
  fi
  if [[ -n "${LAUNCH_PID}" ]]; then
    wait "${LAUNCH_PID}" 2>/dev/null || true
    LAUNCH_PID=""
  fi
  if (( ${#CHILD_PIDS[@]} > 0 )); then
    wait "${CHILD_PIDS[@]}" 2>/dev/null || true
  fi
  if [[ -n "${BT_CONFIG_FILE}" ]]; then
    rm -f "${BT_CONFIG_FILE}"
  fi
}

BT_CONFIG_FILE="$(mktemp "/tmp/ly_regional_area_${AREA_LABEL}_XXXXXX.json")"
trap cleanup_children EXIT INT TERM
BT_PROFILE_ARGS=(
  --template "${BT_CONFIG_TEMPLATE}"
  --area "${AREA_LABEL}"
  --output "${BT_CONFIG_FILE}"
)
if (( PURE_MODE == 1 )); then
  BT_PROFILE_ARGS+=(--pure)
fi
python3 "${BT_PROFILE_GENERATOR}" "${BT_PROFILE_ARGS[@]}"

add_launch_arg_if_missing "bt_config_file" "${BT_CONFIG_FILE}"
add_launch_arg_if_missing "debug_bypass_is_start" "true"
add_launch_arg_if_missing "wait_for_game_start_timeout_sec" "0"
add_launch_arg_if_missing "competition_profile" "regional"

start_fake_referee_publishers() {
  source_ros_workspace "${ROOT_DIR}"

  ros2 topic pub -r 5 /ly/game/all gimbal_driver/msg/GameData \
    "{gamecode: 0, ammoleft: ${FAKE_AMMO}, timeleft: 360, selfhealth: ${FAKE_HP}, exteventdata: 0}" \
    >/dev/null 2>&1 &
  CHILD_PIDS+=("$!")

  ros2 topic pub -r 5 /ly/friend/ammo_left std_msgs/msg/UInt16 \
    "{data: ${FAKE_AMMO}}" \
    >/dev/null 2>&1 &
  CHILD_PIDS+=("$!")

  ros2 topic pub -r 2 /ly/game/is_start std_msgs/msg/Bool \
    "{data: true}" \
    >/dev/null 2>&1 &
  CHILD_PIDS+=("$!")

  if [[ -n "${FAKE_TEAM_RED}" ]]; then
    ros2 topic pub -r 2 /ly/friend/is_team_red std_msgs/msg/Bool \
      "{data: ${FAKE_TEAM_RED}}" \
      >/dev/null 2>&1 &
    CHILD_PIDS+=("$!")
  fi
}

start_mock_gimbal_state() {
  source_ros_workspace "${ROOT_DIR}"

  ros2 run navi_tf_bridge mock_gimbal_state_node --ros-args \
    -p gimbal_angles_topic:=/ly/gimbal/angles \
    -p yaw_deg:="${MOCK_GIMBAL_YAW}" \
    -p pitch_deg:="${MOCK_GIMBAL_PITCH}" \
    -p publish_big_yaw:=false \
    -p publish_hz:=30.0 \
    >/dev/null 2>&1 &
  CHILD_PIDS+=("$!")
}

echo "[INFO] Regional single-area test: area=${AREA_LABEL}"
echo "[INFO] BT config: ${BT_CONFIG_FILE}"
echo "[INFO] Uses real sentry_all regional chain; this is not navi_debug and not a direct goal publisher."
if (( PURE_MODE == 1 )); then
  echo "[INFO] Pure mode enabled: firing/chase/posture disabled; formal sentry_all does not start internal vision nodes."
fi
SHOULD_MOCK_GIMBAL=0
if [[ "${MOCK_GIMBAL_STATE}" == "true" ]] ||
   [[ "${MOCK_GIMBAL_STATE}" == "auto" && "${PURE_MODE}" == "1" ]]; then
  SHOULD_MOCK_GIMBAL=1
fi
if (( FAKE_REFEREE == 1 )); then
  echo "[INFO] Fake referee enabled: hp=${FAKE_HP}, ammo=${FAKE_AMMO}"
fi
if (( SHOULD_MOCK_GIMBAL == 1 )); then
  echo "[INFO] Mock gimbal state enabled: yaw=${MOCK_GIMBAL_YAW}, pitch=${MOCK_GIMBAL_PITCH}"
fi
if (( FAKE_REFEREE == 1 || SHOULD_MOCK_GIMBAL == 1 )); then
  if (( FAKE_REFEREE == 1 )); then
    start_fake_referee_publishers
  fi
  if (( SHOULD_MOCK_GIMBAL == 1 )); then
    start_mock_gimbal_state
  fi
  "${ROOT_DIR}/scripts/launch/start_sentry_all.sh" "${START_ARGS[@]}" -- "${LAUNCH_ARGS[@]}" &
  LAUNCH_PID="$!"
  set +e
  wait "${LAUNCH_PID}"
  STATUS="$?"
  set -e
  exit "${STATUS}"
else
  "${ROOT_DIR}/scripts/launch/start_sentry_all.sh" "${START_ARGS[@]}" -- "${LAUNCH_ARGS[@]}"
fi
