#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
CLEANUP_EXISTING=1
OFFLINE_MODE=0
PROMPT_MODE=1
MODE_ARG=""
LAUNCH_ARGS=()

STACK_NODE_REGEX="/(gimbal_driver_node|detector_node|tracker_solver_node|predictor_node|outpost_hitter_node|buff_hitter_node|behavior_tree_node|mapper_node|fire_flip_test)([[:space:]]|$)"
STACK_LAUNCH_REGEX="ros2 launch behavior_tree sentry_all.launch.py"
BT_CONFIG_REGIONAL_REL="Scripts/ConfigJson/regional_competition.json"
BT_CONFIG_LEAGUE_REL="Scripts/ConfigJson/league_competition.json"
BT_CONFIG_SHOWCASE_REL="Scripts/ConfigJson/showcase_competition.json"
SELECTED_MODE_KIND=""
SELECTED_COMPETITION_PROFILE=""

: "${ROS_LOG_DIR:=/tmp/ros2_logs}"
mkdir -p "${ROS_LOG_DIR}"
export ROS_LOG_DIR

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--cleanup-existing|--no-cleanup-existing] [--offline] [--mode 1|2|3|league|regional|showcase] [--no-prompt] [-- <launch_args...>]

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
      PROMPT_MODE=0
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

normalize_competition_profile() {
  local raw="${1:-}"
  local normalized="${raw,,}"
  case "${normalized}" in
    1|league)
      echo "league"
      return 0
      ;;
    2|regional)
      echo "regional"
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

normalize_mode_kind() {
  local raw="${1:-}"
  local normalized="${raw,,}"
  case "${normalized}" in
    1|league)
      echo "league"
      return 0
      ;;
    2|regional)
      echo "regional"
      return 0
      ;;
    3|showcase|demo)
      echo "showcase"
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

competition_profile_for_mode() {
  local mode_kind="${1:-regional}"
  case "${mode_kind}" in
    league)
      echo "league"
      ;;
    regional|showcase)
      echo "regional"
      ;;
    *)
      return 1
      ;;
  esac
}

has_launch_arg_key() {
  local key="$1"
  local arg
  for arg in "${LAUNCH_ARGS[@]}"; do
    if [[ "${arg}" == "${key}:="* ]] || [[ "${arg}" == "--${key}:="* ]]; then
      return 0
    fi
  done
  return 1
}

launch_arg_value() {
  local key="$1"
  local arg
  for arg in "${LAUNCH_ARGS[@]}"; do
    if [[ "${arg}" == "${key}:="* ]]; then
      echo "${arg#${key}:=}"
      return 0
    fi
    if [[ "${arg}" == "--${key}:="* ]]; then
      echo "${arg#--${key}:=}"
      return 0
    fi
  done
  return 1
}

strip_launch_arg_key() {
  local key="$1"
  local filtered=()
  local arg
  for arg in "${LAUNCH_ARGS[@]}"; do
    if [[ "${arg}" == "${key}:="* ]] || [[ "${arg}" == "--${key}:="* ]]; then
      continue
    fi
    filtered+=("${arg}")
  done
  LAUNCH_ARGS=("${filtered[@]}")
}

select_competition_profile() {
  if has_launch_arg_key "competition_profile"; then
    local arg_value
    arg_value="$(launch_arg_value "competition_profile" || true)"
    if SELECTED_COMPETITION_PROFILE="$(normalize_competition_profile "${arg_value}")"; then
      SELECTED_MODE_KIND="${SELECTED_COMPETITION_PROFILE}"
      strip_launch_arg_key "competition_profile"
      LAUNCH_ARGS=("competition_profile:=${SELECTED_COMPETITION_PROFILE}" "${LAUNCH_ARGS[@]}")
      echo "[INFO] competition_profile from launch args: ${SELECTED_COMPETITION_PROFILE}" >&2
      return
    fi
    echo "[WARN] Invalid launch arg competition_profile: ${arg_value}. Re-selecting." >&2
    strip_launch_arg_key "competition_profile"
  fi

  if [[ -n "${MODE_ARG}" ]]; then
    if ! SELECTED_MODE_KIND="$(normalize_mode_kind "${MODE_ARG}")"; then
      echo "[ERROR] Invalid --mode: ${MODE_ARG}. Use 1|2|3|league|regional|showcase." >&2
      exit 2
    fi
    SELECTED_COMPETITION_PROFILE="$(competition_profile_for_mode "${SELECTED_MODE_KIND}")"
  fi

  if [[ -z "${SELECTED_MODE_KIND}" ]] && (( PROMPT_MODE == 1 )) && [[ -t 0 ]]; then
    while true; do
      echo "[PROMPT] Select startup mode:" >&2
      echo "  1) league" >&2
      echo "  2) regional" >&2
      echo "  3) showcase" >&2
      read -r -p "Input 1, 2 or 3 [default: 2]: " choice
      choice="${choice:-2}"
      if SELECTED_MODE_KIND="$(normalize_mode_kind "${choice}")"; then
        SELECTED_COMPETITION_PROFILE="$(competition_profile_for_mode "${SELECTED_MODE_KIND}")"
        break
      fi
      echo "[WARN] Invalid input: ${choice}" >&2
    done
  fi

  if [[ -z "${SELECTED_MODE_KIND}" ]]; then
    SELECTED_MODE_KIND="regional"
    SELECTED_COMPETITION_PROFILE="regional"
    echo "[INFO] Non-interactive mode: default mode:=regional competition_profile:=regional" >&2
  fi

  LAUNCH_ARGS=("competition_profile:=${SELECTED_COMPETITION_PROFILE}" "${LAUNCH_ARGS[@]}")
}

set_bt_config_by_profile() {
  if has_launch_arg_key "bt_config_file"; then
    echo "[INFO] bt_config_file provided by launch args: $(launch_arg_value bt_config_file || true)" >&2
    return
  fi

  case "${SELECTED_MODE_KIND}" in
    league)
      LAUNCH_ARGS=("bt_config_file:=${BT_CONFIG_LEAGUE_REL}" "${LAUNCH_ARGS[@]}")
      ;;
    showcase)
      LAUNCH_ARGS=("bt_config_file:=${BT_CONFIG_SHOWCASE_REL}" "${LAUNCH_ARGS[@]}")
      ;;
    *)
      LAUNCH_ARGS=("bt_config_file:=${BT_CONFIG_REGIONAL_REL}" "${LAUNCH_ARGS[@]}")
      ;;
  esac
}

if (( OFFLINE_MODE == 1 )); then
  if ! has_launch_arg_key "offline"; then
    LAUNCH_ARGS=("offline:=true" "${LAUNCH_ARGS[@]}")
  fi
  echo "[INFO] Offline mode enabled: passing launch arg offline:=true" >&2
fi

select_competition_profile
set_bt_config_by_profile

echo "[INFO] Effective competition_profile: ${SELECTED_COMPETITION_PROFILE}" >&2
echo "[INFO] Effective mode: ${SELECTED_MODE_KIND}" >&2
echo "[INFO] Effective bt_config_file: $(launch_arg_value bt_config_file || true)" >&2
if has_launch_arg_key "config_file"; then
  echo "[INFO] Effective config_file (from launch args): $(launch_arg_value config_file || true)" >&2
else
  echo "[INFO] Effective config_file: <launch default detector/config/auto_aim_config.yaml>" >&2
fi

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
