#!/usr/bin/env bash

# Quick viewer for /ly/navi/position.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

TOPIC="${NAVI_POSITION_TOPIC:-/ly/navi/position}"
FIELD_ONLY=1
ONCE=0
SHOW_INFO=1
CHECK_ONLY=0
CHECK_TF=1
START_CHAIN=0
KEEP_CHAIN=0
START_CLEANUP_EXISTING=1
START_OFFLINE=0
TIMEOUT_SEC=""
START_WAIT_SEC="${NAVI_POSITION_START_WAIT:-30}"
TF_TIMEOUT_SEC="${NAVI_POSITION_TF_TIMEOUT:-2}"
MAP_FRAME="${NAVI_POSITION_MAP_FRAME:-map}"
BASE_FRAME="${NAVI_POSITION_BASE_FRAME:-base_link}"
FALLBACK_BASE_FRAME="${NAVI_POSITION_FALLBACK_BASE_FRAME:-baselink}"
START_MODE="${NAVI_POSITION_START_MODE:-}"
START_LOG="${NAVI_POSITION_START_LOG:-${ROOT_DIR}/log/navi_position/sentry_all.log}"
START_PID=""
START_EXTRA_ARGS=()

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [options]

Purpose:
  Source the workspace and echo /ly/navi/position.
  Default output is only the data field: [official_map_x_cm, official_map_y_cm].

Options:
  --topic <topic>       Topic to echo. Default: /ly/navi/position
  --check               Only check node/topic/TF readiness, then exit
  --start-chain, --start, --bringup
                        Start sentry_all first, force use_navi_tf_bridge:=true,
                        wait for /ly/navi/position, then echo/check
  --keep-chain          Leave the sentry_all chain running after this script exits
  --start-wait <sec>    Wait time for the first position message. Default: ${START_WAIT_SEC}
  --start-log <path>    sentry_all log path. Default: ${START_LOG}
  --mode <mode>         Pass --mode to sentry_all when --start-chain is used
  --offline             Pass --offline to sentry_all when --start-chain is used
  --cleanup-existing    Let sentry_all clean existing launch tree first. Default
  --no-cleanup-existing Keep existing sentry_all launch tree
  --once                Print one message and exit
  --timeout <seconds>   Stop echo after this many seconds
  --tf-timeout <sec>    Timeout for each TF probe. Default: ${TF_TIMEOUT_SEC}
  --map-frame <frame>   TF map frame to probe. Default: ${MAP_FRAME}
  --base-frame <frame>  TF base frame to probe. Default: ${BASE_FRAME}
  --fallback-base-frame <frame>
                        Fallback TF base frame to probe. Default: ${FALLBACK_BASE_FRAME}
  --full                Print full UInt16MultiArray message
  --no-info             Do not print topic info before echo
  --no-tf-check         Skip map <- base_link TF readiness probe
  --help, -h            Show this help
  -- <launch_args...>   Extra sentry_all launch args for --start-chain

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --check
  ./${SCRIPT_NAME} --start-chain --once
  ./${SCRIPT_NAME} --start-chain --mode regional --timeout 5
  ./${SCRIPT_NAME} --once
  ./${SCRIPT_NAME} --timeout 5
  ./${SCRIPT_NAME} --full
EOF
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "${value}" ]]; then
    echo "[ERROR] ${option} requires a value." >&2
    usage >&2
    exit 2
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --topic)
      require_value "$1" "${2:-}"
      TOPIC="${2:-}"
      shift 2
      ;;
    --check)
      CHECK_ONLY=1
      shift
      ;;
    --start-chain|--start|--bringup)
      START_CHAIN=1
      shift
      ;;
    --keep-chain)
      KEEP_CHAIN=1
      shift
      ;;
    --start-wait)
      require_value "$1" "${2:-}"
      START_WAIT_SEC="${2:-}"
      shift 2
      ;;
    --start-log)
      require_value "$1" "${2:-}"
      START_LOG="${2:-}"
      shift 2
      ;;
    --mode)
      require_value "$1" "${2:-}"
      START_MODE="${2:-}"
      shift 2
      ;;
    --offline)
      START_OFFLINE=1
      shift
      ;;
    --cleanup-existing)
      START_CLEANUP_EXISTING=1
      shift
      ;;
    --no-cleanup-existing)
      START_CLEANUP_EXISTING=0
      shift
      ;;
    --once)
      ONCE=1
      shift
      ;;
    --timeout)
      require_value "$1" "${2:-}"
      TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --tf-timeout)
      require_value "$1" "${2:-}"
      TF_TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --map-frame)
      require_value "$1" "${2:-}"
      MAP_FRAME="${2:-}"
      shift 2
      ;;
    --base-frame)
      require_value "$1" "${2:-}"
      BASE_FRAME="${2:-}"
      shift 2
      ;;
    --fallback-base-frame)
      require_value "$1" "${2:-}"
      FALLBACK_BASE_FRAME="${2:-}"
      shift 2
      ;;
    --full)
      FIELD_ONLY=0
      shift
      ;;
    --no-info)
      SHOW_INFO=0
      shift
      ;;
    --no-tf-check)
      CHECK_TF=0
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      START_EXTRA_ARGS=("$@")
      break
      ;;
    *)
      echo "[ERROR] Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${TOPIC}" ]]; then
  echo "[ERROR] --topic cannot be empty." >&2
  exit 2
fi

if [[ -n "${TIMEOUT_SEC}" && ! "${TIMEOUT_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "[ERROR] --timeout must be a positive number of seconds." >&2
  exit 2
fi
if [[ -z "${START_WAIT_SEC}" || ! "${START_WAIT_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "[ERROR] --start-wait must be a positive number of seconds." >&2
  exit 2
fi
if [[ -z "${TF_TIMEOUT_SEC}" || ! "${TF_TIMEOUT_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "[ERROR] --tf-timeout must be a positive number of seconds." >&2
  exit 2
fi
if [[ -z "${MAP_FRAME}" ]]; then
  echo "[ERROR] --map-frame cannot be empty." >&2
  exit 2
fi
if [[ -z "${BASE_FRAME}" ]]; then
  echo "[ERROR] --base-frame cannot be empty." >&2
  exit 2
fi
if (( START_CHAIN == 0 && ${#START_EXTRA_ARGS[@]} > 0 )); then
  echo "[WARN] Extra launch args after -- are ignored unless --start-chain is set." >&2
fi
if (( START_CHAIN == 1 && -z "${START_LOG}" )); then
  echo "[ERROR] --start-log cannot be empty." >&2
  exit 2
fi

source_ros_workspace "${ROOT_DIR}"

has_start_launch_arg_key() {
  local key="$1"
  local arg
  for arg in "${START_EXTRA_ARGS[@]}"; do
    if [[ "${arg}" == "${key}:="* ]]; then
      return 0
    fi
  done
  return 1
}

cleanup_started_chain() {
  if [[ -n "${START_PID}" ]] && kill -0 "${START_PID}" 2>/dev/null; then
    echo "[INFO] Stopping sentry_all chain pid=${START_PID}"
    kill -INT "${START_PID}" 2>/dev/null || true
    wait "${START_PID}" 2>/dev/null || true
  fi
  START_PID=""
}

start_sentry_all_chain() {
  local -a start_cmd=("${ROOT_DIR}/scripts/start/sentry_all.sh")

  if (( START_CLEANUP_EXISTING == 1 )); then
    start_cmd+=("--cleanup-existing")
  else
    start_cmd+=("--no-cleanup-existing")
  fi
  start_cmd+=("--no-prompt")
  if (( START_OFFLINE == 1 )); then
    start_cmd+=("--offline")
  fi
  if [[ -n "${START_MODE}" ]]; then
    start_cmd+=("--mode" "${START_MODE}")
  fi
  start_cmd+=("--")
  if ! has_start_launch_arg_key "use_navi_tf_bridge"; then
    start_cmd+=("use_navi_tf_bridge:=true")
  fi
  start_cmd+=("${START_EXTRA_ARGS[@]}")

  mkdir -p "$(dirname "${START_LOG}")"
  : > "${START_LOG}"

  printf '[INFO] Starting sentry_all chain:'
  printf ' %q' "${start_cmd[@]}"
  printf '\n'
  echo "[INFO] sentry_all log: ${START_LOG}"
  "${start_cmd[@]}" >"${START_LOG}" 2>&1 &
  START_PID="$!"
  echo "[INFO] sentry_all pid: ${START_PID}"

  if (( KEEP_CHAIN == 0 )); then
    trap cleanup_started_chain EXIT
    trap 'cleanup_started_chain; exit 130' INT
    trap 'cleanup_started_chain; exit 143' TERM
  else
    echo "[INFO] --keep-chain set; this script will not stop sentry_all on exit."
  fi
}

wait_for_position_message() {
  local output_file
  local error_file

  if [[ "${START_WAIT_SEC}" == "0" || "${START_WAIT_SEC}" == "0.0" ]]; then
    return 0
  fi

  output_file="$(mktemp)"
  error_file="$(mktemp)"
  echo "[INFO] Waiting up to ${START_WAIT_SEC}s for first ${TOPIC} message..."
  if timeout "${START_WAIT_SEC}" ros2 topic echo --once --field data "${TOPIC}" \
    >"${output_file}" 2>"${error_file}"
  then
    echo "[INFO] First ${TOPIC} message:"
    sed 's/^/  /' "${output_file}"
  else
    echo "[WARN] Did not receive ${TOPIC} within ${START_WAIT_SEC}s." >&2
    if [[ -s "${error_file}" ]]; then
      sed 's/^/[WARN] ros2 topic echo: /' "${error_file}" >&2
    fi
    echo "[WARN] Check sentry_all log: ${START_LOG}" >&2
  fi
  rm -f "${output_file}" "${error_file}"
}

check_bridge_node() {
  local node_list
  node_list="$(ros2 node list 2>/dev/null || true)"
  if printf '%s\n' "${node_list}" | grep -Eq '(^|/)target_rel_to_goal_pos_node$'; then
    echo "[INFO] Node: target_rel_to_goal_pos_node is running"
  else
    echo "[WARN] Node target_rel_to_goal_pos_node not found. sentry_all may not have launched navi_tf_bridge." >&2
  fi
}

check_topic() {
  echo "[INFO] Topic: ${TOPIC}"
  if ros2 topic type "${TOPIC}" >/dev/null 2>&1; then
    echo "[INFO] Type: $(ros2 topic type "${TOPIC}")"
  else
    echo "[WARN] Topic type is not available yet. Is navi_tf_bridge running?" >&2
  fi
  ros2 topic info "${TOPIC}" || true
}

check_tf_pair() {
  local source_frame="$1"
  local tf_output

  if [[ -z "${source_frame}" ]]; then
    return 1
  fi

  tf_output="$(
    timeout "${TF_TIMEOUT_SEC}" ros2 run tf2_ros tf2_echo "${MAP_FRAME}" "${source_frame}" 2>&1 || true
  )"
  if printf '%s\n' "${tf_output}" | grep -q "Translation:"; then
    echo "[INFO] TF: ${MAP_FRAME} <- ${source_frame} OK"
    return 0
  fi

  echo "[WARN] TF ${MAP_FRAME} <- ${source_frame} not available within ${TF_TIMEOUT_SEC}s" >&2
  return 1
}

check_tf() {
  local ok=1

  if check_tf_pair "${BASE_FRAME}"; then
    ok=0
  fi
  if [[ -n "${FALLBACK_BASE_FRAME}" && "${FALLBACK_BASE_FRAME}" != "${BASE_FRAME}" ]]; then
    if check_tf_pair "${FALLBACK_BASE_FRAME}"; then
      ok=0
    fi
  fi

  if (( ok != 0 )); then
    echo "[WARN] /ly/navi/position needs one valid TF path: ${MAP_FRAME} <- ${BASE_FRAME} or fallback." >&2
  fi
}

if (( START_CHAIN == 1 )); then
  start_sentry_all_chain
  wait_for_position_message
fi

if (( SHOW_INFO == 1 || CHECK_ONLY == 1 )); then
  check_bridge_node
  check_topic
  if (( CHECK_TF == 1 )); then
    check_tf
  fi
  echo
fi

if (( CHECK_ONLY == 1 )); then
  exit 0
fi

cmd=(ros2 topic echo)
if (( ONCE == 1 )); then
  cmd+=(--once)
fi
if (( FIELD_ONLY == 1 )); then
  cmd+=(--field data)
fi
cmd+=("${TOPIC}")

echo "[INFO] Echo command: ${cmd[*]}"
if [[ -n "${TIMEOUT_SEC}" ]]; then
  timeout "${TIMEOUT_SEC}" "${cmd[@]}"
else
  "${cmd[@]}"
fi
