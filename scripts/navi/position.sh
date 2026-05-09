#!/usr/bin/env bash

# Quick viewer for /ly/navi/position.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

TOPIC="${NAVI_POSITION_TOPIC:-/ly/navi/position}"
FIELD_ONLY=1
ONCE=0
SHOW_INFO=1
TIMEOUT_SEC=""

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
  --once                Print one message and exit
  --timeout <seconds>   Stop echo after this many seconds
  --full                Print full UInt16MultiArray message
  --no-info             Do not print topic info before echo
  --help, -h            Show this help

Examples:
  ./${SCRIPT_NAME}
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
    --once)
      ONCE=1
      shift
      ;;
    --timeout)
      require_value "$1" "${2:-}"
      TIMEOUT_SEC="${2:-}"
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
    --help|-h)
      usage
      exit 0
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

source_ros_workspace "${ROOT_DIR}"

if (( SHOW_INFO == 1 )); then
  echo "[INFO] Topic: ${TOPIC}"
  if ros2 topic type "${TOPIC}" >/dev/null 2>&1; then
    echo "[INFO] Type: $(ros2 topic type "${TOPIC}")"
  else
    echo "[WARN] Topic type is not available yet. Is navi_tf_bridge running?" >&2
  fi
  ros2 topic info "${TOPIC}" || true
  echo
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
