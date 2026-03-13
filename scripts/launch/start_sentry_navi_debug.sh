#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
USE_NOGATE=1
START_ARGS=()
LAUNCH_ARGS=()

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--nogate|--with-gate] [start_sentry_all.sh options] [-- <launch_args...>]

Purpose:
  Dedicated navigation debug entry.
  Default behavior:
    - behavior_tree only
    - regional mode
    - bt_config_file:=Scripts/ConfigJson/navi_debug_competition.json
    - default nogate

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --with-gate
  ./${SCRIPT_NAME} -- use_gimbal:=true
EOF
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

append_default_launch_arg() {
  local key="$1"
  local value="$2"
  if ! has_launch_arg_key "${key}"; then
    LAUNCH_ARGS+=("${key}:=${value}")
  fi
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
      START_ARGS+=("$1")
      shift
      ;;
  esac
done

if has_launch_arg_key "competition_profile" || has_launch_arg_key "bt_config_file"; then
  echo "[ERROR] ${SCRIPT_NAME} fixes navigation debug profile/config. Do not pass competition_profile:= or bt_config_file:=." >&2
  exit 2
fi

if (( USE_NOGATE == 1 )); then
  TARGET_SCRIPT="${ROOT_DIR}/scripts/start_sentry_all_nogate.sh"
else
  TARGET_SCRIPT="${ROOT_DIR}/scripts/start_sentry_all.sh"
fi

append_default_launch_arg "use_gimbal" "false"
append_default_launch_arg "use_detector" "false"
append_default_launch_arg "use_tracker" "false"
append_default_launch_arg "use_predictor" "false"
append_default_launch_arg "use_outpost" "false"
append_default_launch_arg "use_buff" "false"
append_default_launch_arg "use_behavior_tree" "true"
append_default_launch_arg "bt_config_file" "Scripts/ConfigJson/navi_debug_competition.json"

cd "${ROOT_DIR}"

exec "${TARGET_SCRIPT}" \
  "${START_ARGS[@]}" \
  --mode regional \
  --no-prompt \
  -- \
  "${LAUNCH_ARGS[@]}"
