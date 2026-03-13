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
  Dedicated showcase/debug entry for sentry demo.
  Fixed defaults:
    --mode 3 --no-prompt
  Default gate behavior:
    --nogate (inject debug_bypass_is_start:=true)

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --offline
  ./${SCRIPT_NAME} --with-gate
  ./${SCRIPT_NAME} -- use_buff:=false use_outpost:=false
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
  echo "[ERROR] ${SCRIPT_NAME} fixes showcase mode. Do not pass competition_profile:= or bt_config_file:=." >&2
  echo "[ERROR] Use ./scripts/start_sentry_all.sh for manual profile/config override." >&2
  exit 2
fi

if (( USE_NOGATE == 1 )); then
  TARGET_SCRIPT="${ROOT_DIR}/scripts/start_sentry_all_nogate.sh"
else
  TARGET_SCRIPT="${ROOT_DIR}/scripts/start_sentry_all.sh"
fi

cd "${ROOT_DIR}"

exec "${TARGET_SCRIPT}" \
  "${START_ARGS[@]}" \
  --mode 3 \
  --no-prompt \
  -- \
  "${LAUNCH_ARGS[@]}"
