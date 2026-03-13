#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [start_sentry_all.sh options] [-- <launch_args...>]

Purpose:
  Launch sentry stack with is_start gate bypassed for debug/link testing.
  This script always injects:
    debug_bypass_is_start:=true

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --mode 1 --no-prompt
  ./${SCRIPT_NAME} --mode 3 --no-prompt
  ./${SCRIPT_NAME} --offline
  ./${SCRIPT_NAME} -- wait_for_game_start_timeout_sec:=8
EOF
}

START_ARGS=()
LAUNCH_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
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

cd "${ROOT_DIR}"

exec "${ROOT_DIR}/scripts/start_sentry_all.sh" \
  "${START_ARGS[@]}" \
  -- \
  "debug_bypass_is_start:=true" \
  "${LAUNCH_ARGS[@]}"
