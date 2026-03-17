#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
LAUNCH_ARGS=()

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [start_sentry_all.sh options] [-- <launch_args...>]

Purpose:
  Thin wrapper for behavior_tree/sentry_all.launch.py with debug_bypass_is_start:=true.
EOF
}

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
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

exec "${ROOT_DIR}/scripts/launch/start_sentry_all.sh" \
  "${LAUNCH_ARGS[@]}" \
  -- \
  "debug_bypass_is_start:=true"
