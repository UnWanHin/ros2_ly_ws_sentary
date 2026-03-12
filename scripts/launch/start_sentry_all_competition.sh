#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [start_sentry_all.sh options] [-- <launch_args...>]

Examples:
  # 兼容旧入口，实际逻辑由 start_sentry_all.sh 处理
  ./${SCRIPT_NAME}

  # 指定模式（非交互）
  ./${SCRIPT_NAME} --mode 1 --no-prompt

  # 仍可传入 start_sentry_all.sh 的选项
  ./${SCRIPT_NAME} --offline

  # 追加 launch 参数
  ./${SCRIPT_NAME} -- use_buff:=false use_outpost:=false
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

exec "${ROOT_DIR}/scripts/start_sentry_all.sh" "${START_ARGS[@]}" -- "${LAUNCH_ARGS[@]}"
