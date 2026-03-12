#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

DEFAULT_CONFIG_REL="scripts/config/auto_aim_config_competition.yaml"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [start_sentry_all.sh options] [-- <launch_args...>]

Examples:
  # 直接使用比赛版配置启动
  ./${SCRIPT_NAME}

  # 仍可传入 start_sentry_all.sh 的选项
  ./${SCRIPT_NAME} --offline

  # 追加 launch 参数（不会覆盖默认比赛配置）
  ./${SCRIPT_NAME} -- use_buff:=false use_outpost:=false

  # 显式指定 config_file（将覆盖默认比赛配置）
  ./${SCRIPT_NAME} -- --config_file:=/abs/path/your_config.yaml
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

cd "${ROOT_DIR}"

if ! has_launch_arg_key "config_file"; then
  LAUNCH_ARGS=("config_file:=${DEFAULT_CONFIG_REL}" "${LAUNCH_ARGS[@]}")
fi

exec "${ROOT_DIR}/scripts/start_sentry_all.sh" "${START_ARGS[@]}" -- "${LAUNCH_ARGS[@]}"
