#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"
CLEANUP_EXISTING=1
OFFLINE_MODE=0
MODE_ARG=""
LAUNCH_ARGS=()
DEFAULT_BASE_CONFIG_FILE="${ROOT_DIR}/config/base_config.yaml"
DEFAULT_DETECTOR_CONFIG_FILE="${ROOT_DIR}/src/detector/config/detector_config.yaml"
DEFAULT_PREDICTOR_CONFIG_FILE="${ROOT_DIR}/src/predictor/config/predictor_config.yaml"
DEFAULT_OUTPOST_CONFIG_FILE="${ROOT_DIR}/src/outpost_hitter/config/outpost_config.yaml"
DEFAULT_BUFF_CONFIG_FILE="${ROOT_DIR}/src/buff_hitter/config/buff_config.yaml"
DEFAULT_OVERRIDE_CONFIG_FILE="${ROOT_DIR}/config/override_config.yaml"
DEFAULT_AREA_MANAGER_CONFIG_FILE="${ROOT_DIR}/config/AreaManager.yaml"
DEFAULT_COMMON_CONFIG_FILE="${ROOT_DIR}/config/common.yaml"

STACK_LAUNCH_REGEX="ros2 launch behavior_tree sentry_all.launch.py"

# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/lib/ros_launch_common.sh"

usage() {
  cat <<EOF2
Usage:
  ${SCRIPT_NAME} [--cleanup-existing|--no-cleanup-existing] [--offline] [--mode 1|2|3|league|regional|regional_simple|showcase] [-- <launch_args...>]

Examples:
  ./${SCRIPT_NAME}
  ./${SCRIPT_NAME} --no-cleanup-existing
  ./${SCRIPT_NAME} --offline
  ./${SCRIPT_NAME} --mode 1
  ./${SCRIPT_NAME} --mode regional --no-prompt
  ./${SCRIPT_NAME} --mode regional_simple --no-prompt
  ./${SCRIPT_NAME} --mode 3 --no-prompt
  ./${SCRIPT_NAME} -- use_buff:=false use_outpost:=false
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

read_common_bt_log_dir() {
  local config_file="$1"
  [[ -f "${config_file}" ]] || return 1

  local value=""
  value="$(read_common_scalar "${config_file}" "bt_log_dir")" || return 1
  [[ -n "${value}" ]] || return 1

  if [[ "${value}" == "~" ]]; then
    value="${HOME}"
  elif [[ "${value}" == "~/"* ]]; then
    value="${HOME}/${value#"~/"}"
  fi

  printf '%s\n' "${value}"
}

read_common_bt_file_log_enable() {
  local config_file="$1"
  [[ -f "${config_file}" ]] || return 1

  local value=""
  value="$(read_common_scalar "${config_file}" "bt_file_log_enable")" || return 1
  read_common_bool_value "${value}"
}

read_common_aim_timer_log_enable() {
  local config_file="$1"
  [[ -f "${config_file}" ]] || return 1

  local value=""
  value="$(read_common_scalar "${config_file}" "aim_timer_log_enable")" || return 1
  read_common_bool_value "${value}"
}

read_common_aim_timer_log_dir() {
  local config_file="$1"
  [[ -f "${config_file}" ]] || return 1

  local value=""
  value="$(read_common_scalar "${config_file}" "aim_timer_log_dir")" || return 1
  [[ -n "${value}" ]] || return 1

  if [[ "${value}" == "~" ]]; then
    value="${HOME}"
  elif [[ "${value}" == "~/"* ]]; then
    value="${HOME}/${value#"~/"}"
  fi

  printf '%s\n' "${value}"
}

read_common_bool_value() {
  local value="$1"
  value="$(printf '%s' "${value}" | tr '[:upper:]' '[:lower:]')"

  case "${value}" in
    true|1|yes|on)
      printf '1\n'
      ;;
    false|0|no|off)
      printf '0\n'
      ;;
    *)
      return 1
      ;;
  esac
}

read_common_scalar() {
  local config_file="$1"
  local key="$2"
  [[ -f "${config_file}" ]] || return 1

  local value=""
  value="$(sed -nE "s/^[[:space:]]*${key}:[[:space:]]*\"?([^\"#]+)\"?[[:space:]]*(#.*)?$/\\1/p" "${config_file}" | head -n 1)"

  # trim leading / trailing spaces
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  [[ -n "${value}" ]] || return 1

  printf '%s\n' "${value}"
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
        echo "[ERROR] --mode requires a value: 1|2|3|league|regional|regional_simple|showcase" >&2
        exit 2
      fi
      MODE_ARG="$2"
      shift 2
      ;;
    --no-prompt)
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

source_ros_workspace "${ROOT_DIR}"
cleanup_existing_launch_tree "${CLEANUP_EXISTING}" "${STACK_LAUNCH_REGEX}"

if [[ -z "${BT_APP_FILE_LOG_ENABLE:-}" ]]; then
  if BT_APP_FILE_LOG_ENABLE_FROM_COMMON="$(read_common_bt_file_log_enable "${DEFAULT_COMMON_CONFIG_FILE}")"; then
    export BT_APP_FILE_LOG_ENABLE="${BT_APP_FILE_LOG_ENABLE_FROM_COMMON}"
    echo "[INFO] default BT_APP_FILE_LOG_ENABLE=${BT_APP_FILE_LOG_ENABLE} (from ${DEFAULT_COMMON_CONFIG_FILE})"
  else
    export BT_APP_FILE_LOG_ENABLE="1"
    echo "[INFO] default BT_APP_FILE_LOG_ENABLE=${BT_APP_FILE_LOG_ENABLE}"
  fi
else
  echo "[INFO] use BT_APP_FILE_LOG_ENABLE=${BT_APP_FILE_LOG_ENABLE}"
fi

if [[ -z "${BT_LOG_DIR:-}" ]]; then
  if BT_LOG_DIR_FROM_COMMON="$(read_common_bt_log_dir "${DEFAULT_COMMON_CONFIG_FILE}")"; then
    export BT_LOG_DIR="${BT_LOG_DIR_FROM_COMMON}"
    echo "[INFO] default BT_LOG_DIR=${BT_LOG_DIR} (from ${DEFAULT_COMMON_CONFIG_FILE})"
  else
    export BT_LOG_DIR="${HOME}/Log/BT"
    echo "[INFO] default BT_LOG_DIR=${BT_LOG_DIR}"
  fi
else
  echo "[INFO] use BT_LOG_DIR=${BT_LOG_DIR}"
fi
if [[ "${BT_APP_FILE_LOG_ENABLE}" != "0" ]]; then
  mkdir -p "${BT_LOG_DIR}"
fi

if ! has_launch_arg_key "aim_timer_log_enable"; then
  if AIM_TIMER_LOG_ENABLE_FROM_COMMON="$(read_common_aim_timer_log_enable "${DEFAULT_COMMON_CONFIG_FILE}")"; then
    if [[ "${AIM_TIMER_LOG_ENABLE_FROM_COMMON}" == "0" ]]; then
      AIM_TIMER_LOG_ENABLE_LAUNCH="false"
    else
      AIM_TIMER_LOG_ENABLE_LAUNCH="true"
    fi
    LAUNCH_ARGS=("aim_timer_log_enable:=${AIM_TIMER_LOG_ENABLE_LAUNCH}" "${LAUNCH_ARGS[@]}")
    echo "[INFO] default aim_timer_log_enable=${AIM_TIMER_LOG_ENABLE_LAUNCH} (from ${DEFAULT_COMMON_CONFIG_FILE})"
  fi
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == aim_timer_log_enable:=* ]] && echo "[INFO] override aim_timer_log_enable=${arg#aim_timer_log_enable:=}"; done
fi

if ! has_launch_arg_key "aim_timer_log_dir"; then
  if AIM_TIMER_LOG_DIR_FROM_COMMON="$(read_common_aim_timer_log_dir "${DEFAULT_COMMON_CONFIG_FILE}")"; then
    LAUNCH_ARGS=("aim_timer_log_dir:=${AIM_TIMER_LOG_DIR_FROM_COMMON}" "${LAUNCH_ARGS[@]}")
    echo "[INFO] default aim_timer_log_dir=${AIM_TIMER_LOG_DIR_FROM_COMMON} (from ${DEFAULT_COMMON_CONFIG_FILE})"
  fi
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == aim_timer_log_dir:=* ]] && echo "[INFO] override aim_timer_log_dir=${arg#aim_timer_log_dir:=}"; done
fi

if ! has_launch_arg_key "base_config_file"; then
  LAUNCH_ARGS=("base_config_file:=${DEFAULT_BASE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default base_config_file=${DEFAULT_BASE_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == base_config_file:=* ]] && echo "[INFO] override base_config_file=${arg#base_config_file:=}"; done
fi

if ! has_launch_arg_key "area_manager_config_file"; then
  LAUNCH_ARGS=("area_manager_config_file:=${DEFAULT_AREA_MANAGER_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default area_manager_config_file=${DEFAULT_AREA_MANAGER_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == area_manager_config_file:=* ]] && echo "[INFO] override area_manager_config_file=${arg#area_manager_config_file:=}"; done
fi

if ! has_launch_arg_key "detector_config_file"; then
  LAUNCH_ARGS=("detector_config_file:=${DEFAULT_DETECTOR_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default detector_config_file=${DEFAULT_DETECTOR_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == detector_config_file:=* ]] && echo "[INFO] override detector_config_file=${arg#detector_config_file:=}"; done
fi

if ! has_launch_arg_key "predictor_config_file"; then
  LAUNCH_ARGS=("predictor_config_file:=${DEFAULT_PREDICTOR_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default predictor_config_file=${DEFAULT_PREDICTOR_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == predictor_config_file:=* ]] && echo "[INFO] override predictor_config_file=${arg#predictor_config_file:=}"; done
fi

if ! has_launch_arg_key "outpost_config_file"; then
  LAUNCH_ARGS=("outpost_config_file:=${DEFAULT_OUTPOST_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default outpost_config_file=${DEFAULT_OUTPOST_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == outpost_config_file:=* ]] && echo "[INFO] override outpost_config_file=${arg#outpost_config_file:=}"; done
fi

if ! has_launch_arg_key "buff_config_file"; then
  LAUNCH_ARGS=("buff_config_file:=${DEFAULT_BUFF_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default buff_config_file=${DEFAULT_BUFF_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == buff_config_file:=* ]] && echo "[INFO] override buff_config_file=${arg#buff_config_file:=}"; done
fi

if ! has_launch_arg_key "config_file"; then
  LAUNCH_ARGS=("config_file:=${DEFAULT_OVERRIDE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default config_file(override)=${DEFAULT_OVERRIDE_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == config_file:=* ]] && echo "[INFO] override config_file=${arg#config_file:=}"; done
fi

if ! has_launch_arg_key "firecode_partial_hold_ms"; then
  if FIRECODE_PARTIAL_HOLD_MS_FROM_COMMON="$(read_common_scalar "${DEFAULT_COMMON_CONFIG_FILE}" "firecode_partial_hold_ms")"; then
    LAUNCH_ARGS=("firecode_partial_hold_ms:=${FIRECODE_PARTIAL_HOLD_MS_FROM_COMMON}" "${LAUNCH_ARGS[@]}")
    echo "[INFO] default firecode_partial_hold_ms=${FIRECODE_PARTIAL_HOLD_MS_FROM_COMMON} (from ${DEFAULT_COMMON_CONFIG_FILE})"
  fi
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == firecode_partial_hold_ms:=* ]] && echo "[INFO] override firecode_partial_hold_ms=${arg#firecode_partial_hold_ms:=}"; done
fi

if ! has_launch_arg_key "velocity_raw_to_mps"; then
  if VELOCITY_RAW_TO_MPS_FROM_COMMON="$(read_common_scalar "${DEFAULT_COMMON_CONFIG_FILE}" "velocity_raw_to_mps")"; then
    LAUNCH_ARGS=("velocity_raw_to_mps:=${VELOCITY_RAW_TO_MPS_FROM_COMMON}" "${LAUNCH_ARGS[@]}")
    echo "[INFO] default velocity_raw_to_mps=${VELOCITY_RAW_TO_MPS_FROM_COMMON} (from ${DEFAULT_COMMON_CONFIG_FILE})"
  fi
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == velocity_raw_to_mps:=* ]] && echo "[INFO] override velocity_raw_to_mps=${arg#velocity_raw_to_mps:=}"; done
fi

if [[ -n "${MODE_ARG}" ]]; then
  LAUNCH_ARGS=("mode:=${MODE_ARG}" "${LAUNCH_ARGS[@]}")
fi
if (( OFFLINE_MODE == 1 )); then
  LAUNCH_ARGS=("offline:=true" "${LAUNCH_ARGS[@]}")
fi

exec ros2 launch behavior_tree sentry_all.launch.py "${LAUNCH_ARGS[@]}"
