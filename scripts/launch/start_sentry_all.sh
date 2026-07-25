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
DEFAULT_OVERRIDE_CONFIG_FILE="${ROOT_DIR}/config/override_config.yaml"
DEFAULT_AREA_MANAGER_CONFIG_FILE="${ROOT_DIR}/src/behavior_tree/config/AreaManager.yaml"
DEFAULT_TASK_CONFIG_FILE="${ROOT_DIR}/src/behavior_tree/config/Task.yaml"
DEFAULT_CHASE_CONFIG_FILE="${ROOT_DIR}/src/behavior_tree/config/Chase.yaml"
DEFAULT_NAVI_ROTATE_CONFIG_FILE="${ROOT_DIR}/src/behavior_tree/config/NaviRotateControl.yaml"
DEFAULT_TACTICAL_CONFIG_FILE="${ROOT_DIR}/src/behavior_tree/config/Tactical.yaml"
DEFAULT_PATROL_CONFIG_FILE="${ROOT_DIR}/src/behavior_tree/config/Patrol.yaml"
DEFAULT_SPECIAL_CONFIG_FILE="${ROOT_DIR}/src/behavior_tree/config/Special.yaml"
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
  ./${SCRIPT_NAME} -- use_gimbal:=false
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

read_common_rosbag() {
  local config_file="$1"
  [[ -f "${config_file}" ]] || return 1

  local value=""
  value="$(read_common_scalar "${config_file}" "rosbag")" || return 1
  read_common_bool_value "${value}"
}

read_common_rosbag_path() {
  local config_file="$1"
  [[ -f "${config_file}" ]] || return 1

  local value=""
  value="$(read_common_scalar "${config_file}" "rosbag_path")" || return 1
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

read_common_path_scalar() {
  local config_file="$1"
  local path="$2"
  [[ -f "${config_file}" ]] || return 1

  python3 - "${config_file}" "${path}" <<'PY'
import sys

config_file = sys.argv[1]
parts = sys.argv[2].split(".")
stack = []

def strip_inline_comment(line: str) -> str:
    out = []
    in_single = False
    in_double = False
    for ch in line:
        if ch == "'" and not in_double:
            in_single = not in_single
        elif ch == '"' and not in_single:
            in_double = not in_double
        elif ch == "#" and not in_single and not in_double:
            break
        out.append(ch)
    return "".join(out).rstrip()

with open(config_file, encoding="utf-8") as fh:
    for raw in fh:
        line = strip_inline_comment(raw.rstrip("\n"))
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        indent = len(line) - len(line.lstrip(" "))
        text = line.strip()
        if ":" not in text:
            continue
        key, value = text.split(":", 1)
        key = key.strip().strip("'\"")
        value = value.strip()

        while stack and stack[-1][0] >= indent:
            stack.pop()
        current = [item[1] for item in stack] + [key]

        if not value:
            stack.append((indent, key))
            continue

        if (value.startswith('"') and value.endswith('"')) or (
            value.startswith("'") and value.endswith("'")
        ):
            value = value[1:-1]

        if current == parts:
            print(value)
            sys.exit(0)

sys.exit(1)
PY
}

read_common_value() {
  local config_file="$1"
  local common_key="$2"
  local legacy_key="${3:-}"

  if [[ "${common_key}" == *.* ]]; then
    read_common_path_scalar "${config_file}" "${common_key}" && return 0
  else
    read_common_scalar "${config_file}" "${common_key}" && return 0
  fi

  if [[ -n "${legacy_key}" ]]; then
    read_common_scalar "${config_file}" "${legacy_key}" && return 0
  fi
  return 1
}

add_common_bool_launch_arg() {
  local common_key="$1"
  local launch_key="$2"
  local legacy_key="${3:-}"

  if ! has_launch_arg_key "${launch_key}"; then
    local raw_value=""
    local bool_value=""
    if raw_value="$(read_common_value "${DEFAULT_COMMON_CONFIG_FILE}" "${common_key}" "${legacy_key}")" &&
       bool_value="$(read_common_bool_value "${raw_value}")"; then
      local launch_value="true"
      if [[ "${bool_value}" == "0" ]]; then
        launch_value="false"
      fi
      LAUNCH_ARGS=("${launch_key}:=${launch_value}" "${LAUNCH_ARGS[@]}")
      echo "[INFO] default ${launch_key}=${launch_value} (from ${DEFAULT_COMMON_CONFIG_FILE}:${common_key})"
    fi
  else
    local arg
    for arg in "${LAUNCH_ARGS[@]}"; do
      [[ "${arg}" == "${launch_key}:="* ]] && echo "[INFO] override ${launch_key}=${arg#${launch_key}:=}"
    done
  fi
}

add_common_scalar_launch_arg() {
  local common_key="$1"
  local launch_key="$2"
  local legacy_key="${3:-}"

  if ! has_launch_arg_key "${launch_key}"; then
    local value=""
    if value="$(read_common_value "${DEFAULT_COMMON_CONFIG_FILE}" "${common_key}" "${legacy_key}")"; then
      LAUNCH_ARGS=("${launch_key}:=${value}" "${LAUNCH_ARGS[@]}")
      echo "[INFO] default ${launch_key}=${value} (from ${DEFAULT_COMMON_CONFIG_FILE}:${common_key})"
    fi
  else
    local arg
    for arg in "${LAUNCH_ARGS[@]}"; do
      [[ "${arg}" == "${launch_key}:="* ]] && echo "[INFO] override ${launch_key}=${arg#${launch_key}:=}"
    done
  fi
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

if (( OFFLINE_MODE == 1 )); then
  LAUNCH_ARGS=("offline:=true" "${LAUNCH_ARGS[@]}")
fi

source_ros_workspace "${ROOT_DIR}"
require_sentry_msgs_for_behavior_tree
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

ROSBAG_LAUNCH=""
if ! has_launch_arg_key "rosbag"; then
  if ROSBAG_FROM_COMMON="$(read_common_rosbag "${DEFAULT_COMMON_CONFIG_FILE}")"; then
    if [[ "${ROSBAG_FROM_COMMON}" == "0" ]]; then
      ROSBAG_LAUNCH="false"
    else
      ROSBAG_LAUNCH="true"
    fi
    LAUNCH_ARGS=("rosbag:=${ROSBAG_LAUNCH}" "${LAUNCH_ARGS[@]}")
    echo "[INFO] default rosbag=${ROSBAG_LAUNCH} (from ${DEFAULT_COMMON_CONFIG_FILE})"
  fi
else
  for arg in "${LAUNCH_ARGS[@]}"; do
    if [[ "${arg}" == rosbag:=* ]]; then
      ROSBAG_LAUNCH="${arg#rosbag:=}"
      if ROSBAG_NORMALIZED="$(read_common_bool_value "${ROSBAG_LAUNCH}")"; then
        if [[ "${ROSBAG_NORMALIZED}" == "0" ]]; then
          ROSBAG_LAUNCH="false"
        else
          ROSBAG_LAUNCH="true"
        fi
      fi
      echo "[INFO] override rosbag=${ROSBAG_LAUNCH}"
    fi
  done
fi

if ! has_launch_arg_key "rosbag_path"; then
  if ROSBAG_PATH_FROM_COMMON="$(read_common_rosbag_path "${DEFAULT_COMMON_CONFIG_FILE}")"; then
    LAUNCH_ARGS=("rosbag_path:=${ROSBAG_PATH_FROM_COMMON}" "${LAUNCH_ARGS[@]}")
    echo "[INFO] default rosbag_path=${ROSBAG_PATH_FROM_COMMON} (from ${DEFAULT_COMMON_CONFIG_FILE})"
  fi
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == rosbag_path:=* ]] && echo "[INFO] override rosbag_path=${arg#rosbag_path:=}"; done
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

if ! has_launch_arg_key "task_config_file"; then
  LAUNCH_ARGS=("task_config_file:=${DEFAULT_TASK_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default task_config_file=${DEFAULT_TASK_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == task_config_file:=* ]] && echo "[INFO] override task_config_file=${arg#task_config_file:=}"; done
fi

if ! has_launch_arg_key "chase_config_file"; then
  LAUNCH_ARGS=("chase_config_file:=${DEFAULT_CHASE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default chase_config_file=${DEFAULT_CHASE_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == chase_config_file:=* ]] && echo "[INFO] override chase_config_file=${arg#chase_config_file:=}"; done
fi

if ! has_launch_arg_key "navi_rotate_config_file"; then
  LAUNCH_ARGS=("navi_rotate_config_file:=${DEFAULT_NAVI_ROTATE_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default navi_rotate_config_file=${DEFAULT_NAVI_ROTATE_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == navi_rotate_config_file:=* ]] && echo "[INFO] override navi_rotate_config_file=${arg#navi_rotate_config_file:=}"; done
fi

if ! has_launch_arg_key "tactical_config_file"; then
  LAUNCH_ARGS=("tactical_config_file:=${DEFAULT_TACTICAL_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default tactical_config_file=${DEFAULT_TACTICAL_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == tactical_config_file:=* ]] && echo "[INFO] override tactical_config_file=${arg#tactical_config_file:=}"; done
fi

if ! has_launch_arg_key "patrol_config_file"; then
  LAUNCH_ARGS=("patrol_config_file:=${DEFAULT_PATROL_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default patrol_config_file=${DEFAULT_PATROL_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == patrol_config_file:=* ]] && echo "[INFO] override patrol_config_file=${arg#patrol_config_file:=}"; done
fi

if ! has_launch_arg_key "special_config_file"; then
  LAUNCH_ARGS=("special_config_file:=${DEFAULT_SPECIAL_CONFIG_FILE}" "${LAUNCH_ARGS[@]}")
  echo "[INFO] default special_config_file=${DEFAULT_SPECIAL_CONFIG_FILE}"
else
  for arg in "${LAUNCH_ARGS[@]}"; do [[ "${arg}" == special_config_file:=* ]] && echo "[INFO] override special_config_file=${arg#special_config_file:=}"; done
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

add_common_bool_launch_arg "start_gate.allow_gimbal_patrol_before_start" "start_gate_allow_gimbal_patrol_before_start"
add_common_scalar_launch_arg "face_mode.max_yaw_step_deg" "face_mode_max_yaw_step_deg"
add_common_scalar_launch_arg "face_mode.max_pitch_step_deg" "face_mode_max_pitch_step_deg"

add_common_bool_launch_arg "gimbal_raw.file.enable" "gimbal_raw_log_enable" "gimbal_raw_log_enable"
add_common_bool_launch_arg "gimbal_raw.file.uplink" "gimbal_raw_log_uplink" "gimbal_raw_log_uplink"
add_common_bool_launch_arg "gimbal_raw.file.downlink" "gimbal_raw_log_downlink" "gimbal_raw_log_downlink"
add_common_bool_launch_arg "gimbal_raw.file.screen" "gimbal_raw_log_screen" "gimbal_raw_log_screen"
add_common_bool_launch_arg "gimbal_raw.file.flush" "gimbal_raw_log_flush" "gimbal_raw_log_flush"
add_common_scalar_launch_arg "gimbal_raw.file.dir" "gimbal_raw_log_dir" "gimbal_raw_log_dir"
add_common_scalar_launch_arg "gimbal_raw.file.type_ids" "gimbal_raw_log_type_ids" "gimbal_raw_log_type_ids"
add_common_bool_launch_arg "gimbal_raw.topic.enable" "gimbal_raw_topic_enable" "gimbal_raw_topic_enable"
add_common_bool_launch_arg "gimbal_raw.topic.uplink" "gimbal_raw_topic_uplink" "gimbal_raw_topic_uplink"
add_common_bool_launch_arg "gimbal_raw.topic.downlink" "gimbal_raw_topic_downlink" "gimbal_raw_topic_downlink"
add_common_scalar_launch_arg "gimbal_raw.topic.type_ids" "gimbal_raw_topic_type_ids" "gimbal_raw_topic_type_ids"

if [[ -n "${MODE_ARG}" ]]; then
  LAUNCH_ARGS=("mode:=${MODE_ARG}" "${LAUNCH_ARGS[@]}")
fi
exec ros2 launch behavior_tree sentry_all.launch.py "${LAUNCH_ARGS[@]}"
