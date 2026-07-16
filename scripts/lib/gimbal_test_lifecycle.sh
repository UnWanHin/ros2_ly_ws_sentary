#!/usr/bin/env bash

# Shared gimbal_driver lifecycle for direct control test scripts.
# Callers must source ros_launch_common.sh first and own GIMBAL_PID.

gimbal_test_launch_driver() {
  local label="$1"
  local config_file="$2"
  local use_virtual_device="$3"
  local output_mode="$4"
  local wait_sec="$5"
  local stack_node_regex="$6"
  local stack_launch_regex="$7"
  shift 7

  if [[ ! -f "${config_file}" ]]; then
    echo "[ERROR] Config file not found: ${config_file}" >&2
    return 1
  fi

  cleanup_existing_stack "1" "${stack_node_regex}" "${stack_launch_regex}"

  echo "[${label}][INFO] Launching gimbal_driver with config=${config_file} use_virtual_device=${use_virtual_device}" >&2
  ros2 launch gimbal_driver gimbal_driver.launch.py \
    "base_config_file:=${config_file}" \
    "use_virtual_device:=${use_virtual_device}" \
    "output:=${output_mode}" \
    "$@" &
  GIMBAL_PID="$!"
  sleep "${wait_sec}"

  if ! kill -0 "${GIMBAL_PID}" 2>/dev/null; then
    echo "[ERROR] gimbal_driver exited early." >&2
    return 1
  fi
}

gimbal_test_stop_driver() {
  if [[ -n "${GIMBAL_PID:-}" ]] && kill -0 "${GIMBAL_PID}" 2>/dev/null; then
    kill -INT "${GIMBAL_PID}" 2>/dev/null || true
    sleep 1
    kill -TERM "${GIMBAL_PID}" 2>/dev/null || true
    wait "${GIMBAL_PID}" 2>/dev/null || true
  fi
}
