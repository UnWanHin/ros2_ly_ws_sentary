#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.


source_optional_sentry_msgs() {
  if ros2 pkg prefix sentry_msgs >/dev/null 2>&1; then
    return 0
  fi

  local -a setup_candidates=()
  if [[ -n "${SENTRY_MSGS_SETUP:-}" ]]; then
    setup_candidates+=("${SENTRY_MSGS_SETUP}")
  fi
  if [[ -n "${SENTRY_COMMON_SETUP:-}" ]]; then
    setup_candidates+=("${SENTRY_COMMON_SETUP}")
  fi
  if [[ -n "${SENTRY_COMMON_ROOT:-}" ]]; then
    setup_candidates+=("${SENTRY_COMMON_ROOT}/install/setup.bash")
  fi
  setup_candidates+=(
    "${HOME}/sentry.common/install/setup.bash"
    "/tmp/sentry_msgs_install/sentry_msgs/share/sentry_msgs/local_setup.bash"
  )

  local setup_file
  for setup_file in "${setup_candidates[@]}"; do
    if [[ -f "${setup_file}" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "${setup_file}"
      set -u
      if ros2 pkg prefix sentry_msgs >/dev/null 2>&1; then
        echo "[INFO] sourced sentry_msgs: ${setup_file}" >&2
        return 0
      fi
    fi
  done

  return 1
}

launch_arg_bool_is_false() {
  local key="$1"
  local arg
  local value
  for arg in "${LAUNCH_ARGS[@]:-}"; do
    if [[ "${arg}" == "${key}:="* ]]; then
      value="${arg#${key}:=}"
      value="$(printf '%s' "${value}" | tr '[:upper:]' '[:lower:]')"
      [[ "${value}" == "false" || "${value}" == "0" || "${value}" == "no" || "${value}" == "off" ]]
      return
    fi
  done
  return 1
}

require_sentry_msgs_for_behavior_tree() {
  if launch_arg_bool_is_false "use_behavior_tree"; then
    return 0
  fi
  if ros2 pkg prefix sentry_msgs >/dev/null 2>&1; then
    return 0
  fi

  echo "[ERROR] sentry_msgs is required for the formal /ly/aim/* chain." >&2
  echo "        Build/source ~/sentry.common, or set SENTRY_MSGS_SETUP to sentry_msgs local_setup.bash." >&2
  exit 1
}

source_ros_workspace() {
  local root_dir="$1"

  : "${ROS_LOG_DIR:=/tmp/ros2_logs}"
  mkdir -p "${ROS_LOG_DIR}"
  export ROS_LOG_DIR

  if [[ -n "${ROS_DISTRO:-}" ]] && [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
  else
    local distro
    for distro in humble iron jazzy rolling; do
      if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
        set +u
        # shellcheck disable=SC1090
        source "/opt/ros/${distro}/setup.bash"
        set -u
        break
      fi
    done
  fi

  source_optional_sentry_msgs || true

  if [[ ! -f "${root_dir}/install/setup.bash" ]]; then
    echo "[ERROR] ${root_dir}/install/setup.bash not found." >&2
    echo "        Run: colcon build" >&2
    exit 1
  fi

  set +u
  # shellcheck disable=SC1091
  source "${root_dir}/install/setup.bash"
  set -u

  cd "${root_dir}"
}

cleanup_existing_stack() {
  local enabled="$1"
  local node_regex="$2"
  local launch_regex="$3"

  if [[ "${enabled}" != "1" ]]; then
    return 0
  fi

  mapfile -t existing_stack_procs < <(
    {
      pgrep -af "${node_regex}" || true
      pgrep -af "${launch_regex}" || true
    } | awk '!seen[$0]++'
  )

  if (( ${#existing_stack_procs[@]} == 0 )); then
    return 0
  fi

  echo "[WARN] Detected existing stack-related processes:" >&2
  printf "  %s\n" "${existing_stack_procs[@]}" >&2
  echo "[INFO] Cleaning up stale processes before launch..." >&2
  pkill -f "${node_regex}" || true
  pkill -f "${launch_regex}" || true
  sleep 1
}

collect_descendant_pids() {
  local parent_pid="$1"
  local -a children=()
  local child

  mapfile -t children < <(pgrep -P "${parent_pid}" || true)
  for child in "${children[@]}"; do
    [[ -n "${child}" ]] || continue
    echo "${child}"
    collect_descendant_pids "${child}"
  done
}

cleanup_existing_launch_tree() {
  local enabled="$1"
  local launch_regex="$2"

  if [[ "${enabled}" != "1" ]]; then
    return 0
  fi

  local -a launch_pids=()
  mapfile -t launch_pids < <(pgrep -f "${launch_regex}" || true)

  if (( ${#launch_pids[@]} == 0 )); then
    return 0
  fi

  echo "[WARN] Detected existing launch processes (strict tree cleanup):" >&2
  local pid
  local cmd
  for pid in "${launch_pids[@]}"; do
    cmd="$(ps -o args= -p "${pid}" 2>/dev/null || true)"
    if [[ -n "${cmd}" ]]; then
      echo "  ${pid} ${cmd}" >&2
    else
      echo "  ${pid}" >&2
    fi
  done

  local -a candidate_pids=()
  local child
  for pid in "${launch_pids[@]}"; do
    candidate_pids+=("${pid}")
    while IFS= read -r child; do
      [[ -n "${child}" ]] || continue
      candidate_pids+=("${child}")
    done < <(collect_descendant_pids "${pid}")
  done

  mapfile -t candidate_pids < <(printf '%s\n' "${candidate_pids[@]}" | awk 'NF && !seen[$0]++')

  echo "[INFO] Cleaning only matched launch process trees before relaunch..." >&2
  kill -INT "${launch_pids[@]}" 2>/dev/null || true
  sleep 1

  local -a alive_pids=()
  for pid in "${candidate_pids[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      alive_pids+=("${pid}")
    fi
  done

  if (( ${#alive_pids[@]} > 0 )); then
    kill -TERM "${alive_pids[@]}" 2>/dev/null || true
    sleep 1
  fi

  alive_pids=()
  for pid in "${candidate_pids[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      alive_pids+=("${pid}")
    fi
  done

  if (( ${#alive_pids[@]} > 0 )); then
    kill -KILL "${alive_pids[@]}" 2>/dev/null || true
  fi
}

read_yaml_path_scalar() {
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

print_raw_goal_map_preview() {
  local config_file="$1"
  local target_x_cm="$2"
  local target_y_cm="$3"
  local target_z_cm="$4"
  local use_static_calibration="$5"
  local raw_goal_target_frame="$6"

  python3 - "${config_file}" "${target_x_cm}" "${target_y_cm}" "${target_z_cm}" \
    "${use_static_calibration}" "${raw_goal_target_frame}" <<'PY'
import sys

try:
    import yaml
except Exception as exc:
    print(f"[WARN] Cannot preview official_map -> map: PyYAML unavailable: {exc}", file=sys.stderr)
    raise SystemExit(0)

config_file, x_cm, y_cm, z_cm, use_cal, frame_override = sys.argv[1:7]

try:
    x = float(x_cm) * 0.01
    y = float(y_cm) * 0.01
    z = float(z_cm) * 0.01
except ValueError as exc:
    print(f"[WARN] Cannot preview official_map -> map: invalid target cm value: {exc}", file=sys.stderr)
    raise SystemExit(0)

print(f"[INFO] map aim raw target: official_map=({x:.3f}, {y:.3f}, {z:.3f})m")
if use_cal.lower() not in ("true", "1", "yes", "on"):
    print("[INFO] map aim active target: raw calibration disabled, target frame unchanged")
    raise SystemExit(0)

try:
    with open(config_file, encoding="utf-8") as fh:
        root = yaml.safe_load(fh) or {}
except Exception as exc:
    print(f"[WARN] Cannot preview official_map -> map: failed to read {config_file}: {exc}", file=sys.stderr)
    raise SystemExit(0)

params = root.get("target_rel_to_goal_pos_node", {}).get("ros__parameters", {})
matrix = params.get("raw_goal_transform_matrix", [])
if not isinstance(matrix, list) or len(matrix) != 16:
    print(f"[WARN] Cannot preview official_map -> map: raw_goal_transform_matrix invalid in {config_file}", file=sys.stderr)
    raise SystemExit(0)

unit = str(params.get("raw_goal_calibration_unit", "m"))
if unit in ("m", "M"):
    unit_scale = 1.0
elif unit in ("cm", "CM"):
    unit_scale = 0.01
else:
    print(f"[WARN] Cannot preview official_map -> map: invalid raw_goal_calibration_unit={unit}", file=sys.stderr)
    raise SystemExit(0)

try:
    m = [float(v) for v in matrix]
except (TypeError, ValueError) as exc:
    print(f"[WARN] Cannot preview official_map -> map: matrix has non-numeric value: {exc}", file=sys.stderr)
    raise SystemExit(0)

out_x = m[0] * x + m[1] * y + m[3] * unit_scale
out_y = m[4] * x + m[5] * y + m[7] * unit_scale
target_frame = frame_override or str(params.get("raw_goal_target_frame", "map"))
print(
    "[INFO] map aim 4x4 XY preview: "
    f"x'={m[0]:.8f}*{x:.3f}+{m[1]:.8f}*{y:.3f}+{m[3] * unit_scale:.3f}, "
    f"y'={m[4]:.8f}*{x:.3f}+{m[5]:.8f}*{y:.3f}+{m[7] * unit_scale:.3f}"
)
print(f"[INFO] map aim active target: {target_frame}=({out_x:.3f}, {out_y:.3f}, {z:.3f})m")
PY
}
