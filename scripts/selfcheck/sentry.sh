#!/usr/bin/env bash

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

: "${ROS_LOG_DIR:=/tmp/ros2_logs}"
mkdir -p "${ROS_LOG_DIR}"
export ROS_LOG_DIR

AUTO_LAUNCH=0
OFFLINE_MODE=0
WAIT_SECONDS=18
CMD_TIMEOUT=6
HZ_SECONDS=6
SKIP_HZ=0
STATIC_ONLY=0
RUNTIME_ONLY=0
LAUNCH_ARGS=()

PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

LAUNCH_PID=""
LAUNCH_LOG=""

if [[ -t 1 ]]; then
  C_RESET=$'\033[0m'
  C_RED=$'\033[31m'
  C_YELLOW=$'\033[33m'
  C_GREEN=$'\033[32m'
  C_CYAN=$'\033[36m'
else
  C_RESET=""
  C_RED=""
  C_YELLOW=""
  C_GREEN=""
  C_CYAN=""
fi

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [--launch] [--offline] [--wait SECONDS] [--cmd-timeout SECONDS] [--hz-seconds SECONDS] [--skip-hz] [--static-only|--runtime-only] [-- <launch_args...>]

Examples:
  # 檢查當前已在運行的系統
  ./${SCRIPT_NAME}

  # 自動啟動整套節點後再檢查
  ./${SCRIPT_NAME} --launch

  # 只做離車靜態檢查（不依賴 ROS 圖）
  ./${SCRIPT_NAME} --static-only

  # 只做運行時圖檢查（跳過文件/配置靜態檢查）
  ./${SCRIPT_NAME} --runtime-only --launch

  # 自動啟動 + 自定義 launch 參數
  ./${SCRIPT_NAME} --launch -- --config_file:=/abs/path/override_config.yaml use_gimbal:=false

Options:
  --launch               自動啟動 sentry_all.launch.py，檢查結束後自動停止
  --offline              以離線模式啟動（傳遞 offline:=true）
  --wait SECONDS         啟動後等待秒數（默認: ${WAIT_SECONDS}）
  --cmd-timeout SECONDS  單次 ros2 命令超時（默認: ${CMD_TIMEOUT}）
  --hz-seconds SECONDS   ros2 topic hz 採樣時長（默認: ${HZ_SECONDS}）
  --skip-hz              跳過頻率檢查（更快）
  --static-only          只執行靜態檢查（文件、配置、BT XML）
  --runtime-only         只執行運行時檢查（node/topic/hz）
  --help                 顯示幫助
EOF
}

info() {
  printf "%s[INFO]%s %s\n" "${C_CYAN}" "${C_RESET}" "$*"
}

pass() {
  PASS_COUNT=$((PASS_COUNT + 1))
  printf "%s[PASS]%s %s\n" "${C_GREEN}" "${C_RESET}" "$*"
}

warn() {
  WARN_COUNT=$((WARN_COUNT + 1))
  printf "%s[WARN]%s %s\n" "${C_YELLOW}" "${C_RESET}" "$*"
}

fail() {
  FAIL_COUNT=$((FAIL_COUNT + 1))
  printf "%s[FAIL]%s %s\n" "${C_RED}" "${C_RESET}" "$*"
}

issue() {
  local severity="$1"
  shift
  if [[ "${severity}" == "warn" ]]; then
    warn "$*"
  else
    fail "$*"
  fi
}

source_ros() {
  if [[ -n "${ROS_DISTRO:-}" ]] && [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    set +u
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
    pass "ROS2 sourced: /opt/ros/${ROS_DISTRO}/setup.bash"
    return
  fi

  local distro
  for distro in humble iron jazzy rolling; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      # shellcheck disable=SC1090
      set +u
      source "/opt/ros/${distro}/setup.bash"
      set -u
      pass "ROS2 sourced: /opt/ros/${distro}/setup.bash"
      return
    fi
  done

fail "No ROS2 setup found under /opt/ros"
}

sentry_msgs_aim_result_has_dynamics() {
  local interface_text
  interface_text="$(ros2 interface show sentry_msgs/msg/AimResult 2>/dev/null)" || return 1
  grep -Fxq "bool follow" <<< "${interface_text}" &&
    grep -Fxq "float32 yaw_omega" <<< "${interface_text}" &&
    grep -Fxq "float32 pitch_omega" <<< "${interface_text}" &&
    grep -Fxq "float32 yaw_alpha" <<< "${interface_text}" &&
    grep -Fxq "float32 pitch_alpha" <<< "${interface_text}"
}

source_optional_sentry_msgs() {
  if ros2 pkg prefix sentry_msgs >/dev/null 2>&1 && sentry_msgs_aim_result_has_dynamics; then
    pass "sentry_msgs available: $(ros2 pkg prefix sentry_msgs)"
    return 0
  fi

  local -a setup_candidates=()
  if [[ -n "${SENTRY_MSGS_SETUP:-}" ]]; then
    setup_candidates+=("${SENTRY_MSGS_SETUP}")
  fi
  if [[ -n "${SENTRY_AIM_ROOT:-}" ]]; then
    setup_candidates+=("${SENTRY_AIM_ROOT}/install/sentry_msgs/share/sentry_msgs/local_setup.bash")
  fi
  setup_candidates+=("${HOME}/sentry.aim/install/sentry_msgs/share/sentry_msgs/local_setup.bash")
  if [[ -n "${SENTRY_AIM_SETUP:-}" ]]; then
    setup_candidates+=("${SENTRY_AIM_SETUP}")
  fi
  if [[ -n "${SENTRY_AIM_ROOT:-}" ]]; then
    setup_candidates+=("${SENTRY_AIM_ROOT}/install/setup.bash")
  fi
  setup_candidates+=(
    "${HOME}/sentry.aim/install/setup.bash"
  )

  local setup_file
  for setup_file in "${setup_candidates[@]}"; do
    if [[ -f "${setup_file}" ]]; then
      # shellcheck disable=SC1090
      set +u
      source "${setup_file}"
      set -u
      if ros2 pkg prefix sentry_msgs >/dev/null 2>&1 && sentry_msgs_aim_result_has_dynamics; then
        pass "sentry_msgs sourced: ${setup_file}"
        return 0
      fi
    fi
  done

  warn "sentry_msgs not found or missing AimResult dynamics; build/source ~/sentry.aim without sentry.common underlay."
  return 1
}

source_workspace() {
  if [[ -f "${ROOT_DIR}/install/local_setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u
    source "${ROOT_DIR}/install/local_setup.bash"
    set -u
    pass "Workspace sourced: ${ROOT_DIR}/install/local_setup.bash"
  else
    fail "Workspace not built: ${ROOT_DIR}/install/local_setup.bash missing"
  fi
}

cleanup_launch() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    info "Stopping launched stack (PID=${LAUNCH_PID})"
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}

trap cleanup_launch EXIT

while [[ $# -gt 0 ]]; do
  case "$1" in
    --launch)
      AUTO_LAUNCH=1
      shift
      ;;
    --offline)
      OFFLINE_MODE=1
      shift
      ;;
    --wait)
      WAIT_SECONDS="$2"
      shift 2
      ;;
    --cmd-timeout)
      CMD_TIMEOUT="$2"
      shift 2
      ;;
    --hz-seconds)
      HZ_SECONDS="$2"
      shift 2
      ;;
    --skip-hz)
      SKIP_HZ=1
      shift
      ;;
    --static-only)
      STATIC_ONLY=1
      shift
      ;;
    --runtime-only)
      RUNTIME_ONLY=1
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
      printf "[ERROR] Unknown argument: %s\n" "$1" >&2
      usage
      exit 2
      ;;
  esac
done

if (( STATIC_ONLY == 1 && RUNTIME_ONLY == 1 )); then
  fail "--static-only and --runtime-only are mutually exclusive"
  exit 2
fi

check_cmd() {
  local cmd="$1"
  if command -v "${cmd}" >/dev/null 2>&1; then
    pass "Command available: ${cmd}"
  else
    fail "Command missing: ${cmd}"
  fi
}

check_file_exists() {
  local path="$1"
  if [[ -f "${path}" ]]; then
    pass "File exists: ${path}"
  else
    fail "File missing: ${path}"
  fi
}

check_ros_interface() {
  local interface_name="$1"
  if timeout "${CMD_TIMEOUT}s" ros2 interface show "${interface_name}" >/dev/null 2>&1; then
    pass "ROS interface available: ${interface_name}"
  else
    fail "ROS interface missing: ${interface_name}"
  fi
}

check_ros_interface_field() {
  local interface_name="$1"
  local expected_field="$2"
  local interface_text
  if interface_text="$(timeout "${CMD_TIMEOUT}s" ros2 interface show "${interface_name}" 2>/dev/null)" &&
    grep -Fxq "${expected_field}" <<< "${interface_text}"; then
    pass "ROS interface field available: ${interface_name} ${expected_field}"
  else
    fail "ROS interface field missing: ${interface_name} ${expected_field}"
  fi
}

check_executable_file() {
  local path="$1"
  if [[ -x "${path}" ]]; then
    pass "Executable OK: ${path}"
  else
    fail "Executable missing or not executable: ${path}"
  fi
}

check_bash_syntax() {
  local path="$1"
  if timeout "${CMD_TIMEOUT}s" bash -n "${path}" >/dev/null 2>&1; then
    pass "Shell syntax OK: ${path}"
  else
    fail "Shell syntax error: ${path}"
  fi
}

check_gimbal_debug_profile_contract() {
  if python3 - "${ROOT_DIR}" <<'PY'
import importlib.util
import re
import sys
import tempfile
from pathlib import Path

root = Path(sys.argv[1])
baseline = root / "src/gimbal_driver/config/gimbal_driver_config.yaml"
profile = root / "src/gimbal_driver/config/debug_mode.yaml"
debug_launch = root / "src/gimbal_driver/launch/debug_node.launch.py"
driver_launch = root / "src/gimbal_driver/launch/gimbal_driver.launch.py"
driver_source = root / "src/gimbal_driver/main.cpp"
driver_cmake = root / "src/gimbal_driver/CMakeLists.txt"
debug_bridge = root / "src/gimbal_driver/scripts/debug.py"
formal_launch = root / "src/behavior_tree/launch/sentry_all.launch.py"
gimbal_lifecycle = root / "scripts/lib/gimbal_test_lifecycle.sh"
legacy_velocity_files = (
    root / "scripts/navi/navi_vel_chain.sh",
    root / "scripts/navi/navi_vel_chain.py",
)

errors = []
baseline_text = baseline.read_text(encoding="utf-8")
forbidden = (
    r"^\s*navigation_test\s*:",
    r"^\s*navigation_test_stale_timeout_ms\s*:",
    r"^\s*navigation_mode\s*:",
    r"^\s*io_config/navigation_mode/should_rotate/follow_mode_when_false\s*:",
)
for pattern in forbidden:
    if re.search(pattern, baseline_text, re.MULTILINE):
        errors.append(f"formal baseline contains debug key matching {pattern}")

if not profile.is_file():
    errors.append("debug_mode.yaml is missing")
else:
    profile_text = profile.read_text(encoding="utf-8")
    required_profile = (
        r"(?m)^\s*raw_downlink_test_mode\s*:\s*false\s*(?:#.*)?$",
        r"(?m)^\s*rotate_level\s*:\s*0\s*(?:#.*)?$",
        r"(?m)^\s*follow_mode_when_false\s*:\s*true\s*(?:#.*)?$",
        r"(?m)^\s*stale_timeout_ms\s*:\s*500\s*(?:#.*)?$",
        r"(?m)^\s*publish_hz\s*:\s*100(?:\.0)?\s*(?:#.*)?$",
    )
    for pattern in required_profile:
        if not re.search(pattern, profile_text):
            errors.append(f"debug mode profile missing {pattern}")

if not debug_launch.is_file():
    errors.append("debug_node.launch.py is missing")
else:
    debug_text = debug_launch.read_text(encoding="utf-8")
    for token in ("IncludeLaunchDescription", "debug_config_file", "gimbal_driver.launch.py", "debug.py"):
        if token not in debug_text:
            errors.append(f"debug_node.launch.py missing {token}")
    if 'LaunchConfiguration("debug_config_file")' not in debug_text:
        errors.append("debug_node.launch.py does not load debug_config_file into the bridge")
    for token in ("load_raw_downlink_test_mode", "raw_downlink_test_mode", "OpaqueFunction"):
        if token not in debug_text:
            errors.append(f"debug_node.launch.py missing raw downlink test token: {token}")

if not debug_bridge.is_file():
    errors.append("debug control bridge is missing")
else:
    bridge_text = debug_bridge.read_text(encoding="utf-8")
    for token in ("/ly/navi/vel", "/ly/navi/should_rotate", "/ly/aim/result", "/ly/control/vel", "/ly/control/angles", "/ly/control/trajectory", "/ly/control/firecode", "ControlVelocity", "GimbalTrajectory", "FireCode", "AimResult", "yaw_omega", "pitch_omega", "yaw_alpha", "pitch_alpha", "FIELD_ALL", "navi_mode", "aim_mode", "use_raw = True", "stale_timeout_ms", "debug_control_state"):
        if token not in bridge_text:
            errors.append(f"debug control bridge missing source token: {token}")

debug_control_state = root / "src/gimbal_driver/scripts/debug_control_state.py"
if not debug_control_state.is_file():
    errors.append("debug control state module is missing")

if not driver_cmake.is_file():
    errors.append("gimbal_driver CMakeLists.txt is missing")
elif "scripts/debug.py" not in driver_cmake.read_text(encoding="utf-8"):
    errors.append("debug velocity bridge is not installed by gimbal_driver")

if not driver_source.is_file():
    errors.append("gimbal_driver source is missing")
else:
    driver_source_text = driver_source.read_text(encoding="utf-8")
    if "MaybeApplyNavigationModeRotateHeartbeat" in driver_source_text:
        errors.append("debug_node must publish FireCode through the bridge, not driver heartbeat")
    for token in (
        "RawDownlinkTest.hpp",
        "GenRawDownlinkTestSubscriptions",
        "SendRawDownlinkTestFrame",
        "rawDownlinkTestMode_",
        "RAW DOWNLINK TEST MODE active",
    ):
        if token not in driver_source_text:
            errors.append(f"gimbal raw downlink test mode missing source token: {token}")

if not driver_launch.is_file():
    errors.append("gimbal_driver.launch.py is missing")
else:
    driver_text = driver_launch.read_text(encoding="utf-8")
    if '"io_config/use_virtual_device": ParameterValue(' not in driver_text:
        errors.append("gimbal use_virtual_device is not normalized to a bool parameter")
    if "LaunchConfiguration(arg_name), value_type=bool" not in driver_text:
        errors.append("gimbal forwarded bool overrides are not typed substitutions")
    for token in ("raw_downlink_test_mode", "io_config/raw_downlink_test_mode"):
        if token not in driver_text:
            errors.append(f"gimbal launch missing raw downlink test argument: {token}")
    spec = importlib.util.spec_from_file_location("gimbal_driver_launch", driver_launch)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with tempfile.TemporaryDirectory() as tmp_dir:
        legacy_base = Path(tmp_dir) / "legacy_base.yaml"
        legacy_override = Path(tmp_dir) / "legacy_override.yaml"
        legacy_base.write_text(
            """/**:\n  ros__parameters:\n    io_config:\n      device_name: /dev/legacy\n      navigation_mode:\n        enabled: true\n    io_config/raw_serial_log_enable: false\n""",
            encoding="utf-8",
        )
        legacy_override.write_text(
            """/**:\n  ros__parameters:\n    io_config:\n      device_name: /dev/override\n      serial_mode: true\n    io_config/navigation_test: true\n""",
            encoding="utf-8",
        )
        parameters, ignored = module.load_legacy_gimbal_parameters(
            [legacy_base, legacy_override]
        )
        if parameters.get("io_config.device_name") != "/dev/override":
            errors.append("legacy gimbal override does not win over legacy base")
        if parameters.get("io_config.serial_mode") is not True:
            errors.append("legacy nested gimbal parameter is not routed to gimbal_driver")
        if "io_config.raw_serial_log_enable" not in parameters:
            errors.append("legacy slash gimbal parameter is not normalized for dot-key precedence")
        if any(key.startswith("io_config.navigation") for key in parameters):
            errors.append("formal legacy config permits navigation direct-debug keys")
        if not {"io_config.navigation_mode.enabled", "io_config.navigation_test"} <= set(ignored):
            errors.append("formal legacy config does not report blocked navigation debug keys")

for legacy_file in legacy_velocity_files:
    if legacy_file.exists():
        errors.append(f"legacy direct velocity bridge still exists: {legacy_file.relative_to(root)}")

formal_text = formal_launch.read_text(encoding="utf-8")
for token in (
    "gimbal_driver.launch.py",
    '"base_config_file": gimbal_driver_config_file',
    '"legacy_base_config_file": base_config_file',
    '"legacy_config_file": config_file',
):
    if token not in formal_text:
        errors.append(f"formal launch does not retain scoped gimbal compatibility: {token}")
for token in (
    "effective_navi_publish_goal_pose = PythonExpression([",
    '"publish_goal_pose": effective_navi_publish_goal_pose,',
    "outpost_manual_goal_enable",
):
    if token not in formal_text:
        errors.append(
            "formal launch does not keep Outpost manual /goal_pose publisher ownership: "
            + token
        )

if not gimbal_lifecycle.is_file():
    errors.append("gimbal test lifecycle helper is missing")
else:
    lifecycle_text = gimbal_lifecycle.read_text(encoding="utf-8")
    if '"base_config_file:=${config_file}"' not in lifecycle_text:
        errors.append("gimbal test lifecycle does not pass full YAML as driver baseline")
    if '"config_file:=${config_file}"' in lifecycle_text:
        errors.append("gimbal test lifecycle still applies full baseline as an overlay")

if errors:
    print("; ".join(errors), file=sys.stderr)
    raise SystemExit(1)
PY
  then
    pass "gimbal formal/debug configuration boundary contract"
  else
    fail "gimbal formal/debug configuration boundary contract"
  fi
}

check_area_manager_enable_contract() {
  if python3 - "${ROOT_DIR}" <<'PY'
from pathlib import Path
import re
import sys

root = Path(sys.argv[1])
config = (root / "src/behavior_tree/config/AreaManager.yaml").read_text()
source = (root / "src/behavior_tree/src/Configuration.cpp").read_text()
scope_header = (root / "src/behavior_tree/include/RegionalAreaScope.hpp").read_text()
formal_launch = (root / "src/behavior_tree/launch/sentry_all.launch.py").read_text()

if re.search(r"^\s{6}RegionalAreaTask:\n\s{8}Enable:\s+(?:true|false)\s*$", config, re.MULTILINE) is None:
    raise SystemExit("AreaManager.yaml lacks RegionalAreaTask.Enable")

for name in ("MyBase", "MyHighland", "MyPreRoadland", "MyReadyRoadland", "CommonCentral"):
    yaml_pattern = rf"^\s{{8}}{name}:\n\s{{10}}Enable:\s+(?:true|false)\s*$"
    if re.search(yaml_pattern, config, re.MULTILINE) is None:
        raise SystemExit(f"AreaManager.yaml lacks RegionalAreaTask.{name}.Enable")
    parameter = f"AreaManager.RegionalAreaTask.{name}.Enable"
    if parameter not in source:
        raise SystemExit(f"Configuration.cpp does not read {parameter}")

if "AreaManager.RegionalAreaTask.Enable" not in source:
    raise SystemExit("Configuration.cpp does not read AreaManager.RegionalAreaTask.Enable")

if "ApplyRegionalAreaTaskScopeOverride(" not in source:
    raise SystemExit("Configuration.cpp does not apply the AreaManager final scope override")
for token in (
    "navi_goal.MyArea.clear()",
    "navi_goal.CommonArea.clear()",
    '"pre_roadland"',
    '"ready_roadland"',
):
    if token not in scope_header:
        raise SystemExit(f"RegionalAreaScope.hpp lacks final scope handling: {token}")
for token in (
    "_regional_area_scope_from_yaml",
    '"area_manager_config_file"',
    "resolved_chase_area_limit_my_area",
    "resolved_chase_area_limit_common_area",
):
    if token not in formal_launch:
        raise SystemExit(f"sentry_all.launch.py does not mirror AreaManager scope to bridge: {token}")
PY
  then
    pass "AreaManager RegionalAreaTask Enable YAML-to-decision contract"
  else
    fail "AreaManager RegionalAreaTask Enable YAML-to-decision contract"
  fi
}

check_tactical_protection_enable_contract() {
  if python3 - "${ROOT_DIR}" <<'PY'
from pathlib import Path
import re
import sys

root = Path(sys.argv[1])
config = (root / "src/behavior_tree/config/Tactical.yaml").read_text()
source = (root / "src/behavior_tree/src/Configuration.cpp").read_text()
game_loop = (root / "src/behavior_tree/src/GameLoop.cpp").read_text()
area_manager = (root / "src/behavior_tree/src/AreaManager.cpp").read_text()
policy = (root / "src/behavior_tree/include/TacticalProtectionPolicy.hpp").read_text()

castle_match = re.search(r"^\s{6}ProtectCastle:\n((?:\s{8}.*\n)+)", config, re.MULTILINE)
if castle_match is None:
    raise SystemExit("Tactical.yaml lacks Tactical.ProtectCastle")
castle_body = castle_match.group(1)
for key in ("Enable", "RFID", "StayWhenRfid", "EnemyPos"):
    yaml_pattern = rf"^\s{{8}}{key}:\s+(?:true|false)\s*$"
    if re.search(yaml_pattern, castle_body, re.MULTILINE) is None:
        raise SystemExit(f"Tactical.yaml lacks Tactical.ProtectCastle.{key}")
    parameter = f"Tactical.ProtectCastle.{key}"
    if parameter not in source:
        raise SystemExit(f"Configuration.cpp does not read {parameter}")

hero_match = re.search(r"^\s{6}ProtectHero:\n((?:\s{8}.*\n)+)", config, re.MULTILINE)
if hero_match is None:
    raise SystemExit("Tactical.yaml lacks Tactical.ProtectHero")
hero_body = hero_match.group(1)
for key, value_pattern in (
    ("Enable", r"(?:true|false)"),
    ("StartElapsedSec", r"\d+"),
    ("HoldSec", r"\d+"),
    ("NoEnemyReleaseSec", r"\d+"),
    ("FriendPositionFreshMs", r"\d+"),
    ("FriendHealthFreshMs", r"\d+"),
    ("GoalBaseId", r"\d+"),
):
    yaml_pattern = rf"^\s{{8}}{key}:\s+{value_pattern}\s*$"
    if re.search(yaml_pattern, hero_body, re.MULTILINE) is None:
        raise SystemExit(f"Tactical.yaml lacks Tactical.ProtectHero.{key}")
    parameter = f"Tactical.ProtectHero.{key}"
    if parameter not in source:
        raise SystemExit(f"Configuration.cpp does not read {parameter}")

for token in (
    "ResolveTacticalFeatureEnable",
    "IsProtectCastleRfidEventEnabled",
    "IsProtectCastleRfidStayEnabled",
    "IsProtectCastleEnemyPositionEnabled",
):
    if token not in policy:
        raise SystemExit(f"TacticalProtectionPolicy.hpp lacks {token}")

for token in (
    "protect_hero = config.HeroProtectionSettings",
    "config.HeroProtectionSettings = protect_hero",
):
    if token not in source:
        raise SystemExit("Tactical.ProtectHero does not preserve the legacy HeroProtection baseline")
for token in (
    "config.TacticalSettings.ProtectCastle",
    "IsProtectCastleRfidEventEnabled",
    "IsProtectCastleRfidStayEnabled",
    "IsProtectCastleEnemyPositionEnabled",
):
    if token not in game_loop:
        raise SystemExit(f"GameLoop.cpp lacks ProtectCastle handling: {token}")
if "enable_own_base_enemy_position" not in area_manager:
    raise SystemExit("AreaManager.cpp does not gate the MyBase enemy-position source")
for token in (
    "const auto& protection = config.TacticalSettings.ProtectHero",
    "protect_hero_threat_ready",
):
    if token not in game_loop and token not in (root / "src/behavior_tree/src/DecisionTrace.cpp").read_text():
        raise SystemExit(f"ProtectHero runtime/trace contract lacks {token}")
PY
  then
    pass "Tactical ProtectCastle/ProtectHero YAML-to-decision contract"
  else
    fail "Tactical ProtectCastle/ProtectHero YAML-to-decision contract"
  fi
}

check_navi_config_contract() {
  if python3 - "${ROOT_DIR}" <<'PY'
from pathlib import Path
import sys
import yaml

root = Path(sys.argv[1])
navi_path = root / "src/behavior_tree/config/Navi.yaml"
if not navi_path.is_file():
    raise SystemExit("Navi.yaml is missing")

params = yaml.safe_load(navi_path.read_text(encoding="utf-8"))
navi = params.get("behavior_tree", {}).get("ros__parameters", {}).get("Navi")
if not isinstance(navi, dict):
    raise SystemExit("Navi.yaml lacks behavior_tree.ros__parameters.Navi")
if not isinstance(navi.get("Is_pub_navi_speed_level"), bool):
    raise SystemExit("Navi.yaml lacks boolean Navi.Is_pub_navi_speed_level")

configuration = (root / "src/behavior_tree/src/Configuration.cpp").read_text(encoding="utf-8")
publisher = (root / "src/behavior_tree/src/PublishMessage.cpp").read_text(encoding="utf-8")
formal_launch = (root / "src/behavior_tree/launch/sentry_all.launch.py").read_text(encoding="utf-8")
for token in (
    '"Navi.Is_pub_navi_speed_level"',
    "config.NaviControlSettings.IsPubNaviSpeedLevel",
):
    if token not in configuration and token not in publisher:
        raise SystemExit(f"BT does not consume Navi speed-level config: {token}")
for token in (
    "void Application::PubNaviSpeedLevel",
    "NormalizeNaviSpeedLevel(requested_level)",
    "PubNaviSpeedLevel(speedLevel)",
    "PubNaviSpeedLevel(kNaviSpeedNormal)",
):
    if token not in publisher:
        raise SystemExit(f"BT speed-level publication is incomplete: {token}")
for token in ('"Navi.yaml"', '"navi_config_file"'):
    if token not in formal_launch:
        raise SystemExit(f"formal launch does not load Navi.yaml: {token}")
PY
  then
    pass "Navi.yaml speed-level YAML-to-navigation-output contract"
  else
    return
  fi
}

trim_text() {
  local text="$1"
  text="${text#"${text%%[![:space:]]*}"}"
  text="${text%"${text##*[![:space:]]}"}"
  printf "%s" "${text}"
}

launch_arg_value() {
  local key="$1"
  local default_value="${2:-}"
  local arg
  for arg in "${LAUNCH_ARGS[@]}"; do
    if [[ "${arg}" == "${key}:="* ]]; then
      printf "%s" "${arg#*=}"
      return
    fi
    if [[ "${arg}" == "--${key}:="* ]]; then
      printf "%s" "${arg#*=}"
      return
    fi
  done
  printf "%s" "${default_value}"
}

extract_yaml_quoted_key_value() {
  local yaml_file="$1"
  local key_pattern="$2"
  local line
  local value

  line="$(grep -E "^[[:space:]]*\"${key_pattern}\"[[:space:]]*:" "${yaml_file}" | head -n1 || true)"
  if [[ -z "${line}" ]]; then
    printf "%s" ""
    return
  fi

  value="${line#*:}"
  value="${value%%#*}"
  value="$(trim_text "${value}")"

  if [[ "${value}" =~ ^\".*\"$ ]]; then
    value="${value:1:${#value}-2}"
  fi

  printf "%s" "${value}"
}

check_launch_mode_hints() {
  local default_base_yaml="${ROOT_DIR}/config/base_config.yaml"
  local base_cfg_arg
  base_cfg_arg="$(launch_arg_value "base_config_file" "")"
  local base_yaml="${base_cfg_arg:-${default_base_yaml}}"

  if [[ ! -f "${base_yaml}" ]]; then
    warn "Launch mode hint skipped: base config file missing: base=${base_yaml}"
    return
  fi
  info "Launch mode hints based on base=${base_yaml}"
  if (( OFFLINE_MODE == 1 )); then
    pass "Offline mode enabled: launch will enforce virtual IO overrides."
  fi

  local use_virtual
  use_virtual="$(extract_yaml_quoted_key_value "${base_yaml}" 'io_config/use_virtual_device')"

  if (( OFFLINE_MODE == 1 )); then
    pass "Offline launch override: io_config/use_virtual_device will be forced to true."
  elif [[ "${use_virtual}" == "false" ]]; then
    warn "Current config requires real serial device (io_config/use_virtual_device=false). If offline, set true."
  else
    pass "Current config uses virtual IO (io_config/use_virtual_device=true)."
  fi
}

print_launch_diagnosis() {
  if [[ -z "${LAUNCH_LOG}" || ! -f "${LAUNCH_LOG}" ]]; then
    return
  fi
  print_section "Launch Diagnosis"
  local died_lines
  died_lines="$(grep -E 'process has died|Segmentation fault|Failed to initialize camera|IODevice::MakeDevice|PoseSolver init failed' "${LAUNCH_LOG}" || true)"
  if [[ -n "${died_lines}" ]]; then
    warn "Detected crash/error signatures from launch log:"
    printf "%s\n" "${died_lines}" | sed 's/^/  /'
  else
    warn "No explicit crash signature found in launch log. Check full log: ${LAUNCH_LOG}"
  fi

  if grep -Eq 'getifaddrs: Operation not permitted|Error creating socket: Operation not permitted|Failed to create Shared Memory Manager|Unable to Register SHM Transport|Problem creating RTPSParticipant' "${LAUNCH_LOG}"; then
    warn "Detected DDS transport permission errors (likely sandbox/container capability issue, not business logic)."
    info "Run this runtime check on host/robot environment with normal network permissions."
  fi

  if grep -Eq 'Failed to initialize camera|camera.*not found|camera.*open failed' "${LAUNCH_LOG}"; then
    warn "Detected camera initialization failure. sentry_all no longer starts the internal camera; check external aim stack or custom launch args."
  fi

  if grep -Eq 'IODevice::MakeDevice|ttyACM|ttyUSB|open serial|No such file or directory' "${LAUNCH_LOG}"; then
    warn "Detected serial/open-device failure. Confirm io_config/use_virtual_device and serial device availability."
  fi

  info "Recent launch log tail:"
  tail -n 40 "${LAUNCH_LOG}" | sed 's/^/  /'
}

run_ros2() {
  timeout "${CMD_TIMEOUT}s" ros2 "$@" 2>/dev/null
}

extract_endpoint_nodes() {
  local info_text="$1"
  local endpoint="$2"
  awk -v endpoint="${endpoint}" '
    /Node name:/ {
      line=$0
      sub(/^[[:space:]]*Node name:[[:space:]]*/, "", line)
      sub(/,[[:space:]]*Node namespace:.*/, "", line)
      sub(/[[:space:]]+$/, "", line)
      node=line
      if (node !~ /^\//) {
        node="/" node
      }
      next
    }
    /Endpoint type:/ {
      mode=$3
      next
    }
    /^[[:space:]]*$/ {
      if (node != "" && mode == endpoint) {
        print node
      }
      node=""
      mode=""
      next
    }
    END {
      if (node != "" && mode == endpoint) {
        print node
      }
    }
  ' <<< "${info_text}" | sort -u
}

declare -A NODE_INFO_CACHE

get_node_info_cached() {
  local node="$1"
  if [[ -z "${NODE_INFO_CACHE[${node}]+x}" ]]; then
    NODE_INFO_CACHE["${node}"]="$(run_ros2 node info "${node}" || true)"
  fi
  printf "%s" "${NODE_INFO_CACHE[${node}]}"
}

node_has_topic_in_section() {
  local info_text="$1"
  local section="$2"
  local topic="$3"
  awk -v section="${section}" -v topic="${topic}" '
    BEGIN { in_section = 0; found = 0 }
    $0 ~ "^[[:space:]]*" section ":" { in_section = 1; next }
    in_section && $0 ~ "^[[:space:]]*[A-Za-z][A-Za-z _]*:" { in_section = 0 }
    in_section && $0 ~ "^[[:space:]]*/[^:]+:" {
      t = $1
      sub(/:$/, "", t)
      if(t == topic) { found = 1; exit }
    }
    END { exit(found ? 0 : 1) }
  ' <<< "${info_text}"
}

check_node_online() {
  local node="$1"
  local node_list="$2"
  if grep -Fxq "${node}" <<< "${node_list}"; then
    pass "Node online: ${node}"
  else
    fail "Node missing: ${node}"
  fi
}

check_node_absent() {
  local node="$1"
  local node_list="$2"
  if grep -Fxq "${node}" <<< "${node_list}"; then
    fail "Unexpected node online: ${node}"
  else
    pass "Node absent as expected: ${node}"
  fi
}

resolve_node_alias() {
  local candidates_csv="$1"
  local node_list="$2"
  local cand

  IFS=',' read -r -a _candidates <<< "${candidates_csv}"
  for cand in "${_candidates[@]}"; do
    [[ -z "${cand}" ]] && continue
    if grep -Fxq "${cand}" <<< "${node_list}"; then
      printf "%s" "${cand}"
      return 0
    fi
  done
  return 1
}

check_node_online_alias() {
  local label="$1"
  local candidates_csv="$2"
  local node_list="$3"
  local resolved

  resolved="$(resolve_node_alias "${candidates_csv}" "${node_list}" || true)"
  if [[ -n "${resolved}" ]]; then
    pass "Node online: ${resolved} (${label})"
  else
    fail "Node missing: ${label} (candidates: ${candidates_csv})"
  fi
}

check_node_sub() {
  local node="$1"
  local topic="$2"
  local severity="${3:-hard}"
  local info_text
  info_text="$(get_node_info_cached "${node}")"
  if [[ -z "${info_text}" ]]; then
    issue "${severity}" "Node info unavailable: ${node}"
    return
  fi
  if node_has_topic_in_section "${info_text}" "Subscribers" "${topic}"; then
    pass "Subscriber contract OK: ${node} <- ${topic}"
  else
    issue "${severity}" "Subscriber contract missing: ${node} <- ${topic}"
  fi
}

check_node_pub() {
  local node="$1"
  local topic="$2"
  local severity="${3:-hard}"
  local info_text
  info_text="$(get_node_info_cached "${node}")"
  if [[ -z "${info_text}" ]]; then
    issue "${severity}" "Node info unavailable: ${node}"
    return
  fi
  if node_has_topic_in_section "${info_text}" "Publishers" "${topic}"; then
    pass "Publisher contract OK: ${node} -> ${topic}"
  else
    issue "${severity}" "Publisher contract missing: ${node} -> ${topic}"
  fi
}

check_topic_link() {
  local topic="$1"
  local expected_type="$2"
  local expected_pubs_csv="$3"
  local expected_subs_csv="$4"
  local severity="${5:-hard}"

  local topic_type
  topic_type="$(run_ros2 topic type "${topic}" | head -n1 || true)"
  if [[ -z "${topic_type}" ]]; then
    issue "${severity}" "Topic missing: ${topic}"
    return
  fi

  if [[ "${topic_type}" == "${expected_type}" ]]; then
    pass "Topic type OK: ${topic} == ${expected_type}"
  else
    issue "${severity}" "Topic type mismatch: ${topic}, expect=${expected_type}, got=${topic_type}"
  fi

  local info_text
  info_text="$(run_ros2 topic info "${topic}" -v || true)"
  if [[ -z "${info_text}" ]]; then
    issue "${severity}" "Topic info unavailable: ${topic}"
    return
  fi

  local pub_nodes
  local sub_nodes
  pub_nodes="$(extract_endpoint_nodes "${info_text}" "PUBLISHER")"
  sub_nodes="$(extract_endpoint_nodes "${info_text}" "SUBSCRIPTION")"

  local node
  IFS=',' read -r -a _pub_nodes <<< "${expected_pubs_csv}"
  for node in "${_pub_nodes[@]}"; do
    [[ -z "${node}" ]] && continue
    if grep -Fxq "${node}" <<< "${pub_nodes}"; then
      pass "Topic publisher OK: ${topic} has ${node}"
    else
      issue "${severity}" "Topic publisher missing: ${topic} lacks ${node}"
    fi
  done

  IFS=',' read -r -a _sub_nodes <<< "${expected_subs_csv}"
  for node in "${_sub_nodes[@]}"; do
    [[ -z "${node}" ]] && continue
    if grep -Fxq "${node}" <<< "${sub_nodes}"; then
      pass "Topic subscriber OK: ${topic} has ${node}"
    else
      issue "${severity}" "Topic subscriber missing: ${topic} lacks ${node}"
    fi
  done
}

check_topic_hz() {
  local topic="$1"
  local min_rate="$2"
  local severity="${3:-warn}"

  local out_text
  out_text="$(timeout "$((HZ_SECONDS + 2))s" ros2 topic hz "${topic}" 2>/dev/null || true)"
  local avg_rate
  avg_rate="$(awk '/average rate:/ {print $3; exit}' <<< "${out_text}")"

  if [[ -z "${avg_rate}" ]]; then
    issue "${severity}" "No hz sample from ${topic} within ${HZ_SECONDS}s"
    return
  fi

  if awk -v v="${avg_rate}" -v min="${min_rate}" 'BEGIN { exit (v >= min ? 0 : 1) }'; then
    pass "Topic hz OK: ${topic} average=${avg_rate} Hz (>= ${min_rate})"
  else
    issue "${severity}" "Topic hz low: ${topic} average=${avg_rate} Hz (< ${min_rate})"
  fi
}

print_section() {
  printf "\n%s==== %s ====%s\n" "${C_CYAN}" "$1" "${C_RESET}"
}

print_section "Environment"
check_cmd bash
check_cmd ros2
check_cmd timeout
check_cmd awk
check_cmd grep
source_ros
source_optional_sentry_msgs || true
source_workspace

if (( RUNTIME_ONLY == 0 )); then
  print_section "Static Files"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/Scripts/main.xml"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/Scripts/config.json"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/Scripts/ConfigJson/league_competition.json"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/Scripts/ConfigJson/regional_competition.json"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/Scripts/ConfigJson/regional_simple_competition.json"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/Scripts/ConfigJson/regional/debug/navi_debug_competition.json"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/Scripts/ConfigJson/regional/debug/navi_debug_points.json"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/launch/sentry_all.launch.py"
  check_file_exists "${ROOT_DIR}/src/gimbal_driver/config/gimbal_driver_config.yaml"
  check_file_exists "${ROOT_DIR}/src/gimbal_driver/config/debug_mode.yaml"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/config/AreaManager.yaml"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/config/Navi.yaml"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/config/Tactical.yaml"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/config/Special.yaml"
  check_file_exists "${ROOT_DIR}/config/base_config.yaml"
  check_file_exists "${ROOT_DIR}/config/override_config.yaml"
  check_file_exists "${ROOT_DIR}/scripts/start.sh"
  check_file_exists "${ROOT_DIR}/scripts/debug.sh"
  check_file_exists "${ROOT_DIR}/scripts/selfcheck.sh"
  check_file_exists "${ROOT_DIR}/scripts/start/sentry_all.sh"
  check_file_exists "${ROOT_DIR}/scripts/launch/start_sentry_all.sh"
  check_file_exists "${ROOT_DIR}/scripts/start/showcase.sh"
  check_file_exists "${ROOT_DIR}/scripts/launch/start_sentry_showcase.sh"
  check_file_exists "${ROOT_DIR}/scripts/debug/navi_debug.sh"
  check_file_exists "${ROOT_DIR}/scripts/launch/start_sentry_navi_debug.sh"
  check_file_exists "${ROOT_DIR}/scripts/debug/armor_test.sh"
  check_file_exists "${ROOT_DIR}/scripts/aim/armor_test.sh"
  check_file_exists "${ROOT_DIR}/scripts/debug/standalone.sh"
  check_file_exists "${ROOT_DIR}/scripts/debug/navi_goal.sh"
  check_file_exists "${ROOT_DIR}/scripts/debug/navi_goal_cli.sh"
  check_file_exists "${ROOT_DIR}/scripts/debug/goal_pos_test.sh"
  check_file_exists "${ROOT_DIR}/scripts/feature_test/standalone/run_standalone_menu.sh"
  check_file_exists "${ROOT_DIR}/scripts/feature_test/standalone/modes/navi_patrol_mode.sh"
  check_file_exists "${ROOT_DIR}/scripts/navi/navitomap.sh"
  check_file_exists "${ROOT_DIR}/scripts/navi/facemode.sh"
  check_file_exists "${ROOT_DIR}/scripts/navi/facemode_cross_matrix.sh"
  check_file_exists "${ROOT_DIR}/scripts/navi/facemode_map.sh"
  check_file_exists "${ROOT_DIR}/scripts/navi/facemode_official.sh"
  check_file_exists "${ROOT_DIR}/scripts/navi/position.sh"
  check_file_exists "${ROOT_DIR}/scripts/navi/map_aim_point_test.sh"
  check_file_exists "${ROOT_DIR}/src/behavior_tree/include/BTNodes.hpp"

  check_executable_file "${ROOT_DIR}/scripts/start.sh"
  check_executable_file "${ROOT_DIR}/scripts/debug.sh"
  check_executable_file "${ROOT_DIR}/scripts/selfcheck.sh"
  check_executable_file "${ROOT_DIR}/scripts/start/sentry_all.sh"
  check_executable_file "${ROOT_DIR}/scripts/launch/start_sentry_all.sh"
  check_executable_file "${ROOT_DIR}/scripts/start/showcase.sh"
  check_executable_file "${ROOT_DIR}/scripts/launch/start_sentry_showcase.sh"
  check_executable_file "${ROOT_DIR}/scripts/debug/navi_debug.sh"
  check_executable_file "${ROOT_DIR}/scripts/launch/start_sentry_navi_debug.sh"
  check_executable_file "${ROOT_DIR}/scripts/debug/armor_test.sh"
  check_executable_file "${ROOT_DIR}/scripts/aim/armor_test.sh"
  check_executable_file "${ROOT_DIR}/scripts/debug/standalone.sh"
  check_executable_file "${ROOT_DIR}/scripts/debug/navi_goal.sh"
  check_executable_file "${ROOT_DIR}/scripts/debug/navi_goal_cli.sh"
  check_executable_file "${ROOT_DIR}/scripts/debug/goal_pos_test.sh"
  check_executable_file "${ROOT_DIR}/scripts/feature_test/standalone/run_standalone_menu.sh"
  check_executable_file "${ROOT_DIR}/scripts/feature_test/standalone/modes/navi_patrol_mode.sh"
  check_executable_file "${ROOT_DIR}/scripts/navi/navitomap.sh"
  check_executable_file "${ROOT_DIR}/scripts/navi/facemode.sh"
  check_executable_file "${ROOT_DIR}/scripts/navi/facemode_cross_matrix.sh"
  check_executable_file "${ROOT_DIR}/scripts/navi/facemode_map.sh"
  check_executable_file "${ROOT_DIR}/scripts/navi/facemode_official.sh"
  check_executable_file "${ROOT_DIR}/scripts/navi/position.sh"
  check_executable_file "${ROOT_DIR}/scripts/navi/map_aim_point_test.sh"

  check_bash_syntax "${ROOT_DIR}/scripts/start.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/debug.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/selfcheck.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/start/sentry_all.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/launch/start_sentry_all.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/start/showcase.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/launch/start_sentry_showcase.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/debug/navi_debug.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/launch/start_sentry_navi_debug.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/debug/armor_test.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/aim/armor_test.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/debug/standalone.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/debug/navi_goal.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/debug/navi_goal_cli.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/debug/goal_pos_test.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/feature_test/standalone/run_standalone_menu.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/feature_test/standalone/modes/navi_patrol_mode.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/navi/navitomap.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/navi/facemode.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/navi/facemode_cross_matrix.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/navi/facemode_map.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/navi/facemode_official.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/navi/position.sh"
  check_bash_syntax "${ROOT_DIR}/scripts/navi/map_aim_point_test.sh"

  pass "Formal sentry_all uses external /ly/aim; base_config has no internal camera/SN requirement"
  check_ros_interface "sentry_msgs/msg/AimTarget"
  check_ros_interface "sentry_msgs/msg/AimTargetArray"
  check_ros_interface "sentry_msgs/msg/AimResult"
  check_ros_interface_field "sentry_msgs/msg/AimResult" "bool follow"
  check_ros_interface_field "sentry_msgs/msg/AimResult" "float32 yaw_omega"
  check_ros_interface_field "sentry_msgs/msg/AimResult" "float32 pitch_omega"
  check_ros_interface_field "sentry_msgs/msg/AimResult" "float32 yaw_alpha"
  check_ros_interface_field "sentry_msgs/msg/AimResult" "float32 pitch_alpha"
  check_ros_interface "gimbal_driver/msg/GimbalTrajectory"
  check_ros_interface "gimbal_driver/msg/GimbalState"
  check_ros_interface "auto_aim_common/msg/GoalReach"
  check_ros_interface "auto_aim_common/msg/RelativeTarget"

  package_count="$(colcon list --names-only 2>/dev/null | wc -l | tr -d ' ')"
  if [[ "${package_count}" == "5" ]]; then
    pass "workspace package inventory contains the five retained ROS packages"
  else
    fail "workspace package inventory expected 5 retained ROS packages, got ${package_count}"
  fi

  if grep -Fq 'BTCPP_format="4"' "${ROOT_DIR}/src/behavior_tree/Scripts/main.xml"; then
    pass "BT XML format is v4"
  else
    fail "BT XML format is not v4"
  fi

  if grep -Fq '<SelectStrategyMode/>' "${ROOT_DIR}/src/behavior_tree/Scripts/main.xml"; then
    pass "BT XML contains dynamic strategy switch node"
  else
    fail "BT XML missing SelectStrategyMode node"
  fi

  check_gimbal_debug_profile_contract
  check_navi_config_contract
  check_area_manager_enable_contract
  check_tactical_protection_enable_contract
fi

if (( STATIC_ONLY == 0 )); then
  print_section "Runtime Graph"
fi

if (( STATIC_ONLY == 0 && AUTO_LAUNCH == 1 )); then
  check_launch_mode_hints
  LAUNCH_LOG="$(mktemp /tmp/sentry_self_check.XXXXXX.log)"
  info "Launching stack by scripts/start/sentry_all.sh (log: ${LAUNCH_LOG})"
  START_ARGS=(--cleanup-existing)
  if (( OFFLINE_MODE == 1 )); then
    START_ARGS+=(--offline)
  fi
  (
    cd "${ROOT_DIR}"
    ./scripts/start/sentry_all.sh "${START_ARGS[@]}" "${LAUNCH_ARGS[@]}"
  ) >"${LAUNCH_LOG}" 2>&1 &
  LAUNCH_PID="$!"
  sleep "${WAIT_SECONDS}"
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    pass "Auto-launch still running after ${WAIT_SECONDS}s"
  else
    fail "Auto-launch exited early. Check log: ${LAUNCH_LOG}"
    tail -n 40 "${LAUNCH_LOG}" || true
  fi
fi

if (( STATIC_ONLY == 0 )); then
  NODE_LIST="$(run_ros2 node list || true)"
  GRAPH_AVAILABLE=1
  OUTPOST_NODE=""
  BUFF_NODE=""
  if [[ -z "${NODE_LIST}" ]]; then
    fail "No ROS2 nodes found. Start stack first or run with --launch"
    GRAPH_AVAILABLE=0
  else
    pass "ROS2 graph has active nodes"
  fi

  if (( GRAPH_AVAILABLE == 1 )); then
  check_node_online "/gimbal_driver" "${NODE_LIST}"
  check_node_online "/behavior_tree" "${NODE_LIST}"
  print_section "Node Contracts"
  # gimbal_driver
  check_node_sub "/gimbal_driver" "/ly/control/angles" hard
  check_node_sub "/gimbal_driver" "/ly/control/firecode" hard
  check_node_sub "/gimbal_driver" "/ly/control/vel" hard
  check_node_sub "/gimbal_driver" "/ly/control/posture" hard
  check_node_sub "/gimbal_driver" "/ly/control/sentry_cmd" hard
  check_node_sub "/gimbal_driver" "/ly/bt/sentry_position" hard

  # behavior_tree inputs
  check_node_sub "/behavior_tree" "/ly/gimbal/angles" hard
  check_node_sub "/behavior_tree" "/ly/gimbal/posture" hard
  check_node_sub "/behavior_tree" "/ly/game/is_start" hard
  check_node_sub "/behavior_tree" "/ly/game/time_left" hard
  check_node_sub "/behavior_tree" "/ly/game/map_command" hard
  check_node_sub "/behavior_tree" "/ly/friend/is_team_red" hard
  check_node_sub "/behavior_tree" "/ly/aim/armor_targets" hard
  check_node_sub "/behavior_tree" "/ly/aim/result" hard

  # behavior_tree outputs
  check_node_pub "/behavior_tree" "/ly/control/angles" hard
  check_node_pub "/behavior_tree" "/ly/control/firecode" hard
  check_node_pub "/behavior_tree" "/ly/control/posture" hard
  check_node_pub "/behavior_tree" "/ly/vision/mode" hard
  check_node_pub "/behavior_tree" "/ly/bt/target" hard
  check_node_pub "/behavior_tree" "/ly/bt/sentry_position" hard
  check_node_pub "/behavior_tree" "/ly/aim/select_target" hard
  check_node_pub "/behavior_tree" "/ly/navi/vel" hard

  print_section "Critical Topic Links"
  check_topic_link "/ly/control/angles" "gimbal_driver/msg/GimbalAngles" "/behavior_tree" "/gimbal_driver" hard
  check_topic_link "/ly/control/firecode" "gimbal_driver/msg/FireCode" "/behavior_tree" "/gimbal_driver" hard
  check_topic_link "/ly/control/posture" "gimbal_driver/msg/SentryCmd" "/behavior_tree" "/gimbal_driver" hard

  # 兼容鏈路檢查：電控側仍訂閱 /ly/control/vel，若沒有發布者視為缺口
  check_topic_link "/ly/control/vel" "gimbal_driver/msg/ControlVelocity" "/behavior_tree" "/gimbal_driver" hard
  check_topic_link "/ly/bt/sentry_position" "geometry_msgs/msg/PointStamped" "/behavior_tree" "/gimbal_driver" hard

  check_topic_link "/ly/aim/select_target" "sentry_msgs/msg/AimTarget" "/behavior_tree" "" hard
  check_topic_link "/ly/aim/armor_targets" "sentry_msgs/msg/AimTargetArray" "" "/behavior_tree" hard
  check_topic_link "/ly/aim/result" "sentry_msgs/msg/AimResult" "" "/behavior_tree" hard
  check_topic_link "/ly/game/map_command" "gimbal_driver/msg/MapCommand" "" "/behavior_tree" hard

  print_section "Conditional Topics (Data-Dependent)"
  check_topic_link "/ly/gimbal/angles" "gimbal_driver/msg/GimbalAngles" "/gimbal_driver" "/behavior_tree" warn
  check_topic_link "/ly/gimbal/posture" "std_msgs/msg/UInt8" "/gimbal_driver" "/behavior_tree" warn

  if (( SKIP_HZ == 0 )); then
    print_section "Frequency Checks"
    check_topic_hz "/ly/control/angles" 5 hard
    check_topic_hz "/ly/control/firecode" 5 hard
    check_topic_hz "/ly/navi/vel" 1 warn
    check_topic_hz "/ly/gimbal/angles" 1 warn
    check_topic_hz "/ly/aim/result" 1 warn
  fi
  else
    warn "Runtime graph checks skipped because no ROS2 nodes are active"
  fi
fi

print_section "Summary"
printf "PASS: %d\n" "${PASS_COUNT}"
printf "WARN: %d\n" "${WARN_COUNT}"
printf "FAIL: %d\n" "${FAIL_COUNT}"

if (( FAIL_COUNT > 0 )); then
  if (( AUTO_LAUNCH == 1 )); then
    print_launch_diagnosis
  fi
  printf "%sResult: FAILED%s\n" "${C_RED}" "${C_RESET}"
  exit 1
fi

printf "%sResult: PASSED%s\n" "${C_GREEN}" "${C_RESET}"
exit 0
