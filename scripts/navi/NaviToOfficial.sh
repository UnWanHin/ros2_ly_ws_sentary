#!/usr/bin/env bash

# Pure static navigation-map -> official-map pointer conversion.
# Reads navi_tf_bridge/config/tf_config.yaml and applies the inverse raw-goal matrix.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT_NAME="$(basename "$0")"

CONFIG_FILE="${ROOT_DIR}/src/navi_tf_bridge/config/tf_config.yaml"
INPUT_UNIT="m"
OUTPUT_UNIT="cm"

usage() {
  cat <<EOF
Usage:
  ${SCRIPT_NAME} [options] <navi_map_x> <navi_map_y> [map_z]

Options:
  --config <path>        tf_config.yaml path (default: src/navi_tf_bridge/config/tf_config.yaml)
  --input-unit m|cm      input unit for x/y/z (default: m)
  --output-unit cm|m     output unit (default: cm)
  --help

Output:
  Converted official-map coordinates only. If z is omitted, prints: x y
  If z is provided, prints: x y z
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      CONFIG_FILE="${2:-}"
      shift 2
      ;;
    --input-unit)
      INPUT_UNIT="${2:-}"
      shift 2
      ;;
    --output-unit)
      OUTPUT_UNIT="${2:-}"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    -*)
      echo "[ERROR] Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      break
      ;;
  esac
done

if (( $# < 2 || $# > 3 )); then
  usage >&2
  exit 2
fi
if [[ -z "${CONFIG_FILE}" || ! -f "${CONFIG_FILE}" ]]; then
  echo "[ERROR] tf_config.yaml not found: ${CONFIG_FILE}" >&2
  exit 1
fi
if [[ "${INPUT_UNIT}" != "cm" && "${INPUT_UNIT}" != "m" ]]; then
  echo "[ERROR] --input-unit must be m or cm." >&2
  exit 2
fi
if [[ "${OUTPUT_UNIT}" != "cm" && "${OUTPUT_UNIT}" != "m" ]]; then
  echo "[ERROR] --output-unit must be cm or m." >&2
  exit 2
fi

python3 - "${CONFIG_FILE}" "${INPUT_UNIT}" "${OUTPUT_UNIT}" "$@" <<'PY'
import re
import sys


def parse_bool(value, default=False):
    if value is None:
        return default
    text = str(value).strip().lower()
    if text in {"true", "1", "yes", "on"}:
        return True
    if text in {"false", "0", "no", "off"}:
        return False
    return default


def scalar_from_text(text, key, default=None):
    match = re.search(rf"^\s*{re.escape(key)}\s*:\s*([^#\n]+)", text, re.MULTILINE)
    if not match:
        return default
    value = match.group(1).strip().strip("'\"")
    return value if value else default


def matrix_from_text(text, key):
    match = re.search(rf"^\s*{re.escape(key)}\s*:\s*\[(.*?)\]", text, re.MULTILINE | re.DOTALL)
    if not match:
        return []
    return [float(v) for v in re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", match.group(1))]


def load_params(config_file):
    with open(config_file, encoding="utf-8") as fh:
        text = fh.read()

    try:
        import yaml

        root = yaml.safe_load(text) or {}
        params = root.get("target_rel_to_goal_pos_node", {}).get("ros__parameters", {})
        if isinstance(params, dict):
            return params
    except Exception:
        pass

    return {
        "use_raw_goal_static_calibration": parse_bool(
            scalar_from_text(text, "use_raw_goal_static_calibration"), False
        ),
        "raw_goal_calibration_model": scalar_from_text(
            text, "raw_goal_calibration_model", "matrix"
        ),
        "raw_goal_calibration_unit": scalar_from_text(text, "raw_goal_calibration_unit", "m"),
        "goal_pos_raw_frame": scalar_from_text(text, "goal_pos_raw_frame", "map"),
        "map_frame": scalar_from_text(text, "map_frame", "map"),
        "raw_goal_transform_matrix": matrix_from_text(text, "raw_goal_transform_matrix"),
    }


config_file, input_unit, output_unit = sys.argv[1:4]
args = sys.argv[4:]

try:
    x_in = float(args[0])
    y_in = float(args[1])
    z_in = float(args[2]) if len(args) >= 3 else None
except ValueError as exc:
    print(f"[ERROR] invalid coordinate: {exc}", file=sys.stderr)
    raise SystemExit(2)

input_scale = 0.01 if input_unit == "cm" else 1.0
output_scale = 100.0 if output_unit == "cm" else 1.0
x_m = x_in * input_scale
y_m = y_in * input_scale
z_m = z_in * input_scale if z_in is not None else None

params = load_params(config_file)
use_static = parse_bool(params.get("use_raw_goal_static_calibration"), False)

if use_static:
    model = str(params.get("raw_goal_calibration_model", "matrix"))
    if model not in {"matrix", "MATRIX", "matrix_4x4"}:
        print(
            f"[ERROR] NaviToOfficial.sh currently supports raw_goal_calibration_model=matrix only, got {model}",
            file=sys.stderr,
        )
        raise SystemExit(1)

    matrix = params.get("raw_goal_transform_matrix", [])
    if not isinstance(matrix, list) or len(matrix) != 16:
        print("[ERROR] raw_goal_transform_matrix must contain 16 values.", file=sys.stderr)
        raise SystemExit(1)

    try:
        m = [float(v) for v in matrix]
    except (TypeError, ValueError) as exc:
        print(f"[ERROR] raw_goal_transform_matrix has non-numeric value: {exc}", file=sys.stderr)
        raise SystemExit(1)

    matrix_unit = str(params.get("raw_goal_calibration_unit", "m"))
    if matrix_unit in {"cm", "CM"}:
        matrix_unit_scale = 0.01
    elif matrix_unit in {"m", "M"}:
        matrix_unit_scale = 1.0
    else:
        print(
            f"[ERROR] raw_goal_calibration_unit must be cm or m, got {matrix_unit}",
            file=sys.stderr,
        )
        raise SystemExit(1)

    a00 = m[0]
    a01 = m[1]
    a10 = m[4]
    a11 = m[5]
    tx_m = m[3] * matrix_unit_scale
    ty_m = m[7] * matrix_unit_scale
    det = a00 * a11 - a01 * a10
    if abs(det) <= 1e-12:
        print("[ERROR] raw_goal_transform_matrix XY part is singular.", file=sys.stderr)
        raise SystemExit(1)

    dx = x_m - tx_m
    dy = y_m - ty_m
    out_x_m = (a11 * dx - a01 * dy) / det
    out_y_m = (-a10 * dx + a00 * dy) / det
else:
    raw_frame = str(params.get("goal_pos_raw_frame", "map"))
    map_frame = str(params.get("map_frame", "map"))
    if raw_frame != map_frame:
        print(
            f"[ERROR] static calibration is disabled and {map_frame}->{raw_frame} needs TF.",
            file=sys.stderr,
        )
        raise SystemExit(1)
    out_x_m = x_m
    out_y_m = y_m

values = [out_x_m * output_scale, out_y_m * output_scale]
if z_m is not None:
    values.append(z_m * output_scale)

print(" ".join(f"{value:.6f}" for value in values))
PY
