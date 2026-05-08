#!/usr/bin/env python3

"""Fit a 2D affine transform and print a 4x4 matrix.

Example YAML:

source_frame: official_map
target_frame: map
# Unit mapping:
#   source official-map points are stored as cm.
#   target navi/map points are stored as m.
#   solved raw_goal_transform_matrix is emitted in m.
source_unit: cm
target_unit: m
output_unit: m
points:
  - name: p1
    source: [574.0, 1206.0, 0.0]
    target: [8.8154, -4.2515, 0.0]
  - name: p2
    source: [1043.0, 329.0, 0.0]
    target: [-0.041879, -9.656739, 0.0]
  - name: p3
    source: [924.0, 834.0, 0.0]
    target: [5.72997, -7.84072, 0.0]

The solved affine matrix maps source XY points to target XY points in output_unit.
Z is passed through unchanged.
The calibration tool defaults to meters for source, target, and matrix output.
When using official-map centimeter points, set source_unit: cm explicitly.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple

try:
    import numpy as np
except Exception as ex:  # pragma: no cover - runtime import guard
    np = None
    NUMPY_IMPORT_ERROR = ex
else:
    NUMPY_IMPORT_ERROR = None

try:
    import yaml
except Exception as ex:  # pragma: no cover - runtime import guard
    yaml = None
    YAML_IMPORT_ERROR = ex
else:
    YAML_IMPORT_ERROR = None


@dataclass
class PointPair3D:
    name: str
    source: "np.ndarray"
    target: "np.ndarray"


def _unit_scale_to_m(unit: str) -> float:
    normalized = unit.strip().lower()
    if normalized in ("m", "meter", "meters"):
        return 1.0
    if normalized in ("cm", "centimeter", "centimeters"):
        return 0.01
    if normalized in ("mm", "millimeter", "millimeters"):
        return 0.001
    raise ValueError(f"unsupported unit '{unit}', expected m/cm/mm")


def _read_xyz(node: Any, field_name: str) -> "np.ndarray":
    if isinstance(node, (list, tuple)):
        if len(node) == 2:
            return np.array([float(node[0]), float(node[1]), 0.0], dtype=float)
        if len(node) >= 3:
            return np.array([float(node[0]), float(node[1]), float(node[2])], dtype=float)

    if isinstance(node, dict):
        for kx, ky, kz in (("x", "y", "z"), ("X", "Y", "Z")):
            if kx in node and ky in node:
                return np.array(
                    [float(node[kx]), float(node[ky]), float(node.get(kz, 0.0))],
                    dtype=float,
                )

    raise ValueError(f"invalid {field_name}: expected [x,y] / [x,y,z] or x/y/z map")


def _find_first_key(raw: Dict[str, Any], keys: Iterable[str]) -> Any:
    for key in keys:
        if key in raw:
            return raw[key]
    return None


def _parse_yaml_pair(raw: Any, index: int) -> PointPair3D:
    if not isinstance(raw, dict):
        raise ValueError(f"points[{index}] must be a map/object")

    source_node = _find_first_key(raw, ("source", "src", "input", "p"))
    target_node = _find_first_key(raw, ("target", "dst", "output", "q"))
    if source_node is None or target_node is None:
        raise ValueError(
            f"points[{index}] must provide source/src/input and target/dst/output"
        )

    return PointPair3D(
        name=str(raw.get("name", f"p{index + 1}")),
        source=_read_xyz(source_node, f"points[{index}].source"),
        target=_read_xyz(target_node, f"points[{index}].target"),
    )


def _parse_point_text(text: str, index: int) -> PointPair3D:
    value = text.strip()
    if not value:
        raise ValueError("empty point pair")

    if ":" in value:
        left, right = value.split(":", 1)
        source_items = [item for item in left.replace(",", " ").split() if item]
        target_items = [item for item in right.replace(",", " ").split() if item]
        if len(source_items) not in (2, 3) or len(target_items) not in (2, 3):
            raise ValueError(f"invalid pair '{text}', expected sx,sy[,sz]:tx,ty[,tz]")
        source = [float(item) for item in source_items]
        target = [float(item) for item in target_items]
        if len(source) == 2:
            source.append(0.0)
        if len(target) == 2:
            target.append(0.0)
    else:
        items = [item for item in value.replace(",", " ").split() if item]
        if len(items) == 4:
            source = [float(items[0]), float(items[1]), 0.0]
            target = [float(items[2]), float(items[3]), 0.0]
        elif len(items) >= 6:
            source = [float(items[0]), float(items[1]), float(items[2])]
            target = [float(items[3]), float(items[4]), float(items[5])]
        else:
            raise ValueError(f"invalid pair '{text}', expected 4 or 6 numeric values")

    return PointPair3D(
        name=f"p{index + 1}",
        source=np.array(source, dtype=float),
        target=np.array(target, dtype=float),
    )


def _prompt_pairs(
    min_points: int,
    source_frame: str,
    target_frame: str,
    source_unit: str,
    target_unit: str,
    output_unit: str,
    default_input_path: str,
) -> List[PointPair3D]:
    print("No --input and no --point provided, entering interactive mode.")
    print(
        f"Unit mapping: {source_frame}({source_unit}) -> "
        f"{target_frame}({target_unit}); matrix/output={output_unit}"
    )
    print(
        f"Input one pair per line: source {source_frame} in {source_unit}, "
        f"then target {target_frame} in {target_unit}."
    )
    print("Format: sx sy [sz] tx ty [tz]  or  sx,sy[,sz]:tx,ty[,tz]")
    print("Example for default m -> m: 10.93 3.66 0 0.413 -9.622 0")
    print("If source points are official cm, run with --source-unit cm.")
    if default_input_path:
        print(f"Press Enter at pair[1] to use YAML points: {default_input_path}")
        print("Typing any pair starts a new calibration and ignores YAML points.")
    print(f"Need at least {min_points} pairs. Empty line finishes.")

    pairs: List[PointPair3D] = []
    while True:
        try:
            line = input(
                f"pair[{len(pairs) + 1}] {source_unit}->{target_unit}> "
            ).strip()
        except EOFError:
            break
        except KeyboardInterrupt:
            print("\n[INFO] interrupted.")
            break

        if not line:
            if not pairs and default_input_path:
                print(f"[INFO] using YAML points: {default_input_path}")
                return []
            if len(pairs) >= min_points:
                break
            print(f"[WARN] currently only {len(pairs)} pair(s), need >= {min_points}.")
            continue

        try:
            pairs.append(_parse_point_text(line, len(pairs)))
        except Exception as ex:
            print(f"[WARN] {ex}")

    return pairs


def _convert_pairs_to_output_unit(
    pairs: Sequence[PointPair3D],
    source_unit: str,
    target_unit: str,
    output_unit: str,
) -> List[PointPair3D]:
    output_scale = _unit_scale_to_m(output_unit)
    source_scale = _unit_scale_to_m(source_unit) / output_scale
    target_scale = _unit_scale_to_m(target_unit) / output_scale

    return [
        PointPair3D(
            name=pair.name,
            source=pair.source * source_scale,
            target=pair.target * target_scale,
        )
        for pair in pairs
    ]


def _validate_pairs(pairs: Sequence[PointPair3D], min_points: int) -> None:
    if len(pairs) < min_points:
        raise ValueError(f"need at least {min_points} point pairs, got {len(pairs)}")

    source = np.array([pair.source for pair in pairs], dtype=float)
    target = np.array([pair.target for pair in pairs], dtype=float)
    if not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError("points contain nan/inf")

    source_rank = int(np.linalg.matrix_rank(source - source.mean(axis=0)))
    target_rank = int(np.linalg.matrix_rank(target - target.mean(axis=0)))
    if source_rank < 2:
        raise ValueError("source points are degenerate; need at least non-collinear points")
    if target_rank < 2:
        raise ValueError("target points are degenerate; need at least non-collinear points")


def _affine(
    pairs: Sequence[PointPair3D],
    snap_epsilon: float,
) -> Tuple["np.ndarray", "np.ndarray", int, "np.ndarray"]:
    design = []
    target = []
    for pair in pairs:
        sx = float(pair.source[0])
        sy = float(pair.source[1])
        design.append([sx, sy, 1.0, 0.0, 0.0, 0.0])
        target.append(float(pair.target[0]))
        design.append([0.0, 0.0, 0.0, sx, sy, 1.0])
        target.append(float(pair.target[1]))

    a = np.array(design, dtype=float)
    b = np.array(target, dtype=float)
    coeffs, _, rank, singular_values = np.linalg.lstsq(a, b, rcond=None)
    if rank < 6:
        raise ValueError(f"affine solve is rank deficient: rank={rank}, expected 6")

    linear = np.array(
        [
            [coeffs[0], coeffs[1], 0.0],
            [coeffs[3], coeffs[4], 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    translation = np.array([coeffs[2], coeffs[5], 0.0], dtype=float)

    if (
        snap_epsilon > 0.0
        and np.allclose(linear, np.eye(3), atol=snap_epsilon, rtol=0.0)
        and np.allclose(translation, np.zeros(3), atol=snap_epsilon, rtol=0.0)
    ):
        linear = np.eye(3)
        translation = np.zeros(3)

    return linear, translation, int(rank), singular_values


def _matrix_4x4(linear: "np.ndarray", translation: "np.ndarray") -> "np.ndarray":
    matrix = np.eye(4)
    matrix[:3, :3] = linear
    matrix[:3, 3] = translation
    return matrix


def _inverse_matrix_4x4(matrix: "np.ndarray") -> "np.ndarray":
    return np.linalg.inv(matrix)


def _residuals(
    pairs: Sequence[PointPair3D],
    linear: "np.ndarray",
    translation: "np.ndarray",
) -> Tuple[float, float, List[Dict[str, Any]]]:
    per_point = []
    sum_sq = 0.0
    max_err = 0.0
    for pair in pairs:
        predicted = linear @ pair.source + translation
        delta = predicted - pair.target
        error = float(np.linalg.norm(delta))
        sum_sq += error * error
        max_err = max(max_err, error)
        per_point.append(
            {
                "name": pair.name,
                "error": error,
                "predicted": predicted.tolist(),
                "target": pair.target.tolist(),
                "delta": delta.tolist(),
            }
        )
    rmse = math.sqrt(sum_sq / max(float(len(pairs)), 1.0))
    return rmse, max_err, per_point


def _load_yaml(path: str) -> Dict[str, Any]:
    if yaml is None:
        raise RuntimeError(f"missing PyYAML dependency: {YAML_IMPORT_ERROR}")
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError("input YAML root must be a map/object")
    return data


def _default_input_candidates(cli_path: str) -> List[Path]:
    if cli_path:
        return [Path(cli_path).expanduser()]

    script_path = Path(__file__).resolve()
    cwd = Path.cwd()
    candidates = [
        script_path.parent.parent / "config" / "navi_calib.yaml",
        cwd / "navi_calib.yaml",
        cwd.parent / "config" / "navi_calib.yaml",
        cwd / "src" / "navi_tf_bridge" / "config" / "navi_calib.yaml",
    ]
    if len(script_path.parents) >= 3:
        candidates.append(
            script_path.parents[2] / "share" / "navi_tf_bridge" / "config" / "navi_calib.yaml"
        )
    return candidates


def _resolve_default_input_path(cli_path: str) -> Path:
    candidates = _default_input_candidates(cli_path)
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return candidates[0]


def _dump_yaml(path: str, data: Dict[str, Any]) -> None:
    if yaml is None:
        raise RuntimeError(f"missing PyYAML dependency: {YAML_IMPORT_ERROR}")
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=False, allow_unicode=False)


def _format_row(row: Sequence[float]) -> str:
    return " ".join(f"{float(value): .9f}" for value in row)


def _flatten_row_major(matrix: "np.ndarray") -> List[float]:
    return [float(value) for value in matrix.reshape(-1)]


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Fit source->target static transform with 2D affine least squares. "
            "Default unit flow: official_map(m) -> map(m), matrix/output=m."
        )
    )
    parser.add_argument("--input", default="", help="Input point YAML path.")
    parser.add_argument(
        "--default-input",
        default="",
        help=(
            "YAML used when running interactively and pressing Enter at pair[1]. "
            "Default: src/navi_tf_bridge/config/navi_calib.yaml."
        ),
    )
    parser.add_argument("--output", default="", help="Optional output YAML path.")
    parser.add_argument(
        "--point",
        action="append",
        default=[],
        help="Repeatable point pair: 'sx,sy[,sz]:tx,ty[,tz]' or 'sx sy [sz] tx ty [tz]'.",
    )
    parser.add_argument("--source-frame", default="", help="Override source_frame.")
    parser.add_argument("--target-frame", default="", help="Override target_frame.")
    parser.add_argument(
        "--source-unit",
        default="",
        help="Override source point unit: m/cm/mm. Default: m.",
    )
    parser.add_argument(
        "--target-unit",
        default="",
        help="Override target point unit: m/cm/mm. Default: m.",
    )
    parser.add_argument(
        "--output-unit",
        default="",
        help="Unit used by the solved matrix. Default: m for raw_goal_transform_matrix.",
    )
    parser.add_argument("--min-points", type=int, default=3, help="Minimum point pairs.")
    parser.add_argument(
        "--snap-epsilon",
        type=float,
        default=1e-9,
        help="Snap near-identity affine transform to exact identity. Set 0 to disable.",
    )
    args = parser.parse_args()

    if np is None:
        print(f"[ERROR] Missing numpy dependency: {NUMPY_IMPORT_ERROR}", file=sys.stderr)
        return 2

    default_input_path = _resolve_default_input_path(args.default_input)
    use_interactive_default_context = (
        not args.input and not args.point and default_input_path.is_file()
    )

    cfg: Dict[str, Any] = {}
    if args.input:
        try:
            cfg = _load_yaml(args.input)
        except Exception as ex:
            print(f"[ERROR] Failed to read input YAML: {ex}", file=sys.stderr)
            return 2
    elif use_interactive_default_context:
        try:
            cfg = _load_yaml(str(default_input_path))
        except Exception as ex:
            print(f"[WARN] Failed to read default input YAML '{default_input_path}': {ex}")
            cfg = {}

    source_frame = str(args.source_frame or cfg.get("source_frame", "official_map"))
    target_frame = str(args.target_frame or cfg.get("target_frame", "map"))
    legacy_unit = cfg.get("unit")
    source_unit = str(args.source_unit or cfg.get("source_unit", legacy_unit or "m"))
    target_unit = str(args.target_unit or cfg.get("target_unit", legacy_unit or "m"))
    output_unit = str(args.output_unit or cfg.get("output_unit", target_unit if legacy_unit else "m"))

    try:
        _unit_scale_to_m(source_unit)
        _unit_scale_to_m(target_unit)
        output_scale_to_m = _unit_scale_to_m(output_unit)
    except Exception as ex:
        print(f"[ERROR] {ex}", file=sys.stderr)
        return 2

    try:
        if args.point:
            raw_pairs = [_parse_point_text(text, i) for i, text in enumerate(args.point)]
        elif args.input:
            points_raw = cfg.get("points", [])
            if not isinstance(points_raw, list):
                raise ValueError("'points' must be a list")
            raw_pairs = [_parse_yaml_pair(raw, i) for i, raw in enumerate(points_raw)]
        else:
            raw_pairs = _prompt_pairs(
                max(args.min_points, 3),
                source_frame,
                target_frame,
                source_unit,
                target_unit,
                output_unit,
                str(default_input_path) if default_input_path.is_file() else "",
            )
            if not raw_pairs and default_input_path.is_file():
                points_raw = cfg.get("points", [])
                if not isinstance(points_raw, list):
                    raise ValueError(f"default input YAML '{default_input_path}' points must be a list")
                raw_pairs = [_parse_yaml_pair(raw, i) for i, raw in enumerate(points_raw)]
        pairs = _convert_pairs_to_output_unit(raw_pairs, source_unit, target_unit, output_unit)
        _validate_pairs(pairs, max(args.min_points, 3))
        linear, translation, rank, singular_values = _affine(pairs, args.snap_epsilon)
    except Exception as ex:
        print(f"[ERROR] Failed to solve affine transform: {ex}", file=sys.stderr)
        return 2

    matrix = _matrix_4x4(linear, translation)
    inverse = _inverse_matrix_4x4(matrix)
    rmse, max_err, per_point = _residuals(pairs, linear, translation)

    result = {
        "source_frame": source_frame,
        "target_frame": target_frame,
        "source_unit": source_unit,
        "target_unit": target_unit,
        "output_unit": output_unit,
        "model": "affine",
        "unit_mapping": {
            "source": f"{source_frame}({source_unit})",
            "target": f"{target_frame}({target_unit})",
            "matrix": f"output({output_unit})",
        },
        "num_points": len(pairs),
        "affine_rank": rank,
        "linear_det_2d": float(linear[0, 0] * linear[1, 1] - linear[0, 1] * linear[1, 0]),
        "singular_values": [float(v) for v in singular_values],
        "translation": {
            "x": float(translation[0]),
            "y": float(translation[1]),
            "z": float(translation[2]),
            "unit": output_unit,
        },
        "translation_m": {
            "x": float(translation[0] * output_scale_to_m),
            "y": float(translation[1] * output_scale_to_m),
            "z": float(translation[2] * output_scale_to_m),
        },
        "affine_matrix_2x3": [
            [float(linear[0, 0]), float(linear[0, 1]), float(translation[0])],
            [float(linear[1, 0]), float(linear[1, 1]), float(translation[1])],
        ],
        "matrix_4x4": matrix.tolist(),
        "matrix_4x4_row_major": _flatten_row_major(matrix),
        "inverse_matrix_4x4": inverse.tolist(),
        "inverse_matrix_4x4_row_major": _flatten_row_major(inverse),
        "residual": {
            "rmse": rmse,
            "max": max_err,
            "unit": output_unit,
            "rmse_m": rmse * output_scale_to_m,
            "max_m": max_err * output_scale_to_m,
            "per_point": per_point,
        },
    }

    if args.output:
        try:
            _dump_yaml(args.output, result)
        except Exception as ex:
            print(f"[ERROR] Failed to write output YAML: {ex}", file=sys.stderr)
            return 2

    print("Affine solve completed.")
    print(f"source_frame={source_frame}, target_frame={target_frame}, points={len(pairs)}")
    print(
        f"unit mapping: {source_frame}({source_unit}) -> "
        f"{target_frame}({target_unit}); matrix/output={output_unit}"
    )
    print(f"units: source={source_unit}, target={target_unit}, matrix={output_unit}")
    print(f"affine_rank={rank}")
    print(
        "linear_2x2="
        f"[[{linear[0, 0]:.9f}, {linear[0, 1]:.9f}], "
        f"[{linear[1, 0]:.9f}, {linear[1, 1]:.9f}]]"
    )
    print(
        f"linear_det_2d={float(linear[0, 0] * linear[1, 1] - linear[0, 1] * linear[1, 0]):.9f}"
    )
    print(
        f"translation_{output_unit}="
        f"(x={translation[0]:.9f}, y={translation[1]:.9f}, z={translation[2]:.9f})"
    )
    print(
        f"residual_rmse_{output_unit}={rmse:.9f}, residual_max_{output_unit}={max_err:.9f}"
    )
    print("source_to_target_4x4:")
    for row in matrix:
        print("  " + _format_row(row))
    print("raw_goal_transform_matrix row-major:")
    print("  [" + ", ".join(f"{value:.9f}" for value in _flatten_row_major(matrix)) + "]")
    if args.output:
        print(f"output_yaml={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
