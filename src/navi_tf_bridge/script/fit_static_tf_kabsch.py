#!/usr/bin/env python3

"""Fit a multi-point Kabsch transform and print a 4x4 matrix.

Example YAML:

source_frame: official_map
target_frame: map
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

The solved matrix maps source points to target points in output_unit.
For navi_tf_bridge raw_goal_transform_matrix, use output_unit=m.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
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


def _prompt_pairs(min_points: int) -> List[PointPair3D]:
    print("No --input and no --point provided, entering interactive mode.")
    print("Input one pair per line: sx sy [sz] tx ty [tz]  or  sx,sy[,sz]:tx,ty[,tz]")
    print(f"Need at least {min_points} pairs. Empty line finishes.")

    pairs: List[PointPair3D] = []
    while True:
        try:
            line = input(f"pair[{len(pairs) + 1}]> ").strip()
        except EOFError:
            break
        except KeyboardInterrupt:
            print("\n[INFO] interrupted.")
            break

        if not line:
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


def _kabsch(
    pairs: Sequence[PointPair3D],
    allow_reflection: bool,
    snap_epsilon: float,
) -> Tuple["np.ndarray", "np.ndarray", "np.ndarray"]:
    source = np.array([pair.source for pair in pairs], dtype=float)
    target = np.array([pair.target for pair in pairs], dtype=float)

    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean

    h = source_centered.T @ target_centered
    u, singular_values, vt = np.linalg.svd(h)
    r = vt.T @ u.T

    if np.linalg.det(r) < 0.0 and not allow_reflection:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T

    t = target_mean - r @ source_mean

    if (
        snap_epsilon > 0.0
        and np.allclose(r, np.eye(3), atol=snap_epsilon, rtol=0.0)
        and np.allclose(t, np.zeros(3), atol=snap_epsilon, rtol=0.0)
    ):
        r = np.eye(3)
        t = np.zeros(3)

    return r, t, singular_values


def _matrix_4x4(r: "np.ndarray", t: "np.ndarray") -> "np.ndarray":
    matrix = np.eye(4)
    matrix[:3, :3] = r
    matrix[:3, 3] = t
    return matrix


def _inverse_matrix_4x4(r: "np.ndarray", t: "np.ndarray") -> "np.ndarray":
    inv = np.eye(4)
    inv[:3, :3] = r.T
    inv[:3, 3] = -(r.T @ t)
    return inv


def _rotation_to_quaternion(r: "np.ndarray") -> Tuple[float, float, float, float]:
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        qw = (r[2, 1] - r[1, 2]) / s
        qx = 0.25 * s
        qy = (r[0, 1] + r[1, 0]) / s
        qz = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        qw = (r[0, 2] - r[2, 0]) / s
        qx = (r[0, 1] + r[1, 0]) / s
        qy = 0.25 * s
        qz = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        qw = (r[1, 0] - r[0, 1]) / s
        qx = (r[0, 2] + r[2, 0]) / s
        qy = (r[1, 2] + r[2, 1]) / s
        qz = 0.25 * s

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def _residuals(
    pairs: Sequence[PointPair3D],
    r: "np.ndarray",
    t: "np.ndarray",
) -> Tuple[float, float, List[Dict[str, Any]]]:
    per_point = []
    sum_sq = 0.0
    max_err = 0.0
    for pair in pairs:
        predicted = r @ pair.source + t
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
        description="Fit source->target static transform with multi-point Kabsch."
    )
    parser.add_argument("--input", default="", help="Input point YAML path.")
    parser.add_argument("--output", default="", help="Optional output YAML path.")
    parser.add_argument(
        "--point",
        action="append",
        default=[],
        help="Repeatable point pair: 'sx,sy[,sz]:tx,ty[,tz]' or 'sx sy [sz] tx ty [tz]'.",
    )
    parser.add_argument("--source-frame", default="", help="Override source_frame.")
    parser.add_argument("--target-frame", default="", help="Override target_frame.")
    parser.add_argument("--source-unit", default="", help="Override source point unit: m/cm/mm.")
    parser.add_argument("--target-unit", default="", help="Override target point unit: m/cm/mm.")
    parser.add_argument(
        "--output-unit",
        default="",
        help="Unit used by the solved matrix. Use m for raw_goal_transform_matrix.",
    )
    parser.add_argument("--min-points", type=int, default=3, help="Minimum point pairs.")
    parser.add_argument(
        "--allow-reflection",
        action="store_true",
        help="Allow det(R)<0 reflection. Default rejects reflection by correcting SVD.",
    )
    parser.add_argument(
        "--snap-epsilon",
        type=float,
        default=1e-9,
        help="Snap near-identity transform to exact identity. Set 0 to disable.",
    )
    args = parser.parse_args()

    if np is None:
        print(f"[ERROR] Missing numpy dependency: {NUMPY_IMPORT_ERROR}", file=sys.stderr)
        return 2

    cfg: Dict[str, Any] = {}
    if args.input:
        try:
            cfg = _load_yaml(args.input)
        except Exception as ex:
            print(f"[ERROR] Failed to read input YAML: {ex}", file=sys.stderr)
            return 2

    source_frame = str(args.source_frame or cfg.get("source_frame", "source_map"))
    target_frame = str(args.target_frame or cfg.get("target_frame", "target_map"))
    source_unit = str(args.source_unit or cfg.get("source_unit", cfg.get("unit", "cm")))
    target_unit = str(args.target_unit or cfg.get("target_unit", cfg.get("unit", source_unit)))
    output_unit = str(args.output_unit or cfg.get("output_unit", target_unit))

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
            raw_pairs = _prompt_pairs(max(args.min_points, 3))
        pairs = _convert_pairs_to_output_unit(raw_pairs, source_unit, target_unit, output_unit)
        _validate_pairs(pairs, max(args.min_points, 3))
        r, t, singular_values = _kabsch(pairs, args.allow_reflection, args.snap_epsilon)
    except Exception as ex:
        print(f"[ERROR] Failed to solve Kabsch transform: {ex}", file=sys.stderr)
        return 2

    matrix = _matrix_4x4(r, t)
    inverse = _inverse_matrix_4x4(r, t)
    rmse, max_err, per_point = _residuals(pairs, r, t)
    qx, qy, qz, qw = _rotation_to_quaternion(r)

    result = {
        "source_frame": source_frame,
        "target_frame": target_frame,
        "source_unit": source_unit,
        "target_unit": target_unit,
        "output_unit": output_unit,
        "num_points": len(pairs),
        "rotation_det": float(np.linalg.det(r)),
        "singular_values": [float(v) for v in singular_values],
        "translation": {
            "x": float(t[0]),
            "y": float(t[1]),
            "z": float(t[2]),
            "unit": output_unit,
        },
        "translation_m": {
            "x": float(t[0] * output_scale_to_m),
            "y": float(t[1] * output_scale_to_m),
            "z": float(t[2] * output_scale_to_m),
        },
        "quaternion_xyzw": [qx, qy, qz, qw],
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

    print("Kabsch solve completed.")
    print(f"source_frame={source_frame}, target_frame={target_frame}, points={len(pairs)}")
    print(f"units: source={source_unit}, target={target_unit}, matrix={output_unit}")
    print(f"rotation_det={float(np.linalg.det(r)):.9f}")
    print(
        f"translation_{output_unit}=(x={t[0]:.9f}, y={t[1]:.9f}, z={t[2]:.9f})"
    )
    print(
        f"residual_rmse_{output_unit}={rmse:.9f}, residual_max_{output_unit}={max_err:.9f}"
    )
    print("source_to_target_4x4:")
    for row in matrix:
        print("  " + _format_row(row))
    print("raw_goal_transform_matrix row-major:")
    print("  [" + ", ".join(f"{value:.9f}" for value in _flatten_row_major(matrix)) + "]")
    print("quaternion_xyzw:")
    print(f"  {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}")
    if args.output:
        print(f"output_yaml={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
