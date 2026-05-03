#!/usr/bin/env python3

# AUTO-COMMENT: file overview
# This file belongs to the ROS2 sentry workspace codebase.
# Keep behavior and interface changes synchronized with related modules.

"""Solve a 2D rigid transform from paired points and output a 4x4 matrix.

Input YAML format example:

source_frame: lower_map
target_frame: radar_map
unit: cm
tz: 0
points:
  - name: p1
    source: [100, 100]
    target: [120, 130]
  - name: p2
    source: [700, 100]
    target: [715, 135]
  - name: p3
    source: [100, 500]
    target: [122, 530]
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Tuple

try:
    import yaml
except Exception as ex:  # pragma: no cover - runtime import guard
    yaml = None
    IMPORT_ERROR = ex
else:
    IMPORT_ERROR = None


@dataclass
class PointPair2D:
    name: str
    sx: float
    sy: float
    tx: float
    ty: float


def _parse_point_pair_text(text: str, index: int) -> PointPair2D:
    # Format: "sx,sy:tx,ty" or "sx sy tx ty"
    value = text.strip()
    if not value:
        raise ValueError("empty point pair text")

    if ":" in value:
        left, right = value.split(":", 1)
        left_items = [s for s in left.replace(",", " ").split() if s]
        right_items = [s for s in right.replace(",", " ").split() if s]
        if len(left_items) < 2 or len(right_items) < 2:
            raise ValueError(f"invalid pair '{text}', expected sx,sy:tx,ty")
        sx = float(left_items[0])
        sy = float(left_items[1])
        tx = float(right_items[0])
        ty = float(right_items[1])
    else:
        items = [s for s in value.replace(",", " ").split() if s]
        if len(items) < 4:
            raise ValueError(f"invalid pair '{text}', expected sx sy tx ty")
        sx = float(items[0])
        sy = float(items[1])
        tx = float(items[2])
        ty = float(items[3])

    return PointPair2D(name=f"p{index + 1}", sx=sx, sy=sy, tx=tx, ty=ty)


def _prompt_points_interactive(min_points: int) -> List[PointPair2D]:
    print("No --input and no --point provided, entering interactive mode.")
    print("Input one pair per line: sx sy tx ty  (or sx,sy:tx,ty)")
    print(f"Need at least {min_points} pairs. Press Enter on empty line to finish.")

    pairs: List[PointPair2D] = []
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
            pair = _parse_point_pair_text(line, len(pairs))
        except Exception as ex:
            print(f"[WARN] {ex}")
            continue
        pairs.append(pair)

    return pairs


def _read_xy(node: Any, field_name: str) -> Tuple[float, float]:
    if isinstance(node, (list, tuple)) and len(node) >= 2:
        return float(node[0]), float(node[1])

    if isinstance(node, dict):
        for kx, ky in (("x", "y"), ("X", "Y")):
            if kx in node and ky in node:
                return float(node[kx]), float(node[ky])

    raise ValueError(f"Invalid {field_name}: expected [x,y] or {{x:..., y:...}}.")


def _find_first_key(raw: Dict[str, Any], keys: Iterable[str]) -> Any:
    for key in keys:
        if key in raw:
            return raw[key]
    return None


def _parse_pair(raw: Any, index: int) -> PointPair2D:
    if not isinstance(raw, dict):
        raise ValueError(f"points[{index}] must be a map/object.")

    source_node = _find_first_key(raw, ("source", "src", "input"))
    target_node = _find_first_key(raw, ("target", "dst", "output"))
    if source_node is None or target_node is None:
        raise ValueError(
            f"points[{index}] must provide both source/src/input and target/dst/output."
        )

    sx, sy = _read_xy(source_node, f"points[{index}].source")
    tx, ty = _read_xy(target_node, f"points[{index}].target")
    name = str(raw.get("name", f"p{index + 1}"))
    return PointPair2D(name=name, sx=sx, sy=sy, tx=tx, ty=ty)


def _triangle_area2(p1: Tuple[float, float], p2: Tuple[float, float], p3: Tuple[float, float]) -> float:
    return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])


def _max_abs_triangle_area2(points: List[Tuple[float, float]]) -> float:
    n = len(points)
    best = 0.0
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                area2 = abs(_triangle_area2(points[i], points[j], points[k]))
                if area2 > best:
                    best = area2
    return best


def _validate_geometry(pairs: List[PointPair2D], min_points: int) -> None:
    if len(pairs) < min_points:
        raise ValueError(f"Need at least {min_points} point pairs, got {len(pairs)}.")

    src_points = [(p.sx, p.sy) for p in pairs]
    dst_points = [(p.tx, p.ty) for p in pairs]

    src_area2 = _max_abs_triangle_area2(src_points)
    dst_area2 = _max_abs_triangle_area2(dst_points)
    if src_area2 <= 1e-9:
        raise ValueError("Source points are degenerate (nearly collinear).")
    if dst_area2 <= 1e-9:
        raise ValueError("Target points are degenerate (nearly collinear).")


def _solve_rigid_2d(pairs: List[PointPair2D]) -> Tuple[float, float, float, float, float]:
    n = float(len(pairs))
    src_cx = sum(p.sx for p in pairs) / n
    src_cy = sum(p.sy for p in pairs) / n
    dst_cx = sum(p.tx for p in pairs) / n
    dst_cy = sum(p.ty for p in pairs) / n

    cross_term = 0.0
    dot_term = 0.0
    for p in pairs:
        xs = p.sx - src_cx
        ys = p.sy - src_cy
        xt = p.tx - dst_cx
        yt = p.ty - dst_cy
        cross_term += xs * yt - ys * xt
        dot_term += xs * xt + ys * yt

    if abs(cross_term) + abs(dot_term) <= 1e-12:
        raise ValueError("Failed to solve rotation: covariance is near zero.")

    yaw = math.atan2(cross_term, dot_term)
    c = math.cos(yaw)
    s = math.sin(yaw)
    tx = dst_cx - (c * src_cx - s * src_cy)
    ty = dst_cy - (s * src_cx + c * src_cy)
    return c, s, tx, ty, yaw


def _residuals(
    pairs: List[PointPair2D], c: float, s: float, tx: float, ty: float
) -> Tuple[float, float, List[Dict[str, float]]]:
    per_point = []
    sum_sq = 0.0
    max_err = 0.0
    for p in pairs:
        pred_x = c * p.sx - s * p.sy + tx
        pred_y = s * p.sx + c * p.sy + ty
        err = math.hypot(pred_x - p.tx, pred_y - p.ty)
        sum_sq += err * err
        max_err = max(max_err, err)
        per_point.append(
            {
                "name": p.name,
                "error": err,
                "pred_x": pred_x,
                "pred_y": pred_y,
            }
        )
    rmse = math.sqrt(sum_sq / max(float(len(pairs)), 1.0))
    return rmse, max_err, per_point


def _matrix_4x4(c: float, s: float, tx: float, ty: float, tz: float) -> List[List[float]]:
    return [
        [c, -s, 0.0, tx],
        [s, c, 0.0, ty],
        [0.0, 0.0, 1.0, tz],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _inverse_matrix_4x4(c: float, s: float, tx: float, ty: float, tz: float) -> List[List[float]]:
    inv_tx = -(c * tx + s * ty)
    inv_ty = s * tx - c * ty
    return [
        [c, s, 0.0, inv_tx],
        [-s, c, 0.0, inv_ty],
        [0.0, 0.0, 1.0, -tz],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _format_float(v: float) -> str:
    return f"{v:.9f}"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Solve 2D rigid transform from paired points and output 4x4 matrix "
            "(source -> target)."
        )
    )
    parser.add_argument("--input", default="", help="Optional input YAML path.")
    parser.add_argument("--output", default="", help="Optional output YAML path.")
    parser.add_argument(
        "--point",
        action="append",
        default=[],
        help=(
            "Point pair text. Repeat this option. "
            "Format: 'sx,sy:tx,ty' or 'sx sy tx ty'."
        ),
    )
    parser.add_argument("--source-frame", default="", help="Override source_frame.")
    parser.add_argument("--target-frame", default="", help="Override target_frame.")
    parser.add_argument("--unit", default="", help="Override unit: cm or m.")
    parser.add_argument(
        "--min-points",
        type=int,
        default=3,
        help="Minimum required point pairs (default: 3).",
    )
    parser.add_argument(
        "--tz",
        type=float,
        default=None,
        help="Optional z translation in the same unit as points (override YAML tz).",
    )
    args = parser.parse_args()

    if yaml is None:
        print(
            f"[ERROR] Missing PyYAML dependency: {IMPORT_ERROR}. "
            "Install python3-yaml first.",
            file=sys.stderr,
        )
        return 2

    cfg: Dict[str, Any] = {}
    if args.input:
        try:
            with open(args.input, encoding="utf-8") as f:
                cfg = yaml.safe_load(f) or {}
        except FileNotFoundError:
            print(f"[ERROR] Input file not found: {args.input}", file=sys.stderr)
            return 2
        except Exception as ex:
            print(f"[ERROR] Failed to read YAML '{args.input}': {ex}", file=sys.stderr)
            return 2
        if not isinstance(cfg, dict):
            print("[ERROR] Input YAML root must be a map/object.", file=sys.stderr)
            return 2

    source_frame = str(args.source_frame or cfg.get("source_frame", "source_map"))
    target_frame = str(args.target_frame or cfg.get("target_frame", "target_map"))
    unit = str(args.unit or cfg.get("unit", "cm")).strip().lower()
    if unit not in ("cm", "m"):
        print(f"[ERROR] Unsupported unit '{unit}', expected 'cm' or 'm'.", file=sys.stderr)
        return 2

    pairs: List[PointPair2D] = []
    if args.point:
        try:
            pairs = [_parse_point_pair_text(text, i) for i, text in enumerate(args.point)]
        except Exception as ex:
            print(f"[ERROR] Failed to parse --point: {ex}", file=sys.stderr)
            return 2
    elif args.input:
        points_raw = cfg.get("points", [])
        if not isinstance(points_raw, list):
            print("[ERROR] 'points' must be a list.", file=sys.stderr)
            return 2
        try:
            pairs = [_parse_pair(raw, i) for i, raw in enumerate(points_raw)]
        except Exception as ex:
            print(f"[ERROR] Failed to parse YAML points: {ex}", file=sys.stderr)
            return 2
    else:
        pairs = _prompt_points_interactive(max(args.min_points, 2))
        if len(pairs) < max(args.min_points, 2):
            print(
                "[ERROR] Not enough point pairs. Use --input or repeat --point, or provide "
                "enough pairs in interactive mode.",
                file=sys.stderr,
            )
            return 2

    try:
        _validate_geometry(pairs, max(args.min_points, 2))
        c, s, tx, ty, yaw = _solve_rigid_2d(pairs)
    except Exception as ex:
        print(f"[ERROR] Failed to solve transform: {ex}", file=sys.stderr)
        return 2

    tz = float(args.tz if args.tz is not None else cfg.get("tz", 0.0))
    rmse, max_err, per_point = _residuals(pairs, c, s, tx, ty)

    unit_to_meter = 0.01 if unit == "cm" else 1.0
    tx_m = tx * unit_to_meter
    ty_m = ty * unit_to_meter
    tz_m = tz * unit_to_meter
    rmse_m = rmse * unit_to_meter
    max_err_m = max_err * unit_to_meter

    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)

    matrix = _matrix_4x4(c, s, tx, ty, tz)
    matrix_inv = _inverse_matrix_4x4(c, s, tx, ty, tz)

    result = {
        "source_frame": source_frame,
        "target_frame": target_frame,
        "unit": unit,
        "num_points": len(pairs),
        "yaw_rad": yaw,
        "yaw_deg": math.degrees(yaw),
        "translation": {"x": tx, "y": ty, "z": tz, "unit": unit},
        "translation_m": {"x": tx_m, "y": ty_m, "z": tz_m},
        "matrix_4x4": matrix,
        "inverse_matrix_4x4": matrix_inv,
        "residual": {
            "rmse": rmse,
            "max": max_err,
            "unit": unit,
            "rmse_m": rmse_m,
            "max_m": max_err_m,
            "per_point": per_point,
        },
        "tf": {
            "lookup_transform_expectation": (
                "For lookupTransform(target_frame, source_frame), publish parent=target_frame, "
                "child=source_frame."
            ),
            "source_to_target_static_tf_cmd": (
                "ros2 run tf2_ros static_transform_publisher "
                f"{_format_float(tx_m)} {_format_float(ty_m)} {_format_float(tz_m)} "
                f"{_format_float(qx)} {_format_float(qy)} {_format_float(qz)} {_format_float(qw)} "
                f"{target_frame} {source_frame}"
            ),
            "target_to_source_static_tf_cmd": (
                "ros2 run tf2_ros static_transform_publisher "
                f"{_format_float(matrix_inv[0][3] * unit_to_meter)} "
                f"{_format_float(matrix_inv[1][3] * unit_to_meter)} "
                f"{_format_float(matrix_inv[2][3] * unit_to_meter)} "
                f"{_format_float(qx)} {_format_float(qy)} {_format_float(-qz)} {_format_float(qw)} "
                f"{source_frame} {target_frame}"
            ),
        },
    }

    if args.output:
        try:
            with open(args.output, "w", encoding="utf-8") as f:
                yaml.safe_dump(result, f, sort_keys=False, allow_unicode=False)
        except Exception as ex:
            print(f"[ERROR] Failed to write output YAML '{args.output}': {ex}", file=sys.stderr)
            return 2

    print("Solve completed.")
    print(f"source_frame={source_frame}, target_frame={target_frame}, points={len(pairs)}")
    print(f"yaw_deg={math.degrees(yaw):.6f}")
    print(
        f"translation_{unit}=(x={tx:.6f}, y={ty:.6f}, z={tz:.6f}), "
        f"translation_m=(x={tx_m:.6f}, y={ty_m:.6f}, z={tz_m:.6f})"
    )
    print(f"residual_rmse_{unit}={rmse:.6f}, residual_max_{unit}={max_err:.6f}")
    print("source_to_target_4x4:")
    for row in matrix:
        print("  " + " ".join(f"{v: .9f}" for v in row))
    print("static_tf_cmd(source->target):")
    print("  " + result["tf"]["source_to_target_static_tf_cmd"])
    if args.output:
        print(f"output_yaml={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
