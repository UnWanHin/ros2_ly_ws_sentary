#!/usr/bin/env python3
"""Generate one temporary regional single-area BT profile from the canonical template."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


AREA_TARGETS = {
    "my_base": ("MyArea", "Base"),
    "my_highland": ("MyArea", "Highland"),
    "my_roadland": ("MyArea", "Roadland"),
    "common_central": ("CommonArea", "Central"),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a single-area regional BT JSON profile."
    )
    parser.add_argument("--template", required=True, type=Path)
    parser.add_argument("--area", required=True, choices=AREA_TARGETS)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--pure",
        action="store_true",
        help="Disable firing and posture and ignore recovery for route/hold tests.",
    )
    return parser.parse_args()


def set_single_area(profile: dict, area: str) -> None:
    navi_goal = profile["DecisionAutonomy"]["NaviGoal"]
    for scope in ("MyArea", "EnemyArea", "CommonArea"):
        for key in navi_goal[scope]:
            navi_goal[scope][key] = False
    scope, key = AREA_TARGETS[area]
    navi_goal[scope][key] = True


def apply_pure_profile(profile: dict) -> None:
    profile["AimDebug"]["StopFire"] = True
    profile["AimDebug"]["FireRequireTargetStatus"] = True
    profile["RegionalAreaTask"] = {"IgnoreRecovery": True}
    profile["Posture"] = {"Enable": False}


def main() -> None:
    args = parse_args()
    with args.template.open("r", encoding="utf-8") as template_file:
        profile = json.load(template_file)

    set_single_area(profile, args.area)
    if args.pure:
        apply_pure_profile(profile)

    with args.output.open("w", encoding="utf-8") as output_file:
        json.dump(profile, output_file, indent=2, ensure_ascii=False)
        output_file.write("\n")


if __name__ == "__main__":
    main()
