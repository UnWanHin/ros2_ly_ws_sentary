#!/usr/bin/env python3
"""Regression checks for generated single-area regional BT profiles."""

from __future__ import annotations

import copy
import json
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[2]
GENERATOR = ROOT_DIR / "scripts/areatest/regional_area_profile.py"
TEMPLATE = (
    ROOT_DIR
    / "src/behavior_tree/Scripts/ConfigJson/regional/test/regional_area_template.json"
)
AREA_TARGETS = {
    "my_base": ("MyArea", "Base"),
    "my_highland": ("MyArea", "Highland"),
    "my_roadland": ("MyArea", "Roadland"),
    "common_central": ("CommonArea", "Central"),
}


def expected_profile(template: dict, area: str, pure: bool) -> dict:
    expected = copy.deepcopy(template)
    navi_goal = expected["DecisionAutonomy"]["NaviGoal"]
    for scope in ("MyArea", "EnemyArea", "CommonArea"):
        for key in navi_goal[scope]:
            navi_goal[scope][key] = False
    scope, key = AREA_TARGETS[area]
    navi_goal[scope][key] = True

    if pure:
        expected["AimDebug"]["StopFire"] = True
        expected["AimDebug"]["FireRequireTargetStatus"] = True
        expected["RegionalAreaTask"] = {"IgnoreRecovery": True}
        expected["Posture"] = {"Enable": False}
    return expected


class RegionalAreaProfileTest(unittest.TestCase):
    def test_all_area_and_pure_combinations_match_the_template_contract(self) -> None:
        template = json.loads(TEMPLATE.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as temp_dir:
            for area in AREA_TARGETS:
                for pure in (False, True):
                    output = Path(temp_dir) / f"{area}_{pure}.json"
                    command = [
                        "python3",
                        str(GENERATOR),
                        "--template",
                        str(TEMPLATE),
                        "--area",
                        area,
                        "--output",
                        str(output),
                    ]
                    if pure:
                        command.append("--pure")
                    subprocess.run(command, check=True, capture_output=True, text=True)
                    actual = json.loads(output.read_text(encoding="utf-8"))

                    self.assertEqual(actual, expected_profile(template, area, pure))


if __name__ == "__main__":
    unittest.main()
