#!/usr/bin/env python3
"""Regression checks for generated single-area regional BT profiles."""

from __future__ import annotations

import copy
import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[2]
GENERATOR = ROOT_DIR / "scripts/areatest/regional_area_profile.py"
RUNNER = ROOT_DIR / "scripts/areatest/regional_area_test.sh"
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

    def test_runner_installs_cleanup_before_profile_generation(self) -> None:
        runner_text = RUNNER.read_text(encoding="utf-8")
        mktemp_offset = runner_text.index('BT_CONFIG_FILE="$(mktemp')
        trap_offset = runner_text.index("trap cleanup_children EXIT INT TERM")
        generator_offset = runner_text.index('python3 "${BT_PROFILE_GENERATOR}"')

        self.assertLess(mktemp_offset, trap_offset)
        self.assertLess(trap_offset, generator_offset)

    def test_runner_waits_for_launch_before_deleting_profile(self) -> None:
        runner_text = RUNNER.read_text(encoding="utf-8")
        cleanup_body = runner_text.split("cleanup_children() {", 1)[1].split(
            "start_fake_referee_publishers()", 1
        )[0]

        self.assertLess(
            cleanup_body.index('kill -INT "${LAUNCH_PID}"'),
            cleanup_body.index('wait "${LAUNCH_PID}"'),
        )
        self.assertLess(
            cleanup_body.index('wait "${LAUNCH_PID}"'),
            cleanup_body.index('rm -f "${BT_CONFIG_FILE}"'),
        )

    def test_runner_removes_temp_profile_when_generator_fails(self) -> None:
        pattern = "ly_regional_area_my_base_*.json"
        before = set(Path("/tmp").glob(pattern))
        with tempfile.TemporaryDirectory() as temp_dir:
            fake_python = Path(temp_dir) / "python3"
            fake_python.write_text("#!/usr/bin/env bash\nexit 19\n", encoding="utf-8")
            fake_python.chmod(0o755)
            env = os.environ | {"PATH": f"{temp_dir}{os.pathsep}{os.environ['PATH']}"}
            result = subprocess.run(
                ["bash", str(RUNNER), "base"],
                cwd=ROOT_DIR,
                env=env,
                capture_output=True,
                text=True,
            )

        self.assertEqual(result.returncode, 19)
        self.assertEqual(set(Path("/tmp").glob(pattern)), before)


if __name__ == "__main__":
    unittest.main()
