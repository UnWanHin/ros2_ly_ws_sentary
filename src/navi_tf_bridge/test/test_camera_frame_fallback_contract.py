#!/usr/bin/env python3
"""Static contract checks for the external sentry.aim camera-frame migration."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[3]


class CameraFrameFallbackContractTest(unittest.TestCase):
    def test_formal_and_standalone_facemode_use_long_focus_then_short_focus(self):
        launch = (ROOT / "src/navi_tf_bridge/launch/map_aim_point.launch.py").read_text()
        solver = (ROOT / "src/navi_tf_bridge/src/pointer_solver_node.cpp").read_text()
        sentry_all = (ROOT / "src/behavior_tree/launch/sentry_all.launch.py").read_text()

        self.assertIn('DeclareLaunchArgument("camera_frame", default_value="gx_camera_0")', launch)
        self.assertIn(
            'DeclareLaunchArgument("camera_fallback_frame", default_value="gx_camera_1")', launch)
        self.assertIn('"camera_frame", "gx_camera_0"', solver)
        self.assertIn('"camera_fallback_frame", "gx_camera_1"', solver)
        self.assertIn('"camera_frame": "gx_camera_0"', sentry_all)
        self.assertIn('"camera_fallback_frame": "gx_camera_1"', sentry_all)
        self.assertIn("solveCameraProjectionAnglesForFrame", solver)


if __name__ == "__main__":
    unittest.main()
