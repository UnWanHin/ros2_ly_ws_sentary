from __future__ import annotations

import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]


def test_offline_launch_does_not_require_external_aim_dynamics() -> None:
    script = """
set -euo pipefail
source scripts/lib/ros_launch_common.sh
sentry_msgs_aim_result_has_dynamics() { return 1; }
LAUNCH_ARGS=(offline:=true use_behavior_tree:=true)
require_sentry_msgs_for_behavior_tree
"""

    result = subprocess.run(
        ["bash", "-c", script],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
