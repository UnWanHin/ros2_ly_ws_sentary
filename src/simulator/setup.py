from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "simulator"


def nested_data_files(root: str) -> list[tuple[str, list[str]]]:
    entries: list[tuple[str, list[str]]] = []
    root_path = Path(root)
    if not root_path.exists():
        return entries
    for child in sorted(root_path.rglob("*")):
        if not child.is_file():
            continue
        target = (Path(f"share/{package_name}") / child.parent).as_posix()
        entries.append((target, [child.as_posix()]))
    return entries


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (
            f"share/{package_name}",
            ["package.xml", "README.md", "requirements.txt", "requirements-foxglove.txt", "requirements-browser.txt"],
        ),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (
            f"share/{package_name}/sample",
            glob("sample/*.jsonl") + glob("sample/*.json") + glob("sample/*.yaml") + glob("sample/*.yml"),
        ),
        (
            f"share/{package_name}/sample/scenarios",
            glob("sample/scenarios/*.jsonl") + glob("sample/scenarios/*.json"),
        ),
        (
            f"share/{package_name}/sample/mock_sequences",
            glob("sample/mock_sequences/*.json")
            + glob("sample/mock_sequences/*.yaml")
            + glob("sample/mock_sequences/*.yml"),
        ),
        (
            f"share/{package_name}/sample/unit_scenes",
            glob("sample/unit_scenes/*.json")
            + glob("sample/unit_scenes/*.yaml")
            + glob("sample/unit_scenes/*.yml"),
        ),
    ]
    + nested_data_files("assets"),
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="unwanhin",
    maintainer_email="unwanhin@example.com",
    description="Offline pygame simulator for sentry behavior-tree decision traces.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "simulator = simulator.main:main",
            "simulator-start = simulator.start:main",
            "simulator-foxglove = simulator.foxglove_export:main",
            "simulator-mock-inputs = simulator.mock_inputs:main",
            "simulator-mock-sequence = simulator.mock_sequence:main",
            "simulator-offline-workflows = simulator.offline_workflow:main",
            "simulator-decision-input-coverage = simulator.decision_input_coverage:main",
            "simulator-unit-scene = simulator.unit_scene:main",
            "simulator-unit-trace = simulator.unit_trace:main",
            "simulator-ros-topic-monitor = simulator.ros_topic_monitor:main",
            "simulator-web-visual-check = simulator.web_visual_check:main",
            "simulator-visual-asset-qa = simulator.visual_asset_qa:main",
            "simulator-quality = simulator.quality:main",
        ],
    },
)
