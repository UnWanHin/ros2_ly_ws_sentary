from glob import glob
from setuptools import find_packages, setup

package_name = "simulator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md", "requirements.txt"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/sample", glob("sample/*.jsonl")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="unwanhin",
    maintainer_email="unwanhin@example.com",
    description="Offline pygame simulator for sentry behavior-tree decision traces.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "simulator = simulator.main:main",
            "simulator-start = simulator.start:main",
            "simulator-mock-inputs = simulator.mock_inputs:main",
            "simulator-ros-topic-monitor = simulator.ros_topic_monitor:main",
        ],
    },
)
