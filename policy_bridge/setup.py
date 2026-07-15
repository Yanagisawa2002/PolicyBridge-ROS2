"""Setuptools entry point for the policy_bridge ROS 2 package."""

from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "policy_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [str(Path("resource") / package_name)],
        ),
        (str(Path("share") / package_name), ["package.xml"]),
        (str(Path("share") / package_name / "launch"), glob("launch/*.launch.py")),
        (str(Path("share") / package_name / "config"), glob("config/*.yaml")),
    ],
    install_requires=["numpy", "setuptools"],
    zip_safe=True,
    maintainer="PolicyBridge-ROS2 contributors",
    maintainer_email="maintainers@example.com",
    description="ROS 2 M0 runtime for a scripted six-joint policy demo.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "demo_client = policy_bridge.demo_client:main",
            "mock_manipulator = policy_bridge.mock_manipulator:main",
            "policy_server = policy_bridge.policy_server:main",
        ],
    },
)
