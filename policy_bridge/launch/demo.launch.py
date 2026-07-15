"""Launch the complete PolicyBridge-ROS2 M0 demonstration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start the mock, action server, and delayed one-shot demo client."""

    package_share = Path(get_package_share_directory("policy_bridge"))
    config_path = str(package_share / "config" / "demo.yaml")

    mock_manipulator = Node(
        package="policy_bridge",
        executable="mock_manipulator",
        name="mock_manipulator",
        output="screen",
        parameters=[config_path],
    )
    policy_server = Node(
        package="policy_bridge",
        executable="policy_server",
        name="policy_server",
        output="screen",
        parameters=[config_path],
    )
    demo_client = Node(
        package="policy_bridge",
        executable="demo_client",
        name="demo_client",
        output="screen",
        parameters=[config_path],
    )

    return LaunchDescription(
        [
            mock_manipulator,
            policy_server,
            TimerAction(period=1.0, actions=[demo_client]),
        ]
    )
