"""Launch the complete synchronized RGB-plus-joint PolicyBridge M2 demonstration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start the manipulator, RGB camera, policy server, and one-shot client."""

    package_share = Path(get_package_share_directory("policy_bridge"))
    config_path = str(package_share / "config" / "multimodal_demo.yaml")

    mock_manipulator = Node(
        package="policy_bridge",
        executable="mock_manipulator",
        name="mock_manipulator",
        output="screen",
        parameters=[config_path],
    )
    mock_rgb_camera = Node(
        package="policy_bridge",
        executable="mock_rgb_camera",
        name="mock_rgb_camera",
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
            mock_rgb_camera,
            policy_server,
            TimerAction(period=1.0, actions=[demo_client]),
        ]
    )
