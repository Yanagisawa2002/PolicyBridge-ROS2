"""Standalone six-joint mock manipulator for the M0 demonstration."""

from __future__ import annotations

import math
import threading
from typing import Final

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .action_validation import JOINT_COUNT, validate_action
from .mock_dynamics import step_toward_target

DEFAULT_JOINT_NAMES: Final[tuple[str, ...]] = tuple(
    f"joint_{index}" for index in range(1, JOINT_COUNT + 1)
)
DEFAULT_INITIAL_POSITIONS: Final[tuple[float, ...]] = (
    0.5,
    -0.4,
    0.3,
    -0.2,
    0.1,
    -0.5,
)


class MockManipulator(Node):
    """Move six in-memory joints toward validated absolute targets."""

    def __init__(self) -> None:
        """Declare parameters and start command, motion, and state callbacks."""

        super().__init__("mock_manipulator")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("motion_rate_hz", 50.0)
        self.declare_parameter("max_delta_per_step", 0.02)
        self.declare_parameter("joint_names", list(DEFAULT_JOINT_NAMES))
        self.declare_parameter("initial_positions", list(DEFAULT_INITIAL_POSITIONS))
        self.declare_parameter("goal_tolerance", 0.005)

        publish_rate_hz = _positive_finite_parameter(
            "publish_rate_hz", self.get_parameter("publish_rate_hz").value
        )
        motion_rate_hz = _positive_finite_parameter(
            "motion_rate_hz", self.get_parameter("motion_rate_hz").value
        )
        self._max_delta_per_step = _positive_finite_parameter(
            "max_delta_per_step",
            self.get_parameter("max_delta_per_step").value,
        )
        self._goal_tolerance = _nonnegative_finite_parameter(
            "goal_tolerance", self.get_parameter("goal_tolerance").value
        )
        self._joint_names = _validate_joint_names(self.get_parameter("joint_names").value)
        self._current_positions = validate_action(self.get_parameter("initial_positions").value)
        self._target_positions = self._current_positions.copy()
        self._state_lock = threading.Lock()

        self._joint_state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self._command_subscription = self.create_subscription(
            Float64MultiArray,
            "/joint_command",
            self._command_callback,
            10,
        )
        self._motion_timer = self.create_timer(1.0 / motion_rate_hz, self._motion_callback)
        self._publish_timer = self.create_timer(1.0 / publish_rate_hz, self._publish_joint_state)

        self.get_logger().info(
            f"Mock manipulator ready with {JOINT_COUNT} joints; "
            f"publishing at {publish_rate_hz:.1f} Hz and updating at "
            f"{motion_rate_hz:.1f} Hz"
        )

    def _command_callback(self, message: Float64MultiArray) -> None:
        try:
            target = validate_action(message.data)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Rejected joint command: {exc}")
            return

        with self._state_lock:
            target_changed = not np.array_equal(target, self._target_positions)
            self._target_positions = target
        if target_changed:
            self.get_logger().info("Accepted a new absolute joint target")

    def _motion_callback(self) -> None:
        with self._state_lock:
            self._current_positions = step_toward_target(
                self._current_positions,
                self._target_positions,
                max_delta_per_step=self._max_delta_per_step,
                goal_tolerance=self._goal_tolerance,
            )

    def _publish_joint_state(self) -> None:
        with self._state_lock:
            positions = self._current_positions.copy()

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self._joint_names)
        message.position = positions.tolist()
        self._joint_state_publisher.publish(message)


def _validate_joint_names(value: object) -> tuple[str, ...]:
    if not isinstance(value, (list, tuple)):
        raise TypeError("joint_names must be a string array")
    names = tuple(value)
    if len(names) != JOINT_COUNT:
        raise ValueError(f"joint_names must contain exactly {JOINT_COUNT} names")
    if any(not isinstance(name, str) or not name for name in names):
        raise ValueError("joint_names must contain non-empty strings")
    if len(set(names)) != len(names):
        raise ValueError("joint_names must be unique")
    return names


def _positive_finite_parameter(name: str, value: object) -> float:
    numeric = _finite_real_parameter(name, value)
    if numeric <= 0.0:
        raise ValueError(f"{name} must be greater than zero")
    return numeric


def _nonnegative_finite_parameter(name: str, value: object) -> float:
    numeric = _finite_real_parameter(name, value)
    if numeric < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return numeric


def _finite_real_parameter(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a real number")
    try:
        numeric = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(numeric):
        raise ValueError(f"{name} must be finite")
    return numeric


def main(args: list[str] | None = None) -> None:
    """Run the mock manipulator node."""

    rclpy.init(args=args)
    node = MockManipulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
