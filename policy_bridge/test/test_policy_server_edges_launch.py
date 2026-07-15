"""Deterministic ROS 2 launch tests for policy-server step and cancel boundaries."""

from __future__ import annotations

import importlib.util
import time
import unittest
from collections import deque
from collections.abc import Callable

import pytest

if importlib.util.find_spec("rclpy") is None:
    pytest.skip(
        "rclpy is not installed; ROS 2 launch tests are unavailable", allow_module_level=True
    )

# These imports intentionally follow the rclpy availability guard so the regular
# non-ROS pytest run can collect this module on development hosts.
import launch  # noqa: E402
import launch_ros.actions  # noqa: E402
import launch_testing.actions  # noqa: E402
import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

from policy_bridge_interfaces.action import ExecutePolicy  # noqa: E402

_ACTION_NAME = "/policy_bridge_edges_launch/execute_policy"
_JOINT_STATE_TOPIC = "/policy_bridge_edges_launch/joint_states"
_JOINT_COMMAND_TOPIC = "/policy_bridge_edges_launch/joint_command"
_CONTROL_RATE_HZ = 5.0
_CONTROL_PERIOD = 1.0 / _CONTROL_RATE_HZ
_DISCOVERY_TIMEOUT = 15.0
_FUTURE_TIMEOUT = 10.0
_RESULT_TIMEOUT = 15.0

_JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))
_START = (1.0,) * 6
_PARTIAL = (0.5,) * 6
_HOME = (0.0,) * 6


@pytest.mark.launch_test
def generate_test_description() -> tuple[launch.LaunchDescription, dict[str, object]]:
    """Launch only the policy server, isolated behind test-specific ROS names."""

    policy_server = launch_ros.actions.Node(
        package="policy_bridge",
        executable="policy_server",
        name="policy_server_edges_launch",
        output="screen",
        parameters=[
            {
                "action_name": _ACTION_NAME,
                "joint_state_topic": _JOINT_STATE_TOPIC,
                "joint_command_topic": _JOINT_COMMAND_TOPIC,
                "control_rate_hz": _CONTROL_RATE_HZ,
            }
        ],
    )

    return (
        launch.LaunchDescription(
            [
                policy_server,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"policy_server": policy_server},
    )


class TestPolicyServerEdges(unittest.TestCase):
    """Drive exact observations around the server's step and cancel boundaries."""

    @classmethod
    def setUpClass(cls) -> None:
        """Initialize the ROS client library for the in-process test node."""

        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        """Shut down the test process's ROS context."""

        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self) -> None:
        """Create the controlled state publisher, command observer, and action client."""

        self.node = rclpy.create_node("policy_server_edges_launch_test")
        self.joint_state_publisher = self.node.create_publisher(
            JointState,
            _JOINT_STATE_TOPIC,
            10,
        )
        self.commands: list[tuple[float, ...]] = []
        self.state_updates_after_commands = 0
        self.state_responses: deque[tuple[float, ...]] = deque()
        self.joint_command_subscription = self.node.create_subscription(
            Float64MultiArray,
            _JOINT_COMMAND_TOPIC,
            self._on_joint_command,
            10,
        )
        self.action_client = ActionClient(self.node, ExecutePolicy, _ACTION_NAME)

        self.assertTrue(
            self.action_client.wait_for_server(timeout_sec=_DISCOVERY_TIMEOUT),
            "policy action server was not discovered before the hard timeout",
        )
        self._spin_until(
            lambda: (
                self.node.count_subscribers(_JOINT_STATE_TOPIC) >= 1
                and self.node.count_publishers(_JOINT_COMMAND_TOPIC) >= 1
            ),
            _DISCOVERY_TIMEOUT,
            "isolated joint-state/command endpoints were not discovered",
        )

    def tearDown(self) -> None:
        """Release action and node resources after the launch scenario."""

        self.action_client.destroy()
        self.node.destroy_node()

    def _on_joint_command(self, message: Float64MultiArray) -> None:
        """Record a command and publish the next explicitly configured observation."""

        self.commands.append(tuple(message.data))
        if self.state_responses:
            self._publish_joint_state(self.state_responses.popleft())
            self.state_updates_after_commands += 1

    def _publish_joint_state(self, positions: tuple[float, ...]) -> None:
        message = JointState()
        message.name = list(_JOINT_NAMES)
        message.position = list(positions)
        self.joint_state_publisher.publish(message)

    def _spin_until(
        self,
        predicate: Callable[[], bool],
        timeout: float,
        failure_message: str,
    ) -> None:
        deadline = time.monotonic() + timeout
        while not predicate():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.fail(failure_message)
            rclpy.spin_once(self.node, timeout_sec=min(0.05, remaining))

    def _spin_for(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            rclpy.spin_once(self.node, timeout_sec=min(0.05, remaining))

    def _prime_server_state(self, positions: tuple[float, ...]) -> None:
        """Repeat an idle observation long enough to cross a full control period."""

        deadline = time.monotonic() + 2.0 * _CONTROL_PERIOD
        while True:
            self._publish_joint_state(positions)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            rclpy.spin_once(self.node, timeout_sec=min(0.05, remaining))

    def _start_scenario(self, responses: list[tuple[float, ...]]) -> None:
        self.state_responses.clear()
        self._prime_server_state(_START)
        self.commands.clear()
        self.state_updates_after_commands = 0
        self.state_responses.extend(responses)

    def _wait_for_future(self, future: object, timeout: float, description: str) -> object:
        self._spin_until(
            lambda: future.done(),  # type: ignore[attr-defined]
            timeout,
            f"timed out waiting for {description}",
        )
        return future.result()  # type: ignore[attr-defined, no-any-return]

    def _send_goal(self, max_steps: int) -> tuple[object, list[int]]:
        feedback_steps: list[int] = []
        goal = ExecutePolicy.Goal()
        goal.instruction = "move to home"
        goal.max_steps = max_steps
        goal.timeout_seconds = 0.0

        send_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=lambda message: feedback_steps.append(message.feedback.current_step),
        )
        goal_handle = self._wait_for_future(send_future, _FUTURE_TIMEOUT, "goal acceptance")
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted, "policy server rejected an idle valid goal")
        return goal_handle, feedback_steps

    def _get_result(self, goal_handle: object) -> object:
        result_future = goal_handle.get_result_async()  # type: ignore[attr-defined]
        return self._wait_for_future(result_future, _RESULT_TIMEOUT, "action result")

    def _assert_home_commands(self, expected_count: int) -> None:
        self.assertEqual(len(self.commands), expected_count)
        for command in self.commands:
            self.assertEqual(command, _HOME)

    def test_step_limits_cancel_and_fresh_goal(self) -> None:
        """Verify exact step outcomes, cancellation gating, and goal-local cleanup."""

        # The first update remains outside tolerance; the second reaches home.
        self._start_scenario([_PARTIAL, _HOME])
        two_step_goal, two_step_feedback = self._send_goal(max_steps=2)
        two_step_wrapped = self._get_result(two_step_goal)

        self.assertEqual(two_step_wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(two_step_wrapped.result.success)
        self.assertEqual(two_step_wrapped.result.termination_reason, "goal_reached")
        self.assertEqual(two_step_feedback, [1, 2])
        self._assert_home_commands(expected_count=2)
        self.assertEqual(self.state_updates_after_commands, 2)
        self.assertFalse(self.state_responses)

        # A single partial update consumes the only allowed step and must abort.
        self._start_scenario([_PARTIAL])
        one_step_goal, one_step_feedback = self._send_goal(max_steps=1)
        one_step_wrapped = self._get_result(one_step_goal)

        self.assertEqual(one_step_wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(one_step_wrapped.result.success)
        self.assertEqual(one_step_wrapped.result.termination_reason, "max_steps_exceeded")
        self.assertEqual(one_step_feedback, [1])
        self.assertEqual(self.commands, [_HOME, _PARTIAL])
        self.assertEqual(self.state_updates_after_commands, 1)
        self.assertFalse(self.state_responses)

        # Cancel immediately after observing the first command.  No state response is
        # necessary: the server's cancel callback wakes its control-period wait.
        self._start_scenario([])
        cancel_goal, _cancel_feedback = self._send_goal(max_steps=50)
        self._spin_until(
            lambda: len(self.commands) >= 1,
            _FUTURE_TIMEOUT,
            "the cancel scenario did not publish its first command",
        )
        self._assert_home_commands(expected_count=1)

        cancel_future = cancel_goal.cancel_goal_async()  # type: ignore[attr-defined]
        cancel_response = self._wait_for_future(
            cancel_future,
            _FUTURE_TIMEOUT,
            "cancel acknowledgement",
        )
        self.assertEqual(len(cancel_response.goals_canceling), 1)
        canceled_wrapped = self._get_result(cancel_goal)

        self.assertEqual(canceled_wrapped.status, GoalStatus.STATUS_CANCELED)
        self.assertFalse(canceled_wrapped.result.success)
        self.assertEqual(canceled_wrapped.result.termination_reason, "goal_canceled")
        canceled_episode_id = canceled_wrapped.result.episode_id
        self.assertTrue(canceled_episode_id)

        self._spin_until(
            lambda: len(self.commands) >= 2,
            _FUTURE_TIMEOUT,
            "the cancel scenario did not publish its hold-position command",
        )
        self.assertEqual(self.commands, [_HOME, _START])
        command_count_after_cancel = len(self.commands)
        self._spin_for(2.5 * _CONTROL_PERIOD)
        self.assertEqual(len(self.commands), command_count_after_cancel)
        self.assertEqual(command_count_after_cancel, 2)

        # A new goal must not inherit the prior goal UUID's cancellation state.
        self._start_scenario([_HOME])
        fresh_goal, fresh_feedback = self._send_goal(max_steps=2)
        fresh_wrapped = self._get_result(fresh_goal)

        self.assertEqual(fresh_wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(fresh_wrapped.result.success)
        self.assertEqual(fresh_wrapped.result.termination_reason, "goal_reached")
        self.assertTrue(fresh_wrapped.result.episode_id)
        self.assertNotEqual(fresh_wrapped.result.episode_id, canceled_episode_id)
        self.assertEqual(fresh_feedback, [1])
        self._assert_home_commands(expected_count=1)
        self.assertEqual(self.state_updates_after_commands, 1)
        self.assertFalse(self.state_responses)
