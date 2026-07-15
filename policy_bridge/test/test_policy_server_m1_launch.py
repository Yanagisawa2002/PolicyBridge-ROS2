"""ROS 2 Humble integration tests for M1 fault handling and deterministic hold."""

from __future__ import annotations

import importlib.util
import math
import time
import unittest
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import pytest

if importlib.util.find_spec("rclpy") is None:
    pytest.skip(
        "rclpy is not installed; ROS 2 launch tests are unavailable", allow_module_level=True
    )

# ROS imports intentionally follow the module-level availability guard so the
# regular non-ROS test suite remains collectable on development hosts.
import launch  # noqa: E402
import launch_ros.actions  # noqa: E402
import launch_testing.actions  # noqa: E402
import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

from policy_bridge_interfaces.action import ExecutePolicy  # noqa: E402

_DISCOVERY_TIMEOUT = 20.0
_FUTURE_TIMEOUT = 10.0
_RESULT_TIMEOUT = 15.0
_CONTROL_RATE_HZ = 20.0
_CONTROL_PERIOD = 1.0 / _CONTROL_RATE_HZ
_DIAGNOSTICS_RATE_HZ = 10.0

_JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))
_START = (1.0,) * 6
_HALF = (0.5,) * 6
_PARTIAL = (0.25,) * 6
_HOME = (0.0,) * 6


@dataclass(frozen=True)
class _ServerSpec:
    key: str
    backend: str
    inference_timeout: float
    observation_timeout: float
    backend_delay: float = 1.0

    @property
    def action(self) -> str:
        return f"/policy_bridge_m1/{self.key}/execute_policy"

    @property
    def joint_state(self) -> str:
        return f"/policy_bridge_m1/{self.key}/joint_states"

    @property
    def joint_command(self) -> str:
        return f"/policy_bridge_m1/{self.key}/joint_command"

    @property
    def diagnostics(self) -> str:
        return f"/policy_bridge_m1/{self.key}/diagnostics"


_SPECS = {
    spec.key: spec
    for spec in (
        _ServerSpec("observation", "scripted", 1.0, 0.40),
        _ServerSpec("scripted", "scripted", 1.0, 0.80),
        _ServerSpec("delayed", "delayed", 0.35, 2.0, backend_delay=1.50),
        _ServerSpec("delayed_race", "delayed", 0.55, 2.0, backend_delay=0.90),
        _ServerSpec("invalid_wrong_shape", "invalid_wrong_shape", 1.0, 1.0),
        _ServerSpec("invalid_nan", "invalid_nan", 1.0, 1.0),
        _ServerSpec("invalid_inf", "invalid_inf", 1.0, 1.0),
        _ServerSpec("raising", "raising", 1.0, 1.0),
    )
}


def _launch_server(spec: _ServerSpec) -> launch_ros.actions.Node:
    return launch_ros.actions.Node(
        package="policy_bridge",
        executable="policy_server",
        name=f"policy_server_m1_{spec.key}",
        output="screen",
        parameters=[
            {
                "action_name": spec.action,
                "joint_state_topic": spec.joint_state,
                "joint_command_topic": spec.joint_command,
                "control_rate_hz": _CONTROL_RATE_HZ,
                "policy_backend": spec.backend,
                "inference_timeout_seconds": spec.inference_timeout,
                "joint_state_timeout_seconds": spec.observation_timeout,
                "diagnostics_rate_hz": _DIAGNOSTICS_RATE_HZ,
                "delayed_policy_delay_seconds": spec.backend_delay,
                "delayed_policy_first_call_only": True,
            }
        ],
        remappings=[("/policy_bridge/diagnostics", spec.diagnostics)],
    )


@pytest.mark.launch_test
def generate_test_description() -> launch.LaunchDescription:
    """Launch isolated servers so each persistent backend has deterministic state."""

    return launch.LaunchDescription(
        [*(_launch_server(spec) for spec in _SPECS.values()), launch_testing.actions.ReadyToTest()]
    )


class _Endpoint:
    """Controlled observation publisher and command/diagnostics recorder."""

    def __init__(self, node: rclpy.node.Node, spec: _ServerSpec) -> None:
        self.node = node
        self.spec = spec
        self.commands: list[tuple[float, ...]] = []
        self.command_times: list[float] = []
        self.diagnostics: list[DiagnosticArray] = []
        self.state_responses: deque[tuple[float, ...]] = deque()

        self.state_publisher = node.create_publisher(JointState, spec.joint_state, 10)
        self.command_subscription = node.create_subscription(
            Float64MultiArray, spec.joint_command, self._on_command, 10
        )
        self.diagnostics_subscription = node.create_subscription(
            DiagnosticArray, spec.diagnostics, self.diagnostics.append, 10
        )
        self.action_client = ActionClient(node, ExecutePolicy, spec.action)

    def destroy(self) -> None:
        self.action_client.destroy()

    def clear_recordings(self) -> None:
        self.commands.clear()
        self.command_times.clear()
        self.diagnostics.clear()
        self.state_responses.clear()

    def _on_command(self, message: Float64MultiArray) -> None:
        self.commands.append(tuple(message.data))
        self.command_times.append(time.monotonic())
        if self.state_responses:
            self.publish_state(self.state_responses.popleft())

    def publish_state(self, positions: tuple[float, ...]) -> None:
        message = JointState()
        message.name = list(_JOINT_NAMES)
        message.position = list(positions)
        self.state_publisher.publish(message)

    def publish_raw_state(self, names: tuple[str, ...], positions: tuple[float, ...]) -> None:
        message = JointState()
        message.name = list(names)
        message.position = list(positions)
        self.state_publisher.publish(message)


class TestPolicyServerM1(unittest.TestCase):
    """Exercise M1 termination paths against real ROS action and topic transport."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node(f"policy_server_m1_test_{self._testMethodName}")
        self.endpoints = {key: _Endpoint(self.node, spec) for key, spec in _SPECS.items()}
        for endpoint in self.endpoints.values():
            self.assertTrue(
                endpoint.action_client.wait_for_server(timeout_sec=_DISCOVERY_TIMEOUT),
                f"{endpoint.spec.key} action server was not discovered",
            )
            self._spin_until(
                lambda endpoint=endpoint: (
                    self.node.count_subscribers(endpoint.spec.joint_state) >= 1
                    and self.node.count_publishers(endpoint.spec.joint_command) >= 1
                    and self.node.count_publishers(endpoint.spec.diagnostics) >= 1
                ),
                _DISCOVERY_TIMEOUT,
                f"{endpoint.spec.key} ROS endpoints were not discovered",
            )

    def tearDown(self) -> None:
        for endpoint in self.endpoints.values():
            endpoint.destroy()
        self.node.destroy_node()

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

    def _wait_future(self, future: Any, timeout: float, description: str) -> Any:
        self._spin_until(future.done, timeout, f"timed out waiting for {description}")
        return future.result()

    def _send_goal(
        self,
        endpoint: _Endpoint,
        *,
        instruction: str = "move to home",
        max_steps: int = 50,
        timeout_seconds: float = 0.0,
    ) -> tuple[Any, list[int]]:
        feedback_steps: list[int] = []
        goal = ExecutePolicy.Goal()
        goal.instruction = instruction
        goal.max_steps = max_steps
        goal.timeout_seconds = timeout_seconds
        send_future = endpoint.action_client.send_goal_async(
            goal,
            feedback_callback=lambda message: feedback_steps.append(message.feedback.current_step),
        )
        handle = self._wait_future(send_future, _FUTURE_TIMEOUT, "goal response")
        return handle, feedback_steps

    def _get_result(self, handle: Any) -> Any:
        return self._wait_future(
            handle.get_result_async(), _RESULT_TIMEOUT, "terminal action result"
        )

    def _cancel(self, handle: Any) -> Any:
        return self._wait_future(handle.cancel_goal_async(), _FUTURE_TIMEOUT, "cancel response")

    def _prime_valid_state(self, endpoint: _Endpoint, positions: tuple[float, ...]) -> None:
        """Publish repeatedly and confirm validity through the public diagnostics."""

        endpoint.diagnostics.clear()
        # Multiple sends make DDS discovery and receipt explicit without an arbitrary sleep.
        for _ in range(5):
            endpoint.publish_state(positions)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self._spin_until(
            lambda: (
                self._latest_value(endpoint, "policy_bridge/observation", "joint_state_valid")
                == "true"
            ),
            2.0,
            f"{endpoint.spec.key} did not report a valid joint state",
        )

    @staticmethod
    def _values(status: DiagnosticStatus) -> dict[str, str]:
        return {item.key: item.value.strip().lower() for item in status.values}

    def _matching_statuses(self, endpoint: _Endpoint, name: str) -> list[DiagnosticStatus]:
        return [
            status
            for message in endpoint.diagnostics
            for status in message.status
            if status.name == name
        ]

    def _latest_value(self, endpoint: _Endpoint, name: str, key: str) -> str | None:
        statuses = self._matching_statuses(endpoint, name)
        if not statuses:
            return None
        return self._values(statuses[-1]).get(key)

    def _wait_for_value(
        self,
        endpoint: _Endpoint,
        name: str,
        key: str,
        value: str,
        timeout: float = 3.0,
    ) -> None:
        expected = value.lower()
        self._spin_until(
            lambda: self._latest_value(endpoint, name, key) == expected,
            timeout,
            f"{endpoint.spec.key} diagnostics never reported {key}={expected}",
        )

    def _assert_required_diagnostics(self, endpoint: _Endpoint) -> None:
        required = {
            "policy_bridge/runtime": {
                "runtime_state",
                "active_goal",
                "episode_id",
                "last_termination_reason",
                "goal_elapsed_ms",
                "safe_stop_count",
            },
            "policy_bridge/observation": {
                "joint_state_received",
                "joint_state_valid",
                "joint_state_age_ms",
                "joint_state_timeout_ms",
            },
            "policy_bridge/policy": {
                "backend_name",
                "backend_busy",
                "last_inference_latency_ms",
                "inference_timeout_ms",
                "last_policy_error",
            },
        }
        self._spin_until(
            lambda: all(self._matching_statuses(endpoint, name) for name in required),
            3.0,
            f"{endpoint.spec.key} did not publish all diagnostic statuses",
        )
        for name, keys in required.items():
            present = set(self._values(self._matching_statuses(endpoint, name)[-1]))
            self.assertTrue(keys <= present, f"{name} missing keys {sorted(keys - present)}")

    def _assert_fault_diagnostic(self, endpoint: _Endpoint, reason: str) -> None:
        def matching_fault_seen() -> bool:
            for status in self._matching_statuses(endpoint, "policy_bridge/runtime"):
                values = self._values(status)
                if (
                    values.get("last_termination_reason") == reason
                    and status.level != DiagnosticStatus.OK
                ):
                    return True
            return False

        self._spin_until(
            matching_fault_seen,
            3.0,
            f"{endpoint.spec.key} did not publish non-OK diagnostics for {reason}",
        )

    def _assert_component_fault_diagnostic(
        self,
        endpoint: _Endpoint,
        component: str,
        reason: str,
    ) -> None:
        self._spin_until(
            lambda: any(
                status.level == DiagnosticStatus.ERROR and status.message == reason
                for status in self._matching_statuses(endpoint, component)
            ),
            3.0,
            f"{component} did not report ERROR/{reason}",
        )

    def _assert_idle_diagnostics_recovered(
        self,
        endpoint: _Endpoint,
        reason: str,
    ) -> None:
        component_names = (
            "policy_bridge/runtime",
            "policy_bridge/observation",
            "policy_bridge/policy",
        )

        def recovered() -> bool:
            latest = {
                name: self._matching_statuses(endpoint, name)[-1]
                for name in component_names
                if self._matching_statuses(endpoint, name)
            }
            if len(latest) != len(component_names):
                return False
            runtime_values = self._values(latest["policy_bridge/runtime"])
            return (
                all(status.level == DiagnosticStatus.OK for status in latest.values())
                and runtime_values.get("runtime_state") == "idle"
                and runtime_values.get("active_goal") == "false"
                and runtime_values.get("last_termination_reason") == reason
            )

        self._spin_until(
            recovered,
            3.0,
            f"{endpoint.spec.key} diagnostics did not recover to idle/OK after {reason}",
        )

    def _assert_diagnostics_contain(self, endpoint: _Endpoint, text: str) -> None:
        needle = text.lower()

        def contains() -> bool:
            for message in endpoint.diagnostics:
                for status in message.status:
                    fields = [status.message, *(item.value for item in status.values)]
                    if any(needle in field.lower() for field in fields):
                        return True
            return False

        self._spin_until(
            contains,
            3.0,
            f"{endpoint.spec.key} diagnostics did not contain {text!r}",
        )

    def _assert_aborted(self, wrapped: Any, reason: str) -> None:
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(wrapped.result.success)
        self.assertEqual(wrapped.result.termination_reason, reason)
        self.assertTrue(wrapped.result.episode_id)

    def _assert_no_commands(self, endpoint: _Endpoint) -> None:
        self._spin_for(3.0 * _CONTROL_PERIOD)
        self.assertEqual(endpoint.commands, [])

    def _assert_hold_is_last(self, endpoint: _Endpoint, expected: tuple[float, ...]) -> None:
        # Action result and topic samples travel over different DDS entities;
        # do not assume their arrival order in the test process.
        self._spin_until(
            lambda: expected in endpoint.commands,
            2.0,
            "fault did not publish the expected hold command",
        )
        self.assertEqual(endpoint.commands[-1], expected)
        self.assertEqual(endpoint.commands.count(expected), 1, "hold was published more than once")
        count_at_result = len(endpoint.commands)
        self._spin_for(3.0 * _CONTROL_PERIOD)
        self.assertEqual(
            len(endpoint.commands), count_at_result, "normal commands continued after termination"
        )

    def test_goal_validation_and_observation_faults(self) -> None:
        endpoint = self.endpoints["observation"]
        self._assert_required_diagnostics(endpoint)

        negative_timeout, _ = self._send_goal(endpoint, timeout_seconds=-0.1)
        self.assertFalse(negative_timeout.accepted)
        invalid_steps, _ = self._send_goal(endpoint, max_steps=0)
        self.assertFalse(invalid_steps.accepted)

        # Keep the first valid goal waiting for observation while probing active-goal rejection.
        waiting_goal, _ = self._send_goal(endpoint)
        self.assertTrue(waiting_goal.accepted)
        concurrent_goal, _ = self._send_goal(endpoint)
        self.assertFalse(concurrent_goal.accepted)
        waiting_result = self._get_result(waiting_goal)
        self._assert_aborted(waiting_result, "observation_timeout")
        self._assert_no_commands(endpoint)
        self._assert_fault_diagnostic(endpoint, "observation_timeout")
        self._assert_component_fault_diagnostic(
            endpoint, "policy_bridge/observation", "observation_timeout"
        )
        self._assert_diagnostics_contain(endpoint, "safe_stop_unavailable_no_valid_state")
        self._assert_idle_diagnostics_recovered(endpoint, "observation_timeout")

        # Cancel as soon as the accepted handle is returned.  The server must
        # accept this even if execute_callback has not yet bound the ROS UUID.
        endpoint.clear_recordings()
        early_cancel_goal, _ = self._send_goal(endpoint)
        self.assertTrue(early_cancel_goal.accepted)
        early_cancel_response = self._cancel(early_cancel_goal)
        self.assertEqual(len(early_cancel_response.goals_canceling), 1)
        early_canceled = self._get_result(early_cancel_goal)
        self.assertEqual(early_canceled.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(early_canceled.result.termination_reason, "goal_canceled")
        self._assert_no_commands(endpoint)

        invalid_observations = (
            ("wrong_shape", _JOINT_NAMES[:-1], _START[:-1]),
            ("extra_joint", (*_JOINT_NAMES, "joint_7"), (*_START, 0.0)),
            ("nan", _JOINT_NAMES, (math.nan, *_START[1:])),
            ("inf", _JOINT_NAMES, (math.inf, *_START[1:])),
        )
        for label, names, positions in invalid_observations:
            with self.subTest(observation=label):
                endpoint.clear_recordings()
                for _ in range(3):
                    endpoint.publish_raw_state(names, positions)
                    rclpy.spin_once(self.node, timeout_sec=0.05)
                self._wait_for_value(
                    endpoint,
                    "policy_bridge/observation",
                    "joint_state_received",
                    "true",
                )
                self._wait_for_value(
                    endpoint,
                    "policy_bridge/observation",
                    "joint_state_valid",
                    "false",
                )
                handle, _ = self._send_goal(endpoint)
                self.assertTrue(handle.accepted)
                wrapped = self._get_result(handle)
                self._assert_aborted(wrapped, "observation_timeout")
                self._assert_no_commands(endpoint)

    def test_inference_timeout_busy_late_result_and_deadline_race(self) -> None:
        endpoint = self.endpoints["delayed"]
        self._prime_valid_state(endpoint, _START)
        endpoint.clear_recordings()

        handle, _ = self._send_goal(endpoint, timeout_seconds=2.0)
        self.assertTrue(handle.accepted)
        wrapped = self._get_result(handle)
        self._assert_aborted(wrapped, "policy_inference_timeout")
        self._assert_hold_is_last(endpoint, _START)
        self._assert_fault_diagnostic(endpoint, "policy_inference_timeout")
        self._assert_component_fault_diagnostic(
            endpoint, "policy_bridge/policy", "policy_inference_timeout"
        )
        self._wait_for_value(endpoint, "policy_bridge/policy", "backend_busy", "true")

        busy_goal, _ = self._send_goal(endpoint, timeout_seconds=2.0)
        self.assertFalse(busy_goal.accepted, "a busy backend accepted a second goal")

        # The delayed result is the normal home target.  It must never appear after timeout.
        self._wait_for_value(
            endpoint,
            "policy_bridge/policy",
            "backend_busy",
            "false",
            timeout=3.0,
        )
        self._spin_for(2.0 * _CONTROL_PERIOD)
        self.assertNotIn(_HOME, endpoint.commands, "late inference result was executed")

        # Cancellation must be polled while synchronous policy code is still
        # running on the worker; it cannot wait for the full backend delay.
        self._prime_valid_state(endpoint, _START)
        endpoint.clear_recordings()
        cancel_handle, _ = self._send_goal(endpoint, timeout_seconds=2.0)
        self.assertTrue(cancel_handle.accepted)
        self._wait_for_value(endpoint, "policy_bridge/policy", "backend_busy", "true")
        cancel_started = time.monotonic()
        cancel_response = self._cancel(cancel_handle)
        self.assertEqual(len(cancel_response.goals_canceling), 1)
        canceled = self._get_result(cancel_handle)
        cancel_elapsed = time.monotonic() - cancel_started
        self.assertEqual(canceled.status, GoalStatus.STATUS_CANCELED)
        self.assertFalse(canceled.result.success)
        self.assertEqual(canceled.result.termination_reason, "goal_canceled")
        self.assertLess(
            cancel_elapsed,
            1.00,
            "cancellation blocked until the delayed synchronous backend returned",
        )
        self._assert_hold_is_last(endpoint, _START)
        canceled_busy_goal, _ = self._send_goal(endpoint, timeout_seconds=2.0)
        self.assertFalse(canceled_busy_goal.accepted)
        self._wait_for_value(
            endpoint,
            "policy_bridge/policy",
            "backend_busy",
            "false",
            timeout=3.0,
        )
        self._spin_for(2.0 * _CONTROL_PERIOD)
        self.assertNotIn(_HOME, endpoint.commands, "canceled inference result was executed")

        # Put goal and inference deadlines close together, but with goal timeout first.
        race = self.endpoints["delayed_race"]
        self._prime_valid_state(race, _START)
        race.clear_recordings()
        race_handle, _ = self._send_goal(race, timeout_seconds=0.45)
        self.assertTrue(race_handle.accepted)
        race_wrapped = self._get_result(race_handle)
        self._assert_aborted(race_wrapped, "goal_timeout")
        self._assert_hold_is_last(race, _START)
        self._wait_for_value(
            race,
            "policy_bridge/policy",
            "backend_busy",
            "false",
            timeout=3.0,
        )
        self._spin_for(2.0 * _CONTROL_PERIOD)
        self.assertNotIn(_HOME, race.commands, "deadline-race late result was executed")

    def test_invalid_actions_and_policy_exception(self) -> None:
        for key in ("invalid_wrong_shape", "invalid_nan", "invalid_inf"):
            with self.subTest(backend=key):
                endpoint = self.endpoints[key]
                self._prime_valid_state(endpoint, _START)
                endpoint.clear_recordings()
                handle, _ = self._send_goal(endpoint)
                self.assertTrue(handle.accepted)
                wrapped = self._get_result(handle)
                self._assert_aborted(wrapped, "invalid_action")
                self._assert_hold_is_last(endpoint, _START)
                self._assert_fault_diagnostic(endpoint, "invalid_action")
                self._assert_component_fault_diagnostic(
                    endpoint, "policy_bridge/policy", "invalid_action"
                )

        raising = self.endpoints["raising"]
        self._prime_valid_state(raising, _START)
        raising.clear_recordings()
        first_handle, _ = self._send_goal(raising)
        self.assertTrue(first_handle.accepted)
        first_result = self._get_result(first_handle)
        self._assert_aborted(first_result, "policy_error")
        self._assert_hold_is_last(raising, _START)
        self._assert_fault_diagnostic(raising, "policy_error")
        self._assert_component_fault_diagnostic(raising, "policy_bridge/policy", "policy_error")
        policy_errors = self._matching_statuses(raising, "policy_bridge/policy")
        self.assertTrue(
            any(self._values(status).get("last_policy_error") for status in policy_errors)
        )

        # The deterministic raising backend still fails, but accepting another goal proves
        # that the node and action server survived the exception and released active state.
        self._prime_valid_state(raising, _START)
        raising.clear_recordings()
        second_handle, _ = self._send_goal(raising)
        self.assertTrue(second_handle.accepted)
        self._assert_aborted(self._get_result(second_handle), "policy_error")

    def test_goal_timeout_stale_cancel_hold_recovery_and_first_wins(self) -> None:
        endpoint = self.endpoints["scripted"]

        # Unsupported instructions before motion do not require a hold.
        self._prime_valid_state(endpoint, _START)
        endpoint.clear_recordings()
        unsupported, _ = self._send_goal(endpoint, instruction="unsupported")
        self.assertTrue(unsupported.accepted)
        self._assert_aborted(self._get_result(unsupported), "unsupported_instruction")
        self._assert_no_commands(endpoint)

        # A monotonic goal deadline wins well before observation freshness expires.
        self._prime_valid_state(endpoint, _START)
        endpoint.clear_recordings()
        timeout_handle, _ = self._send_goal(endpoint, max_steps=100, timeout_seconds=0.45)
        self.assertTrue(timeout_handle.accepted)
        timeout_result = self._get_result(timeout_handle)
        self._assert_aborted(timeout_result, "goal_timeout")
        self.assertIn(_HOME, endpoint.commands, "goal timeout occurred before any policy command")
        self._assert_hold_is_last(endpoint, _START)
        self._assert_fault_diagnostic(endpoint, "goal_timeout")

        # Exhaustion after motion also uses the centralized one-shot hold path.
        self._prime_valid_state(endpoint, _START)
        endpoint.clear_recordings()
        step_handle, _ = self._send_goal(endpoint, max_steps=1)
        self.assertTrue(step_handle.accepted)
        step_result = self._get_result(step_handle)
        self._assert_aborted(step_result, "max_steps_exceeded")
        self.assertIn(_HOME, endpoint.commands)
        self._assert_hold_is_last(endpoint, _START)

        # Stop observations after a valid sample.  The last valid sample is the hold target.
        self._prime_valid_state(endpoint, _START)
        endpoint.clear_recordings()
        stale_handle, _ = self._send_goal(endpoint, max_steps=100)
        self.assertTrue(stale_handle.accepted)
        stale_result = self._get_result(stale_handle)
        self._assert_aborted(stale_result, "stale_observation")
        self.assertIn(_HOME, endpoint.commands)
        self._assert_hold_is_last(endpoint, _START)
        self._assert_fault_diagnostic(endpoint, "stale_observation")
        self._assert_component_fault_diagnostic(
            endpoint, "policy_bridge/observation", "stale_observation"
        )

        # Put max-step exhaustion within roughly one control period of the
        # freshness deadline.  Scheduling may let either condition lock first,
        # but one coherent result and one hold must be the only observable end.
        self._prime_valid_state(endpoint, _START)
        endpoint.clear_recordings()
        stale_step_race, _ = self._send_goal(endpoint, max_steps=15)
        self.assertTrue(stale_step_race.accepted)
        stale_step_result = self._get_result(stale_step_race)
        self.assertEqual(stale_step_result.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(stale_step_result.result.success)
        self.assertIn(
            stale_step_result.result.termination_reason,
            {"stale_observation", "max_steps_exceeded"},
        )
        self._assert_hold_is_last(endpoint, _START)

        # Cancel after the robot reports a distinct in-flight position.  Hold must use it.
        self._prime_valid_state(endpoint, _START)
        endpoint.clear_recordings()
        cancel_handle, _ = self._send_goal(endpoint, max_steps=100)
        self.assertTrue(cancel_handle.accepted)
        self._spin_until(
            lambda: _HOME in endpoint.commands,
            _FUTURE_TIMEOUT,
            "cancel scenario never published a policy command",
        )
        for _ in range(4):
            endpoint.publish_state(_HALF)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        cancel_response = self._cancel(cancel_handle)
        self.assertEqual(len(cancel_response.goals_canceling), 1)
        canceled = self._get_result(cancel_handle)
        self.assertEqual(canceled.status, GoalStatus.STATUS_CANCELED)
        self.assertFalse(canceled.result.success)
        self.assertEqual(canceled.result.termination_reason, "goal_canceled")
        self._assert_hold_is_last(endpoint, _HALF)

        # A fresh UUID must not inherit cancellation, and a controlled home update succeeds.
        self._prime_valid_state(endpoint, _PARTIAL)
        endpoint.clear_recordings()
        endpoint.state_responses.append(_HOME)
        fresh_handle, fresh_feedback = self._send_goal(endpoint, max_steps=3)
        self.assertTrue(fresh_handle.accepted)
        fresh = self._get_result(fresh_handle)
        self.assertEqual(fresh.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(fresh.result.success)
        self.assertEqual(fresh.result.termination_reason, "goal_reached")
        self.assertNotEqual(fresh.result.episode_id, canceled.result.episode_id)
        self.assertEqual(fresh_feedback, [1])

        # Race cancellation against the state update that makes the goal successful.
        # Either terminal state may lock first; it must be internally coherent and unique.
        self._prime_valid_state(endpoint, _START)
        safe_stop_count_before_race = int(
            self._latest_value(
                endpoint,
                "policy_bridge/runtime",
                "safe_stop_count",
            )
            or "0"
        )
        endpoint.clear_recordings()
        endpoint.state_responses.append(_HOME)
        race_handle, _ = self._send_goal(endpoint, max_steps=3)
        self.assertTrue(race_handle.accepted)
        self._spin_until(
            lambda: bool(endpoint.commands),
            _FUTURE_TIMEOUT,
            "success/cancel race never published its policy command",
        )
        race_cancel = self._cancel(race_handle)
        race_result = self._get_result(race_handle)
        if race_cancel.goals_canceling:
            self.assertEqual(race_result.status, GoalStatus.STATUS_CANCELED)
            self.assertFalse(race_result.result.success)
            self.assertEqual(race_result.result.termination_reason, "goal_canceled")
            self._spin_until(
                lambda: len(endpoint.commands) >= 2,
                2.0,
                "cancel-won race did not publish one hold command",
            )
            self.assertEqual(endpoint.commands[-1], _HOME)
            self._wait_for_value(
                endpoint,
                "policy_bridge/runtime",
                "safe_stop_count",
                str(safe_stop_count_before_race + 1),
            )
            count_after_hold = len(endpoint.commands)
            self._spin_for(3.0 * _CONTROL_PERIOD)
            self.assertEqual(len(endpoint.commands), count_after_hold)
        else:
            self.assertEqual(race_result.status, GoalStatus.STATUS_SUCCEEDED)
            self.assertTrue(race_result.result.success)
            self.assertEqual(race_result.result.termination_reason, "goal_reached")

        # Finish with another normal goal to prove the server remains reusable after races.
        self._prime_valid_state(endpoint, _PARTIAL)
        endpoint.clear_recordings()
        endpoint.state_responses.append(_HOME)
        final_handle, _ = self._send_goal(endpoint, max_steps=3)
        self.assertTrue(final_handle.accepted)
        final_result = self._get_result(final_handle)
        self.assertEqual(final_result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(final_result.result.success)
        self.assertEqual(final_result.result.termination_reason, "goal_reached")
        self._assert_required_diagnostics(endpoint)
        self._wait_for_value(endpoint, "policy_bridge/runtime", "runtime_state", "idle")
        for name in (
            "policy_bridge/runtime",
            "policy_bridge/observation",
            "policy_bridge/policy",
        ):
            self.assertEqual(
                self._matching_statuses(endpoint, name)[-1].level,
                DiagnosticStatus.OK,
                f"normal terminal state left {name} non-OK",
            )
