"""ROS 2 Humble integration tests for synchronized M2 observations."""

from __future__ import annotations

import importlib.util
import time
import unittest
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import pytest

if importlib.util.find_spec("rclpy") is None:
    pytest.skip(
        "rclpy is not installed; ROS 2 launch tests are unavailable", allow_module_level=True
    )

# ROS imports intentionally follow the availability guard so non-ROS pytest can
# collect this module on development hosts.
import launch  # noqa: E402
import launch_ros.actions  # noqa: E402
import launch_testing.actions  # noqa: E402
import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image, JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

from policy_bridge_interfaces.action import ExecutePolicy  # noqa: E402

_DISCOVERY_TIMEOUT = 20.0
_FUTURE_TIMEOUT = 10.0
_RESULT_TIMEOUT = 10.0
_CONTROL_RATE_HZ = 20.0
_CONTROL_PERIOD = 1.0 / _CONTROL_RATE_HZ
_DIAGNOSTICS_RATE_HZ = 20.0

_JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))
_START = (1.0,) * 6
_HALF = (0.5,) * 6
_PARTIAL = (0.25,) * 6
_HOME = (0.0,) * 6


@dataclass(frozen=True)
class _ServerSpec:
    key: str
    image_timeout: float
    synchronization_timeout: float
    sync_slop: float
    joint_timeout: float = 0.8

    @property
    def action(self) -> str:
        return f"/policy_bridge_m2/{self.key}/execute_policy"

    @property
    def joint_state(self) -> str:
        return f"/policy_bridge_m2/{self.key}/joint_states"

    @property
    def image(self) -> str:
        return f"/policy_bridge_m2/{self.key}/image_raw"

    @property
    def joint_command(self) -> str:
        return f"/policy_bridge_m2/{self.key}/joint_command"

    @property
    def diagnostics(self) -> str:
        return f"/policy_bridge_m2/{self.key}/diagnostics"


_SPECS = {
    spec.key: spec
    for spec in (
        # Exact synchronization exercises the dedicated TimeSynchronizer path.
        _ServerSpec("normal", 1.2, 0.8, 0.0),
        _ServerSpec("missing", 0.30, 0.60, 0.02),
        # Image expiry must win before synchronization expiry in this scenario.
        _ServerSpec("stale", 0.35, 1.20, 0.02),
        _ServerSpec("unsynchronized", 1.50, 0.35, 0.01),
        _ServerSpec("invalid", 0.30, 0.60, 0.02),
        _ServerSpec("image_goal_race", 0.40, 1.20, 0.02),
        _ServerSpec("sync_joint_race", 1.50, 0.42, 0.01, joint_timeout=0.42),
    )
}


def _launch_server(spec: _ServerSpec) -> launch_ros.actions.Node:
    return launch_ros.actions.Node(
        package="policy_bridge",
        executable="policy_server",
        name=f"policy_server_m2_{spec.key}",
        output="screen",
        parameters=[
            {
                "action_name": spec.action,
                "joint_state_topic": spec.joint_state,
                "joint_command_topic": spec.joint_command,
                "image_topic": spec.image,
                "observation_mode": "rgb_joint",
                "policy_backend": "multimodal_scripted",
                "control_rate_hz": _CONTROL_RATE_HZ,
                "joint_state_timeout_seconds": spec.joint_timeout,
                "image_timeout_seconds": spec.image_timeout,
                "synchronized_observation_timeout_seconds": (spec.synchronization_timeout),
                "sync_queue_size": 4,
                "sync_slop_seconds": spec.sync_slop,
                "max_image_pixels": 64,
                "diagnostics_rate_hz": _DIAGNOSTICS_RATE_HZ,
            }
        ],
        remappings=[("/policy_bridge/diagnostics", spec.diagnostics)],
    )


@pytest.mark.launch_test
def generate_test_description() -> launch.LaunchDescription:
    """Launch isolated servers so persistent observation state cannot cross scenarios."""

    return launch.LaunchDescription(
        [*(_launch_server(spec) for spec in _SPECS.values()), launch_testing.actions.ReadyToTest()]
    )


class _Endpoint:
    """Publish controlled multimodal inputs and record action-server outputs."""

    def __init__(self, node: rclpy.node.Node, spec: _ServerSpec) -> None:
        self.node = node
        self.spec = spec
        self.commands: list[tuple[float, ...]] = []
        self.command_times: list[float] = []
        self.diagnostics: list[DiagnosticArray] = []
        self._next_stamp_ns = 10_000_000_000

        self.joint_publisher = node.create_publisher(
            JointState, spec.joint_state, qos_profile_sensor_data
        )
        self.image_publisher = node.create_publisher(Image, spec.image, qos_profile_sensor_data)
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

    def _on_command(self, message: Float64MultiArray) -> None:
        self.commands.append(tuple(message.data))
        self.command_times.append(time.monotonic())

    def next_stamp_ns(self) -> int:
        stamp_ns = self._next_stamp_ns
        self._next_stamp_ns += 100_000_000
        return stamp_ns

    def publish_joint(
        self,
        positions: tuple[float, ...],
        *,
        stamp_ns: int | None = None,
    ) -> int:
        if stamp_ns is None:
            stamp_ns = self.next_stamp_ns()
        message = JointState()
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        message.name = list(_JOINT_NAMES)
        message.position = list(positions)
        self.joint_publisher.publish(message)
        return stamp_ns

    def publish_image(
        self,
        *,
        stamp_ns: int | None = None,
        encoding: str = "rgb8",
        frame_id: str = "m2_test_camera",
        truncated: bool = False,
    ) -> int:
        if stamp_ns is None:
            stamp_ns = self.next_stamp_ns()
        message = Image()
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        message.header.frame_id = frame_id
        message.height = 2
        message.width = 2
        message.encoding = encoding
        message.is_bigendian = 0
        message.step = 6
        pixels = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
        message.data = pixels[:-1] if truncated else pixels
        self.image_publisher.publish(message)
        return stamp_ns

    def publish_pair(
        self,
        positions: tuple[float, ...],
        *,
        image_offset_ns: int = 0,
        encoding: str = "rgb8",
    ) -> int:
        stamp_ns = self.next_stamp_ns()
        self.publish_joint(positions, stamp_ns=stamp_ns)
        self.publish_image(stamp_ns=stamp_ns + image_offset_ns, encoding=encoding)
        return stamp_ns


class TestPolicyServerM2(unittest.TestCase):
    """Exercise M2 synchronization, faults, sequence gating, and recovery."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node(f"policy_server_m2_test_{self._testMethodName}")
        self.endpoints = {key: _Endpoint(self.node, spec) for key, spec in _SPECS.items()}
        for endpoint in self.endpoints.values():
            self.assertTrue(
                endpoint.action_client.wait_for_server(timeout_sec=_DISCOVERY_TIMEOUT),
                f"{endpoint.spec.key} action server was not discovered",
            )
            self._spin_until(
                lambda endpoint=endpoint: (
                    self.node.count_subscribers(endpoint.spec.joint_state) >= 1
                    and self.node.count_subscribers(endpoint.spec.image) >= 1
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
        *,
        pump: Callable[[], None] | None = None,
    ) -> None:
        deadline = time.monotonic() + timeout
        next_pump = float("-inf")
        while not predicate():
            now = time.monotonic()
            remaining = deadline - now
            if remaining <= 0.0:
                self.fail(failure_message)
            if pump is not None and now >= next_pump:
                pump()
                next_pump = now + 0.04
            rclpy.spin_once(self.node, timeout_sec=min(0.02, remaining))

    def _spin_for(self, duration: float, *, pump: Callable[[], None] | None = None) -> None:
        deadline = time.monotonic() + duration
        next_pump = float("-inf")
        while True:
            now = time.monotonic()
            remaining = deadline - now
            if remaining <= 0.0:
                return
            if pump is not None and now >= next_pump:
                pump()
                next_pump = now + 0.04
            rclpy.spin_once(self.node, timeout_sec=min(0.02, remaining))

    def _wait_future(
        self,
        future: Any,
        timeout: float,
        description: str,
        *,
        pump: Callable[[], None] | None = None,
    ) -> Any:
        self._spin_until(
            future.done,
            timeout,
            f"timed out waiting for {description}",
            pump=pump,
        )
        return future.result()

    def _send_goal(
        self,
        endpoint: _Endpoint,
        *,
        max_steps: int = 20,
        timeout_seconds: float = 0.0,
    ) -> tuple[Any, list[int]]:
        feedback_steps: list[int] = []
        goal = ExecutePolicy.Goal()
        goal.instruction = "move to home"
        goal.max_steps = max_steps
        goal.timeout_seconds = timeout_seconds
        future = endpoint.action_client.send_goal_async(
            goal,
            feedback_callback=lambda message: feedback_steps.append(message.feedback.current_step),
        )
        handle = self._wait_future(future, _FUTURE_TIMEOUT, "goal acceptance")
        self.assertIsNotNone(handle)
        self.assertTrue(handle.accepted)
        return handle, feedback_steps

    def _get_result(
        self,
        handle: Any,
        *,
        pump: Callable[[], None] | None = None,
    ) -> Any:
        return self._wait_future(
            handle.get_result_async(),
            _RESULT_TIMEOUT,
            "action result",
            pump=pump,
        )

    def _cancel(self, handle: Any) -> Any:
        return self._wait_future(
            handle.cancel_goal_async(), _FUTURE_TIMEOUT, "cancel acknowledgement"
        )

    @staticmethod
    def _values(status: DiagnosticStatus) -> dict[str, str]:
        return {item.key: item.value for item in status.values}

    def _matching_statuses(self, endpoint: _Endpoint, name: str) -> list[DiagnosticStatus]:
        return [
            status
            for message in endpoint.diagnostics
            for status in message.status
            if status.name == name
        ]

    def _wait_for_status(
        self,
        endpoint: _Endpoint,
        name: str,
        predicate: Callable[[DiagnosticStatus], bool],
        description: str,
        *,
        timeout: float = 3.0,
    ) -> DiagnosticStatus:
        matching: list[DiagnosticStatus] = []

        def found() -> bool:
            nonlocal matching
            matching = [
                status for status in self._matching_statuses(endpoint, name) if predicate(status)
            ]
            return bool(matching)

        self._spin_until(
            found,
            timeout,
            f"{endpoint.spec.key} diagnostics did not report {description}",
        )
        return matching[-1]

    def _assert_aborted(self, wrapped: Any, reason: str) -> None:
        self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(wrapped.result.success)
        self.assertEqual(wrapped.result.termination_reason, reason)
        self.assertTrue(wrapped.result.episode_id)

    def _assert_hold_is_last(
        self, endpoint: _Endpoint, expected: tuple[float, ...], normal_count: int = 0
    ) -> None:
        self._spin_until(
            lambda: len(endpoint.commands) >= normal_count + 1,
            2.0,
            f"{endpoint.spec.key} did not publish a hold command",
        )
        self.assertEqual(endpoint.commands[-1], expected)
        command_count = len(endpoint.commands)
        self._spin_for(3.0 * _CONTROL_PERIOD)
        self.assertEqual(len(endpoint.commands), command_count)

    def _publish_initial_pair_for_active_goal(
        self,
        endpoint: _Endpoint,
        positions: tuple[float, ...],
        *,
        image_offset_ns: int = 0,
    ) -> None:
        """Publish once after the server has captured this goal's observation epoch."""

        self._wait_for_status(
            endpoint,
            "policy_bridge/runtime",
            lambda status: self._values(status).get("active_goal") == "true",
            "an active goal observation epoch",
        )
        endpoint.publish_pair(
            positions,
            image_offset_ns=image_offset_ns,
        )

    def test_normal_sequence_gate_cancel_race_and_diagnostics(self) -> None:
        endpoint = self.endpoints["normal"]
        endpoint.clear_recordings()

        handle, feedback = self._send_goal(endpoint, max_steps=3)
        self._publish_initial_pair_for_active_goal(endpoint, _START)
        self._spin_until(
            lambda: len(endpoint.commands) == 1,
            _FUTURE_TIMEOUT,
            "normal multimodal goal did not publish its first command",
        )
        self.assertEqual(endpoint.commands, [_HOME])
        initial_sync_status = self._wait_for_status(
            endpoint,
            "policy_bridge/synchronization",
            lambda status: int(self._values(status).get("snapshot_sequence_id", "0")) > 0,
            "the goal-local initial snapshot sequence",
        )
        initial_sequence = int(self._values(initial_sync_status)["snapshot_sequence_id"])

        # A single synchronized snapshot may drive at most one inference/command.
        self._spin_for(4.0 * _CONTROL_PERIOD)
        self.assertEqual(endpoint.commands, [_HOME])

        endpoint.publish_pair(_HALF)
        self._spin_until(
            lambda: len(endpoint.commands) == 2,
            _FUTURE_TIMEOUT,
            "a newer synchronized snapshot did not continue policy execution",
        )
        self.assertEqual(endpoint.commands, [_HOME, _HOME])
        self._spin_for(4.0 * _CONTROL_PERIOD)
        self.assertEqual(endpoint.commands, [_HOME, _HOME])

        endpoint.publish_pair(_HOME)
        wrapped = self._get_result(handle)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(wrapped.result.success)
        self.assertEqual(wrapped.result.termination_reason, "goal_reached")
        self._spin_until(
            lambda: len(feedback) == 2,
            2.0,
            "normal goal feedback did not arrive after the action result",
        )
        self.assertEqual(feedback, [1, 2])
        self.assertEqual(endpoint.commands, [_HOME, _HOME])

        image_status = self._wait_for_status(
            endpoint,
            "policy_bridge/image",
            lambda status: self._values(status).get("image_valid") == "true",
            "a valid image",
        )
        image_values = self._values(image_status)
        self.assertEqual(image_values["image_required"], "true")
        self.assertEqual(image_values["encoding"], "rgb8")
        self.assertEqual(image_values["width"], "2")
        self.assertEqual(image_values["height"], "2")
        self.assertEqual(image_values["frame_id"], "m2_test_camera")

        sync_status = self._wait_for_status(
            endpoint,
            "policy_bridge/synchronization",
            lambda status: (
                int(self._values(status).get("snapshot_sequence_id", "0")) >= initial_sequence + 2
            ),
            "strictly increasing synchronized snapshot sequence IDs",
        )
        sync_values = self._values(sync_status)
        self.assertEqual(sync_values["synchronized_snapshot_available"], "true")
        self.assertEqual(float(sync_values["last_sync_skew_ms"]), 0.0)
        self.assertEqual(float(sync_values["sync_slop_ms"]), 0.0)
        self.assertEqual(sync_values["sync_queue_size"], "4")
        policy_status = self._wait_for_status(
            endpoint,
            "policy_bridge/policy",
            lambda status: self._values(status).get("backend_name") == "multimodal_scripted",
            "the multimodal backend name",
        )
        self.assertEqual(policy_status.level, DiagnosticStatus.OK)

        # Cancel while the sequence gate is waiting.  A newer raw joint sample
        # changes the hold target but cannot trigger another policy command.
        endpoint.clear_recordings()
        cancel_handle, _ = self._send_goal(endpoint, max_steps=20)
        self._publish_initial_pair_for_active_goal(endpoint, _START)
        self._spin_until(
            lambda: endpoint.commands == [_HOME],
            _FUTURE_TIMEOUT,
            "cancel scenario did not reach the new-snapshot wait",
        )
        for _ in range(3):
            endpoint.publish_joint(_HALF)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        cancel_response = self._cancel(cancel_handle)
        self.assertEqual(len(cancel_response.goals_canceling), 1)
        canceled = self._get_result(cancel_handle)
        self.assertEqual(canceled.status, GoalStatus.STATUS_CANCELED)
        self.assertFalse(canceled.result.success)
        self.assertEqual(canceled.result.termination_reason, "goal_canceled")
        self._assert_hold_is_last(endpoint, _HALF, normal_count=1)

        # Race a goal-completing pair against cancellation.  Exactly one coherent
        # terminal state may win; a cancel winner issues one final hold.
        endpoint.clear_recordings()
        race_handle, _ = self._send_goal(endpoint, max_steps=3)
        self._publish_initial_pair_for_active_goal(endpoint, _START)
        self._spin_until(
            lambda: endpoint.commands == [_HOME],
            _FUTURE_TIMEOUT,
            "success/cancel race did not publish its first policy command",
        )
        endpoint.publish_pair(_HOME)
        race_cancel_future = race_handle.cancel_goal_async()
        race_result_future = race_handle.get_result_async()
        self._spin_until(
            lambda: race_cancel_future.done() and race_result_future.done(),
            _RESULT_TIMEOUT,
            "success/cancel race did not terminate",
        )
        race_cancel = race_cancel_future.result()
        race_result = race_result_future.result()
        if race_cancel.goals_canceling:
            self.assertEqual(race_result.status, GoalStatus.STATUS_CANCELED)
            self.assertFalse(race_result.result.success)
            self.assertEqual(race_result.result.termination_reason, "goal_canceled")
            self._assert_hold_is_last(endpoint, _HOME, normal_count=1)
        else:
            self.assertEqual(race_result.status, GoalStatus.STATUS_SUCCEEDED)
            self.assertTrue(race_result.result.success)
            self.assertEqual(race_result.result.termination_reason, "goal_reached")
            self.assertEqual(endpoint.commands, [_HOME])

    def test_missing_unsynchronized_and_invalid_image_faults(self) -> None:
        missing = self.endpoints["missing"]
        missing.clear_recordings()
        missing_handle, _ = self._send_goal(missing)
        missing_result = self._get_result(
            missing_handle, pump=lambda: missing.publish_joint(_START)
        )
        self._assert_aborted(missing_result, "image_timeout")
        self._assert_hold_is_last(missing, _START)
        self.assertEqual(missing.commands, [_START])
        missing_status = self._wait_for_status(
            missing,
            "policy_bridge/image",
            lambda status: status.message == "image_timeout",
            "image_timeout",
        )
        self.assertEqual(missing_status.level, DiagnosticStatus.ERROR)
        self.assertEqual(self._values(missing_status)["image_received"], "false")

        unsynchronized = self.endpoints["unsynchronized"]
        unsynchronized.clear_recordings()
        unsynchronized_handle, _ = self._send_goal(unsynchronized)

        def publish_skewed_inputs() -> None:
            # The two disjoint timestamp regions ensure ATS cannot cross-match
            # adjacent samples while both raw streams remain fresh and valid.
            joint_stamp = unsynchronized.next_stamp_ns()
            unsynchronized.publish_joint(_START, stamp_ns=joint_stamp)
            unsynchronized.publish_image(stamp_ns=joint_stamp + 5_000_000_000)

        unsynchronized_result = self._get_result(unsynchronized_handle, pump=publish_skewed_inputs)
        self._assert_aborted(unsynchronized_result, "observation_sync_timeout")
        self._assert_hold_is_last(unsynchronized, _START)
        self.assertEqual(unsynchronized.commands, [_START])
        sync_status = self._wait_for_status(
            unsynchronized,
            "policy_bridge/synchronization",
            lambda status: status.message == "observation_sync_timeout",
            "observation_sync_timeout",
        )
        self.assertEqual(sync_status.level, DiagnosticStatus.ERROR)
        sync_values = self._values(sync_status)
        self.assertEqual(sync_values["synchronized_snapshot_available"], "false")
        self.assertGreater(float(sync_values["last_sync_skew_ms"]), 10.0)
        self.assertEqual(sync_values["last_sync_error"], "timestamp_skew_exceeds_sync_slop")

        invalid = self.endpoints["invalid"]
        invalid.clear_recordings()
        invalid_handle, _ = self._send_goal(invalid)

        def publish_invalid_inputs() -> None:
            stamp_ns = invalid.next_stamp_ns()
            invalid.publish_joint(_START, stamp_ns=stamp_ns)
            invalid.publish_image(stamp_ns=stamp_ns, encoding="mono8")

        invalid_result = self._get_result(invalid_handle, pump=publish_invalid_inputs)
        self._assert_aborted(invalid_result, "invalid_image")
        self._assert_hold_is_last(invalid, _START)
        invalid_status = self._wait_for_status(
            invalid,
            "policy_bridge/image",
            lambda status: status.message == "invalid_image",
            "invalid_image",
        )
        invalid_values = self._values(invalid_status)
        self.assertEqual(invalid_status.level, DiagnosticStatus.ERROR)
        self.assertEqual(invalid_values["image_received"], "true")
        self.assertEqual(invalid_values["image_valid"], "false")
        self.assertIn("encoding", invalid_values["last_image_error"].lower())
        self.assertEqual(invalid.commands, [_START])

    def test_multimodal_timeout_races_are_first_wins(self) -> None:
        image_goal = self.endpoints["image_goal_race"]
        image_goal.clear_recordings()
        # The image arrives approximately one diagnostics period after the goal
        # begins, putting its 0.40 s expiry near the 0.45 s goal deadline.
        # Scheduling may let either boundary linearize first.
        image_goal_handle, _ = self._send_goal(image_goal, max_steps=100, timeout_seconds=0.45)
        self._publish_initial_pair_for_active_goal(image_goal, _START, image_offset_ns=5_000_000)
        self._spin_until(
            lambda: image_goal.commands == [_HOME],
            _FUTURE_TIMEOUT,
            "image/goal timeout race did not publish its policy command",
        )
        image_goal_result = self._get_result(
            image_goal_handle, pump=lambda: image_goal.publish_joint(_HALF)
        )
        self.assertEqual(image_goal_result.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(image_goal_result.result.success)
        self.assertIn(
            image_goal_result.result.termination_reason,
            {"stale_image", "goal_timeout"},
        )
        self.assertTrue(image_goal_result.result.episode_id)
        self._assert_hold_is_last(image_goal, _HALF, normal_count=1)
        self.assertEqual(image_goal.commands, [_HOME, _HALF])
        image_goal_runtime = self._wait_for_status(
            image_goal,
            "policy_bridge/runtime",
            lambda status: (
                self._values(status).get("last_termination_reason")
                == image_goal_result.result.termination_reason
            ),
            "the coherent image/goal race result",
        )
        self.assertEqual(self._values(image_goal_runtime)["safe_stop_count"], "1")

        sync_joint = self.endpoints["sync_joint_race"]
        sync_joint.clear_recordings()
        sync_joint_handle, _ = self._send_goal(sync_joint, max_steps=100)
        self._publish_initial_pair_for_active_goal(sync_joint, _START, image_offset_ns=5_000_000)
        self._spin_until(
            lambda: sync_joint.commands == [_HOME],
            _FUTURE_TIMEOUT,
            "sync/joint-stale race did not publish its policy command",
        )

        # Start joint freshness near the snapshot-wait boundary, then keep only
        # valid images fresh in a timestamp region that cannot match the joint.
        joint_stamp_ns = sync_joint.next_stamp_ns()
        sync_joint.publish_joint(_HALF, stamp_ns=joint_stamp_ns)
        sync_joint.publish_image(stamp_ns=joint_stamp_ns + 5_000_000_000)
        self._spin_for(0.02)

        def publish_unmatched_image() -> None:
            sync_joint.publish_image(stamp_ns=sync_joint.next_stamp_ns() + 5_000_000_000)

        sync_joint_result = self._get_result(sync_joint_handle, pump=publish_unmatched_image)
        self.assertEqual(sync_joint_result.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(sync_joint_result.result.success)
        self.assertIn(
            sync_joint_result.result.termination_reason,
            {"observation_sync_timeout", "stale_observation"},
        )
        self.assertTrue(sync_joint_result.result.episode_id)
        self._assert_hold_is_last(sync_joint, _HALF, normal_count=1)
        self.assertEqual(sync_joint.commands, [_HOME, _HALF])
        sync_joint_runtime = self._wait_for_status(
            sync_joint,
            "policy_bridge/runtime",
            lambda status: (
                self._values(status).get("last_termination_reason")
                == sync_joint_result.result.termination_reason
            ),
            "the coherent synchronization/joint-stale race result",
        )
        self.assertEqual(self._values(sync_joint_runtime)["safe_stop_count"], "1")

    def test_stale_image_holds_latest_joint_and_recovers(self) -> None:
        endpoint = self.endpoints["stale"]
        endpoint.clear_recordings()
        handle, _ = self._send_goal(endpoint, max_steps=20)
        self._publish_initial_pair_for_active_goal(endpoint, _START, image_offset_ns=5_000_000)
        self._spin_until(
            lambda: endpoint.commands == [_HOME],
            _FUTURE_TIMEOUT,
            "stale-image scenario did not publish its policy command",
        )
        result = self._get_result(handle, pump=lambda: endpoint.publish_joint(_HALF))
        self._assert_aborted(result, "stale_image")
        self._assert_hold_is_last(endpoint, _HALF, normal_count=1)
        stale_status = self._wait_for_status(
            endpoint,
            "policy_bridge/image",
            lambda status: status.message == "stale_image",
            "stale_image",
        )
        self.assertEqual(stale_status.level, DiagnosticStatus.ERROR)

        # A new synchronized pair and a new goal must recover normally after the
        # image fault; this also proves no prior termination leaks across UUIDs.
        endpoint.clear_recordings()
        recovery_handle, recovery_feedback = self._send_goal(endpoint, max_steps=3)
        self._publish_initial_pair_for_active_goal(endpoint, _PARTIAL, image_offset_ns=5_000_000)
        self._spin_until(
            lambda: endpoint.commands == [_HOME],
            _FUTURE_TIMEOUT,
            "recovery goal did not publish its first policy command",
        )
        endpoint.publish_pair(_HOME, image_offset_ns=5_000_000)
        recovered = self._get_result(recovery_handle)
        self.assertEqual(recovered.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(recovered.result.success)
        self.assertEqual(recovered.result.termination_reason, "goal_reached")
        self.assertNotEqual(recovered.result.episode_id, result.result.episode_id)
        self.assertEqual(recovery_feedback, [1])
        self.assertEqual(endpoint.commands, [_HOME])

        approximate_sync = self._wait_for_status(
            endpoint,
            "policy_bridge/synchronization",
            lambda status: (
                status.level == DiagnosticStatus.OK
                and status.message == "synchronized"
                and float(self._values(status).get("last_sync_skew_ms", "nan")) == 5.0
            ),
            "a successful nonzero-slop approximate synchronization",
        )
        approximate_values = self._values(approximate_sync)
        self.assertEqual(float(approximate_values["sync_slop_ms"]), 20.0)

        for component in (
            "policy_bridge/runtime",
            "policy_bridge/observation",
            "policy_bridge/policy",
            "policy_bridge/image",
            "policy_bridge/synchronization",
        ):
            status = self._wait_for_status(
                endpoint,
                component,
                lambda candidate: candidate.level == DiagnosticStatus.OK,
                f"recovered OK state for {component}",
            )
            self.assertEqual(status.level, DiagnosticStatus.OK)
