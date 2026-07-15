"""ROS 2 action server that executes the deterministic M0 policy."""

from __future__ import annotations

import math
import threading
import time
from typing import Final

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from policy_bridge_interfaces.action import ExecutePolicy

from .action_validation import JOINT_COUNT, validate_action
from .scripted_policy import ScriptedPolicy, UnsupportedInstructionError

DEFAULT_JOINT_NAMES: Final[tuple[str, ...]] = tuple(
    f"joint_{index}" for index in range(1, JOINT_COUNT + 1)
)


class PolicyActionServer(Node):
    """Expose ``ScriptedPolicy`` through the ``ExecutePolicy`` action."""

    def __init__(self) -> None:
        """Create subscriptions, command publisher, and action server."""

        super().__init__("policy_server")

        self.declare_parameter("action_name", "execute_policy")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_command_topic", "/joint_command")
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("goal_tolerance", 0.005)
        self.declare_parameter("joint_names", list(DEFAULT_JOINT_NAMES))

        self._action_name = str(self.get_parameter("action_name").value)
        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._joint_command_topic = str(self.get_parameter("joint_command_topic").value)
        self._control_rate_hz = _positive_finite_parameter(
            "control_rate_hz", self.get_parameter("control_rate_hz").value
        )
        self._goal_tolerance = _nonnegative_finite_parameter(
            "goal_tolerance", self.get_parameter("goal_tolerance").value
        )
        self._joint_names = _validate_joint_names(self.get_parameter("joint_names").value)

        if not self._action_name:
            raise ValueError("action_name must not be empty")
        if not self._joint_state_topic or not self._joint_command_topic:
            raise ValueError("joint-state and joint-command topics must not be empty")

        self._policy = ScriptedPolicy()
        self._latest_joint_positions: np.ndarray | None = None
        self._joint_state_lock = threading.Lock()
        self._active_goal_lock = threading.Lock()
        self._goal_is_active = False
        self._command_gate_lock = threading.Lock()
        self._canceling_goal_ids: set[bytes] = set()
        self._shutdown_requested = threading.Event()
        self._execution_wake_event = threading.Event()
        self._last_joint_state_warning_time = float("-inf")

        self._callback_group = ReentrantCallbackGroup()
        self._command_publisher = self.create_publisher(
            Float64MultiArray, self._joint_command_topic, 10
        )
        self._joint_state_subscription = self.create_subscription(
            JointState,
            self._joint_state_topic,
            self._joint_state_callback,
            10,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            ExecutePolicy,
            self._action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            f"ExecutePolicy action server ready on '{self._action_name}'; "
            f"waiting for valid joint state on '{self._joint_state_topic}'"
        )

    def destroy_node(self) -> None:
        """Destroy the action server before destroying the ROS node."""

        self._action_server.destroy()
        super().destroy_node()

    def request_stop(self) -> None:
        """Ask active execution callbacks to finish before executor shutdown."""

        self._shutdown_requested.set()
        self._execution_wake_event.set()

    def _goal_callback(self, _goal_request: ExecutePolicy.Goal) -> GoalResponse:
        with self._active_goal_lock:
            if self._goal_is_active:
                self.get_logger().warning(
                    "Rejecting ExecutePolicy goal because another goal is active"
                )
                return GoalResponse.REJECT
            self._goal_is_active = True
            self._execution_wake_event.clear()

        self.get_logger().info("Accepted ExecutePolicy goal")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        with self._command_gate_lock:
            if not goal_handle.is_active:
                return CancelResponse.REJECT
            self._canceling_goal_ids.add(_goal_id(goal_handle))
            self._execution_wake_event.set()
        self.get_logger().info("Accepted ExecutePolicy cancellation request")
        return CancelResponse.ACCEPT

    def _joint_state_callback(self, message: JointState) -> None:
        try:
            positions = self._positions_in_configured_order(message)
            validated = validate_action(positions)
        except (TypeError, ValueError) as exc:
            self._warn_about_joint_state(str(exc))
            return

        with self._joint_state_lock:
            self._latest_joint_positions = validated

    def _positions_in_configured_order(self, message: JointState) -> list[float]:
        if not message.name:
            return list(message.position)
        if len(message.name) != len(message.position):
            raise ValueError("joint state name and position lengths differ")
        if len(set(message.name)) != len(message.name):
            raise ValueError("joint state contains duplicate joint names")

        position_by_name = dict(zip(message.name, message.position, strict=True))
        missing_names = [name for name in self._joint_names if name not in position_by_name]
        if missing_names:
            raise ValueError("joint state does not contain all configured joint names")
        return [position_by_name[name] for name in self._joint_names]

    def _warn_about_joint_state(self, reason: str) -> None:
        now = time.monotonic()
        if now - self._last_joint_state_warning_time >= 5.0:
            self.get_logger().warning(f"Ignoring invalid joint state: {reason}")
            self._last_joint_state_warning_time = now

    def _latest_positions(self) -> np.ndarray | None:
        with self._joint_state_lock:
            if self._latest_joint_positions is None:
                return None
            return self._latest_joint_positions.copy()

    def _wait_for_control_period(self, control_period: float) -> None:
        self._execution_wake_event.wait(control_period)
        self._execution_wake_event.clear()

    def _episode_id(self, goal_handle: ServerGoalHandle) -> str:
        return f"episode-{_goal_id(goal_handle).hex()}"

    def _release_active_goal(self) -> None:
        with self._active_goal_lock:
            self._goal_is_active = False

    def _is_cancel_requested(self, goal_handle: ServerGoalHandle) -> bool:
        goal_id = _goal_id(goal_handle)
        with self._command_gate_lock:
            return goal_id in self._canceling_goal_ids or goal_handle.is_cancel_requested

    def _publish_command(
        self,
        goal_handle: ServerGoalHandle,
        target: np.ndarray,
    ) -> bool:
        goal_id = _goal_id(goal_handle)
        with self._command_gate_lock:
            if (
                self._shutdown_requested.is_set()
                or goal_id in self._canceling_goal_ids
                or goal_handle.is_cancel_requested
            ):
                return False
            command = Float64MultiArray()
            command.data = target.tolist()
            self._command_publisher.publish(command)
            return True

    def _canceled_result(
        self,
        goal_handle: ServerGoalHandle,
        episode_id: str,
    ) -> ExecutePolicy.Result:
        self._mark_canceled_when_ready(goal_handle)
        result = ExecutePolicy.Result()
        result.success = False
        result.termination_reason = "goal_canceled"
        result.episode_id = episode_id
        self.get_logger().info(f"Episode {episode_id} canceled")
        return result

    def _aborted_result(
        self,
        goal_handle: ServerGoalHandle,
        episode_id: str,
        reason: str,
    ) -> ExecutePolicy.Result:
        canceled = self._abort_unless_canceling(goal_handle)
        result = ExecutePolicy.Result()
        result.success = False
        result.termination_reason = "goal_canceled" if canceled else reason
        result.episode_id = episode_id
        if canceled:
            self.get_logger().info(f"Episode {episode_id} canceled")
        else:
            self.get_logger().warning(f"Episode {episode_id} aborted: {reason}")
        return result

    def _succeeded_result(
        self,
        goal_handle: ServerGoalHandle,
        episode_id: str,
        step: int,
    ) -> ExecutePolicy.Result:
        canceled = self._succeed_unless_canceling(goal_handle)

        result = ExecutePolicy.Result()
        result.success = not canceled
        result.termination_reason = "goal_canceled" if canceled else "goal_reached"
        result.episode_id = episode_id
        if canceled:
            self.get_logger().info(f"Episode {episode_id} canceled")
        else:
            self.get_logger().info(f"Episode {episode_id} succeeded in {step} steps")
        return result

    def _mark_canceled_when_ready(self, goal_handle: ServerGoalHandle) -> None:
        goal_id = _goal_id(goal_handle)
        while True:
            with self._command_gate_lock:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return
                callback_seen = goal_id in self._canceling_goal_ids
            if not callback_seen:
                raise RuntimeError("cannot cancel a goal without a cancellation request")
            # ActionServer changes the state to CANCELING just after the
            # user cancel callback returns.  Do not call canceled() early.
            time.sleep(0.001)

    def _abort_unless_canceling(self, goal_handle: ServerGoalHandle) -> bool:
        goal_id = _goal_id(goal_handle)
        while True:
            with self._command_gate_lock:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return True
                if goal_id not in self._canceling_goal_ids:
                    goal_handle.abort()
                    return False
            time.sleep(0.001)

    def _succeed_unless_canceling(self, goal_handle: ServerGoalHandle) -> bool:
        goal_id = _goal_id(goal_handle)
        while True:
            with self._command_gate_lock:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return True
                if goal_id not in self._canceling_goal_ids:
                    goal_handle.succeed()
                    return False
            time.sleep(0.001)

    async def _execute_callback(self, goal_handle: ServerGoalHandle) -> ExecutePolicy.Result:
        episode_id = self._episode_id(goal_handle)
        request = goal_handle.request
        control_period = 1.0 / self._control_rate_hz

        try:
            self.get_logger().info(
                f"Starting episode {episode_id} for instruction {request.instruction!r}"
            )

            if request.max_steps <= 0:
                return self._aborted_result(goal_handle, episode_id, "invalid_max_steps")

            positions = self._latest_positions()
            waiting_logged = False
            while positions is None:
                if self._shutdown_requested.is_set():
                    return self._aborted_result(goal_handle, episode_id, "server_shutting_down")
                if self._is_cancel_requested(goal_handle):
                    return self._canceled_result(goal_handle, episode_id)
                if not waiting_logged:
                    self.get_logger().info(f"Episode {episode_id} waiting for valid joint state")
                    waiting_logged = True
                self._wait_for_control_period(control_period)
                positions = self._latest_positions()

            self._policy.reset()
            initial_distance: float | None = None

            for step in range(1, request.max_steps + 1):
                if self._shutdown_requested.is_set():
                    return self._aborted_result(goal_handle, episode_id, "server_shutting_down")
                if self._is_cancel_requested(goal_handle):
                    return self._canceled_result(goal_handle, episode_id)

                positions = self._latest_positions()
                if positions is None:
                    self._wait_for_control_period(control_period)
                    continue

                inference_started = time.perf_counter()
                try:
                    policy_action = self._policy.predict(positions, request.instruction)
                except UnsupportedInstructionError as exc:
                    self.get_logger().warning(str(exc))
                    return self._aborted_result(goal_handle, episode_id, "unsupported_instruction")
                except (TypeError, ValueError) as exc:
                    self.get_logger().error(f"Policy input failure: {exc}")
                    return self._aborted_result(goal_handle, episode_id, "policy_input_invalid")
                inference_latency_ms = (time.perf_counter() - inference_started) * 1000.0

                try:
                    target = validate_action(policy_action)
                except (TypeError, ValueError) as exc:
                    self.get_logger().error(f"Policy action validation failed: {exc}")
                    return self._aborted_result(goal_handle, episode_id, "invalid_policy_action")

                distance = float(np.max(np.abs(target - positions)))
                if initial_distance is None:
                    initial_distance = distance
                if initial_distance <= self._goal_tolerance:
                    progress = 1.0
                else:
                    progress = float(np.clip(1.0 - distance / initial_distance, 0.0, 1.0))

                # The gate makes cancellation observation and publication atomic.
                if not self._publish_command(goal_handle, target):
                    if self._shutdown_requested.is_set():
                        return self._aborted_result(goal_handle, episode_id, "server_shutting_down")
                    return self._canceled_result(goal_handle, episode_id)

                if distance > self._goal_tolerance:
                    self._wait_for_control_period(control_period)
                    if self._shutdown_requested.is_set():
                        return self._aborted_result(goal_handle, episode_id, "server_shutting_down")
                    if self._is_cancel_requested(goal_handle):
                        return self._canceled_result(goal_handle, episode_id)

                    updated_positions = self._latest_positions()
                    if updated_positions is not None:
                        distance = float(np.max(np.abs(target - updated_positions)))
                    if initial_distance <= self._goal_tolerance:
                        progress = 1.0
                    else:
                        progress = float(np.clip(1.0 - distance / initial_distance, 0.0, 1.0))

                feedback = ExecutePolicy.Feedback()
                feedback.current_step = step
                feedback.progress = progress
                feedback.inference_latency_ms = float(inference_latency_ms)
                goal_handle.publish_feedback(feedback)

                if distance <= self._goal_tolerance:
                    return self._succeeded_result(goal_handle, episode_id, step)

            return self._aborted_result(goal_handle, episode_id, "max_steps_exceeded")
        finally:
            with self._command_gate_lock:
                self._canceling_goal_ids.discard(_goal_id(goal_handle))
            self._release_active_goal()


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


def _goal_id(goal_handle: ServerGoalHandle) -> bytes:
    return bytes(goal_handle.goal_id.uuid)


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
    """Run the policy action server with concurrent ROS callbacks."""

    rclpy.init(args=args)
    node = PolicyActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.request_stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
