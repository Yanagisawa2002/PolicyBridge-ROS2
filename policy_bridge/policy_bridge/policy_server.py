"""ROS 2 action server with deterministic M1 runtime fault handling."""

from __future__ import annotations

import math
import threading
import time
import traceback
from concurrent.futures import CancelledError, Future, ThreadPoolExecutor
from dataclasses import dataclass
from typing import Final

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from policy_bridge_interfaces.action import ExecutePolicy

from .action_validation import JOINT_COUNT, validate_action
from .policy_backend import PolicyBackend, create_policy_backend
from .runtime_state import (
    GoalAdmission,
    RuntimeSnapshot,
    RuntimeState,
    RuntimeStateMachine,
    TerminationDecision,
    TerminationStatus,
)
from .scripted_policy import UnsupportedInstructionError

DEFAULT_JOINT_NAMES: Final[tuple[str, ...]] = tuple(
    f"joint_{index}" for index in range(1, JOINT_COUNT + 1)
)

DIAGNOSTICS_TOPIC: Final[str] = "/policy_bridge/diagnostics"
_POLL_INTERVAL_SECONDS: Final[float] = 0.02
_OBSERVATION_WARNING_FRACTION: Final[float] = 0.8
_POLICY_FAULT_REASONS: Final[frozenset[str]] = frozenset(
    {"policy_inference_timeout", "invalid_action", "policy_error"}
)
_OBSERVATION_FAULT_REASONS: Final[frozenset[str]] = frozenset(
    {"observation_timeout", "stale_observation"}
)


@dataclass(frozen=True, slots=True)
class _PolicyCall:
    """One bounded worker submission owned by a runtime inference token."""

    future: Future[object]
    token: int
    started_at: float
    completion: _PolicyCallCompletion


class _PolicyCallCompletion:
    """Thread-safe actual completion timestamp set inside the worker callable."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._completed_at: float | None = None

    def mark(self) -> None:
        with self._lock:
            if self._completed_at is None:
                self._completed_at = time.monotonic()

    def read(self) -> float | None:
        with self._lock:
            return self._completed_at


class PolicyActionServer(Node):
    """Execute a synchronous policy through a bounded, fault-aware ROS runtime."""

    def __init__(self) -> None:
        """Validate parameters and create the M1 action, I/O, worker, and diagnostics."""

        super().__init__("policy_server")

        self.declare_parameter("action_name", "execute_policy")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_command_topic", "/joint_command")
        self.declare_parameter("policy_backend", "scripted")
        self.declare_parameter("inference_timeout_seconds", 1.0)
        self.declare_parameter("joint_state_timeout_seconds", 1.0)
        self.declare_parameter("diagnostics_rate_hz", 1.0)
        self.declare_parameter("delayed_policy_delay_seconds", 2.0)
        self.declare_parameter("delayed_policy_first_call_only", True)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("goal_tolerance", 0.005)
        self.declare_parameter("joint_names", list(DEFAULT_JOINT_NAMES))

        self._action_name = str(self.get_parameter("action_name").value)
        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._joint_command_topic = str(self.get_parameter("joint_command_topic").value)
        backend_selector = str(self.get_parameter("policy_backend").value)
        inference_timeout_seconds = _positive_finite_parameter(
            "inference_timeout_seconds",
            self.get_parameter("inference_timeout_seconds").value,
        )
        joint_state_timeout_seconds = _positive_finite_parameter(
            "joint_state_timeout_seconds",
            self.get_parameter("joint_state_timeout_seconds").value,
        )
        diagnostics_rate_hz = _positive_finite_parameter(
            "diagnostics_rate_hz",
            self.get_parameter("diagnostics_rate_hz").value,
        )
        delayed_policy_delay_seconds = _nonnegative_finite_parameter(
            "delayed_policy_delay_seconds",
            self.get_parameter("delayed_policy_delay_seconds").value,
        )
        delayed_policy_first_call_only = _boolean_parameter(
            "delayed_policy_first_call_only",
            self.get_parameter("delayed_policy_first_call_only").value,
        )
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

        self._policy: PolicyBackend = create_policy_backend(
            backend_selector,
            delayed_policy_delay_seconds=delayed_policy_delay_seconds,
            delayed_policy_first_call_only=delayed_policy_first_call_only,
        )
        backend_name = str(getattr(self._policy, "backend_name", backend_selector))
        self._runtime = RuntimeStateMachine(
            backend_name=backend_name,
            inference_timeout_seconds=inference_timeout_seconds,
            joint_state_timeout_seconds=joint_state_timeout_seconds,
        )

        self._latest_joint_positions: np.ndarray | None = None
        self._joint_state_lock = threading.Lock()
        self._command_gate_lock = threading.RLock()
        self._finalization_lock = threading.RLock()
        self._finalized_episode_id = ""
        self._finalized_result: ExecutePolicy.Result | None = None
        self._diagnostic_event_lock = threading.Lock()
        self._diagnostic_event_level = DiagnosticStatus.OK
        self._diagnostic_event_message = "idle"
        self._shutdown_requested = threading.Event()
        self._execution_wake_event = threading.Event()
        self._last_joint_state_warning_time = float("-inf")

        self._policy_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="policy_bridge_worker",
        )

        self._callback_group = ReentrantCallbackGroup()
        self._command_publisher = self.create_publisher(
            Float64MultiArray, self._joint_command_topic, 10
        )
        self._diagnostics_publisher = self.create_publisher(DiagnosticArray, DIAGNOSTICS_TOPIC, 10)
        self._joint_state_subscription = self.create_subscription(
            JointState,
            self._joint_state_topic,
            self._joint_state_callback,
            10,
            callback_group=self._callback_group,
        )
        self._diagnostics_timer = self.create_timer(
            1.0 / diagnostics_rate_hz,
            self._publish_diagnostics,
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
            f"ExecutePolicy action server ready on '{self._action_name}' with "
            f"backend '{backend_name}'; waiting for valid joint state on "
            f"'{self._joint_state_topic}'"
        )
        self._publish_diagnostics()

    def destroy_node(self) -> None:
        """Stop new worker submissions and release ROS resources without waiting forever."""

        self.request_stop()
        self._diagnostics_timer.cancel()
        self._action_server.destroy()
        self._policy_executor.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()

    def request_stop(self) -> None:
        """Wake active execution so ROS shutdown does not wait for a control period."""

        self._shutdown_requested.set()
        self._execution_wake_event.set()

    def _goal_callback(self, goal_request: ExecutePolicy.Goal) -> GoalResponse:
        try:
            timeout_seconds = _nonnegative_finite_parameter(
                "timeout_seconds", goal_request.timeout_seconds
            )
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Rejecting ExecutePolicy goal: {exc}")
            return GoalResponse.REJECT

        if goal_request.max_steps <= 0:
            self.get_logger().warning(
                "Rejecting ExecutePolicy goal: max_steps must be greater than zero"
            )
            return GoalResponse.REJECT

        admission = self._runtime.reserve_goal()
        if admission is not GoalAdmission.ACCEPTED:
            self.get_logger().warning(
                "Rejecting ExecutePolicy goal because "
                + (
                    "another goal is active"
                    if admission is GoalAdmission.REJECTED_ACTIVE_GOAL
                    else "the policy backend is still busy"
                )
            )
            return GoalResponse.REJECT

        del timeout_seconds  # Validation happens here; the deadline starts in execute_callback.
        with self._finalization_lock:
            self._finalized_episode_id = ""
            self._finalized_result = None
        self._runtime.record_policy_error("")
        self._set_diagnostic_event(DiagnosticStatus.OK, "goal_accepted")
        try:
            self._publish_diagnostics()
        except BaseException as exc:
            # Admission has already reserved the single-goal slot.  Keep the
            # action callback total so a diagnostics transport failure cannot
            # strand an unbound reservation.
            self.get_logger().error(
                f"Goal-admission diagnostics failed ({type(exc).__name__}): {exc}"
            )
        self.get_logger().info("Accepted ExecutePolicy goal")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        episode_id = self._episode_id(goal_handle)
        with self._command_gate_lock:
            if not goal_handle.is_active:
                return CancelResponse.REJECT
            if not self._runtime.start_reserved_goal(
                episode_id,
                timeout_seconds=float(goal_handle.request.timeout_seconds),
            ):
                return CancelResponse.REJECT
            if self._runtime.termination_decision(episode_id) is not None:
                return CancelResponse.REJECT
            decision = self._runtime.claim_termination(
                episode_id,
                status=TerminationStatus.CANCELED,
                reason="goal_canceled",
                issue_hold=True,
            )
            if decision is None:
                return CancelResponse.REJECT
            self._execution_wake_event.set()

        self._set_diagnostic_event(DiagnosticStatus.WARN, decision.reason)
        try:
            self._publish_diagnostics()
        except BaseException as exc:
            # The cancellation decision is already linearized.  Diagnostics
            # transport must not prevent this callback from returning ACCEPT
            # and allowing rclpy to enter the CANCELING state.
            self.get_logger().error(
                f"Cancellation diagnostics failed ({type(exc).__name__}): {exc}"
            )
        self.get_logger().info("Accepted ExecutePolicy cancellation request")
        return CancelResponse.ACCEPT

    def _joint_state_callback(self, message: JointState) -> None:
        prior_snapshot = self._runtime.snapshot()
        try:
            positions = self._positions_in_configured_order(message)
            validated = validate_action(positions)
        except (TypeError, ValueError) as exc:
            self._runtime.record_observation(valid=False)
            self._warn_about_joint_state(str(exc))
            if prior_snapshot.joint_state_valid:
                self._publish_diagnostics()
            return

        # The command gate is the linearization point shared with conditional
        # freshness faults, cancellation, normal command publication, and
        # hold-position.  Whichever side acquires it first commits completely.
        with self._command_gate_lock, self._joint_state_lock:
            self._runtime.record_observation(valid=True)
            self._latest_joint_positions = validated
        if not prior_snapshot.joint_state_received or not prior_snapshot.joint_state_valid:
            self._publish_diagnostics()

    def _positions_in_configured_order(self, message: JointState) -> list[float]:
        if len(message.position) != JOINT_COUNT:
            raise ValueError(f"joint state must contain exactly {JOINT_COUNT} positions")
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

    def _episode_id(self, goal_handle: ServerGoalHandle) -> str:
        return f"episode-{_goal_id(goal_handle).hex()}"

    def _claim_termination(
        self,
        episode_id: str,
        *,
        status: TerminationStatus,
        reason: str,
        issue_hold: bool,
    ) -> TerminationDecision | None:
        """Claim a first-wins outcome while excluding normal command publication."""

        with self._command_gate_lock:
            existing = self._runtime.termination_decision(episode_id)
            if existing is not None:
                return existing
            decision = self._runtime.claim_termination(
                episode_id,
                status=status,
                reason=reason,
                issue_hold=issue_hold,
            )
            return decision or self._runtime.termination_decision(episode_id)

    def _check_active_faults(
        self,
        episode_id: str,
        *,
        motion_started: bool,
        check_stale: bool,
    ) -> TerminationDecision | None:
        existing = self._runtime.termination_decision(episode_id)
        if existing is not None:
            return existing
        if self._shutdown_requested.is_set():
            return self._claim_termination(
                episode_id,
                status=TerminationStatus.ABORTED,
                reason="server_shutting_down",
                issue_hold=motion_started,
            )
        if self._runtime.goal_timed_out(episode_id):
            return self._claim_termination(
                episode_id,
                status=TerminationStatus.ABORTED,
                reason="goal_timeout",
                issue_hold=True,
            )
        if check_stale:
            with self._command_gate_lock:
                existing = self._runtime.termination_decision(episode_id)
                if existing is not None:
                    return existing
                decision = self._runtime.claim_stale_observation(episode_id)
                if decision is not None:
                    return decision
        return None

    def _wait_for_initial_observation(
        self,
        episode_id: str,
    ) -> tuple[np.ndarray | None, TerminationDecision | None]:
        """Wait for the first valid local observation with bounded monotonic polling."""

        self._runtime.mark_waiting_for_observation(episode_id)
        waiting_started = time.monotonic()
        while True:
            decision = self._check_active_faults(
                episode_id,
                motion_started=False,
                check_stale=True,
            )
            if decision is not None:
                return None, decision

            positions = self._latest_positions()
            if positions is not None:
                self._runtime.mark_running(episode_id)
                self._publish_diagnostics()
                return positions, None

            observation_elapsed = time.monotonic() - waiting_started
            if observation_elapsed >= self._runtime.joint_state_timeout_seconds:
                with self._command_gate_lock:
                    decision = self._runtime.termination_decision(episode_id)
                    if decision is None:
                        decision = self._runtime.claim_observation_timeout(episode_id)
                if decision is not None:
                    return None, decision
                # A valid callback won the runtime lock at the boundary.  Loop
                # once more and read its matching positions under joint lock.
                continue

            wait_seconds = min(
                _POLL_INTERVAL_SECONDS,
                self._runtime.joint_state_timeout_seconds - observation_elapsed,
            )
            remaining_goal = self._runtime.remaining_goal_seconds(episode_id)
            if remaining_goal is not None:
                wait_seconds = min(wait_seconds, remaining_goal)
            self._execution_wake_event.wait(max(0.0, wait_seconds))
            self._execution_wake_event.clear()

    def _submit_policy_call(
        self,
        episode_id: str,
        positions: np.ndarray,
        instruction: str,
    ) -> _PolicyCall:
        token = self._runtime.begin_inference(episode_id)
        if token is None:
            raise RuntimeError("policy backend slot was unavailable for an active goal")

        started_at = time.monotonic()
        completion = _PolicyCallCompletion()
        try:
            future = self._policy_executor.submit(
                self._invoke_policy,
                completion,
                positions.copy(),
                instruction,
            )
        except BaseException:
            self._runtime.finish_inference(token, error_type="worker_submit_error")
            raise

        call = _PolicyCall(
            future=future,
            token=token,
            started_at=started_at,
            completion=completion,
        )
        future.add_done_callback(lambda completed: self._policy_call_completed(call, completed))
        return call

    def _invoke_policy(
        self,
        completion: _PolicyCallCompletion,
        positions: np.ndarray,
        instruction: str,
    ) -> object:
        try:
            return self._policy.predict(positions, instruction)
        finally:
            completion.mark()

    def _policy_call_completed(self, call: _PolicyCall, future: Future[object]) -> None:
        completed_at = call.completion.read()
        if completed_at is None:
            completed_at = time.monotonic()
        latency_ms = max(0.0, completed_at - call.started_at) * 1000.0
        error_type: str | None = None
        try:
            exception = future.exception()
        except CancelledError:
            error_type = "CancelledError"
        else:
            if exception is not None:
                error_type = type(exception).__name__
        finished = self._runtime.finish_inference(
            call.token,
            latency_ms=latency_ms,
            error_type=error_type,
        )
        if not finished:
            return
        snapshot = self._runtime.snapshot()
        if not snapshot.active_goal and snapshot.runtime_state is RuntimeState.IDLE:
            self._set_diagnostic_event(DiagnosticStatus.OK, "idle")
        self._execution_wake_event.set()
        if not self._shutdown_requested.is_set():
            self._publish_diagnostics()

    def _wait_for_policy_call(
        self,
        episode_id: str,
        call: _PolicyCall,
        *,
        motion_started: bool,
    ) -> TerminationDecision | None:
        inference_deadline = call.started_at + self._runtime.inference_timeout_seconds
        while True:
            decision = self._check_active_faults(
                episode_id,
                motion_started=motion_started,
                check_stale=True,
            )
            if decision is not None:
                return decision

            now = time.monotonic()
            completed_at = call.completion.read()
            if completed_at is not None:
                self._policy_call_completed(call, call.future)
                if completed_at > inference_deadline:
                    self._runtime.record_policy_error("policy_inference_timeout")
                    return self._claim_termination(
                        episode_id,
                        status=TerminationStatus.ABORTED,
                        reason="policy_inference_timeout",
                        issue_hold=True,
                    )
                return self._check_active_faults(
                    episode_id,
                    motion_started=motion_started,
                    check_stale=True,
                )
            if now >= inference_deadline:
                self._runtime.record_policy_error("policy_inference_timeout")
                return self._claim_termination(
                    episode_id,
                    status=TerminationStatus.ABORTED,
                    reason="policy_inference_timeout",
                    issue_hold=True,
                )

            if call.future.done():
                # Future waiters may observe completion just before the worker
                # thread has run its callback.  Finish the generation token
                # idempotently here so the next control step never sees a
                # spurious busy backend.
                self._policy_call_completed(call, call.future)
                return self._check_active_faults(
                    episode_id,
                    motion_started=motion_started,
                    check_stale=True,
                )

            wait_seconds = min(_POLL_INTERVAL_SECONDS, inference_deadline - now)
            remaining_goal = self._runtime.remaining_goal_seconds(episode_id)
            if remaining_goal is not None:
                wait_seconds = min(wait_seconds, remaining_goal)
            observation_age = self._runtime.observation_age_seconds()
            if observation_age is not None:
                wait_seconds = min(
                    wait_seconds,
                    max(
                        0.0,
                        self._runtime.joint_state_timeout_seconds - observation_age,
                    ),
                )
            self._execution_wake_event.wait(max(0.0, wait_seconds))
            self._execution_wake_event.clear()

    def _publish_policy_command(
        self,
        episode_id: str,
        target: np.ndarray,
        *,
        motion_started: bool,
    ) -> tuple[bool, TerminationDecision | None]:
        """Atomically recheck terminal guards and publish one normal policy command."""

        with self._command_gate_lock:
            decision = self._runtime.termination_decision(episode_id)
            if decision is None and self._shutdown_requested.is_set():
                decision = self._runtime.claim_termination(
                    episode_id,
                    status=TerminationStatus.ABORTED,
                    reason="server_shutting_down",
                    issue_hold=motion_started,
                )
            if decision is None and self._runtime.goal_timed_out(episode_id):
                decision = self._runtime.claim_termination(
                    episode_id,
                    status=TerminationStatus.ABORTED,
                    reason="goal_timeout",
                    issue_hold=True,
                )
            if decision is None:
                decision = self._runtime.claim_stale_observation(episode_id)
            if decision is not None:
                return False, decision

            command = Float64MultiArray()
            command.data = target.tolist()
            self._command_publisher.publish(command)
            return True, None

    def _wait_control_period(
        self,
        episode_id: str,
        control_period: float,
        *,
        motion_started: bool,
    ) -> TerminationDecision | None:
        deadline = time.monotonic() + control_period
        while True:
            decision = self._check_active_faults(
                episode_id,
                motion_started=motion_started,
                check_stale=True,
            )
            if decision is not None:
                return decision
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            self._execution_wake_event.wait(min(_POLL_INTERVAL_SECONDS, remaining))
            self._execution_wake_event.clear()

    def _publish_hold_position(self, decision: TerminationDecision) -> bool:
        """Publish the last valid six-joint observation once for this termination."""

        with self._command_gate_lock:
            if not self._runtime.begin_safe_stop_attempt(decision.episode_id):
                return self._runtime.snapshot().last_safe_stop_succeeded is True
            positions = self._latest_positions()
            if positions is None:
                unavailable_reason = "safe_stop_unavailable_no_valid_state"
                self._runtime.finish_safe_stop_attempt(
                    decision.episode_id,
                    published=False,
                    reason=unavailable_reason,
                )
                self.get_logger().error(unavailable_reason)
                return False
            try:
                hold_target = validate_action(positions)
            except (TypeError, ValueError):
                unavailable_reason = "safe_stop_unavailable_no_valid_state"
                self._runtime.finish_safe_stop_attempt(
                    decision.episode_id,
                    published=False,
                    reason=unavailable_reason,
                )
                self.get_logger().error(unavailable_reason)
                return False

            hold_command = Float64MultiArray()
            hold_command.data = hold_target.tolist()
            try:
                self._command_publisher.publish(hold_command)
            except BaseException as exc:
                self._runtime.finish_safe_stop_attempt(
                    decision.episode_id,
                    published=False,
                    reason="safe_stop_publish_error",
                )
                self.get_logger().error(
                    f"Hold-position publication failed ({type(exc).__name__}): {exc}\n"
                    + traceback.format_exc()
                )
                return False
            recorded = self._runtime.finish_safe_stop_attempt(
                decision.episode_id,
                published=True,
                reason=decision.reason,
            )
            if not recorded:
                self.get_logger().error("Hold-position outcome could not be recorded")
                return False
            self.get_logger().warning(
                f"Published hold-position for episode {decision.episode_id}: {decision.reason}"
            )
            return True

    def _finish_action_state(
        self,
        goal_handle: ServerGoalHandle,
        decision: TerminationDecision,
    ) -> None:
        if decision.status is TerminationStatus.SUCCEEDED:
            goal_handle.succeed()
            return
        if decision.status is TerminationStatus.ABORTED:
            goal_handle.abort()
            return

        cancel_state_deadline = time.monotonic() + 2.0
        while not goal_handle.is_cancel_requested:
            if time.monotonic() >= cancel_state_deadline:
                raise RuntimeError("accepted cancellation did not enter the ROS CANCELING state")
            time.sleep(0.001)
        goal_handle.canceled()

    def _finalize_goal(
        self,
        goal_handle: ServerGoalHandle,
        decision: TerminationDecision,
    ) -> ExecutePolicy.Result:
        """Apply the single winning decision to hold, diagnostics, Action, and result."""

        with self._finalization_lock:
            if (
                self._finalized_episode_id == decision.episode_id
                and self._finalized_result is not None
            ):
                return self._finalized_result

            if decision.issue_hold:
                self._publish_hold_position(decision)

            if decision.status is TerminationStatus.SUCCEEDED:
                level = DiagnosticStatus.OK
            elif decision.status is TerminationStatus.CANCELED:
                level = DiagnosticStatus.WARN
            else:
                level = DiagnosticStatus.ERROR
            self._set_diagnostic_event(level, decision.reason)
            self._publish_diagnostics()

            self._finish_action_state(goal_handle, decision)
            result = ExecutePolicy.Result()
            result.success = decision.success
            result.termination_reason = decision.reason
            result.episode_id = decision.episode_id
            self._finalized_episode_id = decision.episode_id
            self._finalized_result = result

            if decision.status is TerminationStatus.SUCCEEDED:
                self.get_logger().info(
                    f"Episode {decision.episode_id} succeeded: {decision.reason}"
                )
            elif decision.status is TerminationStatus.CANCELED:
                self.get_logger().info(f"Episode {decision.episode_id} canceled")
            else:
                self.get_logger().warning(
                    f"Episode {decision.episode_id} aborted: {decision.reason}"
                )
            return result

    async def _execute_callback(self, goal_handle: ServerGoalHandle) -> ExecutePolicy.Result:
        episode_id = self._episode_id(goal_handle)
        request = goal_handle.request
        control_period = 1.0 / self._control_rate_hz
        motion_started = False
        started = self._runtime.start_reserved_goal(
            episode_id,
            timeout_seconds=float(request.timeout_seconds),
        )
        if not started:
            self.get_logger().error("Accepted goal could not bind to the reserved runtime slot")
            self._runtime.abandon_reservation()
            goal_handle.abort()
            result = ExecutePolicy.Result()
            result.success = False
            result.termination_reason = "runtime_state_error"
            result.episode_id = episode_id
            return result

        self._execution_wake_event.clear()
        self.get_logger().info(
            f"Starting episode {episode_id} for instruction {request.instruction!r}"
        )
        self._publish_diagnostics()

        try:
            positions, decision = self._wait_for_initial_observation(episode_id)
            if decision is not None:
                return self._finalize_goal(goal_handle, decision)
            if positions is None:
                raise RuntimeError("observation wait ended without positions or termination")

            try:
                self._policy.reset()
            except BaseException as exc:
                self._runtime.record_policy_error(type(exc).__name__)
                self.get_logger().error(
                    f"Policy reset failed ({type(exc).__name__}): {exc}\n" + traceback.format_exc()
                )
                decision = self._claim_termination(
                    episode_id,
                    status=TerminationStatus.ABORTED,
                    reason="policy_error",
                    issue_hold=True,
                )
                if decision is None:
                    raise RuntimeError("policy reset fault could not claim termination") from exc
                return self._finalize_goal(goal_handle, decision)

            initial_distance: float | None = None
            for step in range(1, request.max_steps + 1):
                decision = self._check_active_faults(
                    episode_id,
                    motion_started=motion_started,
                    check_stale=True,
                )
                if decision is not None:
                    return self._finalize_goal(goal_handle, decision)

                positions = self._latest_positions()
                if positions is None:
                    decision = self._claim_termination(
                        episode_id,
                        status=TerminationStatus.ABORTED,
                        reason="observation_timeout",
                        issue_hold=True,
                    )
                    if decision is None:
                        raise RuntimeError("missing observation could not claim termination")
                    return self._finalize_goal(goal_handle, decision)

                call = self._submit_policy_call(episode_id, positions, request.instruction)
                decision = self._wait_for_policy_call(
                    episode_id,
                    call,
                    motion_started=motion_started,
                )
                if decision is not None:
                    return self._finalize_goal(goal_handle, decision)

                inference_latency_ms = max(0.0, time.monotonic() - call.started_at) * 1000.0
                try:
                    policy_action = call.future.result()
                except UnsupportedInstructionError as exc:
                    self.get_logger().warning(str(exc))
                    decision = self._claim_termination(
                        episode_id,
                        status=TerminationStatus.ABORTED,
                        reason="unsupported_instruction",
                        issue_hold=motion_started,
                    )
                    if decision is None:
                        raise RuntimeError("unsupported instruction could not terminate") from exc
                    return self._finalize_goal(goal_handle, decision)
                except BaseException as exc:
                    self._runtime.record_policy_error(type(exc).__name__)
                    self.get_logger().error(
                        f"Policy inference failed ({type(exc).__name__}): {exc}\n"
                        + traceback.format_exc()
                    )
                    decision = self._claim_termination(
                        episode_id,
                        status=TerminationStatus.ABORTED,
                        reason="policy_error",
                        issue_hold=True,
                    )
                    if decision is None:
                        raise RuntimeError("policy exception could not terminate") from exc
                    return self._finalize_goal(goal_handle, decision)

                try:
                    target = validate_action(policy_action)
                except (TypeError, ValueError) as exc:
                    self._runtime.record_policy_error(type(exc).__name__)
                    self.get_logger().error(f"Policy action validation failed: {exc}")
                    decision = self._claim_termination(
                        episode_id,
                        status=TerminationStatus.ABORTED,
                        reason="invalid_action",
                        issue_hold=True,
                    )
                    if decision is None:
                        raise RuntimeError("invalid action could not terminate") from exc
                    return self._finalize_goal(goal_handle, decision)

                decision = self._check_active_faults(
                    episode_id,
                    motion_started=motion_started,
                    check_stale=True,
                )
                if decision is not None:
                    return self._finalize_goal(goal_handle, decision)

                distance = float(np.max(np.abs(target - positions)))
                if initial_distance is None:
                    initial_distance = distance
                progress = _progress(distance, initial_distance, self._goal_tolerance)

                published, decision = self._publish_policy_command(
                    episode_id,
                    target,
                    motion_started=motion_started,
                )
                if decision is not None:
                    return self._finalize_goal(goal_handle, decision)
                if not published:
                    raise RuntimeError("policy command was neither published nor terminated")
                motion_started = True

                if distance > self._goal_tolerance:
                    decision = self._wait_control_period(
                        episode_id,
                        control_period,
                        motion_started=motion_started,
                    )
                    if decision is not None:
                        return self._finalize_goal(goal_handle, decision)

                    updated_positions = self._latest_positions()
                    if updated_positions is not None:
                        distance = float(np.max(np.abs(target - updated_positions)))
                    progress = _progress(distance, initial_distance, self._goal_tolerance)

                decision = self._check_active_faults(
                    episode_id,
                    motion_started=motion_started,
                    check_stale=True,
                )
                if decision is not None:
                    return self._finalize_goal(goal_handle, decision)

                feedback = ExecutePolicy.Feedback()
                feedback.current_step = step
                feedback.progress = progress
                feedback.inference_latency_ms = float(inference_latency_ms)
                goal_handle.publish_feedback(feedback)

                if distance <= self._goal_tolerance:
                    decision = self._check_active_faults(
                        episode_id,
                        motion_started=motion_started,
                        check_stale=True,
                    )
                    if decision is None:
                        decision = self._claim_termination(
                            episode_id,
                            status=TerminationStatus.SUCCEEDED,
                            reason="goal_reached",
                            issue_hold=False,
                        )
                    if decision is None:
                        raise RuntimeError("success could not claim termination")
                    return self._finalize_goal(goal_handle, decision)

            decision = self._check_active_faults(
                episode_id,
                motion_started=motion_started,
                check_stale=True,
            )
            if decision is None:
                decision = self._claim_termination(
                    episode_id,
                    status=TerminationStatus.ABORTED,
                    reason="max_steps_exceeded",
                    issue_hold=motion_started,
                )
            if decision is None:
                raise RuntimeError("max-steps path could not claim termination")
            return self._finalize_goal(goal_handle, decision)
        except BaseException as exc:
            existing = self._runtime.termination_decision(episode_id)
            if existing is not None:
                return self._finalize_goal(goal_handle, existing)
            self._runtime.record_policy_error(type(exc).__name__)
            self.get_logger().error(
                f"Unexpected runtime failure ({type(exc).__name__}): {exc}\n"
                + traceback.format_exc()
            )
            decision = self._claim_termination(
                episode_id,
                status=TerminationStatus.ABORTED,
                reason="policy_error",
                issue_hold=True,
            )
            if decision is None:
                raise
            return self._finalize_goal(goal_handle, decision)
        finally:
            self._runtime.complete_goal(episode_id)
            snapshot = self._runtime.snapshot()
            if not snapshot.backend_busy and snapshot.runtime_state is RuntimeState.IDLE:
                self._set_diagnostic_event(DiagnosticStatus.OK, "idle")
            self._publish_diagnostics()

    def _set_diagnostic_event(self, level: int, message: str) -> None:
        with self._diagnostic_event_lock:
            self._diagnostic_event_level = level
            self._diagnostic_event_message = message

    def _diagnostic_event(self) -> tuple[int, str]:
        with self._diagnostic_event_lock:
            return self._diagnostic_event_level, self._diagnostic_event_message

    def _publish_diagnostics(self) -> None:
        """Publish one consistent three-component diagnostic snapshot."""

        snapshot = self._runtime.snapshot()
        event_level, event_message = self._diagnostic_event()

        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [
            self._runtime_diagnostic(snapshot, event_level, event_message),
            self._observation_diagnostic(snapshot, event_level, event_message),
            self._policy_diagnostic(snapshot, event_level, event_message),
        ]
        self._diagnostics_publisher.publish(message)

    def _runtime_diagnostic(
        self,
        snapshot: RuntimeSnapshot,
        event_level: int,
        event_message: str,
    ) -> DiagnosticStatus:
        safe_stop_status = "not_requested"
        if snapshot.last_safe_stop_succeeded is True:
            safe_stop_status = "published"
        elif snapshot.last_safe_stop_succeeded is False:
            safe_stop_status = snapshot.last_safe_stop_reason
        return _diagnostic_status(
            name="policy_bridge/runtime",
            level=event_level,
            message=event_message,
            values={
                "runtime_state": snapshot.runtime_state.value,
                "active_goal": _bool_text(snapshot.active_goal),
                "episode_id": snapshot.episode_id,
                "last_termination_reason": snapshot.last_termination_reason,
                "goal_elapsed_ms": _float_text(snapshot.goal_elapsed_ms),
                "safe_stop_count": str(snapshot.safe_stop_count),
                "safe_stop_status": safe_stop_status,
                "last_safe_stop_reason": snapshot.last_safe_stop_reason,
            },
        )

    def _observation_diagnostic(
        self,
        snapshot: RuntimeSnapshot,
        event_level: int,
        event_message: str,
    ) -> DiagnosticStatus:
        level = DiagnosticStatus.OK
        status_message = "observation_ok"
        if event_message in _OBSERVATION_FAULT_REASONS and event_level != DiagnosticStatus.OK:
            level = event_level
            status_message = event_message
        elif snapshot.active_goal and not snapshot.joint_state_received:
            level = DiagnosticStatus.WARN
            status_message = "waiting_for_observation"
        elif snapshot.active_goal and snapshot.joint_state_age_ms is not None:
            if snapshot.joint_state_age_ms > snapshot.joint_state_timeout_ms:
                level = DiagnosticStatus.ERROR
                status_message = "stale_observation"
            elif (
                snapshot.joint_state_age_ms
                >= snapshot.joint_state_timeout_ms * _OBSERVATION_WARNING_FRACTION
            ):
                level = DiagnosticStatus.WARN
                status_message = "observation_near_timeout"
        return _diagnostic_status(
            name="policy_bridge/observation",
            level=level,
            message=status_message,
            values={
                "joint_state_received": _bool_text(snapshot.joint_state_received),
                "joint_state_valid": _bool_text(snapshot.joint_state_valid),
                "joint_state_age_ms": _optional_float_text(snapshot.joint_state_age_ms),
                "joint_state_timeout_ms": _float_text(snapshot.joint_state_timeout_ms),
            },
        )

    def _policy_diagnostic(
        self,
        snapshot: RuntimeSnapshot,
        event_level: int,
        event_message: str,
    ) -> DiagnosticStatus:
        level = DiagnosticStatus.WARN if snapshot.backend_busy else DiagnosticStatus.OK
        status_message = "backend_busy" if snapshot.backend_busy else "policy_ok"
        if event_message in _POLICY_FAULT_REASONS and event_level != DiagnosticStatus.OK:
            level = event_level
            status_message = event_message
        return _diagnostic_status(
            name="policy_bridge/policy",
            level=level,
            message=status_message,
            values={
                "backend_name": snapshot.backend_name,
                "backend_busy": _bool_text(snapshot.backend_busy),
                "last_inference_latency_ms": _optional_float_text(
                    snapshot.last_inference_latency_ms
                ),
                "inference_timeout_ms": _float_text(snapshot.inference_timeout_ms),
                "last_policy_error": snapshot.last_policy_error,
            },
        )


def _diagnostic_status(
    *,
    name: str,
    level: int,
    message: str,
    values: dict[str, str],
) -> DiagnosticStatus:
    status = DiagnosticStatus()
    status.level = level
    status.name = name
    status.message = message
    status.hardware_id = "policy_bridge"
    status.values = []
    for key, value in values.items():
        item = KeyValue()
        item.key = key
        item.value = value
        status.values.append(item)
    return status


def _progress(distance: float, initial_distance: float, tolerance: float) -> float:
    if initial_distance <= tolerance:
        return 1.0
    return float(np.clip(1.0 - distance / initial_distance, 0.0, 1.0))


def _bool_text(value: bool) -> str:
    return "true" if value else "false"


def _float_text(value: float) -> str:
    return f"{value:.3f}"


def _optional_float_text(value: float | None) -> str:
    return "unknown" if value is None else _float_text(value)


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


def _boolean_parameter(name: str, value: object) -> bool:
    if not isinstance(value, bool):
        raise TypeError(f"{name} must be a bool")
    return value


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
