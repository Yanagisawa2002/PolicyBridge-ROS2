"""ROS 2 action server with deterministic M1 faults and M2 observations."""

from __future__ import annotations

import math
import threading
import time
import traceback
from concurrent.futures import CancelledError, Future, ThreadPoolExecutor
from dataclasses import dataclass
from typing import Final

import message_filters
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray

from policy_bridge_interfaces.action import ExecutePolicy

from .action_validation import JOINT_COUNT, validate_action
from .observation import (
    DEFAULT_MAX_IMAGE_PIXELS,
    MAX_CONFIGURABLE_IMAGE_PIXELS,
    ImageValidationError,
    ObservationSnapshot,
    image_data_to_rgb,
    validate_image_layout,
)
from .observation_runtime import (
    JOINT_ONLY,
    RGB_JOINT,
    ObservationEpoch,
    ObservationHealthSnapshot,
    ObservationStore,
    validate_backend_observation_mode,
    validate_observation_mode,
    validate_sync_queue_size,
    validate_sync_slop_seconds,
)
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
_IMAGE_FAULT_REASONS: Final[frozenset[str]] = frozenset(
    {"image_timeout", "stale_image", "invalid_image"}
)
_SYNCHRONIZATION_FAULT_REASONS: Final[frozenset[str]] = frozenset({"observation_sync_timeout"})


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
        """Validate parameters and create the action, observations, worker, and diagnostics."""

        super().__init__("policy_server")

        self.declare_parameter("action_name", "execute_policy")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_command_topic", "/joint_command")
        self.declare_parameter("image_topic", "/camera/rgb/image_raw")
        self.declare_parameter("observation_mode", JOINT_ONLY)
        self.declare_parameter("policy_backend", "scripted")
        self.declare_parameter("inference_timeout_seconds", 1.0)
        self.declare_parameter("joint_state_timeout_seconds", 1.0)
        self.declare_parameter("image_timeout_seconds", 1.0)
        self.declare_parameter("synchronized_observation_timeout_seconds", 1.0)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_seconds", 0.05)
        self.declare_parameter("max_image_pixels", DEFAULT_MAX_IMAGE_PIXELS)
        self.declare_parameter("diagnostics_rate_hz", 1.0)
        self.declare_parameter("delayed_policy_delay_seconds", 2.0)
        self.declare_parameter("delayed_policy_first_call_only", True)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("goal_tolerance", 0.005)
        self.declare_parameter("joint_names", list(DEFAULT_JOINT_NAMES))

        self._action_name = str(self.get_parameter("action_name").value)
        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._joint_command_topic = str(self.get_parameter("joint_command_topic").value)
        self._image_topic = str(self.get_parameter("image_topic").value)
        observation_mode = validate_observation_mode(self.get_parameter("observation_mode").value)
        backend_selector, observation_mode = validate_backend_observation_mode(
            self.get_parameter("policy_backend").value,
            observation_mode,
        )
        inference_timeout_seconds = _positive_finite_parameter(
            "inference_timeout_seconds",
            self.get_parameter("inference_timeout_seconds").value,
        )
        joint_state_timeout_seconds = _positive_finite_parameter(
            "joint_state_timeout_seconds",
            self.get_parameter("joint_state_timeout_seconds").value,
        )
        image_timeout_seconds = _positive_finite_parameter(
            "image_timeout_seconds",
            self.get_parameter("image_timeout_seconds").value,
        )
        synchronized_observation_timeout_seconds = _positive_finite_parameter(
            "synchronized_observation_timeout_seconds",
            self.get_parameter("synchronized_observation_timeout_seconds").value,
        )
        sync_queue_size = validate_sync_queue_size(self.get_parameter("sync_queue_size").value)
        sync_slop_seconds = validate_sync_slop_seconds(
            self.get_parameter("sync_slop_seconds").value
        )
        self._max_image_pixels = _max_image_pixels_parameter(
            self.get_parameter("max_image_pixels").value
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
        if observation_mode == RGB_JOINT and not self._image_topic:
            raise ValueError("image_topic must not be empty in rgb_joint mode")

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
        self._observations = ObservationStore(
            observation_mode=observation_mode,
            image_timeout_seconds=image_timeout_seconds,
            synchronized_observation_timeout_seconds=(synchronized_observation_timeout_seconds),
            sync_queue_size=sync_queue_size,
            sync_slop_seconds=sync_slop_seconds,
        )

        self._latest_joint_positions: np.ndarray | None = None
        self._joint_state_lock = threading.Lock()
        self._command_gate_lock = threading.RLock()
        self._active_goal_observation_epoch: ObservationEpoch | None = None
        self._finalization_lock = threading.RLock()
        self._finalized_episode_id = ""
        self._finalized_result: ExecutePolicy.Result | None = None
        self._diagnostic_event_lock = threading.Lock()
        self._diagnostic_event_level = DiagnosticStatus.OK
        self._diagnostic_event_message = "idle"
        self._shutdown_requested = threading.Event()
        self._execution_wake_event = threading.Event()
        self._last_joint_state_warning_time = float("-inf")
        self._last_image_warning_time = float("-inf")
        self._last_sync_warning_time = float("-inf")

        self._policy_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="policy_bridge_worker",
        )

        self._callback_group = ReentrantCallbackGroup()
        self._observation_callback_group = MutuallyExclusiveCallbackGroup()
        self._command_publisher = self.create_publisher(
            Float64MultiArray, self._joint_command_topic, 10
        )
        self._diagnostics_publisher = self.create_publisher(DiagnosticArray, DIAGNOSTICS_TOPIC, 10)
        sensor_qos = _sensor_data_qos(sync_queue_size)
        self._joint_state_subscription = None
        self._joint_filter_subscriber = None
        self._image_filter_subscriber = None
        self._validated_joint_filter = None
        self._validated_image_filter = None
        self._time_synchronizer = None
        if observation_mode == JOINT_ONLY:
            self._joint_state_subscription = self.create_subscription(
                JointState,
                self._joint_state_topic,
                self._joint_state_callback,
                sensor_qos,
                callback_group=self._observation_callback_group,
            )
        else:
            self._joint_filter_subscriber = message_filters.Subscriber(
                self,
                JointState,
                self._joint_state_topic,
                qos_profile=sensor_qos,
                callback_group=self._observation_callback_group,
            )
            self._image_filter_subscriber = message_filters.Subscriber(
                self,
                Image,
                self._image_topic,
                qos_profile=sensor_qos,
                callback_group=self._observation_callback_group,
            )
            self._validated_joint_filter = message_filters.SimpleFilter()
            self._validated_image_filter = message_filters.SimpleFilter()
            self._joint_filter_subscriber.registerCallback(self._joint_filter_callback)
            self._image_filter_subscriber.registerCallback(self._image_filter_callback)
            filters = [self._validated_joint_filter, self._validated_image_filter]
            if sync_slop_seconds == 0.0:
                self._time_synchronizer = message_filters.TimeSynchronizer(
                    filters,
                    sync_queue_size,
                )
            else:
                self._time_synchronizer = message_filters.ApproximateTimeSynchronizer(
                    filters,
                    sync_queue_size,
                    sync_slop_seconds,
                    allow_headerless=False,
                )
            self._time_synchronizer.registerCallback(self._synchronized_observation_callback)
        self._diagnostics_timer = self.create_timer(
            1.0 / diagnostics_rate_hz,
            self._try_publish_diagnostics,
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
            f"backend '{backend_name}' and observation_mode={observation_mode!r}; "
            f"joint topic='{self._joint_state_topic}'"
            + ("" if observation_mode == JOINT_ONLY else f", image topic='{self._image_topic}'")
        )
        self._try_publish_diagnostics()

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

        # The observation epoch is linearized before the acceptance response,
        # so any RGB pair published after the client observes acceptance is
        # unambiguously goal-local even if execute_callback is scheduled later.
        with self._command_gate_lock:
            self._active_goal_observation_epoch = self._observations.capture_epoch()

        del timeout_seconds  # Validation happens here; the deadline starts in execute_callback.
        with self._finalization_lock:
            self._finalized_episode_id = ""
            self._finalized_result = None
        self._runtime.record_policy_error("")
        self._set_diagnostic_event(DiagnosticStatus.OK, "goal_accepted")
        try:
            self._try_publish_diagnostics()
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
            self._try_publish_diagnostics()
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
        """Accept one joint-only observation and create a policy snapshot."""

        self._process_joint_state(message, create_snapshot=True)

    def _joint_filter_callback(self, message: JointState) -> None:
        """Validate a raw RGB-mode joint message before entering the synchronizer."""

        valid_for_synchronization = self._process_joint_state(
            message,
            create_snapshot=False,
        )
        # Never signal message_filters while holding the command gate.  Humble
        # synchronizers invoke callbacks under their own lock, so doing so
        # would create an ATS-lock/command-gate inversion with cancellation.
        if valid_for_synchronization and self._validated_joint_filter is not None:
            self._validated_joint_filter.signalMessage(message)

    def _process_joint_state(
        self,
        message: JointState,
        *,
        create_snapshot: bool,
    ) -> bool:
        """Update hold/freshness state and optionally construct a joint-only snapshot."""

        prior_snapshot = self._runtime.snapshot()
        try:
            positions = self._positions_in_configured_order(message)
            validated = validate_action(positions)
            joint_stamp_ns = _header_stamp_ns(message.header.stamp)
        except (TypeError, ValueError) as exc:
            with self._command_gate_lock:
                self._runtime.record_observation(valid=False)
            self._warn_about_joint_state(str(exc))
            if prior_snapshot.joint_state_valid:
                self._try_publish_diagnostics()
            self._execution_wake_event.set()
            return False

        policy_snapshot: ObservationSnapshot | None = None
        if create_snapshot:
            sequence_id = self._observations.reserve_sequence_id()
            policy_snapshot = ObservationSnapshot(
                sequence_id=sequence_id,
                joint_positions=validated,
                joint_names=self._joint_names,
                rgb=None,
                joint_stamp_ns=joint_stamp_ns,
                image_stamp_ns=None,
                received_monotonic_ns=time.monotonic_ns(),
                synchronization_skew_ms=None,
                image_frame_id=None,
            )

        # The command gate is the linearization point shared with conditional
        # freshness faults, cancellation, normal command publication, and
        # hold-position.  Whichever side acquires it first commits completely.
        with self._command_gate_lock, self._joint_state_lock:
            self._runtime.record_observation(valid=True)
            self._latest_joint_positions = validated
            self._observations.record_joint_stamp(joint_stamp_ns)
            if policy_snapshot is not None:
                self._observations.commit_snapshot(policy_snapshot)
            elif joint_stamp_ns <= 0:
                self._observations.record_sync_error("missing_joint_header_stamp")
        if not prior_snapshot.joint_state_received or not prior_snapshot.joint_state_valid:
            self._try_publish_diagnostics()
        self._execution_wake_event.set()
        return create_snapshot or joint_stamp_ns > 0

    def _image_filter_callback(self, message: Image) -> None:
        """Validate raw image layout before allowing it into the bounded synchronizer."""

        prior_health = self._observations.snapshot()
        try:
            image_stamp_ns = _header_stamp_ns(message.header.stamp)
            if image_stamp_ns <= 0:
                raise ImageValidationError("image header stamp must be positive")
            if not message.header.frame_id:
                raise ImageValidationError("image frame_id must not be empty")
            validate_image_layout(
                height=message.height,
                width=message.width,
                encoding=message.encoding,
                step=message.step,
                data=message.data,
                max_pixels=self._max_image_pixels,
            )
        except (TypeError, ValueError, ImageValidationError) as exc:
            error = f"{type(exc).__name__}: {exc}"
            with self._command_gate_lock:
                self._observations.record_image(
                    valid=False,
                    stamp_ns=_safe_header_stamp_ns(message.header.stamp),
                    encoding=str(message.encoding),
                    width=int(message.width),
                    height=int(message.height),
                    frame_id=str(message.header.frame_id),
                    error=error,
                )
            self._warn_about_image(error)
            self._execution_wake_event.set()
            if not prior_health.image_received or prior_health.image_valid:
                self._try_publish_diagnostics()
            return

        with self._command_gate_lock:
            self._observations.record_image(
                valid=True,
                stamp_ns=image_stamp_ns,
                encoding=message.encoding,
                width=message.width,
                height=message.height,
                frame_id=message.header.frame_id,
            )
        self._execution_wake_event.set()
        if not prior_health.image_received or not prior_health.image_valid:
            self._try_publish_diagnostics()
        if self._validated_image_filter is not None:
            self._validated_image_filter.signalMessage(message)

    def _synchronized_observation_callback(
        self,
        joint_message: JointState,
        image_message: Image,
    ) -> None:
        """Keep user code from escaping Humble's non-reentrant synchronizer lock."""

        try:
            self._process_synchronized_observation(joint_message, image_message)
        except Exception as exc:
            error = f"synchronized_callback_error ({type(exc).__name__}): {exc}"
            try:
                with self._command_gate_lock:
                    self._observations.record_sync_error(error)
            except Exception as record_exc:
                try:
                    self.get_logger().error(
                        "Could not record synchronized callback failure "
                        f"({type(record_exc).__name__}): {record_exc}"
                    )
                except Exception:
                    pass
            self._execution_wake_event.set()
            try:
                self.get_logger().error(error + "\n" + traceback.format_exc())
            except Exception:
                pass
            try:
                self._try_publish_diagnostics()
            except Exception as diagnostics_exc:
                try:
                    self.get_logger().error(
                        "Synchronized callback diagnostics failed "
                        f"({type(diagnostics_exc).__name__}): {diagnostics_exc}"
                    )
                except Exception:
                    pass

    def _process_synchronized_observation(
        self,
        joint_message: JointState,
        image_message: Image,
    ) -> None:
        """Build and atomically commit one independently revalidated RGB snapshot."""

        prior_health = self._observations.snapshot()
        try:
            positions = validate_action(self._positions_in_configured_order(joint_message))
            joint_stamp_ns = _header_stamp_ns(joint_message.header.stamp)
            image_stamp_ns = _header_stamp_ns(image_message.header.stamp)
            if joint_stamp_ns <= 0 or image_stamp_ns <= 0:
                raise ValueError("synchronized messages require positive header stamps")
            if not image_message.header.frame_id:
                raise ValueError("synchronized image frame_id must not be empty")
            rgb = image_data_to_rgb(
                height=image_message.height,
                width=image_message.width,
                encoding=image_message.encoding,
                step=image_message.step,
                data=image_message.data,
                max_pixels=self._max_image_pixels,
            )
            skew_ms = abs(joint_stamp_ns - image_stamp_ns) / 1_000_000.0
            if skew_ms > self._observations.sync_slop_seconds * 1000.0:
                raise ValueError("synchronized pair exceeds sync_slop_seconds")
            sequence_id = self._observations.reserve_sequence_id()
            snapshot = ObservationSnapshot(
                sequence_id=sequence_id,
                joint_positions=positions,
                joint_names=self._joint_names,
                rgb=rgb,
                joint_stamp_ns=joint_stamp_ns,
                image_stamp_ns=image_stamp_ns,
                received_monotonic_ns=time.monotonic_ns(),
                synchronization_skew_ms=skew_ms,
                image_frame_id=image_message.header.frame_id,
            )
        except (TypeError, ValueError, ImageValidationError) as exc:
            error = f"{type(exc).__name__}: {exc}"
            with self._command_gate_lock:
                self._observations.record_sync_error(error)
            self._warn_about_sync(error)
            self._execution_wake_event.set()
            if prior_health.last_sync_error != error:
                self._try_publish_diagnostics()
            return

        with self._command_gate_lock:
            committed = self._observations.commit_snapshot(snapshot)
        if not committed:
            return
        self._execution_wake_event.set()
        if not prior_health.synchronized_snapshot_available:
            self._try_publish_diagnostics()

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

    def _warn_about_image(self, reason: str) -> None:
        now = time.monotonic()
        if now - self._last_image_warning_time >= 5.0:
            self.get_logger().warning(f"Ignoring invalid RGB image: {reason}")
            self._last_image_warning_time = now

    def _warn_about_sync(self, reason: str) -> None:
        now = time.monotonic()
        if now - self._last_sync_warning_time >= 5.0:
            self.get_logger().warning(f"Ignoring invalid synchronized pair: {reason}")
            self._last_sync_warning_time = now

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

    def _claim_multimodal_fault(
        self,
        episode_id: str,
        *,
        snapshot_wait_elapsed_seconds: float | None = None,
        required_sequence_id: int = 0,
        wait_epoch: ObservationEpoch | None = None,
    ) -> TerminationDecision | None:
        """Atomically classify and claim an M2 image or synchronization fault."""

        if not self._observations.image_required:
            return None
        with self._command_gate_lock:
            existing = self._runtime.termination_decision(episode_id)
            if existing is not None:
                return existing
            goal_elapsed = self._runtime.goal_elapsed_seconds(episode_id)
            if goal_elapsed is None:
                return None
            goal_epoch = self._active_goal_observation_epoch
            if goal_epoch is None:
                return None
            reason = self._observations.image_fault_reason(
                goal_elapsed_seconds=goal_elapsed,
                goal_epoch=goal_epoch,
            )
            if (
                reason is None
                and snapshot_wait_elapsed_seconds is not None
                and wait_epoch is not None
            ):
                reason = self._observations.sync_fault_reason(
                    wait_elapsed_seconds=snapshot_wait_elapsed_seconds,
                    required_sequence_id=required_sequence_id,
                    wait_epoch=wait_epoch,
                )
            if reason is None:
                return None
            return self._runtime.claim_termination(
                episode_id,
                status=TerminationStatus.ABORTED,
                reason=reason,
                issue_hold=True,
            )

    def _check_active_faults(
        self,
        episode_id: str,
        *,
        motion_started: bool,
        check_stale: bool,
        snapshot_wait_elapsed_seconds: float | None = None,
        required_sequence_id: int = 0,
        wait_epoch: ObservationEpoch | None = None,
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
                decision = self._runtime.claim_observation_timeout(episode_id)
                if decision is not None:
                    return decision
        return self._claim_multimodal_fault(
            episode_id,
            snapshot_wait_elapsed_seconds=snapshot_wait_elapsed_seconds,
            required_sequence_id=required_sequence_id,
            wait_epoch=wait_epoch,
        )

    def _wait_for_snapshot(
        self,
        episode_id: str,
        *,
        after_sequence_id: int,
        not_before_monotonic: float,
        motion_started: bool,
    ) -> tuple[ObservationSnapshot | None, TerminationDecision | None]:
        """Wait for one fresh, newer policy snapshot with bounded monotonic polling."""

        self._runtime.mark_waiting_for_observation(episode_id)
        waiting_started = time.monotonic()
        wait_epoch = self._observations.capture_epoch()
        while True:
            now = time.monotonic()
            wait_elapsed = max(0.0, now - waiting_started)
            decision = self._check_active_faults(
                episode_id,
                motion_started=motion_started,
                check_stale=True,
                snapshot_wait_elapsed_seconds=wait_elapsed,
                required_sequence_id=after_sequence_id,
                wait_epoch=wait_epoch,
            )
            if decision is not None:
                return None, decision

            snapshot = self._observations.latest_snapshot(after_sequence_id=after_sequence_id)
            if snapshot is not None and now >= not_before_monotonic:
                self._runtime.mark_running(episode_id)
                return snapshot, None

            goal_elapsed = self._runtime.goal_elapsed_seconds(episode_id)
            if (
                goal_elapsed is not None
                and not self._runtime.has_valid_observation()
                and goal_elapsed >= self._runtime.joint_state_timeout_seconds
            ):
                with self._command_gate_lock:
                    decision = self._runtime.termination_decision(episode_id)
                    if decision is None:
                        decision = self._runtime.claim_observation_timeout(episode_id)
                if decision is not None:
                    return None, decision
                continue

            wait_seconds = _POLL_INTERVAL_SECONDS
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
            elif goal_elapsed is not None:
                wait_seconds = min(
                    wait_seconds,
                    max(
                        0.0,
                        self._runtime.joint_state_timeout_seconds - goal_elapsed,
                    ),
                )
            if now < not_before_monotonic:
                wait_seconds = min(wait_seconds, not_before_monotonic - now)
            self._execution_wake_event.wait(max(0.0, wait_seconds))
            self._execution_wake_event.clear()

    def _wait_control_period(
        self,
        episode_id: str,
        control_period: float,
        *,
        motion_started: bool,
    ) -> TerminationDecision | None:
        """Preserve the M1 final-step observation window without requiring another frame."""

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

    def _submit_policy_call(
        self,
        episode_id: str,
        observation: ObservationSnapshot,
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
                observation,
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
        observation: ObservationSnapshot,
        instruction: str,
    ) -> object:
        try:
            return self._policy.predict(observation, instruction)
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
            self._try_publish_diagnostics()

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
            if decision is None and self._observations.image_required:
                goal_elapsed = self._runtime.goal_elapsed_seconds(episode_id)
                goal_epoch = self._active_goal_observation_epoch
                reason = (
                    None
                    if goal_elapsed is None or goal_epoch is None
                    else self._observations.image_fault_reason(
                        goal_elapsed_seconds=goal_elapsed,
                        goal_epoch=goal_epoch,
                    )
                )
                if reason is not None:
                    decision = self._runtime.claim_termination(
                        episode_id,
                        status=TerminationStatus.ABORTED,
                        reason=reason,
                        issue_hold=True,
                    )
            if decision is not None:
                return False, decision

            command = Float64MultiArray()
            command.data = target.tolist()
            self._command_publisher.publish(command)
            return True, None

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
            self._try_publish_diagnostics()

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
        with self._command_gate_lock:
            goal_observation_epoch = self._active_goal_observation_epoch
            if goal_observation_epoch is None:
                goal_observation_epoch = self._observations.capture_epoch()
                self._active_goal_observation_epoch = goal_observation_epoch
        started = self._runtime.start_reserved_goal(
            episode_id,
            timeout_seconds=float(request.timeout_seconds),
        )
        if not started:
            self.get_logger().error("Accepted goal could not bind to the reserved runtime slot")
            self._runtime.abandon_reservation()
            with self._command_gate_lock:
                if self._active_goal_observation_epoch is goal_observation_epoch:
                    self._active_goal_observation_epoch = None
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
        self._try_publish_diagnostics()

        try:
            observation, decision = self._wait_for_snapshot(
                episode_id,
                after_sequence_id=(
                    goal_observation_epoch.snapshot_sequence_id
                    if self._observations.image_required
                    else 0
                ),
                not_before_monotonic=0.0,
                motion_started=False,
            )
            if decision is not None:
                return self._finalize_goal(goal_handle, decision)
            if observation is None:
                raise RuntimeError("snapshot wait ended without an observation or termination")

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

                positions = observation.joint_positions

                call = self._submit_policy_call(
                    episode_id,
                    observation,
                    request.instruction,
                )
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
                command_published_at = time.monotonic()

                if distance > self._goal_tolerance:
                    if step < request.max_steps:
                        next_observation, decision = self._wait_for_snapshot(
                            episode_id,
                            after_sequence_id=observation.sequence_id,
                            not_before_monotonic=command_published_at + control_period,
                            motion_started=motion_started,
                        )
                        if decision is not None:
                            return self._finalize_goal(goal_handle, decision)
                        if next_observation is None:
                            raise RuntimeError(
                                "new snapshot wait ended without observation or termination"
                            )
                        observation = next_observation
                    else:
                        decision = self._wait_control_period(
                            episode_id,
                            control_period,
                            motion_started=motion_started,
                        )
                        if decision is not None:
                            return self._finalize_goal(goal_handle, decision)
                        next_observation = self._observations.latest_snapshot(
                            after_sequence_id=observation.sequence_id
                        )
                        if next_observation is not None:
                            observation = next_observation

                    distance = float(np.max(np.abs(target - observation.joint_positions)))
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
            with self._command_gate_lock:
                if self._active_goal_observation_epoch is goal_observation_epoch:
                    self._active_goal_observation_epoch = None
            snapshot = self._runtime.snapshot()
            if not snapshot.backend_busy and snapshot.runtime_state is RuntimeState.IDLE:
                self._set_diagnostic_event(DiagnosticStatus.OK, "idle")
            self._try_publish_diagnostics()

    def _set_diagnostic_event(self, level: int, message: str) -> None:
        with self._diagnostic_event_lock:
            self._diagnostic_event_level = level
            self._diagnostic_event_message = message

    def _diagnostic_event(self) -> tuple[int, str]:
        with self._diagnostic_event_lock:
            return self._diagnostic_event_level, self._diagnostic_event_message

    def _try_publish_diagnostics(self) -> None:
        """Diagnostics must not prevent Action completion or release admission."""
        try:
            self._publish_diagnostics()
        except Exception as exc:
            # Logging can itself fail during ROS shutdown. Neither optional
            # transport is allowed to replace a terminal Action result.
            try:
                self.get_logger().error(f"Diagnostic publication failed: {exc}")
            except Exception:
                pass

    def _publish_diagnostics(self) -> None:
        """Publish one consistent five-component diagnostic snapshot."""

        snapshot = self._runtime.snapshot()
        observation_health = self._observations.snapshot()
        event_level, event_message = self._diagnostic_event()

        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [
            self._runtime_diagnostic(snapshot, event_level, event_message),
            self._observation_diagnostic(snapshot, event_level, event_message),
            self._policy_diagnostic(snapshot, event_level, event_message),
            self._image_diagnostic(
                snapshot,
                observation_health,
                event_level,
                event_message,
            ),
            self._synchronization_diagnostic(
                snapshot,
                observation_health,
                event_level,
                event_message,
            ),
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

    def _image_diagnostic(
        self,
        runtime: RuntimeSnapshot,
        health: ObservationHealthSnapshot,
        event_level: int,
        event_message: str,
    ) -> DiagnosticStatus:
        if not health.image_required:
            level = DiagnosticStatus.OK
            status_message = "disabled"
        elif event_message in _IMAGE_FAULT_REASONS and event_level != DiagnosticStatus.OK:
            level = DiagnosticStatus.ERROR
            status_message = event_message
        elif not runtime.active_goal:
            level = DiagnosticStatus.OK
            status_message = "image_idle"
        elif not health.image_received:
            level = DiagnosticStatus.WARN
            status_message = "waiting_for_image"
        elif not health.image_valid:
            level = DiagnosticStatus.WARN
            status_message = "invalid_image_waiting"
        elif health.image_age_ms is not None and health.image_age_ms >= health.image_timeout_ms:
            level = DiagnosticStatus.ERROR
            status_message = "stale_image"
        elif (
            health.image_age_ms is not None
            and health.image_age_ms >= health.image_timeout_ms * _OBSERVATION_WARNING_FRACTION
        ):
            level = DiagnosticStatus.WARN
            status_message = "image_near_timeout"
        else:
            level = DiagnosticStatus.OK
            status_message = "image_ok"
        return _diagnostic_status(
            name="policy_bridge/image",
            level=level,
            message=status_message,
            values={
                "image_required": _bool_text(health.image_required),
                "image_received": _bool_text(health.image_received),
                "image_valid": _bool_text(health.image_valid),
                "image_age_ms": _optional_float_text(health.image_age_ms),
                "image_timeout_ms": _float_text(health.image_timeout_ms),
                "encoding": health.encoding,
                "width": str(health.width),
                "height": str(health.height),
                "frame_id": health.frame_id,
                "last_image_error": health.last_image_error,
            },
        )

    def _synchronization_diagnostic(
        self,
        runtime: RuntimeSnapshot,
        health: ObservationHealthSnapshot,
        event_level: int,
        event_message: str,
    ) -> DiagnosticStatus:
        if not health.image_required:
            level = DiagnosticStatus.OK
            status_message = "disabled"
        elif event_message in _SYNCHRONIZATION_FAULT_REASONS and event_level != DiagnosticStatus.OK:
            level = DiagnosticStatus.ERROR
            status_message = event_message
        elif not runtime.active_goal:
            level = DiagnosticStatus.OK
            status_message = "synchronization_idle"
        elif runtime.runtime_state is RuntimeState.WAITING_FOR_OBSERVATION:
            level = DiagnosticStatus.WARN
            status_message = (
                "waiting_for_synchronization"
                if not health.synchronized_snapshot_available
                else "waiting_for_new_synchronization"
            )
        elif not health.synchronized_snapshot_available:
            level = DiagnosticStatus.WARN
            status_message = "waiting_for_synchronization"
        elif health.current_sync_error:
            level = DiagnosticStatus.WARN
            status_message = "synchronization_waiting"
        else:
            level = DiagnosticStatus.OK
            status_message = "synchronized"
        return _diagnostic_status(
            name="policy_bridge/synchronization",
            level=level,
            message=status_message,
            values={
                "synchronized_snapshot_available": _bool_text(
                    health.synchronized_snapshot_available
                ),
                "snapshot_sequence_id": str(health.snapshot_sequence_id),
                "snapshot_age_ms": _optional_float_text(health.snapshot_age_ms),
                "last_sync_skew_ms": _optional_float_text(health.last_sync_skew_ms),
                "sync_slop_ms": _float_text(health.sync_slop_ms),
                "sync_queue_size": str(health.sync_queue_size),
                "synchronized_observation_timeout_ms": _float_text(
                    health.synchronized_observation_timeout_ms
                ),
                "last_sync_error": health.last_sync_error,
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


def _header_stamp_ns(stamp: object) -> int:
    """Convert a ROS header stamp to nanoseconds without substituting a clock."""

    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if (
        isinstance(sec, bool)
        or not isinstance(sec, int)
        or isinstance(nanosec, bool)
        or not isinstance(nanosec, int)
    ):
        raise TypeError("header stamp sec/nanosec must be integers")
    if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
        raise ValueError("header stamp is outside the ROS time range")
    return sec * 1_000_000_000 + nanosec


def _safe_header_stamp_ns(stamp: object) -> int:
    try:
        return _header_stamp_ns(stamp)
    except (TypeError, ValueError):
        return 0


def _sensor_data_qos(depth: int) -> QoSProfile:
    """Return explicit bounded volatile best-effort sensor-data QoS."""

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _max_image_pixels_parameter(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError("max_image_pixels must be an integer")
    if value <= 0 or value > MAX_CONFIGURABLE_IMAGE_PIXELS:
        raise ValueError(
            "max_image_pixels must be greater than zero and at most "
            f"{MAX_CONFIGURABLE_IMAGE_PIXELS}"
        )
    return value


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
