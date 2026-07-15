"""Thread-safe, ROS-independent runtime state for policy execution.

The action server owns ROS concerns such as goal handles and publishers.  This
module owns the small amount of shared state that must remain coherent when
goal, cancellation, worker-completion, and timer callbacks run concurrently.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum


class RuntimeState(Enum):
    """Externally observable execution state."""

    IDLE = "idle"
    WAITING_FOR_OBSERVATION = "waiting_for_observation"
    RUNNING = "running"
    STOPPING = "stopping"
    FAULTED = "faulted"


class GoalAdmission(Enum):
    """Result of atomically attempting to admit a goal."""

    ACCEPTED = "accepted"
    REJECTED_ACTIVE_GOAL = "active_goal"
    REJECTED_BACKEND_BUSY = "backend_busy"

    @property
    def accepted(self) -> bool:
        """Return whether the goal acquired the single active-goal slot."""

        return self is GoalAdmission.ACCEPTED


class TerminationStatus(Enum):
    """ROS-independent terminal status chosen for one goal."""

    SUCCEEDED = "succeeded"
    ABORTED = "aborted"
    CANCELED = "canceled"


@dataclass(frozen=True, slots=True)
class TerminationDecision:
    """Immutable first-wins decision returned to the termination owner."""

    episode_id: str
    status: TerminationStatus
    reason: str
    issue_hold: bool
    elapsed_ms: float

    @property
    def success(self) -> bool:
        """Derive result success from terminal status without a second flag."""

        return self.status is TerminationStatus.SUCCEEDED


@dataclass(frozen=True, slots=True)
class RuntimeSnapshot:
    """Consistent diagnostics input captured under one lock."""

    runtime_state: RuntimeState
    active_goal: bool
    episode_id: str
    last_termination_reason: str
    goal_elapsed_ms: float
    safe_stop_count: int
    last_safe_stop_reason: str
    last_safe_stop_succeeded: bool | None

    joint_state_received: bool
    joint_state_valid: bool
    joint_state_age_ms: float | None
    joint_state_timeout_ms: float

    backend_name: str
    backend_busy: bool
    last_inference_latency_ms: float | None
    inference_timeout_ms: float
    last_policy_error: str


class RuntimeStateMachine:
    """Coordinate goal lifetime, backend ownership, and diagnostics state.

    The class deliberately contains no ROS types.  Methods that can lose a
    race return a value instead of relying on a prior check.  In particular,
    :meth:`claim_termination` is the linearization point for all competing
    success, cancellation, timeout, and fault paths.
    """

    def __init__(
        self,
        *,
        backend_name: str,
        inference_timeout_seconds: float,
        joint_state_timeout_seconds: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """Initialize an idle runtime with validated timeout settings."""

        if not isinstance(backend_name, str) or not backend_name:
            raise ValueError("backend_name must be a non-empty string")
        if not callable(clock):
            raise TypeError("clock must be callable")

        self._inference_timeout_seconds = _positive_finite(
            "inference_timeout_seconds", inference_timeout_seconds
        )
        self._joint_state_timeout_seconds = _positive_finite(
            "joint_state_timeout_seconds", joint_state_timeout_seconds
        )
        self._backend_name = backend_name
        self._clock = clock
        self._lock = threading.RLock()

        self._state = RuntimeState.IDLE
        self._active_goal = False
        self._episode_id = ""
        self._goal_started_at: float | None = None
        self._goal_deadline: float | None = None
        self._termination: TerminationDecision | None = None
        self._last_termination_reason = ""
        self._last_goal_elapsed_ms = 0.0

        self._backend_busy = False
        self._inference_token = 0
        self._active_inference_token: int | None = None
        self._last_inference_latency_ms: float | None = None
        self._last_policy_error = ""

        self._joint_state_received = False
        self._joint_state_valid = False
        self._last_valid_joint_state_at: float | None = None

        self._safe_stop_count = 0
        self._safe_stop_attempt_started_for_goal = False
        self._safe_stop_attempt_finished_for_goal = False
        self._last_safe_stop_reason = ""
        self._last_safe_stop_succeeded: bool | None = None

    @property
    def inference_timeout_seconds(self) -> float:
        """Configured per-inference timeout."""

        return self._inference_timeout_seconds

    @property
    def joint_state_timeout_seconds(self) -> float:
        """Configured valid-observation freshness timeout."""

        return self._joint_state_timeout_seconds

    def reserve_goal(self) -> GoalAdmission:
        """Atomically reserve the active-goal slot before a ROS UUID exists.

        ROS action goal callbacks receive the request but not the accepted
        goal handle.  Reserving and starting are therefore separate operations:
        this method gates admission, while :meth:`start_reserved_goal` binds the
        goal-derived episode ID and starts its monotonic deadline.
        """

        with self._lock:
            admission = self._admission_unlocked()
            if admission is not GoalAdmission.ACCEPTED:
                return admission
            self._reserve_unlocked()
            return GoalAdmission.ACCEPTED

    def start_reserved_goal(self, episode_id: str, *, timeout_seconds: float) -> bool:
        """Bind and start a reservation when its ROS goal UUID becomes available.

        Rebinding the same live episode is idempotent and never resets its
        deadline, including when an early cancel callback already claimed the
        terminal decision before the execute callback entered.
        """

        _validate_episode_id(episode_id)
        goal_timeout = _nonnegative_finite("timeout_seconds", timeout_seconds)

        with self._lock:
            if (
                self._active_goal
                and self._episode_id == episode_id
                and self._goal_started_at is not None
            ):
                return True
            if (
                not self._active_goal
                or self._episode_id
                or self._goal_started_at is not None
                or self._termination is not None
            ):
                return False
            self._start_reserved_unlocked(episode_id, goal_timeout, self._now())
            return True

    def abandon_reservation(self) -> bool:
        """Release an unbound, non-terminal goal reservation.

        This is intentionally unavailable after an episode ID is bound, so an
        execution callback cannot accidentally discard a live goal.
        """

        with self._lock:
            if (
                not self._active_goal
                or self._episode_id
                or self._goal_started_at is not None
                or self._termination is not None
            ):
                return False
            self._active_goal = False
            self._goal_deadline = None
            self._state = RuntimeState.IDLE
            return True

    def begin_goal(self, episode_id: str, *, timeout_seconds: float) -> GoalAdmission:
        """Atomically admit one goal unless a goal or worker is already active.

        A zero goal timeout disables its wall-clock deadline.  Invalid timeout
        values are programmer/input validation errors and are rejected before
        shared state changes.
        """

        _validate_episode_id(episode_id)
        goal_timeout = _nonnegative_finite("timeout_seconds", timeout_seconds)

        with self._lock:
            admission = self._admission_unlocked()
            if admission is not GoalAdmission.ACCEPTED:
                return admission
            self._reserve_unlocked()
            self._start_reserved_unlocked(episode_id, goal_timeout, self._now())
            return GoalAdmission.ACCEPTED

    def mark_waiting_for_observation(self, episode_id: str) -> bool:
        """Move the current non-terminal goal to the observation-wait state."""

        return self._set_active_state(episode_id, RuntimeState.WAITING_FOR_OBSERVATION)

    def mark_running(self, episode_id: str) -> bool:
        """Move the current non-terminal goal to the running state."""

        return self._set_active_state(episode_id, RuntimeState.RUNNING)

    def remaining_goal_seconds(self, episode_id: str) -> float | None:
        """Return remaining goal time, ``None`` if disabled, or zero if inactive."""

        with self._lock:
            if not self._owns_live_goal(episode_id):
                return 0.0
            if self._goal_deadline is None:
                return None
            now = self._now()
            return max(0.0, self._goal_deadline - now)

    def goal_elapsed_seconds(self, episode_id: str) -> float | None:
        """Return monotonic elapsed time for the current live episode."""

        with self._lock:
            if not self._owns_live_goal(episode_id) or self._goal_started_at is None:
                return None
            return max(0.0, self._now() - self._goal_started_at)

    def goal_timed_out(self, episode_id: str) -> bool:
        """Return whether an enabled deadline has elapsed for the live goal."""

        remaining = self.remaining_goal_seconds(episode_id)
        return remaining is not None and remaining <= 0.0

    def begin_inference(self, episode_id: str) -> int | None:
        """Acquire the single backend slot and return its generation token."""

        with self._lock:
            if not self._owns_live_goal(episode_id) or self._backend_busy:
                return None
            self._inference_token += 1
            self._active_inference_token = self._inference_token
            self._backend_busy = True
            return self._inference_token

    def finish_inference(
        self,
        token: int,
        *,
        latency_ms: float | None = None,
        error_type: str | None = None,
    ) -> bool:
        """Release a matching backend call, ignoring stale completion callbacks.

        A timed-out worker can finish after its goal has already completed.  Its
        token is still accepted and clears the busy gate, but it cannot change
        that goal's terminal decision.
        """

        if latency_ms is not None:
            latency_ms = _nonnegative_finite("latency_ms", latency_ms)
        if error_type is not None and not isinstance(error_type, str):
            raise TypeError("error_type must be a string or None")

        with self._lock:
            if not self._backend_busy or token != self._active_inference_token:
                return False
            self._backend_busy = False
            self._active_inference_token = None
            if latency_ms is not None:
                self._last_inference_latency_ms = latency_ms
            if error_type is not None:
                self._last_policy_error = error_type
            if not self._active_goal:
                self._state = RuntimeState.IDLE
            return True

    def record_policy_error(self, error_type: str) -> None:
        """Record a sanitized exception or validation category for diagnostics."""

        if not isinstance(error_type, str):
            raise TypeError("error_type must be a string")
        with self._lock:
            self._last_policy_error = error_type

    def record_observation(self, *, valid: bool) -> None:
        """Record local receipt and freshness of the latest joint-state message."""

        if not isinstance(valid, bool):
            raise TypeError("valid must be a bool")
        with self._lock:
            self._joint_state_received = True
            self._joint_state_valid = valid
            if valid:
                self._last_valid_joint_state_at = self._now()

    def has_valid_observation(self) -> bool:
        """Return whether at least one valid joint state has ever been received."""

        with self._lock:
            return self._last_valid_joint_state_at is not None

    def observation_age_seconds(self) -> float | None:
        """Return local monotonic age of the last valid joint state."""

        with self._lock:
            if self._last_valid_joint_state_at is None:
                return None
            now = self._now()
            return max(0.0, now - self._last_valid_joint_state_at)

    def observation_is_stale(self) -> bool:
        """Return true only when an existing valid observation is too old."""

        age = self.observation_age_seconds()
        return age is not None and age > self._joint_state_timeout_seconds

    def claim_observation_timeout(self, episode_id: str) -> TerminationDecision | None:
        """Atomically fault a live goal that never obtained a valid observation.

        The timeout begins with execution, not reservation.  A valid observation
        recorded before this lock is acquired prevents the claim; an observation
        recorded after a winning claim cannot replace its terminal decision.
        """

        with self._lock:
            if (
                not self._owns_live_goal(episode_id)
                or self._goal_started_at is None
                or self._last_valid_joint_state_at is not None
            ):
                return None
            now = self._now()
            if now - self._goal_started_at < self._joint_state_timeout_seconds:
                return None
            return self._claim_termination_unlocked(
                episode_id,
                status=TerminationStatus.ABORTED,
                reason="observation_timeout",
                issue_hold=True,
                now=now,
            )

    def claim_stale_observation(self, episode_id: str) -> TerminationDecision | None:
        """Atomically fault a live goal whose last valid observation is stale."""

        with self._lock:
            if not self._owns_live_goal(episode_id) or self._last_valid_joint_state_at is None:
                return None
            now = self._now()
            if now - self._last_valid_joint_state_at <= self._joint_state_timeout_seconds:
                return None
            return self._claim_termination_unlocked(
                episode_id,
                status=TerminationStatus.ABORTED,
                reason="stale_observation",
                issue_hold=True,
                now=now,
            )

    def claim_termination(
        self,
        episode_id: str,
        *,
        status: TerminationStatus,
        reason: str,
        issue_hold: bool,
    ) -> TerminationDecision | None:
        """Atomically choose one terminal outcome for a goal.

        Exactly one caller receives a decision.  Losing success/fault/cancel
        races receive ``None`` and therefore must not mutate the ROS Action
        state, publish a hold, or produce a second result.
        """

        if not isinstance(status, TerminationStatus):
            raise TypeError("status must be a TerminationStatus")
        if not isinstance(reason, str) or not reason:
            raise ValueError("reason must be a non-empty string")
        if not isinstance(issue_hold, bool):
            raise TypeError("issue_hold must be a bool")
        with self._lock:
            if not self._active_goal or episode_id != self._episode_id:
                return None
            if self._termination is not None:
                return None
            return self._claim_termination_unlocked(
                episode_id,
                status=status,
                reason=reason,
                issue_hold=issue_hold,
                now=self._now(),
            )

    def termination_decision(self, episode_id: str) -> TerminationDecision | None:
        """Return the immutable winning decision for an episode, if one exists.

        This lets an execute callback observe a decision first claimed by a
        concurrent cancellation callback without attempting a second claim.
        """

        with self._lock:
            if self._termination is None or self._termination.episode_id != episode_id:
                return None
            return self._termination

    def begin_safe_stop_attempt(self, episode_id: str) -> bool:
        """Atomically grant the one hold-publication attempt for a goal.

        Callers must acquire this permission *before* publishing.  This makes
        the at-most-once guarantee cover the side effect itself rather than
        merely deduplicating diagnostics after multiple publications.
        """

        with self._lock:
            if (
                self._termination is None
                or self._termination.episode_id != episode_id
                or not self._termination.issue_hold
                or self._safe_stop_attempt_started_for_goal
            ):
                return False
            self._safe_stop_attempt_started_for_goal = True
            return True

    def finish_safe_stop_attempt(
        self,
        episode_id: str,
        published: bool,
        reason: str,
    ) -> bool:
        """Commit the result of a previously claimed hold attempt.

        ``safe_stop_count`` counts successfully published hold-position
        commands.  A failed attempt can instead record a reason such as
        ``safe_stop_unavailable_no_valid_state`` without fabricating a command.
        """

        if not isinstance(published, bool):
            raise TypeError("published must be a bool")
        if not isinstance(reason, str) or not reason:
            raise ValueError("reason must be a non-empty string")

        with self._lock:
            if (
                self._termination is None
                or self._termination.episode_id != episode_id
                or not self._termination.issue_hold
                or not self._safe_stop_attempt_started_for_goal
                or self._safe_stop_attempt_finished_for_goal
            ):
                return False
            self._safe_stop_attempt_finished_for_goal = True
            self._last_safe_stop_reason = reason
            self._last_safe_stop_succeeded = published
            if published:
                self._safe_stop_count += 1
            return True

    def record_safe_stop_outcome(
        self,
        episode_id: str,
        *,
        published: bool,
        reason: str,
    ) -> bool:
        """Claim and finish a hold attempt for compatibility with pure callers."""

        if not self.begin_safe_stop_attempt(episode_id):
            return False
        return self.finish_safe_stop_attempt(
            episode_id,
            published=published,
            reason=reason,
        )

    def complete_goal(self, episode_id: str) -> bool:
        """Release a terminal goal and settle to idle unless its worker is busy."""

        with self._lock:
            if not self._active_goal or episode_id != self._episode_id or self._termination is None:
                return False
            self._active_goal = False
            self._goal_started_at = None
            self._goal_deadline = None
            self._state = RuntimeState.FAULTED if self._backend_busy else RuntimeState.IDLE
            return True

    def snapshot(self) -> RuntimeSnapshot:
        """Return a self-consistent immutable diagnostics snapshot."""

        with self._lock:
            now = self._now()
            if self._last_valid_joint_state_at is None:
                joint_state_age_ms = None
            else:
                joint_state_age_ms = max(0.0, now - self._last_valid_joint_state_at) * 1000.0

            if self._active_goal and self._termination is None:
                goal_elapsed_ms = self._elapsed_ms(now)
            else:
                goal_elapsed_ms = self._last_goal_elapsed_ms

            return RuntimeSnapshot(
                runtime_state=self._state,
                active_goal=self._active_goal,
                episode_id=self._episode_id,
                last_termination_reason=self._last_termination_reason,
                goal_elapsed_ms=goal_elapsed_ms,
                safe_stop_count=self._safe_stop_count,
                last_safe_stop_reason=self._last_safe_stop_reason,
                last_safe_stop_succeeded=self._last_safe_stop_succeeded,
                joint_state_received=self._joint_state_received,
                joint_state_valid=self._joint_state_valid,
                joint_state_age_ms=joint_state_age_ms,
                joint_state_timeout_ms=self._joint_state_timeout_seconds * 1000.0,
                backend_name=self._backend_name,
                backend_busy=self._backend_busy,
                last_inference_latency_ms=self._last_inference_latency_ms,
                inference_timeout_ms=self._inference_timeout_seconds * 1000.0,
                last_policy_error=self._last_policy_error,
            )

    def _admission_unlocked(self) -> GoalAdmission:
        if self._active_goal:
            return GoalAdmission.REJECTED_ACTIVE_GOAL
        if self._backend_busy:
            return GoalAdmission.REJECTED_BACKEND_BUSY
        return GoalAdmission.ACCEPTED

    def _reserve_unlocked(self) -> None:
        self._active_goal = True
        self._episode_id = ""
        self._goal_started_at = None
        self._goal_deadline = None
        self._termination = None
        self._safe_stop_attempt_started_for_goal = False
        self._safe_stop_attempt_finished_for_goal = False
        self._state = RuntimeState.WAITING_FOR_OBSERVATION

    def _start_reserved_unlocked(
        self, episode_id: str, goal_timeout: float, started_at: float
    ) -> None:
        self._episode_id = episode_id
        self._goal_started_at = started_at
        self._goal_deadline = started_at + goal_timeout if goal_timeout > 0.0 else None

    def _set_active_state(self, episode_id: str, state: RuntimeState) -> bool:
        with self._lock:
            if not self._owns_live_goal(episode_id):
                return False
            self._state = state
            return True

    def _claim_termination_unlocked(
        self,
        episode_id: str,
        *,
        status: TerminationStatus,
        reason: str,
        issue_hold: bool,
        now: float,
    ) -> TerminationDecision:
        elapsed_ms = self._elapsed_ms(now)
        decision = TerminationDecision(
            episode_id=episode_id,
            status=status,
            reason=reason,
            issue_hold=issue_hold,
            elapsed_ms=elapsed_ms,
        )
        self._termination = decision
        self._last_termination_reason = reason
        self._last_goal_elapsed_ms = elapsed_ms
        self._state = (
            RuntimeState.FAULTED if status is TerminationStatus.ABORTED else RuntimeState.STOPPING
        )
        return decision

    def _owns_live_goal(self, episode_id: str) -> bool:
        return self._active_goal and episode_id == self._episode_id and self._termination is None

    def _elapsed_ms(self, now: float) -> float:
        if self._goal_started_at is None:
            return 0.0 if self._active_goal else self._last_goal_elapsed_ms
        return max(0.0, now - self._goal_started_at) * 1000.0

    def _now(self) -> float:
        now = self._clock()
        if isinstance(now, bool) or not isinstance(now, (int, float)):
            raise TypeError("clock must return a real number")
        numeric = float(now)
        if not math.isfinite(numeric):
            raise ValueError("clock must return a finite value")
        return numeric


def _positive_finite(name: str, value: object) -> float:
    numeric = _finite_real(name, value)
    if numeric <= 0.0:
        raise ValueError(f"{name} must be greater than zero")
    return numeric


def _validate_episode_id(episode_id: object) -> None:
    if not isinstance(episode_id, str) or not episode_id:
        raise ValueError("episode_id must be a non-empty string")


def _nonnegative_finite(name: str, value: object) -> float:
    numeric = _finite_real(name, value)
    if numeric < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return numeric


def _finite_real(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a real number")
    numeric = float(value)
    if not math.isfinite(numeric):
        raise ValueError(f"{name} must be finite")
    return numeric
