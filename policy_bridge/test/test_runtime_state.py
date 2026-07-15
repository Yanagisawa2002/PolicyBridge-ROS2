"""Unit tests for the ROS-independent runtime state machine."""

from __future__ import annotations

import threading
from dataclasses import FrozenInstanceError

import pytest
from policy_bridge.runtime_state import (
    GoalAdmission,
    RuntimeState,
    RuntimeStateMachine,
    TerminationStatus,
)


class FakeClock:
    """Manually advanced monotonic clock for deterministic deadline tests."""

    def __init__(self, start: float = 100.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def make_runtime(clock: FakeClock | None = None) -> RuntimeStateMachine:
    """Build a runtime with explicit test timeout values."""

    return RuntimeStateMachine(
        backend_name="scripted",
        inference_timeout_seconds=1.25,
        joint_state_timeout_seconds=0.5,
        clock=clock or FakeClock(),
    )


def test_initial_snapshot_contains_complete_diagnostics_inputs() -> None:
    """The idle snapshot exposes every field required by M1 diagnostics."""

    runtime = make_runtime()

    snapshot = runtime.snapshot()

    assert snapshot.runtime_state is RuntimeState.IDLE
    assert snapshot.active_goal is False
    assert snapshot.episode_id == ""
    assert snapshot.last_termination_reason == ""
    assert snapshot.goal_elapsed_ms == 0.0
    assert snapshot.safe_stop_count == 0
    assert snapshot.joint_state_received is False
    assert snapshot.joint_state_valid is False
    assert snapshot.joint_state_age_ms is None
    assert snapshot.joint_state_timeout_ms == 500.0
    assert snapshot.backend_name == "scripted"
    assert snapshot.backend_busy is False
    assert snapshot.last_inference_latency_ms is None
    assert snapshot.inference_timeout_ms == 1250.0
    assert snapshot.last_policy_error == ""
    with pytest.raises(FrozenInstanceError):
        snapshot.active_goal = True  # type: ignore[misc]


@pytest.mark.parametrize(
    ("field", "value", "exception"),
    [
        ("backend_name", "", ValueError),
        ("inference_timeout_seconds", 0.0, ValueError),
        ("inference_timeout_seconds", float("inf"), ValueError),
        ("joint_state_timeout_seconds", -1.0, ValueError),
        ("joint_state_timeout_seconds", True, TypeError),
    ],
)
def test_runtime_configuration_is_validated(
    field: str, value: object, exception: type[Exception]
) -> None:
    """Invalid names and timeout settings fail before runtime use."""

    arguments: dict[str, object] = {
        "backend_name": "scripted",
        "inference_timeout_seconds": 1.0,
        "joint_state_timeout_seconds": 1.0,
    }
    arguments[field] = value

    with pytest.raises(exception):
        RuntimeStateMachine(**arguments)  # type: ignore[arg-type]


@pytest.mark.parametrize("timeout", [-1.0, float("nan"), float("inf"), True])
def test_invalid_goal_timeout_does_not_change_runtime(timeout: object) -> None:
    """Negative, non-finite, and boolean goal timeouts are rejected atomically."""

    runtime = make_runtime()

    with pytest.raises((TypeError, ValueError)):
        runtime.begin_goal("episode-invalid", timeout_seconds=timeout)  # type: ignore[arg-type]

    assert runtime.snapshot().runtime_state is RuntimeState.IDLE
    assert runtime.snapshot().active_goal is False


def test_reserved_goal_deadline_starts_only_when_execution_starts() -> None:
    """ROS admission reserves a slot without consuming the goal timeout budget."""

    clock = FakeClock()
    runtime = make_runtime(clock)

    assert runtime.reserve_goal() is GoalAdmission.ACCEPTED
    reservation = runtime.snapshot()
    assert reservation.active_goal is True
    assert reservation.episode_id == ""
    assert reservation.runtime_state is RuntimeState.WAITING_FOR_OBSERVATION

    clock.advance(25.0)
    assert runtime.snapshot().goal_elapsed_ms == 0.0
    assert runtime.start_reserved_goal("episode-reserved", timeout_seconds=2.0) is True
    clock.advance(0.25)
    assert runtime.start_reserved_goal("episode-reserved", timeout_seconds=99.0) is True
    assert runtime.start_reserved_goal("episode-rebind", timeout_seconds=2.0) is False
    assert runtime.remaining_goal_seconds("episode-reserved") == 1.75
    assert runtime.snapshot().goal_elapsed_ms == 250.0

    clock.advance(0.25)
    assert runtime.remaining_goal_seconds("episode-reserved") == 1.5
    assert runtime.snapshot().goal_elapsed_ms == 500.0


def test_reservation_rejects_a_second_active_goal() -> None:
    """A request-only ROS callback still atomically gates concurrent goals."""

    runtime = make_runtime()

    assert runtime.reserve_goal() is GoalAdmission.ACCEPTED
    assert runtime.reserve_goal() is GoalAdmission.REJECTED_ACTIVE_GOAL
    assert (
        runtime.begin_goal("episode-too-late", timeout_seconds=1.0)
        is GoalAdmission.REJECTED_ACTIVE_GOAL
    )


def test_unbound_reservation_can_be_abandoned_but_bound_goal_cannot() -> None:
    """A failed admission handoff can release only a still-unbound slot."""

    runtime = make_runtime()

    assert runtime.reserve_goal() is GoalAdmission.ACCEPTED
    assert runtime.abandon_reservation() is True
    assert runtime.abandon_reservation() is False
    assert runtime.snapshot().runtime_state is RuntimeState.IDLE
    assert runtime.snapshot().active_goal is False

    assert runtime.reserve_goal() is GoalAdmission.ACCEPTED
    assert runtime.start_reserved_goal("episode-bound", timeout_seconds=1.0) is True
    assert runtime.abandon_reservation() is False
    assert runtime.snapshot().active_goal is True


def test_bound_start_is_idempotent_after_early_cancel_claim() -> None:
    """Execute can join an episode already bound and canceled by its callback."""

    runtime = make_runtime()
    assert runtime.reserve_goal() is GoalAdmission.ACCEPTED
    assert runtime.start_reserved_goal("episode-early-cancel", timeout_seconds=1.0)
    decision = runtime.claim_termination(
        "episode-early-cancel",
        status=TerminationStatus.CANCELED,
        reason="goal_canceled",
        issue_hold=True,
    )
    assert decision is not None

    assert runtime.start_reserved_goal("episode-early-cancel", timeout_seconds=1.0)
    assert runtime.termination_decision("episode-early-cancel") is decision


def test_goal_lifetime_and_monotonic_deadline() -> None:
    """State, elapsed time, and deadline share the injected monotonic clock."""

    clock = FakeClock()
    runtime = make_runtime(clock)

    assert runtime.begin_goal("episode-1", timeout_seconds=2.0).accepted
    assert runtime.snapshot().runtime_state is RuntimeState.WAITING_FOR_OBSERVATION
    assert runtime.remaining_goal_seconds("episode-1") == 2.0
    assert runtime.goal_timed_out("episode-1") is False

    clock.advance(0.75)
    assert runtime.mark_running("episode-1") is True
    assert runtime.snapshot().runtime_state is RuntimeState.RUNNING
    assert runtime.snapshot().goal_elapsed_ms == 750.0
    assert runtime.remaining_goal_seconds("episode-1") == 1.25

    clock.advance(1.25)
    assert runtime.goal_timed_out("episode-1") is True

    decision = runtime.claim_termination(
        "episode-1",
        status=TerminationStatus.ABORTED,
        reason="goal_timeout",
        issue_hold=True,
    )
    assert decision is not None
    assert decision.success is False
    assert decision.elapsed_ms == 2000.0
    assert runtime.complete_goal("episode-1") is True

    snapshot = runtime.snapshot()
    assert snapshot.runtime_state is RuntimeState.IDLE
    assert snapshot.active_goal is False
    assert snapshot.episode_id == "episode-1"
    assert snapshot.last_termination_reason == "goal_timeout"
    assert snapshot.goal_elapsed_ms == 2000.0


def test_zero_goal_timeout_disables_deadline() -> None:
    """A zero timeout leaves max-steps and other fault guards in control."""

    clock = FakeClock()
    runtime = make_runtime(clock)
    assert runtime.begin_goal("episode-no-deadline", timeout_seconds=0.0).accepted

    clock.advance(10_000.0)

    assert runtime.remaining_goal_seconds("episode-no-deadline") is None
    assert runtime.goal_timed_out("episode-no-deadline") is False


def test_goal_admission_is_thread_safe() -> None:
    """Concurrent callbacks can acquire at most one active-goal slot."""

    runtime = make_runtime()
    worker_count = 8
    barrier = threading.Barrier(worker_count)
    results: list[GoalAdmission] = []
    results_lock = threading.Lock()

    def attempt(index: int) -> None:
        barrier.wait(timeout=2.0)
        result = runtime.reserve_goal()
        with results_lock:
            results.append(result)

    threads = [threading.Thread(target=attempt, args=(index,)) for index in range(worker_count)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=2.0)

    assert all(not thread.is_alive() for thread in threads)
    assert results.count(GoalAdmission.ACCEPTED) == 1
    assert results.count(GoalAdmission.REJECTED_ACTIVE_GOAL) == worker_count - 1


def test_termination_is_first_wins_across_threads() -> None:
    """A success/timeout race produces one immutable terminal decision."""

    runtime = make_runtime()
    assert runtime.begin_goal("episode-race", timeout_seconds=1.0).accepted
    barrier = threading.Barrier(2)
    decisions = []
    decisions_lock = threading.Lock()

    def claim(status: TerminationStatus, reason: str) -> None:
        barrier.wait(timeout=2.0)
        decision = runtime.claim_termination(
            "episode-race",
            status=status,
            reason=reason,
            issue_hold=status is TerminationStatus.ABORTED,
        )
        with decisions_lock:
            decisions.append(decision)

    success_thread = threading.Thread(
        target=claim, args=(TerminationStatus.SUCCEEDED, "goal_reached")
    )
    timeout_thread = threading.Thread(
        target=claim, args=(TerminationStatus.ABORTED, "goal_timeout")
    )
    success_thread.start()
    timeout_thread.start()
    success_thread.join(timeout=2.0)
    timeout_thread.join(timeout=2.0)

    winners = [decision for decision in decisions if decision is not None]
    assert len(winners) == 1
    assert runtime.termination_decision("another-episode") is None
    assert runtime.termination_decision("episode-race") is winners[0]
    assert runtime.snapshot().last_termination_reason == winners[0].reason
    assert runtime.complete_goal("episode-race") is True
    assert runtime.complete_goal("episode-race") is False


def test_backend_busy_survives_timed_out_goal_and_gates_admission() -> None:
    """A late worker keeps new goals out until its matching callback finishes."""

    runtime = make_runtime()
    assert runtime.begin_goal("episode-slow", timeout_seconds=5.0).accepted
    token = runtime.begin_inference("episode-slow")
    assert token is not None
    assert runtime.begin_inference("episode-slow") is None

    decision = runtime.claim_termination(
        "episode-slow",
        status=TerminationStatus.ABORTED,
        reason="policy_inference_timeout",
        issue_hold=True,
    )
    assert decision is not None
    assert runtime.complete_goal("episode-slow") is True
    assert runtime.snapshot().runtime_state is RuntimeState.FAULTED
    assert runtime.snapshot().backend_busy is True
    assert runtime.reserve_goal() is GoalAdmission.REJECTED_BACKEND_BUSY

    assert runtime.finish_inference(token + 1, latency_ms=1500.0) is False
    assert runtime.snapshot().backend_busy is True
    assert runtime.finish_inference(token, latency_ms=1500.0) is True
    assert runtime.snapshot().runtime_state is RuntimeState.IDLE
    assert runtime.snapshot().last_inference_latency_ms == 1500.0
    assert runtime.begin_goal("episode-next", timeout_seconds=1.0).accepted


def test_safe_stop_outcome_is_recorded_at_most_once_per_goal() -> None:
    """Only the winning termination can add one successful hold publication."""

    runtime = make_runtime()
    assert runtime.begin_goal("episode-hold", timeout_seconds=1.0).accepted
    decision = runtime.claim_termination(
        "episode-hold",
        status=TerminationStatus.CANCELED,
        reason="goal_canceled",
        issue_hold=True,
    )
    assert decision is not None

    assert runtime.record_safe_stop_outcome("episode-hold", published=True, reason="goal_canceled")
    assert not runtime.record_safe_stop_outcome("episode-hold", published=True, reason="duplicate")

    snapshot = runtime.snapshot()
    assert snapshot.safe_stop_count == 1
    assert snapshot.last_safe_stop_reason == "goal_canceled"
    assert snapshot.last_safe_stop_succeeded is True


def test_safe_stop_attempt_is_claimed_before_publish_and_is_not_reentrant() -> None:
    """Only one callback receives permission for the external publish side effect."""

    runtime = make_runtime()
    assert runtime.begin_goal("episode-hold-race", timeout_seconds=1.0).accepted
    assert runtime.claim_termination(
        "episode-hold-race",
        status=TerminationStatus.ABORTED,
        reason="goal_timeout",
        issue_hold=True,
    )

    assert runtime.begin_safe_stop_attempt("episode-hold-race") is True
    assert runtime.begin_safe_stop_attempt("episode-hold-race") is False
    assert runtime.finish_safe_stop_attempt(
        "episode-hold-race", published=True, reason="goal_timeout"
    )
    assert not runtime.finish_safe_stop_attempt(
        "episode-hold-race", published=True, reason="duplicate"
    )
    assert runtime.snapshot().safe_stop_count == 1


def test_concurrent_safe_stop_claim_has_one_publish_owner() -> None:
    """Concurrent termination handlers grant one hold publisher before I/O."""

    runtime = make_runtime()
    assert runtime.begin_goal("episode-hold-threads", timeout_seconds=1.0).accepted
    assert runtime.claim_termination(
        "episode-hold-threads",
        status=TerminationStatus.CANCELED,
        reason="goal_canceled",
        issue_hold=True,
    )
    barrier = threading.Barrier(6)
    claims: list[bool] = []
    claims_lock = threading.Lock()

    def attempt() -> None:
        barrier.wait(timeout=2.0)
        claimed = runtime.begin_safe_stop_attempt("episode-hold-threads")
        with claims_lock:
            claims.append(claimed)

    threads = [threading.Thread(target=attempt) for _ in range(6)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=2.0)

    assert all(not thread.is_alive() for thread in threads)
    assert claims.count(True) == 1
    assert claims.count(False) == 5
    assert runtime.finish_safe_stop_attempt(
        "episode-hold-threads", published=True, reason="goal_canceled"
    )
    assert runtime.snapshot().safe_stop_count == 1


def test_unavailable_safe_stop_records_reason_without_incrementing_count() -> None:
    """Missing observation is diagnosed without fabricating a hold command."""

    runtime = make_runtime()
    assert runtime.begin_goal("episode-no-state", timeout_seconds=1.0).accepted
    assert runtime.claim_termination(
        "episode-no-state",
        status=TerminationStatus.ABORTED,
        reason="observation_timeout",
        issue_hold=True,
    )

    assert runtime.record_safe_stop_outcome(
        "episode-no-state",
        published=False,
        reason="safe_stop_unavailable_no_valid_state",
    )

    snapshot = runtime.snapshot()
    assert snapshot.safe_stop_count == 0
    assert snapshot.last_safe_stop_reason == "safe_stop_unavailable_no_valid_state"
    assert snapshot.last_safe_stop_succeeded is False


def test_non_hold_termination_cannot_record_safe_stop() -> None:
    """A pre-motion unsupported instruction does not authorize hold publication."""

    runtime = make_runtime()
    assert runtime.begin_goal("episode-unsupported", timeout_seconds=1.0).accepted
    assert runtime.claim_termination(
        "episode-unsupported",
        status=TerminationStatus.ABORTED,
        reason="unsupported_instruction",
        issue_hold=False,
    )

    assert not runtime.record_safe_stop_outcome(
        "episode-unsupported", published=True, reason="unsupported_instruction"
    )
    assert runtime.snapshot().safe_stop_count == 0


def test_observation_snapshot_tracks_last_valid_freshness() -> None:
    """Invalid messages do not refresh the last known valid observation."""

    clock = FakeClock()
    runtime = make_runtime(clock)

    runtime.record_observation(valid=False)
    snapshot = runtime.snapshot()
    assert snapshot.joint_state_received is True
    assert snapshot.joint_state_valid is False
    assert snapshot.joint_state_age_ms is None
    assert runtime.has_valid_observation() is False
    assert runtime.observation_is_stale() is False

    runtime.record_observation(valid=True)
    clock.advance(0.4)
    runtime.record_observation(valid=False)
    snapshot = runtime.snapshot()
    assert snapshot.joint_state_valid is False
    assert snapshot.joint_state_age_ms == pytest.approx(400.0)
    assert runtime.has_valid_observation() is True
    assert runtime.observation_is_stale() is False

    clock.advance(0.11)
    assert runtime.observation_age_seconds() == pytest.approx(0.51)
    assert runtime.observation_is_stale() is True


def test_observation_timeout_claim_checks_deadline_and_fresh_record_atomically() -> None:
    """A valid state recorded first prevents the no-observation fault claim."""

    clock = FakeClock()
    runtime = make_runtime(clock)
    assert runtime.begin_goal("episode-observation-fresh", timeout_seconds=2.0).accepted

    clock.advance(0.49)
    assert runtime.claim_observation_timeout("episode-observation-fresh") is None
    clock.advance(0.01)
    runtime.record_observation(valid=True)

    assert runtime.claim_observation_timeout("episode-observation-fresh") is None
    assert runtime.termination_decision("episode-observation-fresh") is None


def test_observation_timeout_claim_wins_before_late_valid_record() -> None:
    """A state arriving after the atomic timeout cannot rewrite termination."""

    clock = FakeClock()
    runtime = make_runtime(clock)
    assert runtime.begin_goal("episode-observation-timeout", timeout_seconds=2.0).accepted
    clock.advance(0.5)

    decision = runtime.claim_observation_timeout("episode-observation-timeout")
    assert decision is not None
    assert decision.status is TerminationStatus.ABORTED
    assert decision.reason == "observation_timeout"
    assert decision.issue_hold is True

    runtime.record_observation(valid=True)
    assert runtime.termination_decision("episode-observation-timeout") is decision


def test_stale_observation_claim_uses_strict_timeout_boundary() -> None:
    """An observation is stale only after, not exactly at, the threshold."""

    clock = FakeClock()
    runtime = make_runtime(clock)
    runtime.record_observation(valid=True)
    assert runtime.begin_goal("episode-stale-boundary", timeout_seconds=2.0).accepted

    clock.advance(0.5)
    assert runtime.claim_stale_observation("episode-stale-boundary") is None
    clock.advance(0.001)
    decision = runtime.claim_stale_observation("episode-stale-boundary")

    assert decision is not None
    assert decision.status is TerminationStatus.ABORTED
    assert decision.reason == "stale_observation"
    assert decision.issue_hold is True


def test_fresh_record_wins_stale_race_and_prevents_claim() -> None:
    """Refreshing under the runtime lock makes the following stale claim lose."""

    clock = FakeClock()
    runtime = make_runtime(clock)
    runtime.record_observation(valid=True)
    assert runtime.begin_goal("episode-stale-refresh", timeout_seconds=2.0).accepted
    clock.advance(0.51)

    runtime.record_observation(valid=True)

    assert runtime.claim_stale_observation("episode-stale-refresh") is None
    assert runtime.termination_decision("episode-stale-refresh") is None


def test_stale_claim_wins_before_late_fresh_record() -> None:
    """A refresh after stale termination leaves the first-wins decision intact."""

    clock = FakeClock()
    runtime = make_runtime(clock)
    runtime.record_observation(valid=True)
    assert runtime.begin_goal("episode-stale-winner", timeout_seconds=2.0).accepted
    clock.advance(0.51)

    decision = runtime.claim_stale_observation("episode-stale-winner")
    assert decision is not None
    runtime.record_observation(valid=True)

    assert runtime.termination_decision("episode-stale-winner") is decision
    assert runtime.snapshot().last_termination_reason == "stale_observation"


def test_policy_error_is_sanitized_diagnostics_data() -> None:
    """The model stores an exception category, not traceback or ROS objects."""

    runtime = make_runtime()
    runtime.record_policy_error("RuntimeError")

    assert runtime.snapshot().last_policy_error == "RuntimeError"

    with pytest.raises(TypeError):
        runtime.record_policy_error(123)  # type: ignore[arg-type]
