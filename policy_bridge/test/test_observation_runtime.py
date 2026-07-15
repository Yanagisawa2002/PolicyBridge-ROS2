"""Pure tests for M2 observation health and sequence bookkeeping."""

from __future__ import annotations

import math

import numpy as np
import pytest
from policy_bridge.observation import ObservationSnapshot
from policy_bridge.observation_runtime import (
    JOINT_ONLY,
    RGB_JOINT,
    ObservationStore,
    validate_backend_observation_mode,
    validate_observation_mode,
    validate_sync_queue_size,
    validate_sync_slop_seconds,
)

_JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))


class _Clock:
    def __init__(self) -> None:
        self.now = 10.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def _store(clock: _Clock, *, mode: str = RGB_JOINT) -> ObservationStore:
    return ObservationStore(
        observation_mode=mode,
        image_timeout_seconds=1.0,
        synchronized_observation_timeout_seconds=0.5,
        sync_queue_size=10,
        sync_slop_seconds=0.05,
        now=clock,
    )


def _snapshot(sequence_id: int, *, rgb: bool) -> ObservationSnapshot:
    return ObservationSnapshot(
        sequence_id=sequence_id,
        joint_positions=np.arange(6, dtype=np.float64),
        joint_names=_JOINT_NAMES,
        rgb=np.zeros((2, 3, 3), dtype=np.uint8) if rgb else None,
        joint_stamp_ns=1_000_000_000 if rgb else 0,
        image_stamp_ns=1_010_000_000 if rgb else None,
        received_monotonic_ns=12_000_000_000,
        synchronization_skew_ms=10.0 if rgb else None,
        image_frame_id="camera" if rgb else None,
    )


@pytest.mark.parametrize("value", ["", "latest", 1, None])
def test_observation_mode_rejects_unknown_values(value: object) -> None:
    with pytest.raises((TypeError, ValueError)):
        validate_observation_mode(value)


@pytest.mark.parametrize("value", [1, 101, 2.5, True])
def test_sync_queue_size_is_bounded(value: object) -> None:
    with pytest.raises((TypeError, ValueError)):
        validate_sync_queue_size(value)


@pytest.mark.parametrize("value", [-0.1, 1.1, math.inf, math.nan, True])
def test_sync_slop_is_finite_nonnegative_and_bounded(value: object) -> None:
    with pytest.raises((TypeError, ValueError)):
        validate_sync_slop_seconds(value)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("image_timeout_seconds", 0.0),
        ("image_timeout_seconds", -1.0),
        ("synchronized_observation_timeout_seconds", 0.0),
        ("synchronized_observation_timeout_seconds", math.inf),
    ],
)
def test_observation_timeouts_must_be_positive(field: str, value: float) -> None:
    arguments = {
        "observation_mode": RGB_JOINT,
        "image_timeout_seconds": 1.0,
        "synchronized_observation_timeout_seconds": 1.0,
        "sync_queue_size": 10,
        "sync_slop_seconds": 0.05,
        "now": _Clock(),
    }
    arguments[field] = value
    with pytest.raises(ValueError):
        ObservationStore(**arguments)  # type: ignore[arg-type]


def test_multimodal_backend_requires_rgb_joint_mode() -> None:
    with pytest.raises(ValueError, match="requires observation_mode=rgb_joint"):
        validate_backend_observation_mode("multimodal_scripted", JOINT_ONLY)
    assert validate_backend_observation_mode("multimodal_scripted", RGB_JOINT) == (
        "multimodal_scripted",
        RGB_JOINT,
    )
    assert validate_backend_observation_mode("scripted", JOINT_ONLY) == (
        "scripted",
        JOINT_ONLY,
    )


def test_joint_only_sequence_commit_and_gate() -> None:
    clock = _Clock()
    store = _store(clock, mode=JOINT_ONLY)
    first_id = store.reserve_sequence_id()
    second_id = store.reserve_sequence_id()
    assert (first_id, second_id) == (1, 2)

    first = _snapshot(first_id, rgb=False)
    second = _snapshot(second_id, rgb=False)
    assert store.commit_snapshot(first)
    assert store.latest_snapshot(after_sequence_id=0) is first
    assert store.latest_snapshot(after_sequence_id=first_id) is None
    assert store.commit_snapshot(second)
    assert not store.commit_snapshot(first)
    assert store.latest_snapshot(after_sequence_id=first_id) is second


def test_store_rejects_snapshot_for_wrong_mode() -> None:
    clock = _Clock()
    with pytest.raises(ValueError, match="does not accept RGB"):
        _store(clock, mode=JOINT_ONLY).commit_snapshot(_snapshot(1, rgb=True))
    with pytest.raises(ValueError, match="requires RGB"):
        _store(clock, mode=RGB_JOINT).commit_snapshot(_snapshot(1, rgb=False))


def test_missing_and_invalid_image_reasons_are_distinct() -> None:
    clock = _Clock()
    missing = _store(clock)
    missing_epoch = missing.capture_epoch()
    assert (
        missing.image_fault_reason(
            goal_elapsed_seconds=0.99,
            goal_epoch=missing_epoch,
        )
        is None
    )
    assert (
        missing.image_fault_reason(
            goal_elapsed_seconds=1.0,
            goal_epoch=missing_epoch,
        )
        == "image_timeout"
    )

    invalid = _store(clock)
    invalid_epoch = invalid.capture_epoch()
    invalid.record_image(
        valid=False,
        stamp_ns=1,
        encoding="mono8",
        width=2,
        height=2,
        frame_id="camera",
        error="unsupported_encoding",
    )
    assert (
        invalid.image_fault_reason(
            goal_elapsed_seconds=1.0,
            goal_epoch=invalid_epoch,
        )
        == "invalid_image"
    )
    health = invalid.snapshot()
    assert health.image_received
    assert not health.image_valid
    assert health.last_image_error == "unsupported_encoding"


def test_valid_image_that_stops_becomes_stale() -> None:
    clock = _Clock()
    store = _store(clock)
    goal_epoch = store.capture_epoch()
    store.record_image(
        valid=True,
        stamp_ns=1_000_000_000,
        encoding="rgb8",
        width=4,
        height=3,
        frame_id="camera",
    )
    clock.advance(1.0)
    assert (
        store.image_fault_reason(
            goal_elapsed_seconds=1.0,
            goal_epoch=goal_epoch,
        )
        == "stale_image"
    )


def test_recent_invalid_frames_after_valid_image_report_invalid_image() -> None:
    clock = _Clock()
    store = _store(clock)
    goal_epoch = store.capture_epoch()
    store.record_image(
        valid=True,
        stamp_ns=1_000_000_000,
        encoding="rgb8",
        width=4,
        height=3,
        frame_id="camera",
    )
    clock.advance(0.8)
    store.record_image(
        valid=False,
        stamp_ns=1_900_000_000,
        encoding="mono8",
        width=4,
        height=3,
        frame_id="camera",
        error="unsupported_encoding",
    )
    clock.advance(0.2)
    assert (
        store.image_fault_reason(
            goal_elapsed_seconds=2.0,
            goal_epoch=goal_epoch,
        )
        == "invalid_image"
    )


def test_invalid_image_from_prior_goal_does_not_poison_new_goal() -> None:
    clock = _Clock()
    store = _store(clock)
    store.record_image(
        valid=False,
        stamp_ns=1_000_000_000,
        encoding="mono8",
        width=4,
        height=3,
        frame_id="camera",
        error="unsupported_encoding",
    )

    new_goal_epoch = store.capture_epoch()
    clock.advance(1.0)

    assert (
        store.image_fault_reason(
            goal_elapsed_seconds=1.0,
            goal_epoch=new_goal_epoch,
        )
        == "image_timeout"
    )


def test_sync_timeout_requires_fresh_valid_inputs_and_no_new_sequence() -> None:
    clock = _Clock()
    store = _store(clock)
    wait_epoch = store.capture_epoch()
    store.record_joint_stamp(1_000_000_000)
    store.record_image(
        valid=True,
        stamp_ns=1_010_000_000,
        encoding="rgb8",
        width=4,
        height=3,
        frame_id="camera",
    )
    assert (
        store.sync_fault_reason(
            wait_elapsed_seconds=0.49,
            required_sequence_id=0,
            wait_epoch=wait_epoch,
        )
        is None
    )
    assert (
        store.sync_fault_reason(
            wait_elapsed_seconds=0.5,
            required_sequence_id=0,
            wait_epoch=wait_epoch,
        )
        == "observation_sync_timeout"
    )

    snapshot = _snapshot(store.reserve_sequence_id(), rgb=True)
    assert store.commit_snapshot(snapshot)
    assert (
        store.sync_fault_reason(
            wait_elapsed_seconds=1.0,
            required_sequence_id=snapshot.sequence_id - 1,
            wait_epoch=wait_epoch,
        )
        is None
    )


def test_sync_timeout_ignores_raw_arrivals_before_current_wait() -> None:
    clock = _Clock()
    store = _store(clock)
    store.record_joint_stamp(1_000_000_000)
    store.record_image(
        valid=True,
        stamp_ns=1_200_000_000,
        encoding="rgb8",
        width=4,
        height=3,
        frame_id="camera",
    )
    wait_epoch = store.capture_epoch()
    clock.advance(0.5)

    assert (
        store.sync_fault_reason(
            wait_elapsed_seconds=0.5,
            required_sequence_id=wait_epoch.snapshot_sequence_id,
            wait_epoch=wait_epoch,
        )
        is None
    )


@pytest.mark.parametrize("new_stream", ["joint", "image"])
def test_sync_timeout_requires_both_raw_streams_after_wait(new_stream: str) -> None:
    clock = _Clock()
    store = _store(clock)
    wait_epoch = store.capture_epoch()
    if new_stream == "joint":
        store.record_joint_stamp(1_000_000_000)
    else:
        store.record_image(
            valid=True,
            stamp_ns=1_200_000_000,
            encoding="rgb8",
            width=4,
            height=3,
            frame_id="camera",
        )
    clock.advance(0.5)

    assert (
        store.sync_fault_reason(
            wait_elapsed_seconds=0.5,
            required_sequence_id=0,
            wait_epoch=wait_epoch,
        )
        is None
    )


def test_epoch_captures_latest_snapshot_and_is_store_local() -> None:
    clock = _Clock()
    store = _store(clock)
    snapshot = _snapshot(store.reserve_sequence_id(), rgb=True)
    assert store.commit_snapshot(snapshot)
    epoch = store.capture_epoch()
    assert epoch.snapshot_sequence_id == snapshot.sequence_id

    other = _store(clock)
    with pytest.raises(ValueError, match="different store"):
        other.image_fault_reason(goal_elapsed_seconds=1.0, goal_epoch=epoch)


def test_diagnostics_retain_historical_errors_after_valid_recovery() -> None:
    clock = _Clock()
    store = _store(clock)
    store.record_image(
        valid=False,
        stamp_ns=900_000_000,
        encoding="mono8",
        width=4,
        height=3,
        frame_id="camera",
        error="unsupported_encoding",
    )
    store.record_joint_stamp(1_000_000_000)
    store.record_image(
        valid=True,
        stamp_ns=1_200_000_000,
        encoding="bgr8",
        width=4,
        height=3,
        frame_id="camera",
    )
    health = store.snapshot()
    assert health.image_valid
    assert health.current_image_error == ""
    assert health.last_image_error == "unsupported_encoding"
    assert health.last_sync_skew_ms == pytest.approx(200.0)
    assert health.current_sync_error == "timestamp_skew_exceeds_sync_slop"
    assert health.last_sync_error == "timestamp_skew_exceeds_sync_slop"

    synchronized = ObservationSnapshot(
        sequence_id=store.reserve_sequence_id(),
        joint_positions=np.zeros(6),
        joint_names=_JOINT_NAMES,
        rgb=np.zeros((3, 4, 3), dtype=np.uint8),
        joint_stamp_ns=1_000_000_000,
        image_stamp_ns=1_010_000_000,
        received_monotonic_ns=12_000_000_000,
        synchronization_skew_ms=10.0,
        image_frame_id="camera",
    )
    assert store.commit_snapshot(synchronized)
    health = store.snapshot()
    assert health.synchronized_snapshot_available
    assert health.snapshot_sequence_id == synchronized.sequence_id
    assert health.last_sync_skew_ms == pytest.approx(10.0)
    assert health.current_sync_error == ""
    assert health.last_sync_error == "timestamp_skew_exceeds_sync_slop"


def test_joint_only_health_components_are_explicitly_disabled() -> None:
    health = _store(_Clock(), mode=JOINT_ONLY).snapshot()
    assert not health.image_required
    assert not health.image_received
    assert not health.synchronized_snapshot_available
    assert health.snapshot_sequence_id == 0
