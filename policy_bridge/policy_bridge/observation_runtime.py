"""Thread-safe M2 observation health, sequence, and fault bookkeeping."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Final

from .observation import ObservationSnapshot

JOINT_ONLY: Final[str] = "joint_only"
RGB_JOINT: Final[str] = "rgb_joint"
OBSERVATION_MODES: Final[frozenset[str]] = frozenset((JOINT_ONLY, RGB_JOINT))
MAX_SYNC_QUEUE_SIZE: Final[int] = 100
MAX_SYNC_SLOP_SECONDS: Final[float] = 1.0


@dataclass(frozen=True, slots=True)
class ObservationHealthSnapshot:
    """Immutable diagnostics view of image and synchronization health."""

    observation_mode: str
    image_required: bool
    image_received: bool
    image_valid: bool
    image_age_ms: float | None
    image_timeout_ms: float
    encoding: str
    width: int
    height: int
    frame_id: str
    current_image_error: str
    last_image_error: str
    synchronized_snapshot_available: bool
    snapshot_sequence_id: int
    snapshot_age_ms: float | None
    last_sync_skew_ms: float | None
    sync_slop_ms: float
    sync_queue_size: int
    synchronized_observation_timeout_ms: float
    current_sync_error: str
    last_sync_error: str


@dataclass(frozen=True, slots=True)
class ObservationEpoch:
    """Opaque, immutable baseline for one goal or synchronized-snapshot wait."""

    snapshot_sequence_id: int
    _joint_arrival_epoch: int
    _valid_image_arrival_epoch: int
    _invalid_image_arrival_epoch: int
    _store_identity: object


class ObservationStore:
    """Own the latest immutable snapshot and bounded multimodal health metadata.

    ROS callbacks perform message validation outside this class.  The store
    records only primitive metadata and :class:`ObservationSnapshot` values.
    Its lock never calls ROS or user policy code.
    """

    def __init__(
        self,
        *,
        observation_mode: str,
        image_timeout_seconds: float,
        synchronized_observation_timeout_seconds: float,
        sync_queue_size: int,
        sync_slop_seconds: float,
        now: Callable[[], float] = time.monotonic,
    ) -> None:
        self._observation_mode = validate_observation_mode(observation_mode)
        self._image_timeout_seconds = _positive_finite(
            "image_timeout_seconds", image_timeout_seconds
        )
        self._synchronized_observation_timeout_seconds = _positive_finite(
            "synchronized_observation_timeout_seconds",
            synchronized_observation_timeout_seconds,
        )
        self._sync_queue_size = validate_sync_queue_size(sync_queue_size)
        self._sync_slop_seconds = validate_sync_slop_seconds(sync_slop_seconds)
        self._now = now
        self._lock = threading.RLock()

        self._next_sequence_id = 1
        self._latest_snapshot: ObservationSnapshot | None = None
        self._last_snapshot_received_at: float | None = None

        self._image_received = False
        self._image_valid = False
        self._last_image_received_at: float | None = None
        self._last_valid_image_at: float | None = None
        self._encoding = ""
        self._width = 0
        self._height = 0
        self._frame_id = ""
        self._current_image_error = ""
        self._last_image_error = ""

        self._last_joint_stamp_ns: int | None = None
        self._last_image_stamp_ns: int | None = None
        self._joint_arrival_epoch = 0
        self._valid_image_arrival_epoch = 0
        self._invalid_image_arrival_epoch = 0
        self._epoch_identity = object()
        self._last_sync_skew_ms: float | None = None
        self._current_sync_error = ""
        self._last_sync_error = ""

    @property
    def observation_mode(self) -> str:
        return self._observation_mode

    @property
    def image_required(self) -> bool:
        return self._observation_mode == RGB_JOINT

    @property
    def image_timeout_seconds(self) -> float:
        return self._image_timeout_seconds

    @property
    def synchronized_observation_timeout_seconds(self) -> float:
        return self._synchronized_observation_timeout_seconds

    @property
    def sync_queue_size(self) -> int:
        return self._sync_queue_size

    @property
    def sync_slop_seconds(self) -> float:
        return self._sync_slop_seconds

    def capture_epoch(self) -> ObservationEpoch:
        """Capture an atomic baseline for goal-local and wait-local decisions.

        The returned value is deliberately tied to this store.  It contains
        only bounded state metadata, never a ROS message or image payload.
        """

        with self._lock:
            latest_sequence = (
                0 if self._latest_snapshot is None else self._latest_snapshot.sequence_id
            )
            return ObservationEpoch(
                snapshot_sequence_id=latest_sequence,
                _joint_arrival_epoch=self._joint_arrival_epoch,
                _valid_image_arrival_epoch=self._valid_image_arrival_epoch,
                _invalid_image_arrival_epoch=self._invalid_image_arrival_epoch,
                _store_identity=self._epoch_identity,
            )

    def reserve_sequence_id(self) -> int:
        """Reserve a strictly increasing ID without holding a lock during image copies."""

        with self._lock:
            sequence_id = self._next_sequence_id
            self._next_sequence_id += 1
            return sequence_id

    def commit_snapshot(self, snapshot: ObservationSnapshot) -> bool:
        """Atomically publish a newer snapshot; stale concurrent commits lose."""

        if not isinstance(snapshot, ObservationSnapshot):
            raise TypeError("snapshot must be an ObservationSnapshot")
        if self.image_required and snapshot.rgb is None:
            raise ValueError("rgb_joint mode requires RGB snapshots")
        if not self.image_required and snapshot.rgb is not None:
            raise ValueError("joint_only mode does not accept RGB snapshots")

        with self._lock:
            if (
                self._latest_snapshot is not None
                and snapshot.sequence_id <= self._latest_snapshot.sequence_id
            ):
                return False
            self._latest_snapshot = snapshot
            self._last_snapshot_received_at = self._now()
            self._next_sequence_id = max(self._next_sequence_id, snapshot.sequence_id + 1)
            if snapshot.rgb is not None:
                self._last_sync_skew_ms = snapshot.synchronization_skew_ms
                self._current_sync_error = ""
            return True

    def latest_snapshot(self, *, after_sequence_id: int = 0) -> ObservationSnapshot | None:
        """Return the latest immutable value only when it is newer than the gate."""

        if isinstance(after_sequence_id, bool) or not isinstance(after_sequence_id, int):
            raise TypeError("after_sequence_id must be an integer")
        if after_sequence_id < 0:
            raise ValueError("after_sequence_id must be non-negative")
        with self._lock:
            snapshot = self._latest_snapshot
            if snapshot is None or snapshot.sequence_id <= after_sequence_id:
                return None
            return snapshot

    def record_joint_stamp(self, stamp_ns: int) -> None:
        """Record a validated source header stamp for synchronization diagnostics."""

        normalized = _nonnegative_integer("joint_stamp_ns", stamp_ns)
        with self._lock:
            self._last_joint_stamp_ns = normalized
            if normalized > 0:
                self._joint_arrival_epoch += 1
            self._update_candidate_skew_unlocked()

    def record_image(
        self,
        *,
        valid: bool,
        stamp_ns: int | None,
        encoding: str,
        width: int,
        height: int,
        frame_id: str,
        error: str = "",
    ) -> None:
        """Record one raw image arrival after bounded layout validation."""

        if not isinstance(valid, bool):
            raise TypeError("valid must be a bool")
        if stamp_ns is not None:
            stamp_ns = _nonnegative_integer("image_stamp_ns", stamp_ns)
        if any(not isinstance(value, str) for value in (encoding, frame_id, error)):
            raise TypeError("image text metadata must be strings")
        normalized_width = _nonnegative_integer("image width", width)
        normalized_height = _nonnegative_integer("image height", height)
        if valid and (stamp_ns is None or stamp_ns <= 0):
            raise ValueError("a valid synchronized image requires a positive header stamp")
        if valid and error:
            raise ValueError("a valid image cannot include a validation error")
        if not valid and not error:
            raise ValueError("an invalid image must include a validation error")

        with self._lock:
            now = self._now()
            self._image_received = True
            self._image_valid = valid
            self._last_image_received_at = now
            self._encoding = encoding
            self._width = normalized_width
            self._height = normalized_height
            self._frame_id = frame_id
            if valid:
                self._last_valid_image_at = now
                self._last_image_stamp_ns = stamp_ns
                self._valid_image_arrival_epoch += 1
                self._current_image_error = ""
                self._update_candidate_skew_unlocked()
            else:
                self._invalid_image_arrival_epoch += 1
                self._current_image_error = error
                self._last_image_error = error

    def record_sync_error(self, error: str, *, skew_ms: float | None = None) -> None:
        """Retain a compact reason for the most recent rejected sync candidate."""

        if not isinstance(error, str) or not error:
            raise ValueError("sync error must be a non-empty string")
        if skew_ms is not None:
            skew_ms = _nonnegative_finite("skew_ms", skew_ms)
        with self._lock:
            self._current_sync_error = error
            self._last_sync_error = error
            if skew_ms is not None:
                self._last_sync_skew_ms = skew_ms

    def image_fault_reason(
        self,
        *,
        goal_elapsed_seconds: float,
        goal_epoch: ObservationEpoch,
    ) -> str | None:
        """Classify missing, invalid, or stale image state without claiming termination."""

        elapsed = _nonnegative_finite("goal_elapsed_seconds", goal_elapsed_seconds)
        self._validate_epoch(goal_epoch)
        if not self.image_required:
            return None
        with self._lock:
            now = self._now()
            invalid_image_received_for_goal = (
                self._invalid_image_arrival_epoch > goal_epoch._invalid_image_arrival_epoch
            )
            if self._last_valid_image_at is None:
                if elapsed < self._image_timeout_seconds:
                    return None
                return (
                    "invalid_image"
                    if invalid_image_received_for_goal and not self._image_valid
                    else "image_timeout"
                )

            valid_age = max(0.0, now - self._last_valid_image_at)
            if valid_age < self._image_timeout_seconds:
                return None
            invalid_frame_is_recent = (
                not self._image_valid
                and invalid_image_received_for_goal
                and bool(self._current_image_error)
                and self._last_image_received_at is not None
                and max(0.0, now - self._last_image_received_at) < self._image_timeout_seconds
            )
            return "invalid_image" if invalid_frame_is_recent else "stale_image"

    def sync_fault_reason(
        self,
        *,
        wait_elapsed_seconds: float,
        required_sequence_id: int,
        wait_epoch: ObservationEpoch,
    ) -> str | None:
        """Return sync timeout only when fresh valid inputs exist but no newer pair does."""

        wait_elapsed = _nonnegative_finite("wait_elapsed_seconds", wait_elapsed_seconds)
        required_sequence = _nonnegative_integer("required_sequence_id", required_sequence_id)
        self._validate_epoch(wait_epoch)
        if not self.image_required:
            return None
        with self._lock:
            if (
                self._latest_snapshot is not None
                and self._latest_snapshot.sequence_id > required_sequence
            ):
                return None
            if (
                self._valid_image_arrival_epoch <= wait_epoch._valid_image_arrival_epoch
                or self._joint_arrival_epoch <= wait_epoch._joint_arrival_epoch
            ):
                return None
            image_age = max(0.0, self._now() - self._last_valid_image_at)
            if image_age >= self._image_timeout_seconds:
                return None
            if wait_elapsed < self._synchronized_observation_timeout_seconds:
                return None
            return "observation_sync_timeout"

    def snapshot(self) -> ObservationHealthSnapshot:
        """Return a self-consistent immutable diagnostics snapshot."""

        with self._lock:
            now = self._now()
            image_age_ms = (
                None
                if self._last_valid_image_at is None
                else max(0.0, now - self._last_valid_image_at) * 1000.0
            )
            snapshot_age_ms = (
                None
                if self._last_snapshot_received_at is None
                else max(0.0, now - self._last_snapshot_received_at) * 1000.0
            )
            latest_sequence = (
                0 if self._latest_snapshot is None else self._latest_snapshot.sequence_id
            )
            return ObservationHealthSnapshot(
                observation_mode=self._observation_mode,
                image_required=self.image_required,
                image_received=self._image_received,
                image_valid=self._image_valid,
                image_age_ms=image_age_ms,
                image_timeout_ms=self._image_timeout_seconds * 1000.0,
                encoding=self._encoding,
                width=self._width,
                height=self._height,
                frame_id=self._frame_id,
                current_image_error=self._current_image_error,
                last_image_error=self._last_image_error,
                synchronized_snapshot_available=self._latest_snapshot is not None,
                snapshot_sequence_id=latest_sequence,
                snapshot_age_ms=snapshot_age_ms,
                last_sync_skew_ms=self._last_sync_skew_ms,
                sync_slop_ms=self._sync_slop_seconds * 1000.0,
                sync_queue_size=self._sync_queue_size,
                synchronized_observation_timeout_ms=(
                    self._synchronized_observation_timeout_seconds * 1000.0
                ),
                current_sync_error=self._current_sync_error,
                last_sync_error=self._last_sync_error,
            )

    def _update_candidate_skew_unlocked(self) -> None:
        if self._last_joint_stamp_ns is None or self._last_image_stamp_ns is None:
            return
        if self._last_joint_stamp_ns <= 0 or self._last_image_stamp_ns <= 0:
            return
        skew_ms = abs(self._last_joint_stamp_ns - self._last_image_stamp_ns) / 1_000_000.0
        self._last_sync_skew_ms = skew_ms
        if skew_ms > self._sync_slop_seconds * 1000.0:
            self._current_sync_error = "timestamp_skew_exceeds_sync_slop"
            self._last_sync_error = "timestamp_skew_exceeds_sync_slop"
        else:
            self._current_sync_error = ""

    def _validate_epoch(self, epoch: ObservationEpoch) -> None:
        if not isinstance(epoch, ObservationEpoch):
            raise TypeError("observation epoch must be an ObservationEpoch")
        if epoch._store_identity is not self._epoch_identity:
            raise ValueError("observation epoch belongs to a different store")


def validate_observation_mode(value: object) -> str:
    if not isinstance(value, str):
        raise TypeError("observation_mode must be a string")
    if value not in OBSERVATION_MODES:
        allowed = ", ".join(sorted(OBSERVATION_MODES))
        raise ValueError(f"observation_mode must be one of: {allowed}")
    return value


def validate_backend_observation_mode(
    backend_selector: object,
    observation_mode: object,
) -> tuple[str, str]:
    """Validate the only mode-specific backend restriction in M2."""

    mode = validate_observation_mode(observation_mode)
    if not isinstance(backend_selector, str):
        raise TypeError("policy_backend must be a string")
    if not backend_selector:
        raise ValueError("policy_backend must not be empty")
    if mode == JOINT_ONLY and backend_selector == "multimodal_scripted":
        raise ValueError("multimodal_scripted requires observation_mode=rgb_joint")
    return backend_selector, mode


def validate_sync_queue_size(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError("sync_queue_size must be an integer")
    if value < 2 or value > MAX_SYNC_QUEUE_SIZE:
        raise ValueError(f"sync_queue_size must be between 2 and {MAX_SYNC_QUEUE_SIZE}")
    return value


def validate_sync_slop_seconds(value: object) -> float:
    numeric = _nonnegative_finite("sync_slop_seconds", value)
    if numeric > MAX_SYNC_SLOP_SECONDS:
        raise ValueError(f"sync_slop_seconds must not exceed {MAX_SYNC_SLOP_SECONDS}")
    return numeric


def _positive_finite(name: str, value: object) -> float:
    numeric = _nonnegative_finite(name, value)
    if numeric <= 0.0:
        raise ValueError(f"{name} must be greater than zero")
    return numeric


def _nonnegative_finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a real number")
    numeric = float(value)
    if not math.isfinite(numeric) or numeric < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return numeric


def _nonnegative_integer(name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an integer")
    if value < 0:
        raise ValueError(f"{name} must be non-negative")
    return value
