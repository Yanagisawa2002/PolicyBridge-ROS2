"""ROS-independent multimodal observation values and image conversion.

ROS callbacks are responsible for extracting primitive header and image fields
before calling this module.  Keeping that boundary explicit prevents policy
backends from depending on ROS message classes and makes validation fully unit
testable.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Integral
from typing import Final

import numpy as np
import numpy.typing as npt

JOINT_COUNT: Final[int] = 6
DEFAULT_MAX_IMAGE_PIXELS: Final[int] = 2_073_600
"""Default image limit: 1920 x 1080 pixels."""

MAX_CONFIGURABLE_IMAGE_PIXELS: Final[int] = 16_777_216
"""Hard configuration ceiling: 4096 x 4096 pixels."""

JointVector = npt.NDArray[np.float64]
RgbImage = npt.NDArray[np.uint8]


class ImageValidationError(ValueError):
    """Raised when primitive ``sensor_msgs/Image`` fields are inconsistent."""


@dataclass(frozen=True, slots=True)
class ValidatedImageLayout:
    """Normalized image layout metadata produced without an output allocation."""

    height: int
    width: int
    encoding: str
    step: int
    pixel_count: int
    required_bytes: int


@dataclass(frozen=True, slots=True)
class ObservationSnapshot:
    """One validated policy observation with immutable, owned array storage.

    Stamp fields use nanoseconds in their respective clock domains.  The joint
    and image stamps are source header timestamps used for synchronization;
    ``received_monotonic_ns`` is local monotonic time used for freshness.
    ``sequence_id`` identifies accepted snapshots and starts at one.
    """

    sequence_id: int
    joint_positions: JointVector
    joint_names: tuple[str, ...]
    rgb: RgbImage | None
    joint_stamp_ns: int
    image_stamp_ns: int | None
    received_monotonic_ns: int
    synchronization_skew_ms: float | None
    image_frame_id: str | None

    def __post_init__(self) -> None:
        """Validate all fields and replace arrays with read-only owned copies."""

        sequence_id = _validated_integer(
            self.sequence_id,
            name="sequence_id",
            minimum=1,
        )
        joint_stamp_ns = _validated_integer(
            self.joint_stamp_ns,
            name="joint_stamp_ns",
            minimum=0,
        )
        received_monotonic_ns = _validated_integer(
            self.received_monotonic_ns,
            name="received_monotonic_ns",
            minimum=0,
        )
        joint_positions = _validated_joint_positions(self.joint_positions)
        joint_names = _validated_joint_names(self.joint_names)

        object.__setattr__(self, "sequence_id", sequence_id)
        object.__setattr__(self, "joint_stamp_ns", joint_stamp_ns)
        object.__setattr__(self, "received_monotonic_ns", received_monotonic_ns)
        object.__setattr__(self, "joint_positions", joint_positions)
        object.__setattr__(self, "joint_names", joint_names)

        if self.rgb is None:
            if self.image_stamp_ns is not None:
                raise ValueError("image_stamp_ns must be None when rgb is None")
            if self.synchronization_skew_ms is not None:
                raise ValueError("synchronization_skew_ms must be None when rgb is None")
            if self.image_frame_id is not None:
                raise ValueError("image_frame_id must be None when rgb is None")
            return

        rgb = _validated_rgb(self.rgb)
        if joint_stamp_ns == 0:
            raise ValueError("joint_stamp_ns must be positive when rgb is present")
        if self.image_stamp_ns is None:
            raise ValueError("image_stamp_ns is required when rgb is present")
        image_stamp_ns = _validated_integer(
            self.image_stamp_ns,
            name="image_stamp_ns",
            minimum=1,
        )
        skew_ms = _validated_skew(self.synchronization_skew_ms)
        image_frame_id = _validated_frame_id(self.image_frame_id)

        calculated_skew_ms = abs(joint_stamp_ns - image_stamp_ns) / 1_000_000.0
        if not math.isclose(skew_ms, calculated_skew_ms, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError(
                "synchronization_skew_ms must equal the absolute header stamp "
                f"difference ({calculated_skew_ms} ms)"
            )

        object.__setattr__(self, "rgb", rgb)
        object.__setattr__(self, "image_stamp_ns", image_stamp_ns)
        object.__setattr__(self, "synchronization_skew_ms", skew_ms)
        object.__setattr__(self, "image_frame_id", image_frame_id)


def image_data_to_rgb(
    *,
    height: int,
    width: int,
    encoding: str,
    step: int,
    data: object,
    max_pixels: int = DEFAULT_MAX_IMAGE_PIXELS,
) -> RgbImage:
    """Validate primitive image fields and return an owned RGB ``uint8`` copy.

    Only ``rgb8`` and ``bgr8`` are supported.  Row padding is discarded.  All
    dimension and allocation-bound checks happen before the source buffer is
    viewed or an output array is allocated.

    Args:
        height: Number of image rows.
        width: Number of image columns.
        encoding: Exactly ``rgb8`` or ``bgr8``.
        step: Source bytes per row, including optional padding.
        data: A contiguous object supporting Python's buffer protocol.
        max_pixels: Maximum accepted ``height * width``.

    Returns:
        A C-contiguous, owned array with shape ``(height, width, 3)`` and RGB
        channel order.

    Raises:
        TypeError: If a scalar field or ``data`` has the wrong type.
        ImageValidationError: If the image fields are invalid or inconsistent.
    """

    layout, source_bytes = _validated_image_layout_and_buffer(
        height=height,
        width=width,
        encoding=encoding,
        step=step,
        data=data,
        max_pixels=max_pixels,
    )
    packed_row_bytes = layout.width * 3
    rows = np.frombuffer(
        source_bytes,
        dtype=np.uint8,
        count=layout.required_bytes,
    ).reshape(
        layout.height,
        layout.step,
    )
    packed = rows[:, :packed_row_bytes].reshape(
        layout.height,
        layout.width,
        3,
    )
    if layout.encoding == "bgr8":
        packed = packed[:, :, ::-1]

    # ``np.array(..., copy=True)`` is intentional even for already contiguous
    # RGB input: no returned view may alias a mutable ROS message buffer.
    return np.array(packed, dtype=np.uint8, order="C", copy=True)


def validate_image_layout(
    *,
    height: int,
    width: int,
    encoding: str,
    step: int,
    data: object,
    max_pixels: int = DEFAULT_MAX_IMAGE_PIXELS,
) -> ValidatedImageLayout:
    """Validate image metadata and buffer coverage without allocating pixels.

    This is intended for raw-image diagnostics before a synchronized pair is
    available.  It performs the exact checks used by :func:`image_data_to_rgb`
    but allocates no NumPy output array.
    """

    layout, _ = _validated_image_layout_and_buffer(
        height=height,
        width=width,
        encoding=encoding,
        step=step,
        data=data,
        max_pixels=max_pixels,
    )
    return layout


def _validated_image_layout_and_buffer(
    *,
    height: int,
    width: int,
    encoding: str,
    step: int,
    data: object,
    max_pixels: int,
) -> tuple[ValidatedImageLayout, memoryview]:
    """Return normalized layout plus a zero-copy byte view of valid input."""

    normalized_height = _validated_image_integer(height, name="height", minimum=1)
    normalized_width = _validated_image_integer(width, name="width", minimum=1)
    normalized_step = _validated_image_integer(step, name="step", minimum=0)
    normalized_max_pixels = _validated_image_integer(
        max_pixels,
        name="max_image_pixels",
        minimum=1,
    )
    if normalized_max_pixels > MAX_CONFIGURABLE_IMAGE_PIXELS:
        raise ImageValidationError(
            "max_image_pixels must not exceed "
            f"{MAX_CONFIGURABLE_IMAGE_PIXELS}; got {normalized_max_pixels}"
        )

    if not isinstance(encoding, str):
        raise TypeError("image encoding must be a string")
    if encoding not in ("rgb8", "bgr8"):
        raise ImageValidationError(
            f"unsupported image encoding {encoding!r}; expected 'rgb8' or 'bgr8'"
        )

    pixel_count = normalized_height * normalized_width
    if pixel_count > normalized_max_pixels:
        raise ImageValidationError(
            f"image pixel count {pixel_count} exceeds max_image_pixels {normalized_max_pixels}"
        )

    packed_row_bytes = normalized_width * 3
    if normalized_step < packed_row_bytes:
        raise ImageValidationError(
            f"image step must be at least width * 3 ({packed_row_bytes}); got {normalized_step}"
        )

    required_bytes = normalized_step * normalized_height
    try:
        source = memoryview(data)
    except TypeError as exc:
        raise TypeError("image data must support the contiguous buffer protocol") from exc
    try:
        source_bytes = source.cast("B")
    except (TypeError, ValueError) as exc:
        raise TypeError("image data must be a contiguous byte buffer") from exc

    if source_bytes.nbytes < required_bytes:
        raise ImageValidationError(
            f"image data is truncated: requires at least {required_bytes} bytes; "
            f"got {source_bytes.nbytes}"
        )

    return (
        ValidatedImageLayout(
            height=normalized_height,
            width=normalized_width,
            encoding=encoding,
            step=normalized_step,
            pixel_count=pixel_count,
            required_bytes=required_bytes,
        ),
        source_bytes,
    )


def _validated_integer(value: object, *, name: str, minimum: int) -> int:
    """Return an integral field while rejecting booleans and negative values."""

    if isinstance(value, bool) or not isinstance(value, Integral):
        raise TypeError(f"{name} must be an integer")
    normalized = int(value)
    if normalized < minimum:
        qualifier = "positive" if minimum == 1 else "non-negative"
        raise ValueError(f"{name} must be {qualifier}")
    return normalized


def _validated_image_integer(value: object, *, name: str, minimum: int) -> int:
    """Validate an integral image field using image-specific error types."""

    if isinstance(value, bool) or not isinstance(value, Integral):
        raise TypeError(f"image {name} must be an integer")
    normalized = int(value)
    if normalized < minimum:
        qualifier = "positive" if minimum == 1 else "non-negative"
        raise ImageValidationError(f"image {name} must be {qualifier}")
    return normalized


def _validated_joint_positions(value: object) -> JointVector:
    """Return a finite read-only ``float64`` six-vector with owned storage."""

    try:
        candidate = np.asarray(value)
    except (TypeError, ValueError) as exc:
        raise TypeError("joint_positions must be a real numeric array-like value") from exc
    if not (
        np.issubdtype(candidate.dtype, np.number)
        and not np.issubdtype(candidate.dtype, np.complexfloating)
        and not np.issubdtype(candidate.dtype, np.bool_)
    ):
        raise TypeError("joint_positions must contain only real numeric values")
    if candidate.shape != (JOINT_COUNT,):
        raise ValueError(f"joint_positions must have shape ({JOINT_COUNT},); got {candidate.shape}")
    with np.errstate(over="ignore", invalid="ignore"):
        positions = np.array(candidate, dtype=np.float64, order="C", copy=True)
    if not np.all(np.isfinite(positions)):
        raise ValueError("joint_positions must contain only finite values")
    positions.flags.writeable = False
    return positions


def _validated_joint_names(value: object) -> tuple[str, ...]:
    """Return six non-empty, unique joint names as an immutable tuple."""

    if isinstance(value, (str, bytes)):
        raise TypeError("joint_names must be an iterable of strings")
    try:
        names = tuple(value)  # type: ignore[arg-type]
    except TypeError as exc:
        raise TypeError("joint_names must be an iterable of strings") from exc
    if len(names) != JOINT_COUNT:
        raise ValueError(f"joint_names must contain {JOINT_COUNT} names; got {len(names)}")
    if any(not isinstance(name, str) for name in names):
        raise TypeError("joint_names must contain only strings")
    if any(not name for name in names):
        raise ValueError("joint_names must not contain empty strings")
    if len(set(names)) != JOINT_COUNT:
        raise ValueError("joint_names must be unique")
    return names


def _validated_rgb(value: object) -> RgbImage:
    """Return an owned, read-only RGB image without implicit dtype conversion."""

    if not isinstance(value, np.ndarray):
        raise TypeError("rgb must be a numpy array")
    if value.dtype != np.uint8:
        raise TypeError(f"rgb must have dtype uint8; got {value.dtype}")
    if value.ndim != 3 or value.shape[2:] != (3,):
        raise ValueError(f"rgb must have shape (height, width, 3); got {value.shape}")
    if value.shape[0] <= 0 or value.shape[1] <= 0:
        raise ValueError("rgb height and width must be positive")
    pixel_count = value.shape[0] * value.shape[1]
    if pixel_count > MAX_CONFIGURABLE_IMAGE_PIXELS:
        raise ValueError(
            f"rgb pixel count must not exceed {MAX_CONFIGURABLE_IMAGE_PIXELS}; got {pixel_count}"
        )
    rgb = np.array(value, dtype=np.uint8, order="C", copy=True)
    rgb.flags.writeable = False
    return rgb


def _validated_skew(value: object) -> float:
    """Return a finite, non-negative synchronization skew in milliseconds."""

    if isinstance(value, bool) or not isinstance(value, (int, float, np.number)):
        raise TypeError("synchronization_skew_ms must be a finite number")
    skew_ms = float(value)
    if not math.isfinite(skew_ms) or skew_ms < 0.0:
        raise ValueError("synchronization_skew_ms must be finite and non-negative")
    return skew_ms


def _validated_frame_id(value: object) -> str:
    """Return a non-empty image frame identifier."""

    if not isinstance(value, str):
        raise TypeError("image_frame_id must be a string when rgb is present")
    if not value:
        raise ValueError("image_frame_id must not be empty when rgb is present")
    return value
