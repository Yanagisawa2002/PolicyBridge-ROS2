"""Pure tests for immutable observations and bounded RGB conversion."""

from __future__ import annotations

from dataclasses import FrozenInstanceError

import numpy as np
import pytest
from policy_bridge.observation import (
    DEFAULT_MAX_IMAGE_PIXELS,
    MAX_CONFIGURABLE_IMAGE_PIXELS,
    ImageValidationError,
    ObservationSnapshot,
    ValidatedImageLayout,
    image_data_to_rgb,
    validate_image_layout,
)

JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))


def _joint_only_snapshot(**overrides: object) -> ObservationSnapshot:
    values: dict[str, object] = {
        "sequence_id": 1,
        "joint_positions": np.arange(6, dtype=np.float64),
        "joint_names": JOINT_NAMES,
        "rgb": None,
        "joint_stamp_ns": 1_000_000_000,
        "image_stamp_ns": None,
        "received_monotonic_ns": 8_000_000_000,
        "synchronization_skew_ms": None,
        "image_frame_id": None,
    }
    values.update(overrides)
    return ObservationSnapshot(**values)  # type: ignore[arg-type]


def _rgb_joint_snapshot(**overrides: object) -> ObservationSnapshot:
    values: dict[str, object] = {
        "sequence_id": 2,
        "joint_positions": np.arange(6, dtype=np.float64),
        "joint_names": JOINT_NAMES,
        "rgb": np.arange(24, dtype=np.uint8).reshape(2, 4, 3),
        "joint_stamp_ns": 1_020_000_000,
        "image_stamp_ns": 1_000_000_000,
        "received_monotonic_ns": 8_100_000_000,
        "synchronization_skew_ms": 20.0,
        "image_frame_id": "camera_rgb_optical_frame",
    }
    values.update(overrides)
    return ObservationSnapshot(**values)  # type: ignore[arg-type]


def test_joint_only_snapshot_has_owned_read_only_float64_joints() -> None:
    """Joint-only snapshots contain no image metadata and normalize joints."""

    source = np.arange(6, dtype=np.float32)
    snapshot = _joint_only_snapshot(joint_positions=source)

    assert snapshot.sequence_id == 1
    assert snapshot.joint_names == JOINT_NAMES
    assert snapshot.joint_positions.shape == (6,)
    assert snapshot.joint_positions.dtype == np.float64
    assert snapshot.joint_positions.flags.c_contiguous
    assert not snapshot.joint_positions.flags.writeable
    assert snapshot.rgb is None
    assert snapshot.image_stamp_ns is None
    assert snapshot.synchronization_skew_ms is None
    assert snapshot.image_frame_id is None
    assert snapshot.joint_stamp_ns == 1_000_000_000
    assert snapshot.received_monotonic_ns == 8_000_000_000


def test_rgb_joint_snapshot_has_validated_read_only_rgb_and_timing() -> None:
    """RGB snapshots preserve nanosecond stamps and millisecond skew."""

    snapshot = _rgb_joint_snapshot()

    assert snapshot.sequence_id == 2
    assert snapshot.rgb is not None
    assert snapshot.rgb.shape == (2, 4, 3)
    assert snapshot.rgb.dtype == np.uint8
    assert snapshot.rgb.flags.c_contiguous
    assert not snapshot.rgb.flags.writeable
    assert snapshot.image_stamp_ns == 1_000_000_000
    assert snapshot.joint_stamp_ns == 1_020_000_000
    assert snapshot.synchronization_skew_ms == 20.0
    assert snapshot.image_frame_id == "camera_rgb_optical_frame"


def test_snapshot_defensively_copies_arrays_and_freezes_fields() -> None:
    """Neither source mutation nor caller writes can alter an accepted snapshot."""

    joints = np.arange(6, dtype=np.float64)
    rgb = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
    snapshot = _rgb_joint_snapshot(joint_positions=joints, rgb=rgb)
    expected_joints = joints.copy()
    expected_rgb = rgb.copy()

    joints[:] = -1.0
    rgb[:] = 255

    np.testing.assert_array_equal(snapshot.joint_positions, expected_joints)
    assert snapshot.rgb is not None
    np.testing.assert_array_equal(snapshot.rgb, expected_rgb)
    with pytest.raises(ValueError, match="read-only"):
        snapshot.joint_positions[0] = 10.0
    with pytest.raises(ValueError, match="read-only"):
        snapshot.rgb[0, 0, 0] = 10
    with pytest.raises(FrozenInstanceError):
        snapshot.sequence_id = 10  # type: ignore[misc]


@pytest.mark.parametrize(
    ("positions", "error_type", "message"),
    [
        (np.zeros(5), ValueError, r"shape \(6,\)"),
        (np.zeros((6, 1)), ValueError, r"shape \(6,\)"),
        (np.asarray([0, 1, 2, 3, 4, np.nan]), ValueError, "finite"),
        (np.asarray([0, 1, 2, 3, 4, np.inf]), ValueError, "finite"),
        (np.asarray([True] * 6), TypeError, "real numeric"),
        (np.asarray(["0"] * 6), TypeError, "real numeric"),
    ],
)
def test_snapshot_rejects_invalid_joint_vectors(
    positions: np.ndarray,
    error_type: type[Exception],
    message: str,
) -> None:
    """Snapshot construction enforces six finite real joint positions."""

    with pytest.raises(error_type, match=message):
        _joint_only_snapshot(joint_positions=positions)


@pytest.mark.parametrize(
    ("rgb", "error_type", "message"),
    [
        (np.zeros((2, 3), dtype=np.uint8), ValueError, "shape"),
        (np.zeros((2, 3, 4), dtype=np.uint8), ValueError, "shape"),
        (np.zeros((0, 3, 3), dtype=np.uint8), ValueError, "positive"),
        (np.zeros((2, 3, 3), dtype=np.float32), TypeError, "dtype uint8"),
        ([[[0, 0, 0]]], TypeError, "numpy array"),
    ],
)
def test_snapshot_rejects_invalid_rgb(
    rgb: object,
    error_type: type[Exception],
    message: str,
) -> None:
    """RGB storage is exactly positive H x W x 3 uint8."""

    with pytest.raises(error_type, match=message):
        _rgb_joint_snapshot(rgb=rgb)


@pytest.mark.parametrize(
    ("overrides", "error_type", "message"),
    [
        ({"sequence_id": 0}, ValueError, "sequence_id must be positive"),
        ({"sequence_id": True}, TypeError, "sequence_id must be an integer"),
        ({"joint_stamp_ns": -1}, ValueError, "joint_stamp_ns must be non-negative"),
        (
            {"received_monotonic_ns": -1},
            ValueError,
            "received_monotonic_ns must be non-negative",
        ),
        ({"joint_names": JOINT_NAMES[:-1]}, ValueError, "contain 6 names"),
        ({"joint_names": ("joint_1",) * 6}, ValueError, "unique"),
    ],
)
def test_snapshot_rejects_invalid_sequence_joint_metadata_and_times(
    overrides: dict[str, object],
    error_type: type[Exception],
    message: str,
) -> None:
    """Identifiers, names, and local/source nanosecond fields are strict."""

    with pytest.raises(error_type, match=message):
        _joint_only_snapshot(**overrides)


@pytest.mark.parametrize(
    ("overrides", "error_type", "message"),
    [
        ({"image_stamp_ns": None}, ValueError, "image_stamp_ns is required"),
        ({"image_stamp_ns": -1}, ValueError, "image_stamp_ns must be positive"),
        ({"image_stamp_ns": 0}, ValueError, "image_stamp_ns must be positive"),
        (
            {"joint_stamp_ns": 0, "synchronization_skew_ms": 1_000.0},
            ValueError,
            "joint_stamp_ns must be positive",
        ),
        ({"synchronization_skew_ms": None}, TypeError, "finite number"),
        ({"synchronization_skew_ms": -1.0}, ValueError, "non-negative"),
        ({"synchronization_skew_ms": np.nan}, ValueError, "finite"),
        ({"synchronization_skew_ms": 19.0}, ValueError, "absolute header stamp"),
        ({"image_frame_id": None}, TypeError, "must be a string"),
        ({"image_frame_id": ""}, ValueError, "must not be empty"),
    ],
)
def test_rgb_snapshot_rejects_inconsistent_image_metadata(
    overrides: dict[str, object],
    error_type: type[Exception],
    message: str,
) -> None:
    """RGB metadata is present and internally consistent with header stamps."""

    with pytest.raises(error_type, match=message):
        _rgb_joint_snapshot(**overrides)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("image_stamp_ns", 1, "must be None"),
        ("synchronization_skew_ms", 0.0, "must be None"),
        ("image_frame_id", "camera", "must be None"),
    ],
)
def test_joint_only_snapshot_rejects_orphaned_image_metadata(
    field: str,
    value: object,
    message: str,
) -> None:
    """Joint-only snapshots cannot imply that an absent RGB image exists."""

    with pytest.raises(ValueError, match=message):
        _joint_only_snapshot(**{field: value})


def test_rgb8_conversion_returns_owned_contiguous_rgb() -> None:
    """Packed rgb8 bytes retain channel order and never alias their source."""

    source = bytearray([1, 2, 3, 4, 5, 6])
    rgb = image_data_to_rgb(
        height=1,
        width=2,
        encoding="rgb8",
        step=6,
        data=source,
    )
    source[:] = bytes([255] * 6)

    assert rgb.shape == (1, 2, 3)
    assert rgb.dtype == np.uint8
    assert rgb.flags.c_contiguous
    assert rgb.flags.owndata
    np.testing.assert_array_equal(rgb, [[[1, 2, 3], [4, 5, 6]]])


def test_layout_validation_reuses_conversion_checks_without_pixel_output() -> None:
    """Raw callbacks can validate padded buffers without constructing RGB."""

    layout = validate_image_layout(
        height=2,
        width=2,
        encoding="rgb8",
        step=8,
        data=bytes(16),
    )

    assert layout == ValidatedImageLayout(
        height=2,
        width=2,
        encoding="rgb8",
        step=8,
        pixel_count=4,
        required_bytes=16,
    )


def test_bgr8_conversion_reverses_channels() -> None:
    """bgr8 is the only conversion and is normalized to RGB order."""

    rgb = image_data_to_rgb(
        height=1,
        width=2,
        encoding="bgr8",
        step=6,
        data=bytes([3, 2, 1, 30, 20, 10]),
    )

    np.testing.assert_array_equal(rgb, [[[1, 2, 3], [10, 20, 30]]])


def test_image_conversion_discards_each_rows_padding() -> None:
    """Step padding is handled per row rather than included as pixel data."""

    rgb = image_data_to_rgb(
        height=2,
        width=2,
        encoding="rgb8",
        step=8,
        data=bytes(
            [
                1,
                2,
                3,
                4,
                5,
                6,
                99,
                98,
                7,
                8,
                9,
                10,
                11,
                12,
                97,
                96,
            ]
        ),
    )

    np.testing.assert_array_equal(
        rgb,
        [
            [[1, 2, 3], [4, 5, 6]],
            [[7, 8, 9], [10, 11, 12]],
        ],
    )


@pytest.mark.parametrize("encoding", ["mono8", "rgba8", "RGB8", "", "yuv422"])
def test_image_conversion_rejects_unsupported_encoding(encoding: str) -> None:
    """Image encodings are an exact allow-list and are never guessed."""

    with pytest.raises(ImageValidationError, match="unsupported image encoding"):
        image_data_to_rgb(
            height=1,
            width=1,
            encoding=encoding,
            step=3,
            data=b"\x00\x00\x00",
        )


@pytest.mark.parametrize(("height", "width"), [(0, 1), (1, 0), (-1, 1), (1, -1)])
def test_image_conversion_rejects_zero_or_negative_dimensions(
    height: int,
    width: int,
) -> None:
    """Non-positive declarations fail before buffer conversion."""

    with pytest.raises(ImageValidationError, match="must be positive"):
        image_data_to_rgb(
            height=height,
            width=width,
            encoding="rgb8",
            step=3,
            data=b"",
        )


def test_image_conversion_rejects_step_smaller_than_packed_row() -> None:
    """A row must contain every declared RGB pixel."""

    with pytest.raises(ImageValidationError, match=r"width \* 3 \(6\)"):
        image_data_to_rgb(
            height=1,
            width=2,
            encoding="rgb8",
            step=5,
            data=bytes(6),
        )


def test_image_conversion_rejects_truncated_data() -> None:
    """Data covers all padded rows, not merely the packed pixel count."""

    with pytest.raises(
        ImageValidationError,
        match="requires at least 16 bytes; got 15",
    ):
        image_data_to_rgb(
            height=2,
            width=2,
            encoding="rgb8",
            step=8,
            data=bytes(15),
        )


def test_image_conversion_rejects_pixel_limit_before_touching_data() -> None:
    """Oversized declarations cannot trigger source conversion or allocation."""

    with pytest.raises(ImageValidationError, match="pixel count 6 exceeds"):
        image_data_to_rgb(
            height=2,
            width=3,
            encoding="rgb8",
            step=9,
            data=object(),
            max_pixels=5,
        )


@pytest.mark.parametrize("max_pixels", [0, -1])
def test_image_conversion_rejects_non_positive_max_pixels(max_pixels: int) -> None:
    """The conversion allocation bound itself must be meaningful."""

    with pytest.raises(ImageValidationError, match="max_image_pixels must be positive"):
        image_data_to_rgb(
            height=1,
            width=1,
            encoding="rgb8",
            step=3,
            data=bytes(3),
            max_pixels=max_pixels,
        )


def test_image_conversion_rejects_unbounded_max_pixel_configuration() -> None:
    """The public hard ceiling prevents disabling allocation protection."""

    with pytest.raises(ImageValidationError, match="must not exceed"):
        image_data_to_rgb(
            height=1,
            width=1,
            encoding="rgb8",
            step=3,
            data=bytes(3),
            max_pixels=MAX_CONFIGURABLE_IMAGE_PIXELS + 1,
        )


def test_default_image_limit_is_full_hd() -> None:
    """The documented production default is exactly 1920 x 1080."""

    assert DEFAULT_MAX_IMAGE_PIXELS == 1920 * 1080
