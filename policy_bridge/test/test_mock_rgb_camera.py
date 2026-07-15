"""Pure tests for the deterministic M2 mock RGB camera."""

from __future__ import annotations

import numpy as np
import pytest
from policy_bridge.mock_rgb_camera import (
    DEFAULT_FRAME_ID,
    DEFAULT_HEIGHT,
    DEFAULT_PUBLISH_RATE_HZ,
    DEFAULT_TOPIC_NAME,
    DEFAULT_WIDTH,
    MAX_IMAGE_DIMENSION,
    MAX_PUBLISH_RATE_HZ,
    build_frame_payload,
    generate_deterministic_rgb_frame,
    validate_camera_parameters,
)


def test_default_parameters_are_small_bounded_and_explicit() -> None:
    configuration = validate_camera_parameters(
        publish_rate_hz=DEFAULT_PUBLISH_RATE_HZ,
        width=DEFAULT_WIDTH,
        height=DEFAULT_HEIGHT,
        frame_id=DEFAULT_FRAME_ID,
        topic_name=DEFAULT_TOPIC_NAME,
    )

    assert configuration.publish_rate_hz == 20.0
    assert (configuration.width, configuration.height) == (64, 48)
    assert configuration.frame_id == "camera_rgb_optical_frame"
    assert configuration.topic_name == "/camera/rgb/image_raw"


def test_frame_payload_has_exact_rgb8_layout() -> None:
    payload = build_frame_payload(7, width=5, height=3)

    assert payload.height == 3
    assert payload.width == 5
    assert payload.encoding == "rgb8"
    assert payload.is_bigendian == 0
    assert payload.step == 15
    assert len(payload.data) == payload.height * payload.step
    decoded = np.frombuffer(payload.data, dtype=np.uint8).reshape(3, 5, 3)
    np.testing.assert_array_equal(decoded, generate_deterministic_rgb_frame(7, width=5, height=3))


def test_frames_are_deterministic_and_change_with_index() -> None:
    first = generate_deterministic_rgb_frame(0, width=4, height=3)
    repeated = generate_deterministic_rgb_frame(0, width=4, height=3)
    second = generate_deterministic_rgb_frame(1, width=4, height=3)

    np.testing.assert_array_equal(first, repeated)
    assert not np.array_equal(first, second)
    np.testing.assert_array_equal(first[0, 0], np.asarray([0, 0, 0], dtype=np.uint8))
    np.testing.assert_array_equal(second[0, 0], np.asarray([1, 5, 11], dtype=np.uint8))


def test_generated_frame_is_rgb_uint8_and_c_contiguous() -> None:
    frame = generate_deterministic_rgb_frame(19, width=7, height=2)

    assert frame.shape == (2, 7, 3)
    assert frame.dtype == np.uint8
    assert frame.flags.c_contiguous


@pytest.mark.parametrize("rate", [0.0, -1.0, np.inf, np.nan, MAX_PUBLISH_RATE_HZ + 1.0])
def test_invalid_publish_rate_is_rejected(rate: float) -> None:
    with pytest.raises(ValueError, match="publish_rate_hz"):
        validate_camera_parameters(
            publish_rate_hz=rate,
            width=64,
            height=48,
            frame_id=DEFAULT_FRAME_ID,
            topic_name=DEFAULT_TOPIC_NAME,
        )


@pytest.mark.parametrize("rate", [True, "20"])
def test_non_numeric_publish_rate_is_rejected(rate: object) -> None:
    with pytest.raises(TypeError, match="publish_rate_hz"):
        validate_camera_parameters(
            publish_rate_hz=rate,
            width=64,
            height=48,
            frame_id=DEFAULT_FRAME_ID,
            topic_name=DEFAULT_TOPIC_NAME,
        )


@pytest.mark.parametrize(
    ("width", "height", "error_type"),
    [
        (0, 48, ValueError),
        (64, -1, ValueError),
        (1.5, 48, TypeError),
        (True, 48, TypeError),
        (MAX_IMAGE_DIMENSION + 1, 1, ValueError),
        (1921, 1080, ValueError),
    ],
)
def test_invalid_dimensions_are_rejected(
    width: object,
    height: object,
    error_type: type[Exception],
) -> None:
    with pytest.raises(error_type):
        validate_camera_parameters(
            publish_rate_hz=20.0,
            width=width,
            height=height,
            frame_id=DEFAULT_FRAME_ID,
            topic_name=DEFAULT_TOPIC_NAME,
        )


@pytest.mark.parametrize(
    ("field", "value", "error_type"),
    [
        ("frame_id", "", ValueError),
        ("frame_id", "camera frame", ValueError),
        ("frame_id", None, TypeError),
        ("topic_name", "", ValueError),
        ("topic_name", "/camera/rgb image", ValueError),
        ("topic_name", 3, TypeError),
    ],
)
def test_invalid_names_are_rejected(
    field: str,
    value: object,
    error_type: type[Exception],
) -> None:
    parameters: dict[str, object] = {
        "publish_rate_hz": 20.0,
        "width": 64,
        "height": 48,
        "frame_id": DEFAULT_FRAME_ID,
        "topic_name": DEFAULT_TOPIC_NAME,
    }
    parameters[field] = value
    with pytest.raises(error_type, match=field):
        validate_camera_parameters(**parameters)


@pytest.mark.parametrize("frame_index", [-1, 1.5, True])
def test_invalid_frame_index_is_rejected(frame_index: object) -> None:
    error_type = ValueError if frame_index == -1 else TypeError
    with pytest.raises(error_type, match="frame_index"):
        generate_deterministic_rgb_frame(frame_index, width=64, height=48)
