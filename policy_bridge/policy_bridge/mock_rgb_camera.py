"""Deterministic RGB image publisher for the PolicyBridge M2 demonstration."""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Integral, Real
from typing import Final

import numpy as np
from numpy.typing import NDArray

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
except ModuleNotFoundError as exc:  # Allow pure helpers to be tested without ROS.
    rclpy = None  # type: ignore[assignment]
    ExternalShutdownException = RuntimeError  # type: ignore[assignment,misc]
    Node = object  # type: ignore[assignment,misc]
    qos_profile_sensor_data = None
    Image = None  # type: ignore[assignment,misc]
    _ROS_IMPORT_ERROR: ModuleNotFoundError | None = exc
else:
    _ROS_IMPORT_ERROR = None


DEFAULT_PUBLISH_RATE_HZ: Final[float] = 20.0
DEFAULT_WIDTH: Final[int] = 64
DEFAULT_HEIGHT: Final[int] = 48
DEFAULT_FRAME_ID: Final[str] = "camera_rgb_optical_frame"
DEFAULT_TOPIC_NAME: Final[str] = "/camera/rgb/image_raw"
MAX_PUBLISH_RATE_HZ: Final[float] = 240.0
MAX_IMAGE_DIMENSION: Final[int] = 4096
MAX_IMAGE_PIXELS: Final[int] = 2_073_600
MAX_NAME_LENGTH: Final[int] = 255


@dataclass(frozen=True)
class MockRGBCameraConfiguration:
    """Validated configuration for the bounded mock publisher."""

    publish_rate_hz: float
    width: int
    height: int
    frame_id: str
    topic_name: str


@dataclass(frozen=True)
class RGBFramePayload:
    """ROS-independent fields used to populate one ``sensor_msgs/Image``."""

    height: int
    width: int
    encoding: str
    is_bigendian: int
    step: int
    data: bytes


def validate_camera_parameters(
    *,
    publish_rate_hz: object,
    width: object,
    height: object,
    frame_id: object,
    topic_name: object,
) -> MockRGBCameraConfiguration:
    """Validate and normalize mock-camera parameters without ROS dependencies."""

    rate = _bounded_positive_real(
        "publish_rate_hz",
        publish_rate_hz,
        maximum=MAX_PUBLISH_RATE_HZ,
    )
    validated_width = _bounded_positive_integer("width", width)
    validated_height = _bounded_positive_integer("height", height)
    if validated_width * validated_height > MAX_IMAGE_PIXELS:
        raise ValueError(f"width * height must not exceed {MAX_IMAGE_PIXELS} pixels")

    return MockRGBCameraConfiguration(
        publish_rate_hz=rate,
        width=validated_width,
        height=validated_height,
        frame_id=_bounded_nonempty_text("frame_id", frame_id),
        topic_name=_bounded_nonempty_text("topic_name", topic_name),
    )


def generate_deterministic_rgb_frame(
    frame_index: object,
    *,
    width: object,
    height: object,
) -> NDArray[np.uint8]:
    """Create a bounded, deterministic, C-contiguous ``H x W x 3`` RGB frame.

    Pixel channels vary with both position and frame index.  The frame pattern
    repeats after 256 frames by design, while every adjacent frame differs.
    """

    index = _nonnegative_integer("frame_index", frame_index)
    validated_width = _bounded_positive_integer("width", width)
    validated_height = _bounded_positive_integer("height", height)
    if validated_width * validated_height > MAX_IMAGE_PIXELS:
        raise ValueError(f"width * height must not exceed {MAX_IMAGE_PIXELS} pixels")

    phase = index % 256
    x = np.arange(validated_width, dtype=np.uint16)[np.newaxis, :]
    y = np.arange(validated_height, dtype=np.uint16)[:, np.newaxis]
    red = np.broadcast_to((x + phase) % 256, (validated_height, validated_width))
    green = np.broadcast_to(
        (3 * y + 5 * phase) % 256,
        (validated_height, validated_width),
    )
    blue = (x + y + 11 * phase) % 256
    frame = np.stack((red, green, blue), axis=-1).astype(np.uint8, copy=False)
    return np.ascontiguousarray(frame)


def build_frame_payload(
    frame_index: object,
    *,
    width: object,
    height: object,
) -> RGBFramePayload:
    """Build deterministic ``rgb8`` fields with exact row stride and payload size."""

    frame = generate_deterministic_rgb_frame(
        frame_index,
        width=width,
        height=height,
    )
    frame_height, frame_width, channels = frame.shape
    if channels != 3:  # Defensive invariant: generator always produces RGB.
        raise RuntimeError("deterministic frame generator did not produce RGB data")
    return RGBFramePayload(
        height=frame_height,
        width=frame_width,
        encoding="rgb8",
        is_bigendian=0,
        step=frame_width * channels,
        data=frame.tobytes(order="C"),
    )


class MockRGBCamera(Node):  # type: ignore[misc]
    """Publish small deterministic RGB images using explicit sensor-data QoS."""

    def __init__(self) -> None:
        """Declare parameters and start the periodic image publisher."""

        if _ROS_IMPORT_ERROR is not None:
            raise RuntimeError("mock_rgb_camera requires a ROS 2 Python environment") from (
                _ROS_IMPORT_ERROR
            )
        super().__init__("mock_rgb_camera")

        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter("width", DEFAULT_WIDTH)
        self.declare_parameter("height", DEFAULT_HEIGHT)
        self.declare_parameter("frame_id", DEFAULT_FRAME_ID)
        self.declare_parameter("topic_name", DEFAULT_TOPIC_NAME)

        configuration = validate_camera_parameters(
            publish_rate_hz=self.get_parameter("publish_rate_hz").value,
            width=self.get_parameter("width").value,
            height=self.get_parameter("height").value,
            frame_id=self.get_parameter("frame_id").value,
            topic_name=self.get_parameter("topic_name").value,
        )
        self._configuration = configuration
        self._frame_index = 0
        self._warned_zero_stamp = False
        self._publisher = self.create_publisher(
            Image,
            configuration.topic_name,
            qos_profile_sensor_data,
        )
        self._publish_timer = self.create_timer(
            1.0 / configuration.publish_rate_hz,
            self._publish_image,
        )
        self.get_logger().info(
            "Mock RGB camera ready: "
            f"{configuration.width}x{configuration.height} rgb8 at "
            f"{configuration.publish_rate_hz:.1f} Hz on {configuration.topic_name!r} "
            f"with frame_id={configuration.frame_id!r}"
        )

    def _publish_image(self) -> None:
        stamp = self.get_clock().now().to_msg()
        if stamp.sec == 0 and stamp.nanosec == 0:
            if not self._warned_zero_stamp:
                self.get_logger().warning(
                    "ROS clock is zero; withholding images until a non-zero stamp is available"
                )
                self._warned_zero_stamp = True
            return

        payload = build_frame_payload(
            self._frame_index,
            width=self._configuration.width,
            height=self._configuration.height,
        )
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = self._configuration.frame_id
        message.height = payload.height
        message.width = payload.width
        message.encoding = payload.encoding
        message.is_bigendian = payload.is_bigendian
        message.step = payload.step
        message.data = payload.data
        self._publisher.publish(message)
        self._frame_index += 1


def _bounded_positive_real(name: str, value: object, *, maximum: float) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real number")
    try:
        numeric = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(numeric):
        raise ValueError(f"{name} must be finite")
    if numeric <= 0.0 or numeric > maximum:
        raise ValueError(f"{name} must be greater than zero and at most {maximum}")
    return numeric


def _bounded_positive_integer(name: str, value: object) -> int:
    integer = _nonnegative_integer(name, value)
    if integer == 0 or integer > MAX_IMAGE_DIMENSION:
        raise ValueError(f"{name} must be greater than zero and at most {MAX_IMAGE_DIMENSION}")
    return integer


def _nonnegative_integer(name: str, value: object) -> int:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral):
        raise TypeError(f"{name} must be an integer")
    integer = int(value)
    if integer < 0:
        raise ValueError(f"{name} must be non-negative")
    return integer


def _bounded_nonempty_text(name: str, value: object) -> str:
    if not isinstance(value, str):
        raise TypeError(f"{name} must be a string")
    if not value or value.isspace():
        raise ValueError(f"{name} must not be empty")
    if len(value) > MAX_NAME_LENGTH:
        raise ValueError(f"{name} must contain at most {MAX_NAME_LENGTH} characters")
    if any(character.isspace() for character in value):
        raise ValueError(f"{name} must not contain whitespace")
    return value


def main(args: list[str] | None = None) -> None:
    """Run the mock RGB camera node."""

    if rclpy is None:
        raise RuntimeError("mock_rgb_camera requires a ROS 2 Python environment") from (
            _ROS_IMPORT_ERROR
        )
    rclpy.init(args=args)
    node = MockRGBCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
