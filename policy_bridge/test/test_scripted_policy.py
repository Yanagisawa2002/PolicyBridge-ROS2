"""Unit tests for deterministic joint-only and multimodal scripted policies."""

from __future__ import annotations

import numpy as np
import pytest
from policy_bridge.observation import ObservationSnapshot
from policy_bridge.scripted_policy import (
    SUPPORTED_INSTRUCTION,
    MultimodalScriptedPolicy,
    ScriptedPolicy,
    UnsupportedInstructionError,
)

JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))


def _observation(rgb: np.ndarray | None = None) -> ObservationSnapshot:
    """Build one valid ROS-independent backend request."""

    has_rgb = rgb is not None
    return ObservationSnapshot(
        sequence_id=1,
        joint_positions=np.asarray(
            [0.6, -0.5, 0.4, -0.3, 0.2, -0.1],
            dtype=np.float64,
        ),
        joint_names=JOINT_NAMES,
        rgb=rgb,
        joint_stamp_ns=1_000_000_000,
        image_stamp_ns=1_000_000_000 if has_rgb else None,
        received_monotonic_ns=5_000_000_000,
        synchronization_skew_ms=0.0 if has_rgb else None,
        image_frame_id="camera_rgb_optical_frame" if has_rgb else None,
    )


def test_supported_instruction_returns_finite_six_joint_home_target() -> None:
    """The sole supported task maps to a finite absolute home target."""

    policy = ScriptedPolicy()

    action = policy.predict(
        _observation(),
        SUPPORTED_INSTRUCTION,
    )

    assert action.shape == (6,)
    assert action.dtype == np.float64
    assert np.all(np.isfinite(action))
    np.testing.assert_array_equal(action, np.zeros(6, dtype=np.float64))


def test_predictions_return_independent_arrays() -> None:
    """A caller cannot mutate targets returned by later predictions."""

    policy = ScriptedPolicy()
    observation = _observation()

    first = policy.predict(observation, SUPPORTED_INSTRUCTION)
    first[0] = 9.0
    second = policy.predict(observation, SUPPORTED_INSTRUCTION)

    np.testing.assert_array_equal(second, np.zeros(6, dtype=np.float64))


@pytest.mark.parametrize(
    "instruction",
    ["unknown", "Move to home", "move to home ", ""],
)
def test_unknown_instruction_raises_specific_error(instruction: str) -> None:
    """Unsupported strings fail explicitly rather than selecting a fallback."""

    policy = ScriptedPolicy()

    with pytest.raises(
        UnsupportedInstructionError,
        match="expected exactly 'move to home'",
    ):
        policy.predict(_observation(), instruction)


def test_non_string_instruction_is_rejected() -> None:
    """Instructions must be strings before support is evaluated."""

    policy = ScriptedPolicy()

    with pytest.raises(TypeError, match="instruction must be a string"):
        policy.predict(_observation(), 123)  # type: ignore[arg-type]


def test_reset_is_repeatable() -> None:
    """Repeated resets retain the deterministic policy behavior."""

    policy = ScriptedPolicy()
    policy.reset()
    policy.reset()

    action = policy.predict(
        _observation(),
        SUPPORTED_INSTRUCTION,
    )
    np.testing.assert_array_equal(action, np.zeros(6, dtype=np.float64))


def test_ros_or_array_objects_do_not_leak_into_backend_contract() -> None:
    """Backends accept only ObservationSnapshot, never arrays or ROS objects."""

    class FakeJointState:
        position = [0.0] * 6

    policy = ScriptedPolicy()

    with pytest.raises(TypeError, match="must be an ObservationSnapshot"):
        policy.predict(np.zeros(6), SUPPORTED_INSTRUCTION)  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="must be an ObservationSnapshot"):
        policy.predict(FakeJointState(), SUPPORTED_INSTRUCTION)  # type: ignore[arg-type]


def test_multimodal_policy_consumes_valid_rgb_and_returns_home() -> None:
    """The RGB-aware backend records deterministic evidence of image access."""

    rgb = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
    policy = MultimodalScriptedPolicy()

    action = policy.predict(_observation(rgb), SUPPORTED_INSTRUCTION)

    expected_checksum = int(rgb.reshape(-1)[0])
    expected_checksum += 257 * int(rgb.reshape(-1)[rgb.size // 2])
    expected_checksum += 65_537 * int(rgb.reshape(-1)[-1])
    np.testing.assert_array_equal(action, np.zeros(6, dtype=np.float64))
    assert policy.rgb_observation_count == 1
    assert policy.last_rgb_checksum == expected_checksum


def test_multimodal_policy_rejects_missing_rgb_explicitly() -> None:
    """The multimodal backend never silently degrades to joint-only input."""

    policy = MultimodalScriptedPolicy()

    with pytest.raises(ValueError, match="requires an RGB observation"):
        policy.predict(_observation(), SUPPORTED_INSTRUCTION)
    assert policy.rgb_observation_count == 0
    assert policy.last_rgb_checksum is None


def test_multimodal_policy_reset_clears_rgb_access_evidence() -> None:
    """RGB proof is episode-local and reset deterministically."""

    policy = MultimodalScriptedPolicy()
    policy.predict(
        _observation(np.ones((1, 1, 3), dtype=np.uint8)),
        SUPPORTED_INSTRUCTION,
    )

    policy.reset()

    assert policy.rgb_observation_count == 0
    assert policy.last_rgb_checksum is None
