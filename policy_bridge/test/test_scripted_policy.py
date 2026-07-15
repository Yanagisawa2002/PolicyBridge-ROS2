"""Unit tests for the deterministic M0 scripted policy."""

from __future__ import annotations

import numpy as np
import pytest
from policy_bridge.scripted_policy import (
    SUPPORTED_INSTRUCTION,
    ScriptedPolicy,
    UnsupportedInstructionError,
)


def test_supported_instruction_returns_finite_six_joint_home_target() -> None:
    """The sole supported task maps to a finite absolute home target."""

    policy = ScriptedPolicy()

    action = policy.predict(
        np.asarray([0.6, -0.5, 0.4, -0.3, 0.2, -0.1], dtype=np.float64),
        SUPPORTED_INSTRUCTION,
    )

    assert action.shape == (6,)
    assert action.dtype == np.float64
    assert np.all(np.isfinite(action))
    np.testing.assert_array_equal(action, np.zeros(6, dtype=np.float64))


def test_predictions_return_independent_arrays() -> None:
    """A caller cannot mutate targets returned by later predictions."""

    policy = ScriptedPolicy()
    positions = np.zeros(6, dtype=np.float64)

    first = policy.predict(positions, SUPPORTED_INSTRUCTION)
    first[0] = 9.0
    second = policy.predict(positions, SUPPORTED_INSTRUCTION)

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
        policy.predict(np.zeros(6, dtype=np.float64), instruction)


def test_non_string_instruction_is_rejected() -> None:
    """Instructions must be strings before support is evaluated."""

    policy = ScriptedPolicy()

    with pytest.raises(TypeError, match="instruction must be a string"):
        policy.predict(np.zeros(6, dtype=np.float64), 123)  # type: ignore[arg-type]


def test_reset_is_repeatable() -> None:
    """Repeated resets retain the deterministic policy behavior."""

    policy = ScriptedPolicy()
    policy.reset()
    policy.reset()

    action = policy.predict(
        np.ones(6, dtype=np.float64),
        SUPPORTED_INSTRUCTION,
    )
    np.testing.assert_array_equal(action, np.zeros(6, dtype=np.float64))


@pytest.mark.parametrize(
    "positions",
    [np.zeros(5), np.asarray([0.0, 0.0, np.nan, 0.0, 0.0, 0.0])],
)
def test_invalid_joint_observation_is_rejected(positions: np.ndarray) -> None:
    """The policy validates its six-joint observation contract."""

    policy = ScriptedPolicy()

    with pytest.raises(ValueError, match="invalid joint_positions"):
        policy.predict(positions, SUPPORTED_INSTRUCTION)
