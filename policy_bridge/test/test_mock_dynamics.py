"""Unit tests for the pure mock-manipulator dynamics."""

from __future__ import annotations

import numpy as np
import pytest
from policy_bridge.mock_dynamics import step_toward_target


def test_step_moves_each_unsettled_joint_toward_target() -> None:
    """A dynamics step reduces error for positive and negative directions."""

    current = np.asarray([0.0, 0.0, 0.5, -0.5, 1.0, -1.0])
    target = np.asarray([1.0, -1.0, 0.0, 0.0, 1.5, -1.5])

    updated = step_toward_target(
        current,
        target,
        max_delta_per_step=0.1,
        goal_tolerance=0.0,
    )

    assert np.all(np.abs(target - updated) < np.abs(target - current))


def test_step_change_never_exceeds_limit() -> None:
    """Every joint respects the configured per-step movement bound."""

    current = np.asarray([-2.0, -1.0, 0.0, 1.0, 2.0, 3.0])
    target = -current
    maximum_delta = 0.075

    updated = step_toward_target(
        current,
        target,
        max_delta_per_step=maximum_delta,
        goal_tolerance=0.0,
    )

    movement = np.abs(updated - current)
    assert np.all(movement <= maximum_delta + np.finfo(np.float64).eps)


def test_step_does_not_overshoot_near_target() -> None:
    """A remaining error smaller than the step limit lands on the target."""

    current = np.zeros(6, dtype=np.float64)
    target = np.asarray([0.02, -0.03, 0.04, -0.05, 0.06, -0.07])

    updated = step_toward_target(
        current,
        target,
        max_delta_per_step=0.1,
        goal_tolerance=0.0,
    )

    np.testing.assert_array_equal(updated, target)


def test_positions_within_tolerance_remain_stable() -> None:
    """Settled joints do not jitter on repeated updates."""

    current = np.asarray([0.999, -1.001, 0.002, -0.002, 2.0, -2.0])
    target = np.asarray([1.0, -1.0, 0.0, 0.0, 2.0, -2.0])

    first = step_toward_target(
        current,
        target,
        max_delta_per_step=0.1,
        goal_tolerance=0.01,
    )
    second = step_toward_target(
        first,
        target,
        max_delta_per_step=0.1,
        goal_tolerance=0.01,
    )

    np.testing.assert_array_equal(first, current)
    np.testing.assert_array_equal(second, first)


@pytest.mark.parametrize(
    ("current", "target", "invalid_name"),
    [
        (np.zeros(5), np.zeros(6), "current_positions"),
        (np.zeros(6), np.zeros(7), "target_positions"),
    ],
)
def test_wrong_vector_dimension_is_rejected(
    current: np.ndarray,
    target: np.ndarray,
    invalid_name: str,
) -> None:
    """Both dynamics vectors must contain exactly six joints."""

    with pytest.raises(ValueError, match=invalid_name):
        step_toward_target(
            current,
            target,
            max_delta_per_step=0.1,
            goal_tolerance=0.01,
        )


@pytest.mark.parametrize("maximum_delta", [0.0, -0.1, np.inf, np.nan, 10**1000])
def test_invalid_step_limit_is_rejected(maximum_delta: float) -> None:
    """The movement limit must be positive and finite."""

    with pytest.raises(ValueError, match="max_delta_per_step"):
        step_toward_target(
            np.zeros(6),
            np.ones(6),
            max_delta_per_step=maximum_delta,
            goal_tolerance=0.01,
        )


@pytest.mark.parametrize("tolerance", [-0.1, np.inf, np.nan])
def test_invalid_goal_tolerance_is_rejected(tolerance: float) -> None:
    """Tolerance must be non-negative and finite."""

    with pytest.raises(ValueError, match="goal_tolerance"):
        step_toward_target(
            np.zeros(6),
            np.ones(6),
            max_delta_per_step=0.1,
            goal_tolerance=tolerance,
        )
