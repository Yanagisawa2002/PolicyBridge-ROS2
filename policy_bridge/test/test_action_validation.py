"""Unit tests for ROS-independent action validation."""

from __future__ import annotations

import numpy as np
import pytest
from policy_bridge.action_validation import validate_action


def test_valid_six_joint_action_is_copied_as_float64() -> None:
    """A valid numeric vector passes with a safe, normalized representation."""

    source = np.arange(6, dtype=np.int32)

    validated = validate_action(source)

    assert validated.shape == (6,)
    assert validated.dtype == np.float64
    assert validated.flags.owndata
    np.testing.assert_array_equal(validated, source)
    source[0] = 99
    assert validated[0] == 0.0


@pytest.mark.parametrize(
    "action",
    [
        np.zeros(5),
        np.zeros(7),
        np.zeros((1, 6)),
        np.zeros((6, 1)),
        np.asarray(1.0),
    ],
)
def test_wrong_shape_is_rejected(action: np.ndarray) -> None:
    """Only a flat, six-element vector satisfies the M0 action contract."""

    with pytest.raises(ValueError, match=r"shape \(6,\)"):
        validate_action(action)


def test_nan_is_rejected() -> None:
    """NaN cannot enter the command path."""

    action = np.zeros(6, dtype=np.float64)
    action[2] = np.nan

    with pytest.raises(ValueError, match="finite"):
        validate_action(action)


@pytest.mark.parametrize("non_finite", [np.inf, -np.inf])
def test_infinity_is_rejected(non_finite: float) -> None:
    """Positive and negative infinity cannot enter the command path."""

    action = np.zeros(6, dtype=np.float64)
    action[4] = non_finite

    with pytest.raises(ValueError, match="finite"):
        validate_action(action)


@pytest.mark.parametrize(
    "action",
    [
        ["0", "0", "0", "0", "0", "0"],
        [True, False, True, False, True, False],
        np.asarray([1 + 0j] * 6),
        np.asarray([object()] * 6, dtype=object),
    ],
)
def test_non_real_numeric_input_is_rejected(action: object) -> None:
    """Strings, booleans, complex numbers, and objects are not joint targets."""

    with pytest.raises(TypeError, match="real numeric"):
        validate_action(action)
