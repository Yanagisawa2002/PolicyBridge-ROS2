"""Pure deterministic joint dynamics for the M0 mock manipulator."""

from __future__ import annotations

import math

import numpy as np

from .action_validation import validate_action
from .policy_backend import JointVector


def step_toward_target(
    current_positions: object,
    target_positions: object,
    *,
    max_delta_per_step: float,
    goal_tolerance: float,
) -> JointVector:
    """Advance six joints toward an absolute target by at most one step.

    Each joint moves independently.  A joint already within
    ``goal_tolerance`` remains unchanged, and every other joint moves by the
    smaller of its remaining error and ``max_delta_per_step``.  Consequently,
    a step cannot overshoot its target and repeated calls are deterministic.

    Args:
        current_positions: Current six-joint position vector.
        target_positions: Absolute six-joint target vector.
        max_delta_per_step: Strictly positive per-joint movement limit.
        goal_tolerance: Non-negative distance at which a joint is considered
            settled.

    Returns:
        A new finite ``float64`` vector containing the next positions.

    Raises:
        TypeError: If a vector is non-numeric or either scalar parameter is not
            a real number.
        ValueError: If a vector has the wrong shape or non-finite values, or if
            a scalar parameter is outside its valid range.
    """

    current = _validate_vector(current_positions, name="current_positions")
    target = _validate_vector(target_positions, name="target_positions")
    maximum_delta = _validate_scalar(
        max_delta_per_step,
        name="max_delta_per_step",
        minimum=0.0,
        minimum_is_inclusive=False,
    )
    tolerance = _validate_scalar(
        goal_tolerance,
        name="goal_tolerance",
        minimum=0.0,
        minimum_is_inclusive=True,
    )

    error = target - current
    settled = np.abs(error) <= tolerance
    bounded_delta = np.clip(error, -maximum_delta, maximum_delta)
    bounded_delta[settled] = 0.0
    return current + bounded_delta


def _validate_vector(value: object, *, name: str) -> JointVector:
    """Validate a named joint vector while preserving precise error context."""

    try:
        return validate_action(value)
    except (TypeError, ValueError) as exc:
        error_type = type(exc)
        raise error_type(f"invalid {name}: {exc}") from exc


def _validate_scalar(
    value: object,
    *,
    name: str,
    minimum: float,
    minimum_is_inclusive: bool,
) -> float:
    """Validate a finite real scalar against a lower bound."""

    if isinstance(value, (bool, np.bool_)) or not isinstance(
        value, (int, float, np.integer, np.floating)
    ):
        raise TypeError(f"{name} must be a real number")

    try:
        numeric_value = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(numeric_value):
        raise ValueError(f"{name} must be finite")

    if minimum_is_inclusive:
        is_valid = numeric_value >= minimum
        bound_text = f"greater than or equal to {minimum}"
    else:
        is_valid = numeric_value > minimum
        bound_text = f"greater than {minimum}"
    if not is_valid:
        raise ValueError(f"{name} must be {bound_text}")

    return numeric_value
