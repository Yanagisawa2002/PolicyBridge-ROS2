"""Validation helpers for absolute joint-position actions."""

from __future__ import annotations

from typing import Final

import numpy as np

from .policy_backend import JointVector

JOINT_COUNT: Final[int] = 6
"""Number of joints supported by the M0 runtime."""


def validate_action(action: object) -> JointVector:
    """Validate and copy an M0 absolute joint-position action.

    Numeric array-like inputs are accepted, but the validated result always
    has ``float64`` dtype and owns its data.  Returning a copy prevents a
    policy from mutating a command after it has passed validation.

    Args:
        action: Candidate absolute joint-position target.

    Returns:
        A finite ``float64`` vector with shape ``(6,)``.

    Raises:
        TypeError: If ``action`` is not a real numeric array-like value.
        ValueError: If its shape is not ``(6,)`` or it contains NaN or Inf.
    """

    try:
        candidate = np.asarray(action)
    except (TypeError, ValueError) as exc:
        raise TypeError("action must be a real numeric array-like value") from exc

    if not _is_real_numeric_dtype(candidate.dtype):
        raise TypeError("action must contain only real numeric values")

    expected_shape = (JOINT_COUNT,)
    if candidate.shape != expected_shape:
        raise ValueError(f"action must have shape {expected_shape}; got {candidate.shape}")

    with np.errstate(over="ignore", invalid="ignore"):
        validated = candidate.astype(np.float64, copy=True)

    if not np.all(np.isfinite(validated)):
        raise ValueError("action must contain only finite values")

    return validated


def _is_real_numeric_dtype(dtype: np.dtype[object]) -> bool:
    """Return whether ``dtype`` represents real numbers, excluding booleans."""

    return bool(
        np.issubdtype(dtype, np.number)
        and not np.issubdtype(dtype, np.complexfloating)
        and not np.issubdtype(dtype, np.bool_)
    )
