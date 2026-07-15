"""Deterministic scripted policy used by the M0 demonstration."""

from __future__ import annotations

from typing import Final

import numpy as np

from .action_validation import validate_action
from .policy_backend import JointVector

SUPPORTED_INSTRUCTION: Final[str] = "move to home"
"""The only task instruction supported by the M0 policy."""

_HOME_TARGET: Final[tuple[float, float, float, float, float, float]] = (
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
)


class UnsupportedInstructionError(ValueError):
    """Raised when an instruction is outside the M0 policy contract."""


class ScriptedPolicy:
    """Return a fixed home target for the one instruction supported in M0.

    The returned action is an absolute joint-position target.  This class is
    intentionally stateless and deterministic; ``reset`` is present to honor
    the :class:`~policy_bridge.policy_backend.PolicyBackend` protocol.
    """

    def reset(self) -> None:
        """Reset episode-local state.

        M0 has no episode-local state, so repeated calls are safe and have no
        observable effect.
        """

    def predict(
        self,
        joint_positions: JointVector,
        instruction: str,
    ) -> JointVector:
        """Return the absolute six-joint home target.

        Args:
            joint_positions: Current finite joint positions with shape ``(6,)``.
            instruction: Exactly ``"move to home"``.

        Returns:
            A new finite ``float64`` array containing the home target.

        Raises:
            TypeError: If ``instruction`` is not a string or the observation is
                not numeric.
            UnsupportedInstructionError: If the string instruction is not the
                single instruction supported in M0.
            ValueError: If the observation has the wrong shape or is non-finite.
        """

        if not isinstance(instruction, str):
            raise TypeError("instruction must be a string")
        if instruction != SUPPORTED_INSTRUCTION:
            raise UnsupportedInstructionError(
                f"unsupported instruction {instruction!r}; "
                f"expected exactly {SUPPORTED_INSTRUCTION!r}"
            )

        try:
            validate_action(joint_positions)
        except (TypeError, ValueError) as exc:
            error_type = type(exc)
            raise error_type(f"invalid joint_positions: {exc}") from exc

        return np.asarray(_HOME_TARGET, dtype=np.float64).copy()
