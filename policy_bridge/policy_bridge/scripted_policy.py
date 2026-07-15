"""Deterministic scripted policy used by the M0 demonstration."""

from __future__ import annotations

from typing import Final

import numpy as np

from .action_validation import validate_action
from .observation import MAX_CONFIGURABLE_IMAGE_PIXELS, ObservationSnapshot
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

    backend_name: Final[str] = "scripted"

    def reset(self) -> None:
        """Reset episode-local state.

        M0 has no episode-local state, so repeated calls are safe and have no
        observable effect.
        """

    def predict(
        self,
        observation: ObservationSnapshot,
        instruction: str,
    ) -> JointVector:
        """Return the absolute six-joint home target.

        Args:
            observation: Validated ROS-independent observation snapshot.
            instruction: Exactly ``"move to home"``.

        Returns:
            A new finite ``float64`` array containing the home target.

        Raises:
            TypeError: If ``instruction`` is not a string or ``observation`` is
                not an :class:`ObservationSnapshot`.
            UnsupportedInstructionError: If the string instruction is not the
                single instruction supported in M0.
            ValueError: If the snapshot's joint vector violates the backend
                contract.
        """

        _validate_request(observation, instruction)

        return np.asarray(_HOME_TARGET, dtype=np.float64).copy()


class MultimodalScriptedPolicy:
    """Finite test backend proving that an RGB snapshot reached inference.

    The backend deliberately performs no image preprocessing or model work.  It
    reads three deterministic bytes into a lightweight checksum, then returns
    the same six-joint home target as :class:`ScriptedPolicy`.
    """

    backend_name: Final[str] = "multimodal_scripted"

    def __init__(self) -> None:
        self._rgb_observation_count = 0
        self._last_rgb_checksum: int | None = None

    @property
    def rgb_observation_count(self) -> int:
        """Number of RGB observations consumed since the latest reset."""

        return self._rgb_observation_count

    @property
    def last_rgb_checksum(self) -> int | None:
        """Deterministic checksum of the most recently consumed RGB image."""

        return self._last_rgb_checksum

    def reset(self) -> None:
        """Clear episode-local proof that RGB data reached this backend."""

        self._rgb_observation_count = 0
        self._last_rgb_checksum = None

    def predict(
        self,
        observation: ObservationSnapshot,
        instruction: str,
    ) -> JointVector:
        """Require RGB input, record a deterministic probe, and return home."""

        _validate_request(observation, instruction)
        rgb = observation.rgb
        if rgb is None:
            raise ValueError("multimodal_scripted requires an RGB observation")
        if rgb.dtype != np.uint8:
            raise TypeError(f"multimodal RGB must have dtype uint8; got {rgb.dtype}")
        if rgb.ndim != 3 or rgb.shape[2:] != (3,):
            raise ValueError(f"multimodal RGB must have shape (height, width, 3); got {rgb.shape}")
        pixel_count = rgb.shape[0] * rgb.shape[1]
        if pixel_count <= 0 or pixel_count > MAX_CONFIGURABLE_IMAGE_PIXELS:
            raise ValueError(
                "multimodal RGB pixel count must be between 1 and "
                f"{MAX_CONFIGURABLE_IMAGE_PIXELS}; got {pixel_count}"
            )

        flat = rgb.reshape(-1)
        midpoint = flat.size // 2
        self._last_rgb_checksum = int(flat[0]) + 257 * int(flat[midpoint]) + 65_537 * int(flat[-1])
        self._rgb_observation_count += 1
        return np.asarray(_HOME_TARGET, dtype=np.float64).copy()


def _validate_request(
    observation: ObservationSnapshot,
    instruction: str,
) -> JointVector:
    """Validate the common scripted-backend request and return joint values."""

    if not isinstance(observation, ObservationSnapshot):
        raise TypeError("observation must be an ObservationSnapshot")
    if not isinstance(instruction, str):
        raise TypeError("instruction must be a string")
    if instruction != SUPPORTED_INSTRUCTION:
        raise UnsupportedInstructionError(
            f"unsupported instruction {instruction!r}; expected exactly {SUPPORTED_INSTRUCTION!r}"
        )

    try:
        return validate_action(observation.joint_positions)
    except (TypeError, ValueError) as exc:
        error_type = type(exc)
        raise error_type(f"invalid observation joint_positions: {exc}") from exc
