"""ROS-independent policy backends used by the runtime and its tests.

The public backend contract deliberately stays synchronous.  Runtime code is
responsible for placing calls on a bounded worker when it needs cancellation or
timeout handling; individual backends never depend on ROS.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Final, Literal, Protocol

import numpy as np
import numpy.typing as npt

from .observation import ObservationSnapshot

JointVector = npt.NDArray[np.float64]
InvalidActionMode = Literal["wrong_shape", "nan", "inf"]

_BACKEND_NAMES: Final[tuple[str, ...]] = (
    "scripted",
    "multimodal_scripted",
    "delayed",
    "invalid_wrong_shape",
    "invalid_nan",
    "invalid_inf",
    "raising",
)


class PolicyBackend(Protocol):
    """Structural interface implemented by policy inference backends.

    A backend receives one validated observation snapshot and produces an
    absolute six-joint position target.  The protocol deliberately contains no
    ROS types so that policy implementations remain easy to unit test.
    """

    def reset(self) -> None:
        """Reset any episode-local backend state before a new task."""

        ...

    def predict(
        self,
        observation: ObservationSnapshot,
        instruction: str,
    ) -> JointVector:
        """Return an absolute joint-position target for the observation.

        Args:
            observation: Immutable, ROS-independent observation snapshot.
            instruction: Task instruction understood by the backend.

        Returns:
            A finite ``float64`` array with shape ``(6,)`` representing an
            absolute joint-position target.
        """

        ...


class PolicyBackendTestError(RuntimeError):
    """Deterministic exception raised by :class:`RaisingPolicy`."""


def _new_scripted_policy() -> PolicyBackend:
    """Construct the production default without introducing an import cycle."""

    # ScriptedPolicy imports the action alias from this module, so the import
    # remains local to avoid an import cycle.
    from .scripted_policy import ScriptedPolicy

    return ScriptedPolicy()


def _new_multimodal_scripted_policy() -> PolicyBackend:
    """Construct the finite RGB-aware built-in backend."""

    from .scripted_policy import MultimodalScriptedPolicy

    return MultimodalScriptedPolicy()


class DelayedPolicy:
    """Delegate to another synchronous backend after a deterministic delay.

    By default only the first prediction after each :meth:`reset` is delayed.
    This is useful for exercising the runtime's inference-timeout and late
    result handling without changing the action returned by ScriptedPolicy.
    """

    backend_name: Final[str] = "delayed"

    def __init__(
        self,
        delay_seconds: float = 1.0,
        *,
        first_call_only: bool = True,
        delegate: PolicyBackend | None = None,
    ) -> None:
        """Configure the delay and optional backend receiving the actual call."""

        if isinstance(delay_seconds, bool):
            raise TypeError("delay_seconds must be a finite number")
        try:
            normalized_delay = float(delay_seconds)
        except (TypeError, ValueError) as exc:
            raise TypeError("delay_seconds must be a finite number") from exc
        if not math.isfinite(normalized_delay) or normalized_delay < 0.0:
            raise ValueError("delay_seconds must be finite and non-negative")
        if not isinstance(first_call_only, bool):
            raise TypeError("first_call_only must be a bool")

        self._delay_seconds = normalized_delay
        self._first_call_only = first_call_only
        self._delegate = delegate if delegate is not None else _new_scripted_policy()
        self._call_count = 0
        self._lock = threading.Lock()

    @property
    def delay_seconds(self) -> float:
        """Configured delay in seconds."""

        return self._delay_seconds

    @property
    def first_call_only(self) -> bool:
        """Whether only the first call after reset is delayed."""

        return self._first_call_only

    @property
    def call_count(self) -> int:
        """Number of predictions issued since the most recent reset."""

        with self._lock:
            return self._call_count

    def reset(self) -> None:
        """Reset the delay schedule and the delegated backend."""

        with self._lock:
            self._call_count = 0
        self._delegate.reset()

    def predict(
        self,
        observation: ObservationSnapshot,
        instruction: str,
    ) -> JointVector:
        """Delay when configured, then return the delegated prediction."""

        with self._lock:
            call_index = self._call_count
            self._call_count += 1

        should_delay = not self._first_call_only or call_index == 0
        if should_delay and self._delay_seconds > 0.0:
            time.sleep(self._delay_seconds)
        return self._delegate.predict(observation, instruction)


class InvalidActionPolicy:
    """Return one deliberately invalid action after validating the request."""

    def __init__(
        self,
        mode: InvalidActionMode = "wrong_shape",
        *,
        delegate: PolicyBackend | None = None,
    ) -> None:
        """Select the invalid action kind produced by :meth:`predict`."""

        if mode not in ("wrong_shape", "nan", "inf"):
            raise ValueError("mode must be one of: wrong_shape, nan, inf")
        self._mode = mode
        self._delegate = delegate if delegate is not None else _new_scripted_policy()
        self.backend_name = f"invalid_{mode}"

    @property
    def mode(self) -> InvalidActionMode:
        """Configured invalid action kind."""

        return self._mode

    def reset(self) -> None:
        """Reset the delegated backend."""

        self._delegate.reset()

    def predict(
        self,
        observation: ObservationSnapshot,
        instruction: str,
    ) -> JointVector:
        """Validate through the delegate, then corrupt its valid action."""

        action = np.asarray(
            self._delegate.predict(observation, instruction),
            dtype=np.float64,
        ).copy()
        if self._mode == "wrong_shape":
            return action[:-1]
        if self._mode == "nan":
            action[0] = np.nan
        else:
            action[0] = np.inf
        return action


class RaisingPolicy:
    """Backend that raises a deterministic non-instruction policy error."""

    backend_name: Final[str] = "raising"

    def __init__(self, message: str = "configured policy backend failure") -> None:
        """Configure the stable error message raised by every prediction."""

        if not isinstance(message, str):
            raise TypeError("message must be a string")
        self._message = message

    def reset(self) -> None:
        """Reset episode-local state; this backend is stateless."""

    def predict(
        self,
        observation: ObservationSnapshot,
        instruction: str,
    ) -> JointVector:
        """Raise :class:`PolicyBackendTestError` deterministically."""

        del observation, instruction
        raise PolicyBackendTestError(self._message)


def create_policy_backend(
    name: str = "scripted",
    *,
    delayed_policy_delay_seconds: float = 1.0,
    delayed_policy_first_call_only: bool = True,
) -> PolicyBackend:
    """Create one of the built-in backends through an explicit allow-list.

    No entry-point discovery, dynamic imports, downloads, or model registry are
    involved.  ``scripted`` remains the production default;
    ``multimodal_scripted`` is a finite RGB-aware test backend, and the
    remaining selectors are deterministic fault-injection backends.

    Args:
        name: One of ``scripted``, ``multimodal_scripted``, ``delayed``,
            ``invalid_wrong_shape``, ``invalid_nan``, ``invalid_inf``, or
            ``raising``.
        delayed_policy_delay_seconds: Delay used by the ``delayed`` backend.
        delayed_policy_first_call_only: Delay only the first call after reset.

    Raises:
        TypeError: If ``name`` is not a string.
        ValueError: If ``name`` is not a supported built-in backend.
    """

    if not isinstance(name, str):
        raise TypeError("policy backend name must be a string")
    if name == "scripted":
        return _new_scripted_policy()
    if name == "multimodal_scripted":
        return _new_multimodal_scripted_policy()
    if name == "delayed":
        return DelayedPolicy(
            delayed_policy_delay_seconds,
            first_call_only=delayed_policy_first_call_only,
        )
    if name.startswith("invalid_"):
        mode = name.removeprefix("invalid_")
        if mode in ("wrong_shape", "nan", "inf"):
            return InvalidActionPolicy(mode)  # type: ignore[arg-type]
    if name == "raising":
        return RaisingPolicy()

    supported = ", ".join(_BACKEND_NAMES)
    raise ValueError(f"unsupported policy backend {name!r}; expected one of: {supported}")
