"""ROS-independent interface for policy inference backends."""

from __future__ import annotations

from typing import Protocol

import numpy as np
import numpy.typing as npt

JointVector = npt.NDArray[np.float64]


class PolicyBackend(Protocol):
    """Structural interface implemented by policy inference backends.

    A backend receives the latest six-joint observation and produces an
    absolute six-joint position target.  The protocol deliberately contains
    no ROS types so that policy implementations remain easy to unit test.
    """

    def reset(self) -> None:
        """Reset any episode-local backend state before a new task."""

        ...

    def predict(
        self,
        joint_positions: JointVector,
        instruction: str,
    ) -> JointVector:
        """Return an absolute joint-position target for the observation.

        Args:
            joint_positions: Current robot joint positions with shape ``(6,)``.
            instruction: Task instruction understood by the backend.

        Returns:
            A finite ``float64`` array with shape ``(6,)`` representing an
            absolute joint-position target.
        """

        ...
