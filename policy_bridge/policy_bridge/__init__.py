"""ROS 2 runtime primitives for deterministic policy deployment."""

from .action_validation import JOINT_COUNT, validate_action
from .mock_dynamics import step_toward_target
from .policy_backend import JointVector, PolicyBackend
from .scripted_policy import (
    SUPPORTED_INSTRUCTION,
    ScriptedPolicy,
    UnsupportedInstructionError,
)

__all__ = [
    "JOINT_COUNT",
    "SUPPORTED_INSTRUCTION",
    "JointVector",
    "PolicyBackend",
    "ScriptedPolicy",
    "UnsupportedInstructionError",
    "step_toward_target",
    "validate_action",
]
