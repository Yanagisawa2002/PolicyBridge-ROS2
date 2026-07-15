"""Unit tests for the explicit M1 policy backend factory and test doubles."""

from __future__ import annotations

from typing import Final

import numpy as np
import pytest
from policy_bridge.policy_backend import (
    DelayedPolicy,
    InvalidActionPolicy,
    JointVector,
    PolicyBackendTestError,
    RaisingPolicy,
    create_policy_backend,
)
from policy_bridge.scripted_policy import SUPPORTED_INSTRUCTION, ScriptedPolicy

from policy_bridge import policy_backend as policy_backend_module


class RecordingPolicy:
    """Minimal legacy-style backend without a diagnostic name."""

    target: Final[tuple[float, ...]] = (0.0, 1.0, 2.0, 3.0, 4.0, 5.0)

    def __init__(self) -> None:
        self.reset_count = 0
        self.predict_count = 0

    def reset(self) -> None:
        """Record reset calls."""

        self.reset_count += 1

    def predict(self, joint_positions: JointVector, instruction: str) -> JointVector:
        """Record calls and return one valid deterministic target."""

        del joint_positions, instruction
        self.predict_count += 1
        return np.asarray(self.target, dtype=np.float64)


def test_factory_defaults_to_existing_scripted_policy() -> None:
    """The production default remains the existing deterministic backend."""

    backend = create_policy_backend()

    assert isinstance(backend, ScriptedPolicy)
    assert backend.backend_name == "scripted"  # type: ignore[attr-defined]
    np.testing.assert_array_equal(
        backend.predict(np.ones(6, dtype=np.float64), SUPPORTED_INSTRUCTION),
        np.zeros(6, dtype=np.float64),
    )


@pytest.mark.parametrize(
    ("selector", "expected_type", "backend_name"),
    [
        ("scripted", ScriptedPolicy, "scripted"),
        ("delayed", DelayedPolicy, "delayed"),
        ("invalid_wrong_shape", InvalidActionPolicy, "invalid_wrong_shape"),
        ("invalid_nan", InvalidActionPolicy, "invalid_nan"),
        ("invalid_inf", InvalidActionPolicy, "invalid_inf"),
        ("raising", RaisingPolicy, "raising"),
    ],
)
def test_factory_exposes_only_named_builtin_backends(
    selector: str,
    expected_type: type[object],
    backend_name: str,
) -> None:
    """Every explicit selector produces a backend with a diagnostic name."""

    backend = create_policy_backend(
        selector,
        delayed_policy_delay_seconds=0.0,
    )

    assert isinstance(backend, expected_type)
    assert backend.backend_name == backend_name  # type: ignore[attr-defined]


def test_delayed_policy_delays_only_first_call_after_each_reset(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """First-only mode has deterministic per-episode delay semantics."""

    sleep_calls: list[float] = []
    delegate = RecordingPolicy()
    backend = DelayedPolicy(0.25, first_call_only=True, delegate=delegate)
    monkeypatch.setattr(policy_backend_module.time, "sleep", sleep_calls.append)
    observation = np.zeros(6, dtype=np.float64)

    first = backend.predict(observation, SUPPORTED_INSTRUCTION)
    second = backend.predict(observation, SUPPORTED_INSTRUCTION)

    assert sleep_calls == [0.25]
    assert backend.call_count == 2
    assert delegate.predict_count == 2
    np.testing.assert_array_equal(first, second)

    backend.reset()
    backend.predict(observation, SUPPORTED_INSTRUCTION)

    assert sleep_calls == [0.25, 0.25]
    assert backend.call_count == 1
    assert delegate.reset_count == 1


def test_delayed_policy_can_delay_every_call(monkeypatch: pytest.MonkeyPatch) -> None:
    """Disabling first-only mode applies the configured delay every time."""

    sleep_calls: list[float] = []
    backend = DelayedPolicy(0.5, first_call_only=False, delegate=RecordingPolicy())
    monkeypatch.setattr(policy_backend_module.time, "sleep", sleep_calls.append)
    observation = np.zeros(6, dtype=np.float64)

    backend.predict(observation, SUPPORTED_INSTRUCTION)
    backend.predict(observation, SUPPORTED_INSTRUCTION)

    assert sleep_calls == [0.5, 0.5]


@pytest.mark.parametrize(
    ("mode", "expected_name"),
    [
        ("wrong_shape", "invalid_wrong_shape"),
        ("nan", "invalid_nan"),
        ("inf", "invalid_inf"),
    ],
)
def test_invalid_action_policy_produces_selected_fault(
    mode: str,
    expected_name: str,
) -> None:
    """Each invalid-action mode is stable and distinguishable."""

    backend = InvalidActionPolicy(mode)  # type: ignore[arg-type]
    action = backend.predict(np.zeros(6, dtype=np.float64), SUPPORTED_INSTRUCTION)

    assert backend.backend_name == expected_name
    assert action.dtype == np.float64
    if mode == "wrong_shape":
        assert action.shape == (5,)
    elif mode == "nan":
        assert action.shape == (6,)
        assert np.isnan(action[0])
    else:
        assert action.shape == (6,)
        assert np.isinf(action[0])


def test_invalid_action_policy_resets_and_validates_through_delegate() -> None:
    """The fault injector remains compatible with any synchronous backend."""

    delegate = RecordingPolicy()
    backend = InvalidActionPolicy("nan", delegate=delegate)

    backend.reset()
    backend.predict(np.zeros(6, dtype=np.float64), "any instruction")

    assert delegate.reset_count == 1
    assert delegate.predict_count == 1


def test_raising_policy_uses_stable_non_instruction_exception() -> None:
    """The raising backend gives runtime tests a deterministic policy error."""

    backend = RaisingPolicy("intentional test failure")
    backend.reset()

    with pytest.raises(PolicyBackendTestError, match="^intentional test failure$"):
        backend.predict(np.zeros(6, dtype=np.float64), SUPPORTED_INSTRUCTION)


def test_factory_forwards_delayed_backend_configuration(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Factory delay parameters have the same meaning as direct construction."""

    sleep_calls: list[float] = []
    backend = create_policy_backend(
        "delayed",
        delayed_policy_delay_seconds=0.75,
        delayed_policy_first_call_only=False,
    )
    monkeypatch.setattr(policy_backend_module.time, "sleep", sleep_calls.append)

    backend.predict(np.zeros(6, dtype=np.float64), SUPPORTED_INSTRUCTION)
    backend.predict(np.zeros(6, dtype=np.float64), SUPPORTED_INSTRUCTION)

    assert sleep_calls == [0.75, 0.75]


@pytest.mark.parametrize("value", [-1.0, np.nan, np.inf])
def test_delayed_policy_rejects_invalid_delay(value: float) -> None:
    """Delay configuration cannot be negative or non-finite."""

    with pytest.raises(ValueError, match="finite and non-negative"):
        DelayedPolicy(value)


def test_delayed_policy_rejects_non_numeric_or_boolean_delay() -> None:
    """Implicit boolean delays and non-numeric values are configuration errors."""

    with pytest.raises(TypeError, match="finite number"):
        DelayedPolicy(True)
    with pytest.raises(TypeError, match="finite number"):
        DelayedPolicy("not-a-number")  # type: ignore[arg-type]


def test_backend_configuration_enums_are_strict() -> None:
    """Unknown backend names and invalid test modes fail at construction."""

    with pytest.raises(TypeError, match="name must be a string"):
        create_policy_backend(1)  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="unsupported policy backend 'unknown'"):
        create_policy_backend("unknown")
    with pytest.raises(ValueError, match="mode must be one of"):
        InvalidActionPolicy("unknown")  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="first_call_only must be a bool"):
        DelayedPolicy(0.0, first_call_only=1)  # type: ignore[arg-type]
