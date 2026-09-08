"""Integration checks at the real physics command boundary."""

import numpy as np
import pytest
from run_evidence import PhysicsArm, episode


@pytest.mark.parametrize("action", [[1, 2], [float("nan")] * 6, [float("inf")] * 6])
def test_invalid_action_never_reaches_physics(action):
    arm = PhysicsArm(np.full(6, 0.4))
    try:
        original = arm.positions()
        with pytest.raises(ValueError):
            arm.command(action)
        assert arm.commands == 0
        np.testing.assert_array_equal(arm.positions(), original)
    finally:
        arm.close()


def test_home_and_hold_with_unseen_start():
    initial = np.array([0.63, -0.42, 0.18, -0.55, 0.72, -0.33])
    home, _ = episode(initial, hold=False)
    hold, _ = episode(initial, hold=True)
    assert home["passed"]
    assert hold["passed"]
    assert hold["commands"] == 5
