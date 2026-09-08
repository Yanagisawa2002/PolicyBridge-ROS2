"""Headless physics evidence for the ROS-independent policy boundary.

This does not launch the ROS Action server or exercise a learned policy.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.metadata
import json
import platform
import subprocess
import sys
from pathlib import Path

import numpy as np
import pybullet as bullet

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "policy_bridge"))

from policy_bridge.action_validation import validate_action  # noqa: E402
from policy_bridge.observation import ObservationSnapshot  # noqa: E402
from policy_bridge.policy_backend import create_policy_backend  # noqa: E402

DT = 1.0 / 240.0
SUBSTEPS = 12
JOINTS = tuple(range(6))
TOLERANCE = 0.01


class PhysicsArm:
    """Six motorized rigid links in an isolated PyBullet DIRECT client."""

    def __init__(self, initial: np.ndarray):
        self.client = bullet.connect(bullet.DIRECT)
        if self.client < 0:
            raise RuntimeError("PyBullet DIRECT connection failed")
        bullet.setTimeStep(DT, physicsClientId=self.client)
        bullet.setGravity(0, 0, -9.81, physicsClientId=self.client)
        bullet.setPhysicsEngineParameter(
            numSolverIterations=100,
            deterministicOverlappingPairs=1,
            physicsClientId=self.client,
        )
        shape = bullet.createCollisionShape(
            bullet.GEOM_BOX,
            halfExtents=[0.1, 0.025, 0.025],
            physicsClientId=self.client,
        )
        self.body = bullet.createMultiBody(
            baseMass=0,
            basePosition=[0, 0, 1],
            linkMasses=[0.25] * 6,
            linkCollisionShapeIndices=[shape] * 6,
            linkVisualShapeIndices=[-1] * 6,
            linkPositions=[[0.2, 0, 0]] * 6,
            linkOrientations=[[0, 0, 0, 1]] * 6,
            linkInertialFramePositions=[[0.1, 0, 0]] * 6,
            linkInertialFrameOrientations=[[0, 0, 0, 1]] * 6,
            linkParentIndices=list(range(6)),
            linkJointTypes=[bullet.JOINT_REVOLUTE] * 6,
            linkJointAxis=[[0, 0, 1], [0, 1, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0], [1, 0, 0]],
            physicsClientId=self.client,
        )
        for joint, value in enumerate(validate_action(initial)):
            bullet.resetJointState(
                self.body,
                joint,
                float(value),
                physicsClientId=self.client,
            )
        self.commands = 0

    def positions(self) -> np.ndarray:
        states = bullet.getJointStates(self.body, JOINTS, physicsClientId=self.client)
        return np.asarray([state[0] for state in states], dtype=np.float64)

    def command(self, action: object) -> None:
        target = validate_action(action)
        bullet.setJointMotorControlArray(
            self.body,
            JOINTS,
            bullet.POSITION_CONTROL,
            targetPositions=target.tolist(),
            forces=[40.0] * 6,
            positionGains=[0.15] * 6,
            velocityGains=[1.0] * 6,
            physicsClientId=self.client,
        )
        self.commands += 1

    def advance(self) -> None:
        for _ in range(SUBSTEPS):
            bullet.stepSimulation(physicsClientId=self.client)

    def close(self) -> None:
        bullet.disconnect(physicsClientId=self.client)


def snapshot(sequence: int, positions: np.ndarray) -> ObservationSnapshot:
    stamp = int(sequence * SUBSTEPS * DT * 1e9)
    return ObservationSnapshot(
        sequence_id=sequence,
        joint_positions=positions,
        joint_names=tuple(f"joint_{i + 1}" for i in JOINTS),
        rgb=None,
        joint_stamp_ns=stamp,
        image_stamp_ns=None,
        received_monotonic_ns=stamp,
        synchronization_skew_ms=None,
        image_frame_id=None,
    )


def episode(initial: np.ndarray, *, hold: bool, motors: bool = True):
    arm = PhysicsArm(initial)
    policy = create_policy_backend("scripted")
    policy.reset()
    trace = []
    hold_target = None
    post_hold_peak = 0.0
    try:
        if not motors:
            bullet.setJointMotorControlArray(
                arm.body,
                JOINTS,
                bullet.VELOCITY_CONTROL,
                forces=[0.0] * 6,
                physicsClientId=arm.client,
            )
        for step in range(100):
            observation = snapshot(step + 1, arm.positions())
            if hold and step == 4:
                hold_target = observation.joint_positions.copy()
                arm.command(hold_target)
            elif hold_target is None and motors:
                arm.command(policy.predict(observation, "move to home"))
            arm.advance()
            positions = arm.positions()
            target = np.zeros(6) if hold_target is None else hold_target
            error = float(np.max(np.abs(positions - target)))
            if hold_target is not None:
                post_hold_peak = max(post_hold_peak, error)
            trace.append(
                {
                    "step": step + 1,
                    "sim_seconds": (step + 1) * SUBSTEPS * DT,
                    "phase": "hold" if hold_target is not None else "home",
                    "error_rad": error,
                    **{f"q{i + 1}": float(q) for i, q in enumerate(positions)},
                }
            )
        passed = trace[-1]["error_rad"] <= TOLERANCE
        if hold:
            passed = passed and post_hold_peak <= 0.05 and arm.commands == 5
        return {
            "initial_rad": initial.tolist(),
            "final_error_rad": trace[-1]["error_rad"],
            "peak_hold_error_rad": post_hold_peak if hold else None,
            "commands": arm.commands,
            "passed": bool(passed),
        }, trace
    finally:
        arm.close()


def run(output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=False)
    rng = np.random.default_rng(20260908)
    initial_states = rng.uniform(-0.8, 0.8, size=(8, 6))
    rows = []
    results = []
    for index, initial in enumerate(initial_states):
        for hold in (False, True):
            name = f"start-{index + 1}-{'hold' if hold else 'home'}"
            result, trace = episode(initial, hold=hold)
            results.append({"episode": name, **result})
            rows.extend({"episode": name, **row} for row in trace)
    control, trace = episode(initial_states[0], hold=False, motors=False)
    rows.extend({"episode": "motors-disabled-control", **row} for row in trace)
    with (output / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    source_files = [Path(__file__), *sorted((ROOT / "policy_bridge/policy_bridge").glob("*.py"))]
    hashes = {
        path.relative_to(ROOT).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in source_files
    }
    result = {
        "scope": "ROS-independent scripted-policy boundary in headless rigid-body simulation",
        "not_validated": [
            "ROS transport/action runtime",
            "learned policy",
            "real hardware",
            "collision avoidance",
            "certified stop",
            "real-time deadlines",
        ],
        "environment": {
            "python": platform.python_version(),
            "platform": platform.platform(),
            "numpy": np.__version__,
            "pybullet": importlib.metadata.version("pybullet"),
        },
        "source_base_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
        "source_sha256": hashes,
        "protocol": {
            "seed": 20260908,
            "physics_hz": 240,
            "control_hz": 20,
            "duration_seconds": 5,
            "gravity_m_s2": [0, 0, -9.81],
            "final_error_limit_rad": TOLERANCE,
            "peak_hold_limit_rad": 0.05,
            "model": "procedural six-link chain; no self-collision or contact task",
        },
        "episodes": results,
        "motors_disabled_control": control,
        "trajectory_sha256": hashlib.sha256((output / "trajectory.csv").read_bytes()).hexdigest(),
        "passed": all(result["passed"] for result in results) and not control["passed"],
    }
    (output / "results.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    result = run(parser.parse_args().output)
    print(
        json.dumps(
            {
                "passed": result["passed"],
                "episodes": len(result["episodes"]),
                "control_error": result["motors_disabled_control"]["final_error_rad"],
            }
        )
    )
    raise SystemExit(0 if result["passed"] else 1)
