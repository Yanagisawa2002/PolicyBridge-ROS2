# Headless rigid-body simulation evidence

This example connects the existing `ObservationSnapshot -> ScriptedPolicy ->
validate_action` boundary to actual PyBullet rigid-body stepping and position
motors. It runs without a window or ROS installation. It replaces the simple
joint stepping function with a physics engine for this experiment only; the
ROS Action server and default demos are unchanged.

## Reproduce

From the repository root, in an isolated Python environment:

```bash
python -m pip install -r examples/pybullet/requirements.txt
python examples/pybullet/run_evidence.py --output simulation-results/new-run
```

The output folder must not already exist. The runner writes `results.json` and
`trajectory.csv` before returning an exit code. Exit 1 means a declared evidence
gate failed; the retained run below deliberately includes such a failure.

## Fixed protocol

- PyBullet 3.2.7, NumPy 2.2.6; the recorded run uses Python 3.12 on Windows.
- A procedural fixed-base chain of six 0.25 kg rigid links; revolute joints,
  40 Nm motor force limits, gravity -9.81 m/s², 240 Hz physics and 20 Hz control.
- Eight initial joint vectors from seed 20260908, sampled in [-0.8, 0.8] rad.
- Five simulated seconds for each home and hold episode (16 episodes total).
- Home uses the existing scripted `move to home` policy. Hold sends the latest
  measured joint position once at control step 4 (0.2 simulated seconds), with
  no further commands. This is an adapter-level hold, not a ROS cancellation test.
- Final max joint error must be at most 0.01 rad. Hold's maximum **20 Hz sampled**
  deviation must also stay at most 0.05 rad; inter-sample peaks are not measured.
- One motors-disabled control checks that the home result requires actuation.
- Self-collision, contacts and robot-specific calibration are not modeled.

## Recorded results — 2026-09-08

| Check | Recorded outcome |
| --- | --- |
| Home convergence | 8/8 passed; worst final error 0.001500 rad |
| Hold gates | 7/8 passed; worst sampled transient 0.055593 rad |
| Hold final error | Worst 0.001500 rad |
| Motors-disabled control | Final home error 5.690338 rad; correctly did not converge |
| Overall declared gate | **FAIL**, because one hold transient exceeded 0.05 rad |

The second starting configuration settles near the hold target but exceeds
the transient limit. A single absolute-position hold command does not establish
instantaneous stopping in a system with inertia. No threshold was relaxed and
no failed episode was removed. A robot-specific braking/controller integration
would need separate design and validation before making a stronger stop claim.

The [result JSON](evidence/pybullet-2026-09-08/results.json) includes environment,
source hashes, all episodes and the declared gates. The [20 Hz trajectory](evidence/pybullet-2026-09-08/trajectory.csv)
contains the observed states, not reconstructed animation data. Source hashes
identify the bytes used for this run; `source_base_commit` identifies the checkout
before the newly added example was committed.

## Test scope

```bash
python -m pip install pytest==8.4.2
python -m pytest -q policy_bridge/test examples/pybullet/test_adapter.py
```

The local run passed **205 tests** and skipped **3 ROS launch modules** because
`rclpy` is unavailable. New tests exercise invalid-command rejection before the
physics API and home/hold behavior for an additional fixed initial state.
These passing tests do not override the failed transient in the 16-episode matrix.

The [historical ROS 2 Humble validation](validation.md) remains separate. This
new example does not validate ROS transport, action cancellation, RGB
synchronization, real-time deadlines, a learned policy, or real hardware.
