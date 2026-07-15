# M0 validation record

This record distinguishes checks that actually ran from ROS behavior that was only inspected structurally.

## 2026-07-15 local validation

Environment:

- Windows PowerShell
- Python 3.13.5
- NumPy 2.1.3
- pytest 8.3.4
- `ros2`: not found
- `colcon`: not found
- CMake: not found

Executed checks:

| Check | Recorded result |
| --- | --- |
| `python -m pytest -q` | `37 passed in 0.13s` |
| `python -m ruff check .` | `All checks passed!` |
| `python -m ruff format --check .` | `13 files already formatted` |
| Python bytecode compilation | Passed |
| Parse every Python file using Python 3.10 grammar | Passed |
| Parse both `package.xml` files | Passed |
| Parse `policy_bridge/config/demo.yaml` | Passed |
| Compare `ExecutePolicy.action` with the required field contract | Passed |

The passing pytest suite is ROS-independent. It covers the scripted policy, explicit rejection of unsupported instructions, action shape/type/finite-value validation, and bounded mock dynamics including no overshoot and tolerance stability.

## Not executed in this environment

The following require a sourced ROS 2 Humble environment and remain runtime-unverified here:

- `rosdep` dependency resolution
- `colcon build --symlink-install`
- ROS package tests through `colcon test`
- `ros2 launch policy_bridge demo.launch.py`
- `/joint_states` and `/joint_command` topic transport
- action discovery, successful goal completion, cancellation, and unknown-instruction integration paths

The node files, launch description, package manifests, configuration, and cancellation paths received static review, but that is not equivalent to a ROS runtime or end-to-end result.
