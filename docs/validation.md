# M0.1 ROS 2 Humble validation record

This record separates observed runtime behavior from structural inspection. It covers M0.1 acceptance only; it does not claim any M1 timeout, hold-position, diagnostics, or safe-stop capability.

## Validation environment

Validation ran on 2026-07-15 in a persistent Docker container created from `ros:humble-ros-base-jammy`, with the repository mounted as a ROS workspace package.

| Item | Recorded value |
| --- | --- |
| Container operating system | Ubuntu 22.04.5 LTS (Jammy) |
| Kernel | Linux `6.6.87.2-microsoft-standard-WSL2` |
| ROS distribution | ROS 2 Humble (`ROS_DISTRO=humble`) |
| RMW implementation | `rmw_fastrtps_cpp` |
| Python | 3.10.12, the Humble/Jammy system interpreter |
| colcon | `colcon-core` 0.21.0; `colcon version-check` completed |
| CMake | 3.22.1 |
| ROS graph isolation | `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1` |
| Host used to control Docker | Windows PowerShell |

`ros2 doctor --report` was used to record the ROS distribution, middleware, and platform. Humble's `ros2` CLI does not provide a `ros2 --version` option, so that unsupported command was not treated as version evidence.

The environment was recorded with:

```bash
uname -a
cat /etc/os-release
python3 --version
ros2 --version || true
printenv ROS_DISTRO
colcon version-check || true
cmake --version
ros2 doctor --report
```

## Clean dependency resolution and build

The workspace build products were removed before dependency resolution and build:

```bash
source /opt/ros/humble/setup.bash
cd /root/policybridge_ws
rm -rf build install log
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --event-handlers console_direct+
```

Recorded result:

- `rosdep` reported that all required dependencies were installed.
- `colcon` built both packages: `policy_bridge_interfaces` and `policy_bridge`.
- The generated `ExecutePolicy` action interfaces, three Python executables, launch file, and YAML configuration were installed.

The initial manifest listed `ament_python` as a rosdep-resolved build-tool dependency. Humble has no rosdep key for that package name; removing that invalid dependency brought the manifest in line with a generated Humble `ament_python` package and allowed clean dependency resolution.

## Tests and static checks

The ROS-independent suite was exercised on the Windows development host and then through the Humble workspace. The final recorded results were:

| Check | Recorded result |
| --- | --- |
| Windows `python -m pytest -q` | `37 passed, 1 skipped in 0.16s`; the skip is only the ROS launch test because `rclpy` is unavailable on the host |
| Windows `python -m ruff check .` | `All checks passed!` |
| Windows `python -m ruff format --check .` | `15 files already formatted` |
| Humble `colcon test` and `colcon test-result --verbose` | `38 tests, 0 errors, 0 failures, 0 skipped` |
| Direct Humble `python3 -m pytest -q` | `38 passed in 4.01s` |
| Python 3.10 grammar, XML, YAML, and Action contract checks | Passed |

The Humble test run originally exposed a pytest 6.2 warning for the unsupported `pythonpath` configuration option. Source-tree imports are now supplied by `policy_bridge/test/conftest.py`, eliminating that warning while preserving non-installed local testing.

The final clean test commands were:

```bash
cd /root/policybridge_ws
colcon test --event-handlers console_direct+
colcon test-result --verbose

cd /root/policybridge_ws/src/PolicyBridge-ROS2
python3 -m pytest -q
```

The launch-boundary test was collected by the normal `colcon test` run; it was not accepted solely from a standalone test invocation.

## ROS package, interface, executable, and graph discovery

After sourcing the clean workspace install, the following discovery commands ran successfully:

```bash
source /opt/ros/humble/setup.bash
source /root/policybridge_ws/install/setup.bash
ros2 pkg list | grep policy_bridge
ros2 interface show policy_bridge_interfaces/action/ExecutePolicy
ros2 pkg executables policy_bridge
```

Recorded package and executable discovery:

```text
policy_bridge
policy_bridge_interfaces

policy_bridge demo_client
policy_bridge mock_manipulator
policy_bridge policy_server
```

`ros2 interface show` matched the committed goal, result, and feedback contract. During a live demo the graph was checked with:

```bash
ros2 node list
ros2 topic list
ros2 topic info /joint_states
ros2 topic info /joint_command
ros2 topic echo /joint_states --once
ros2 action list
ros2 action info /execute_policy
```

The graph contained `/mock_manipulator` and `/policy_server`; `/joint_states` and `/joint_command` each had the expected type and one publisher/one subscriber, and `/execute_policy` exposed one action server owned by `/policy_server`.

## Normal end-to-end execution and repeat-launch isolation

The final repeat check ran the following command twice, stopping each launch with SIGINT only after the one-shot client had exited cleanly:

```bash
ros2 launch policy_bridge demo.launch.py

# After each stopped launch:
pgrep -af 'policy_server|mock_manipulator|demo_client'
```

Two consecutive fixed-code runs each produced:

- an accepted `move to home` goal;
- feedback for steps 1 through 6, with final progress `1.000`;
- `success=True` and `termination_reason=goal_reached`;
- clean client, mock-manipulator, and policy-server exits with no traceback or residual process.

The two runs returned distinct goal-derived episode IDs:

```text
episode-83a6ce3a01ed4fb1ab45ae1ceff99283
episode-2d768339f01d45a096c8bc887cbf11bb
```

This replaced a process-local counter that reused `episode-000001` after every launch. The episode identifier is now derived from the ROS action goal UUID, so repeat launches do not share the former counter namespace.

The first runtime attempt also exposed `ExternalShutdownException` during launch shutdown in the mock node. The three node entry points now handle that normal ROS executor shutdown path; the repeated post-fix launches exited cleanly.

## Unsupported-instruction behavior

With the action server and mock manipulator running, the initial joint state was recorded as:

```text
[0.5, -0.4, 0.3, -0.2, 0.1, -0.5]
```

A goal containing `pick up the red mug` was then sent while a `/joint_command` listener watched for three seconds. The recorded result was:

```bash
# Run in separate sourced shells while the server and mock are active.
timeout 3s ros2 topic echo /joint_command
ros2 run policy_bridge demo_client --ros-args \
  -p instruction:="pick up the red mug"
```

- goal accepted at the action-transport layer, then terminated with action status `ABORTED`;
- `success=False` and exact `termination_reason=unsupported_instruction`;
- no `/joint_command` message observed;
- joint state unchanged after the result;
- policy-server process remained healthy.

This verifies that an unknown instruction fails explicitly rather than selecting a fallback action or crashing the server.

## Cancellation and fresh-goal recovery

The cancellation scenario used the same running action server with `control_rate_hz=1.0` and a deliberately slow mock manipulator (`max_delta_per_step=0.0001`) so the goal could not finish before cancellation. After the first command was observed, the client requested cancellation.

The controlled runtime was started from separate sourced shells with:

```bash
ros2 run policy_bridge policy_server --ros-args \
  -p control_rate_hz:=1.0
ros2 run policy_bridge mock_manipulator --ros-args \
  -p max_delta_per_step:=0.0001
ros2 action send_goal /execute_policy \
  policy_bridge_interfaces/action/ExecutePolicy \
  "{instruction: 'move to home', max_steps: 200, timeout_seconds: 0.0}" \
  --feedback
```

The action CLI was interrupted with Ctrl-C after the first observed command, which issues the cancellation request. For the fresh-goal check, only the slow mock was stopped; the same server stayed alive, and the next goal was sent with:

```bash
ros2 run policy_bridge mock_manipulator
ros2 run policy_bridge demo_client
```

Recorded result:

- cancellation was accepted;
- the action finished with status `CANCELED`, `success=False`, and `termination_reason=goal_canceled`;
- command count was 1 at the canceled result and remained 1 after another 2.5 seconds;
- the server did not crash.

The slow mock was then replaced by the default mock without restarting the action server. A fresh `move to home` goal succeeded with `termination_reason=goal_reached`, and its episode ID differed from the canceled goal. The action-server process remained the same throughout, demonstrating that cancellation state did not leak into the next goal.

An earlier attempt with the default fast mock reached home before the cancellation request and therefore was not counted as cancellation evidence; the controlled slow-mock rerun above is the accepted scenario.

## Exact final-step boundary

`policy_bridge/test/test_policy_server_edges_launch.py` provides a controlled ROS launch test that publishes explicit joint observations and observes every joint command. It is designed to verify all of the following against one running policy server:

- with `max_steps=2`, an out-of-tolerance first state followed by home on the second update succeeds after exactly two commands and feedback steps `[1, 2]`;
- with `max_steps=1`, the same out-of-tolerance first update aborts with `max_steps_exceeded` after exactly one command;
- cancellation after the first command returns `goal_canceled` and publishes no new command for more than two control periods;
- a fresh goal then succeeds and receives a distinct episode ID.

The test passed inside the final clean Humble `colcon test` run. Its four scenarios ran against one launched action server, and the complete workspace result was `38 tests, 0 errors, 0 failures, 0 skipped`. This accepts both the exact final-step behavior and the one-step-short failure behavior as repeatable automated checks.

## Safety boundary and excluded claims

Cancellation validation proves only that the policy server stops publishing **new** `/joint_command` messages after accepting cancellation. It does not retract the command already sent, publish a hold command, stop a hardware controller, or prove physical standstill; the mock may continue moving toward its last target according to its own dynamics.

M0/M0.1 does not implement or claim:

- enforcement of `timeout_seconds`;
- a hold-position, emergency-stop, or deterministic safe-stop path;
- diagnostics, fault recovery, lifecycle management, or formal real-time behavior;
- simulator, motion-planning, perception, learned-model, remote-inference, or physical-robot integration.

Those items remain outside this validation and outside M0.1 scope.

## Acceptance status

All M0.1 runtime conditions were observed in the stated Humble environment: clean dependency resolution and build, zero-failure tests, ROS discovery, normal execution, orderly unsupported-instruction failure, cancellation with no later command publication, a successful fresh goal on the same server, exact final-step behavior, and two clean consecutive launches with distinct episode IDs and no residual node process. M0 is therefore accepted at the M0.1 boundary; no M1 behavior was implemented as part of this validation.
