# M1 ROS 2 Humble 验收记录

本记录对应 PolicyBridge-ROS2 M1“Runtime Fault Handling and Deterministic Hold”的实际验收，验证日期为 2026-07-15。M0 的基础动作链路、单活动目标约束和 `ExecutePolicy.action` 接口仍作为基线；本次新增验证聚焦超时、故障终止、确定性 hold、并发竞争、诊断和故障后复用。除“已知边界”明确说明的内容外，本文不把结构检查当作运行时证据。

## 验证环境

| 项目 | 实测值 |
| --- | --- |
| 官方容器镜像 | `ros:humble-ros-base-jammy` |
| 操作系统 | Ubuntu 22.04.5 LTS，x86_64 |
| ROS 发行版 | ROS 2 Humble，`ROS_DISTRO=humble` |
| Python | 3.10.12 |
| ROS 工作区 | `/root/ws` |
| 源码位置 | `/root/ws/src/PolicyBridge-ROS2` |
| 本地开发主机 | Windows / PowerShell |

## 干净依赖解析与构建

构建前实际删除了工作区的 `build`、`install` 和 `log`，随后在 Humble 环境中重新解析依赖和构建：

```bash
source /opt/ros/humble/setup.bash
cd /root/ws
rm -rf build install log
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --event-handlers console_direct+
```

实测结果：

- `rosdep`：`All required rosdeps installed successfully`；
- `colcon build`：`2 packages finished [4.09s]`；
- 两个包 `policy_bridge_interfaces`、`policy_bridge` 均从干净状态成功构建。

## 测试与静态检查

Humble 容器内执行：

```bash
source /opt/ros/humble/setup.bash
cd /root/ws
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose

cd /root/ws/src/PolicyBridge-ROS2
python3 -m pytest -q
```

| 检查 | 实测结果 |
| --- | --- |
| `colcon test --event-handlers console_direct+` | collected 90 items；`90 passed in 19.22s` |
| `colcon test-result --verbose` | `90 tests, 0 errors, 0 failures, 0 skipped` |
| 容器内 `python3 -m pytest -q` | `90 passed in 18.93s` |
| Windows `python -m pytest -q` | `88 passed, 2 skipped`；仅两个需要 `rclpy` 的 ROS launch 测试模块跳过 |
| Windows `python -m ruff check .` | 通过 |
| Windows `python -m ruff format --check .` | 通过 |
| `git diff --check` | 通过 |

Windows 结果只用于非 ROS 单元测试与静态检查；M1 的 ROS 行为结论来自上述 Humble 容器中的 90 项零跳过测试和真实 launch 集成测试。

## 包、接口、可执行文件与安装资产

安装空间来源于本次干净构建。以下发现和导入检查均通过：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
colcon list
ros2 pkg executables policy_bridge
ros2 interface show policy_bridge_interfaces/action/ExecutePolicy
python3 -c "from policy_bridge.policy_server import PolicyActionServer; from policy_bridge.policy_backend import create_policy_backend; from policy_bridge.runtime_state import RuntimeStateMachine"
ros2 pkg prefix policy_bridge --share
test -f /root/ws/install/policy_bridge/share/policy_bridge/launch/demo.launch.py
test -f /root/ws/install/policy_bridge/share/policy_bridge/config/demo.yaml
```

发现两个 ROS 包和三个可执行文件：

```text
policy_bridge
policy_bridge_interfaces

policy_bridge demo_client
policy_bridge mock_manipulator
policy_bridge policy_server
```

launch 文件、YAML 配置和 Python 模块可从安装空间解析。`ExecutePolicy.action` 未为 M1 改动，实测接口仍为：

```text
string instruction
int32 max_steps
float32 timeout_seconds
---
bool success
string termination_reason
string episode_id
---
int32 current_step
float32 progress
float32 inference_latency_ms
```

## M1 launch 故障矩阵

真实 ROS launch 集成测试启动隔离后的 policy server，发送 Action 目标、发布受控 `/joint_states`，并记录 `/joint_command` 与诊断消息。下表记录已通过的行为，不是待办清单。

| 场景 | 实测结果 |
| --- | --- |
| 参数校验 | `timeout_seconds < 0` 和 `max_steps <= 0` 的目标被拒绝；`timeout_seconds=0` 的无期限场景可正常运行 |
| 活动目标门控 | 已有活动目标时新目标被拒绝，单活动目标约束成立 |
| 无有效观测 | 超时返回 `observation_timeout`；未发布零向量或任何伪造 hold，并记录 `safe_stop_unavailable_no_valid_state` |
| 无效观测输入 | 缺少关节、额外关节、长度错误、NaN 和 Inf 的 JointState 均未被当作有效状态 |
| 推理超时 | delayed backend 超时返回 `policy_inference_timeout`；结果晚到后被忽略，未执行晚到 action |
| backend busy | Action 已结束但同步调用未返回期间 `backend_busy=true`，新目标被拒绝；实际调用返回后恢复为 `false` 并可再次接收目标 |
| 取消等待中的推理 | 取消不等待 delayed backend 返回即可完成；晚到结果未变成普通命令，busy 门控持续到实际调用结束 |
| 目标截止时间竞争 | 目标单调时钟截止时间先赢时只返回 `goal_timeout`；晚到推理结果被忽略 |
| 非法 policy action | wrong shape、NaN、Inf 均返回 `invalid_action` 并执行 hold |
| backend 异常 | raising backend 返回 `policy_error` 并执行 hold；后续新目标仍可被同一服务器接收和独立终止，服务器未卡死 |
| 不支持指令 | 返回 `unsupported_instruction`；无运动时不发布 hold 或普通命令 |
| 目标超时 | 返回 `goal_timeout` 并执行 hold |
| 步数上限 | 返回 `max_steps_exceeded` 并执行 hold；边界测试同时覆盖最后一步成功与少一步失败 |
| 过期观测 | 先有有效状态、随后超过本地单调时钟新鲜度限制时返回 `stale_observation` 并执行 hold |
| 取消 | Action 状态为 `CANCELED`，结果为 `goal_canceled`，并执行一次 hold |
| 新 UUID 恢复 | 取消后的新目标使用不同 episode ID，未继承旧目标状态并成功 `goal_reached` |
| first-wins 竞争 | goal deadline/推理、stale/max-steps、cancel/success 三类竞争均只产生一个终止赢家；Action 状态、result 和 hold 决策一致 |
| 最终复用 | 所有故障与竞争场景后，同一 scripted server 的最终正常目标成功 `goal_reached` |

这些用例还验证了目标终止路径不会同时报告两个原因，也不会在结果之后继续发布来自该目标的普通 policy 命令。

## 确定性 hold 证据

hold 使用“最近一次有效的 6 维关节状态”，通过与普通命令相同的 `/joint_command` 发布边界发送。集成测试中的直接观测为：

- 普通故障场景的最近有效状态为 `[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]`，终止时最后一条命令与该状态完全相同；
- 取消场景在取消前注入的新近状态为 `[0.5, 0.5, 0.5, 0.5, 0.5, 0.5]`，最后一条命令与该状态完全相同；
- 每个需要安全停止的终止最多发布一次 hold；hold 是该目标的最后一条命令，继续观察后没有普通命令越过该边界；
- 从未收到有效状态的 `observation_timeout` 不发布 hold，明确记录 `safe_stop_unavailable_no_valid_state`，没有用全零状态掩盖未知姿态。

因此这里的“deterministic hold”是消息层面的确定性命令语义；它不等同于已证明物理机械臂静止。

## Diagnostics 发现与故障恢复

运行 demo 时执行了真实 graph 查询和一次消息抓取：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
ros2 topic list | grep '^/policy_bridge/diagnostics$'
ros2 topic type /policy_bridge/diagnostics
ros2 topic echo /policy_bridge/diagnostics diagnostic_msgs/msg/DiagnosticArray --once
```

实测发现 `/policy_bridge/diagnostics`，类型为 `diagnostic_msgs/msg/DiagnosticArray`。抓取到的一帧包含以下三项，均为 `level=OK`：

```text
policy_bridge/runtime
policy_bridge/observation
policy_bridge/policy
```

该帧对应的活动目标为：

```text
episode-66c7bcb3ca47458491964017e4b1e1bc
```

三项状态的必需 key 均实际存在：

| 状态名 | 必需 key |
| --- | --- |
| `policy_bridge/runtime` | `runtime_state`, `active_goal`, `episode_id`, `last_termination_reason`, `goal_elapsed_ms`, `safe_stop_count` |
| `policy_bridge/observation` | `joint_state_received`, `joint_state_valid`, `joint_state_age_ms`, `joint_state_timeout_ms` |
| `policy_bridge/policy` | `backend_name`, `backend_busy`, `last_inference_latency_ms`, `inference_timeout_ms`, `last_policy_error` |

故障 launch 用例验证了事件触发的即时诊断：观测类故障使 observation 组件进入 `ERROR`，policy 推理超时、非法 action 和 backend 异常使 policy 组件进入 `ERROR`，runtime 同时为 non-OK 并保留对应终止原因。故障处理完成且 backend 不再 busy 后，最新三项均恢复为 `OK`，`runtime_state=idle`，同时 `last_termination_reason` 仍保留最近一次原因。周期低频发布与关键状态变化即时发布均得到覆盖。

## 连续两次正常 launch

干净安装空间中的正常 demo 连续运行两次：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
ros2 launch policy_bridge demo.launch.py

# 每次客户端完成并有序停止 launch 后检查 ROS graph：
ros2 node list
```

两次运行均收到 `success=True`、`termination_reason=goal_reached`，demo client 打印 clean exit 后再有序停止 launch；policy server、mock manipulator 和 client 均无 traceback，停止后无残留进程。两次 episode ID 不同：

```text
episode-da7882f978e4438b80b207e25ae1e85e
episode-0893a87aef824b388ca36f1cce2bf84d
```

## 已知边界

Python 不能安全强杀一个任意且永不返回的同步 backend 线程。M1 会按时终止 Action、忽略迟到结果，并在实际调用结束前维持 `backend_busy`、拒绝新目标；但若第三方 backend 永久不返回，不能承诺该 Python 进程自行干净退出。仓库内置的 scripted、delayed、invalid 和 raising 测试 backend 都是有限执行，以上 launch 与重复运行均已 clean exit。

本记录也不声称：

- hold 已让真实硬件达到物理静止；
- Python/ROS 2 执行器具备硬实时保证；
- 已完成真实机械臂、外部 learned-policy 服务、运动规划或仿真器集成。

## 验收结论

在官方 ROS 2 Humble 容器中，M1 已通过干净依赖解析、双包构建、90 项零失败零跳过测试、真实 Action/Topic/Diagnostics launch 集成、故障后复用和连续两次正常运行。结果支持本阶段声明的运行时超时、first-wins 终止、单 worker busy 门控、迟到结果隔离、最近有效状态 hold、无有效状态不伪造命令以及三组件诊断；声明范围不超过上述消息层与测试环境边界。
