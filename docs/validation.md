# PolicyBridge-ROS2 ROS 2 Humble 验收记录

本文将 M2“Synchronized Multimodal Observation Pipeline”的最终验收与已确认的 M1 历史基线分开记录。M2 结论只采用 ROS 2 Humble 干净工作区的实际运行结果；收集到用例或代码结构本身不算运行时通过证据。

## M2 最终验收（PASS）

> **状态：M2 技术验收通过。** 干净 Humble 构建、隔离 ROS domain 的 204 项全量测试、两个真实 demo、图像/同步故障矩阵、sequence gate、竞争条件、诊断、故障恢复和 clean shutdown 均已取得运行证据。发布 commit 与 annotated tag 在本记录写入后执行，因此本节不虚构尚未生成的 commit hash。

### M2 环境和 Git 基线

| 项目 | M2 实测值 |
| --- | --- |
| 官方容器镜像 | `ros:humble-ros-base-jammy` |
| 操作系统 / 架构 | Ubuntu 22.04.5 LTS / x86_64 |
| ROS 发行版 | ROS 2 Humble |
| Python | 3.10.12 |
| ROS 工作区 | `/root/ws` |
| 源码位置 | `/root/ws/src/PolicyBridge-ROS2` |
| 验证日期 | 2026-07-15 |
| M2 验证起点 | `533564f` |
| `v0.2.0-m1` annotated tag 解引用 | `533564f1c0cc…`，与 M1 commit 一致 |
| 最终隔离 domain | `ROS_DOMAIN_ID=169`（colcon），`ROS_DOMAIN_ID=168`（独立 pytest） |
| Remote | 未配置；`git remote -v` 无输出 |

开始前实际执行：

```bash
git status --short
git log --oneline -5
git rev-parse v0.2.0-m1^{}
git tag --list
git remote -v
```

确认起始工作区位于 `533564f` 的 M1 正式基线，`v0.2.0-m1` 指向该 commit，且没有 remote。M2 发布提交和 `v0.3.0-m2` tag 需在文档完成后创建；本记录不预填最终 hash，也不声称已 push。

### M2 干净依赖解析和构建

在 ROS 2 Humble 环境中实际删除旧产物后执行：

```bash
source /opt/ros/humble/setup.bash
cd /root/ws

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -y

rm -rf build install log

colcon build \
  --symlink-install \
  --event-handlers console_direct+
```

| 检查 | M2 实测结果 |
| --- | --- |
| `rosdep install` | `All required rosdeps installed successfully` |
| clean `colcon build` | `2 packages finished [4.26s]` |
| `policy_bridge_interfaces` | 成功，从空 build/install/log 生成接口 |
| `policy_bridge` | 成功，从空 build/install/log 安装运行时与资产 |

### M2 全量测试和静态检查

最终的全量 ROS 测试使用独立 domain 169：

```bash
source /opt/ros/humble/setup.bash
cd /root/ws
source install/setup.bash

ROS_DOMAIN_ID=169 colcon test --event-handlers console_direct+
colcon test-result --verbose

cd /root/ws/src/PolicyBridge-ROS2
ROS_DOMAIN_ID=168 python3 -m pytest -q
python3 -m ruff check .
python3 -m ruff format --check .
git diff --check
```

| 检查 | M2 实测结果 |
| --- | --- |
| 最终 `colcon test` | collected 204；`204 passed in 26.40s`；2 packages finished in 27.1s |
| `colcon test-result --verbose` | `204 tests, 0 errors, 0 failures, 0 skipped` |
| 独立 Humble `python3 -m pytest -q` | `204 passed in 26.45s` |
| Windows host `python -m pytest -q` | `201 passed, 3 skipped`；三个 skip 均为需要 `rclpy` 的 ROS-only launch 模块 |
| `python -m ruff check .` | 通过 |
| `python -m ruff format --check .` | 通过；27 files already formatted |
| `git diff --check` | 通过，无 whitespace error |

最终正式计数是 Humble 中 **204 项、204 通过、0 errors、0 failures、0 skipped**。Host 结果只证明纯 Python 路径和静态检查；ROS 行为结论来自 Humble 零跳过运行。

#### 首次非隔离 domain 的污染记录

首次在 `ROS_DOMAIN_ID=58` 的 `colcon test` 得到 203 pass 和 1 个外层 launch failure。检查时没有仍存活的本地测试进程，但该共享 domain 中存在重叠/残留发现的同名 Action participant，导致外层测试连接到非本轮隔离的参与者。随后使用独立 ROS domains 重跑，污染消失；最终 post-assertion run 在 domain 169 为 204/204，独立 pytest 在 domain 168 也为 204/204。

因此首次结果保留为环境隔离记录，不归类为代码失败，也没有从最终统计中隐去。正式结论以最新隔离 domain 的 clean run 为准。

### 安装资产、接口和 ROS graph

在本次 clean install 空间实际执行：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash

colcon list
ros2 pkg executables policy_bridge
ros2 interface show policy_bridge_interfaces/action/ExecutePolicy
ros2 pkg prefix policy_bridge --share

test -f /root/ws/install/policy_bridge/share/policy_bridge/launch/demo.launch.py
test -f /root/ws/install/policy_bridge/share/policy_bridge/launch/multimodal_demo.launch.py
test -f /root/ws/install/policy_bridge/share/policy_bridge/config/demo.yaml
test -f /root/ws/install/policy_bridge/share/policy_bridge/config/multimodal_demo.yaml

python3 -c "from policy_bridge.observation import ObservationSnapshot, image_data_to_rgb; from policy_bridge.policy_server import PolicyActionServer; from policy_bridge.mock_rgb_camera import MockRGBCamera"
```

实测发现两个包以及四个 `policy_bridge` 可执行文件：

```text
policy_bridge demo_client
policy_bridge mock_manipulator
policy_bridge mock_rgb_camera
policy_bridge policy_server
```

两个 launch/config 文件对均已安装，M2 Python 导入通过。`ExecutePolicy.action` 实测仍为：

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

运行时 graph 实际发现 `/execute_policy` Action，以及 `/joint_states`、`/camera/rgb/image_raw`、`/joint_command`、`/policy_bridge/diagnostics` Topics。`ros2 topic info --verbose` 显示 camera publisher/subscriber 均为 `BEST_EFFORT + VOLATILE`，验证了实际交付使用兼容的 sensor-data QoS，而不是 transient-local 缓存。

### Joint-only 回归 demo

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
ros2 launch policy_bridge demo.launch.py
```

| 观察项 | M2 实测结果 |
| --- | --- |
| 不启动 camera | 两次均正常完成，无 image timeout |
| Action state / result | 两次均 `SUCCEEDED / success=true / goal_reached` |
| image/synchronization diagnostics | 全量测试确认 `joint_only` 中为 `OK / disabled` |
| M1 timeout、cancel、invalid-action 回归 | M1 launch 模块包含在最终 204/204 中，关键路径全部通过 |
| shutdown | 子进程 clean exit，停止后无残留进程 |

两次正式运行的 episode ID 分别为：

```text
episode-c2a9919c7925486cb4a7080ab32dd1bf
episode-aff4e0668b0b4d6886455b681f8cfcbe
```

### RGB-joint 正常闭环 demo

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
ros2 launch policy_bridge multimodal_demo.launch.py
```

运行期间实际执行 graph 查询、Image 消息抓取和 diagnostics 抓取。观测摘要为：

| 观察项 | M2 实测结果 |
| --- | --- |
| `/camera/rgb/image_raw` | 可发现并实际交付；runtime 将帧验证为有效 |
| mock 图像元数据 | `rgb8`、64×48、`camera_rgb_optical_frame`；有效同步快照证明非零 header stamp 通过运行时校验 |
| JointState/Image 同步 | 正常；抓取 skew 为 1.315 ms，小于 50 ms slop |
| Snapshot | `snapshot_sequence_id=320`，说明序列在持续增长 |
| Backend | diagnostics 为 `multimodal_scripted`；Goal 成功及 launch test 的 RGB-required backend 路径证明 RGB 实际进入 `predict()` |
| Image diagnostics | `image_valid=true`，encoding/尺寸/frame 均正确，`last_image_error` 为空 |
| Synchronization diagnostics | snapshot available，slop 50 ms，queue 10，`last_sync_error` 为空 |
| Action state / result | 两次均 `SUCCEEDED / success=true / goal_reached` |
| shutdown | 子进程 clean exit，停止后无残留进程 |

两次正式运行的 episode ID 分别为：

```text
episode-3452a238d6424d269ea8c72499562fa2
episode-f8bce643b26e4de7912ad9d49c3d38b5
```

### M2 图像和同步故障矩阵

`policy_bridge/test/test_policy_server_m2_launch.py` 在真实 Humble Action/Topic 图中覆盖四条 M2 故障；纯测试补充全部图像布局边界。

| 场景 | M2 实测结果 |
| --- | --- |
| 有 joint、从未收到 image | `ABORTED / image_timeout`；diagnostics 为 `ERROR` 且 `image_received=false`；最后一次命令为最近有效 joint hold |
| 先有同步快照，随后 image 停止而 joint 继续 | `ABORTED / stale_image`；一次 hold 等于等待期间最新 raw joint；同一 server 后续 Goal 恢复 `goal_reached` |
| 两路均持续有有效新数据，但 header 区域故意错开 | 不形成“各取最新”的伪快照；`ABORTED / observation_sync_timeout`；sync diagnostics 为 `ERROR` 并保留 skew/slop 错误；一次 latest-joint hold |
| 不支持 encoding | `mono8` 帧未进入 backend；`ABORTED / invalid_image`；diagnostics 为 `ERROR`、`image_valid=false`，并在 `last_image_error` 保留 encoding 错误；一次 latest-joint hold |
| 截断 data、row padding、`bgr8`、零尺寸、step 过小、超像素上限 | 纯 Python 图像转换/验证测试全部通过；截断和超限在输出分配前被拒绝，padding 与 BGR→RGB 结果正确 |
| 没有任何有效 joint | M1 的无有效状态 launch 回归仍在 204/204 中：不伪造零向量 hold，保留 `safe_stop_unavailable_no_valid_state` 语义 |
| 故障后的新正常 Goal | stale-image 集成场景在同一服务器提交新 UUID，形成新同步对并成功 `goal_reached`；五项 diagnostics 恢复 OK |

四条 M2 故障均验证 Action 最终状态、唯一 `termination_reason`、最后一条 `/joint_command`、组件 diagnostics、服务器存活性和 hold 次数。故障 launch 方法为 `test_missing_unsynchronized_and_invalid_image_faults` 与 `test_stale_image_holds_latest_joint_and_recovers`。

### Sequence gate、Goal/wait epoch、取消和竞争

| 场景 | M2 实测结果 |
| --- | --- |
| 单个同步快照 | 只产生一次 inference/普通命令；跨多个 control periods 未重复发布 |
| `sequence_id > previous_sequence_id` | 新快照前保持等待；新快照后闭环继续；sequence diagnostics 严格增长 |
| 精确同步 | `sync_slop_seconds=0` 的 `TimeSynchronizer` 路径实际成功，报告 0 ms skew |
| 近似同步 | 5 ms header skew 在 20 ms slop 内实际成功，报告 `synchronized` |
| Goal epoch | 测试等待服务器捕获 goal-local observation epoch 后再发布首对消息，旧 Goal 前的 invalid 历史不会污染新 Goal 的 fault 分类 |
| Wait epoch | sync timeout 只在本次等待开始后 joint 和 valid image 都有新到达、但仍无更新 snapshot 时成立；旧流量不会触发立即超时 |
| 最后一步边界 | M1 edge launch 测试仍通过；不会为不存在的下一次 inference 额外等待快照 |
| 等待新快照时取消 | `CANCELED / goal_canceled`；原普通命令后只追加一次等于最新 raw joint 的 hold |
| 新同步快照与 cancellation 竞争 | 只允许 `goal_reached` 或 `goal_canceled` 中一个 coherent 赢家；取消获胜时无普通命令越界并只 hold 一次 |
| image stale 与 goal timeout 竞争 | 结果只可能是其中一个赢家；runtime reason、Action state 和 `safe_stop_count=1` 一致 |
| sync timeout 与 joint stale 竞争 | 结果只可能是其中一个赢家；最新 joint hold 一次，无双结果或双 hold |

对应 Humble launch 方法为 `test_normal_sequence_gate_cancel_race_and_diagnostics` 和 `test_multimodal_timeout_races_are_first_wins`。它们使用状态等待、受控 publisher 和明确时间边界，而不是大量极短 sleep 获得偶然通过；扩展后的 M2 集成套件重复运行仍为 green。

### M2 Diagnostics 实测

实际 `DiagnosticArray` 包含五项：

```text
policy_bridge/runtime
policy_bridge/observation
policy_bridge/policy
policy_bridge/image
policy_bridge/synchronization
```

| 状态名 | M2 实测结果 |
| --- | --- |
| `policy_bridge/runtime` | 原 M1 runtime/termination/hold 字段均存在；竞争测试中 last reason 与 Action 赢家一致 |
| `policy_bridge/observation` | 原 M1 joint received/valid/age/timeout 字段存在；M1 freshness 回归通过 |
| `policy_bridge/policy` | `backend_name=multimodal_scripted`，正常抓取为 OK；busy/latency/timeout/error 回归通过 |
| `policy_bridge/image` | 实测 `image_required=true`, `image_valid=true`, `rgb8`, 64×48, `camera_rgb_optical_frame`，无当前错误；故障时进入相应 ERROR 并保留 `last_image_error` |
| `policy_bridge/synchronization` | 实测 snapshot available、sequence 320、age/skew 字段、1.315 ms skew、50 ms slop、queue 10、无当前错误；sync fault 时进入 ERROR 并保留 `last_sync_error` |

`joint_only` 的 image/synchronization 状态实测/测试为 `OK / disabled`。正常 RGB 为 OK，等待新同步时为 WARN，四条图像/同步故障获胜时对应组件为 ERROR。纯测试还验证“当前错误”和“历史最后错误”分离：后续有效图像/同步会清除当前错误并允许组件恢复 OK，而公开的 `last_image_error` / `last_sync_error` 继续保留最近历史原因；不会因历史字符串仍存在就维持 WARN/ERROR。

### 连续运行、episode ID 和 clean shutdown

分别连续运行两次 joint-only 和两次 multimodal demo。每次客户端完成后有序停止 launch，并检查 ROS graph 与子进程：

| 检查 | M2 实测结果 |
| --- | --- |
| joint-only 连续两次 | 两次 `goal_reached`；episode ID 为 `c2a9919c…`、`aff4e066…`，互不相同 |
| multimodal 连续两次 | 两次 `goal_reached`；episode ID 为 `3452a238…`、`f8bce643…`，互不相同 |
| traceback / shutdown error | 无 |
| 残留 ROS 节点或子进程 | 无；四次 launch 的 child processes 均 clean exit |

### M2 验收结论

M2 **正式技术验收通过**：clean Humble build 成功；最终隔离运行 204/204、零错误、零失败、零跳过；joint-only 与 rgb-joint 两种 demo 均连续两次 `goal_reached`；RGB 实际到达 backend；精确与近似 header 同步、monotonic freshness、snapshot sequence gate、Goal/wait epoch、四条图像/同步故障、one-shot latest-joint hold、等待取消、三类 first-wins 竞争、五组件 diagnostics、故障恢复与 clean shutdown 均有运行或相应边界测试证据。

已知限制仍是 README/architecture 中列出的软件边界：单 raw RGB camera、`rgb8`/`bgr8`、单同步 Python worker、非硬实时、hold 非安全认证、无 depth/TF/模型加载/远程推理等扩展。

发布记录将在本验收文档写入后完成：commit message 应为 `feat: add synchronized multimodal observations`，随后创建 annotated tag `v0.3.0-m2`。此时最终 commit hash 尚未产生，因此没有在本文虚构；仓库未配置 remote，所以完成后也不会虚构 push。

## M1 已确认历史基线（2026-07-15）

以下记录对应 M1“Runtime Fault Handling and Deterministic Hold”的已完成实际验收。M0 的基础动作链路、单活动目标约束和 `ExecutePolicy.action` 接口仍作为基线；该历史结果不能替代上方 M2 的新验收。

### M1 验证环境

| 项目 | 实测值 |
| --- | --- |
| 官方容器镜像 | `ros:humble-ros-base-jammy` |
| 操作系统 | Ubuntu 22.04.5 LTS，x86_64 |
| ROS 发行版 | ROS 2 Humble，`ROS_DISTRO=humble` |
| Python | 3.10.12 |
| ROS 工作区 | `/root/ws` |
| 源码位置 | `/root/ws/src/PolicyBridge-ROS2` |
| 本地开发主机 | Windows / PowerShell |

### M1 干净依赖解析与构建

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

### M1 测试与静态检查

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

### M1 包、接口、可执行文件与安装资产

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

### M1 launch 故障矩阵

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

### M1 确定性 hold 证据

hold 使用“最近一次有效的 6 维关节状态”，通过与普通命令相同的 `/joint_command` 发布边界发送。集成测试中的直接观测为：

- 普通故障场景的最近有效状态为 `[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]`，终止时最后一条命令与该状态完全相同；
- 取消场景在取消前注入的新近状态为 `[0.5, 0.5, 0.5, 0.5, 0.5, 0.5]`，最后一条命令与该状态完全相同；
- 每个需要安全停止的终止最多发布一次 hold；hold 是该目标的最后一条命令，继续观察后没有普通命令越过该边界；
- 从未收到有效状态的 `observation_timeout` 不发布 hold，明确记录 `safe_stop_unavailable_no_valid_state`，没有用全零状态掩盖未知姿态。

因此这里的“deterministic hold”是消息层面的确定性命令语义；它不等同于已证明物理机械臂静止。

### M1 Diagnostics 发现与故障恢复

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

### M1 连续两次正常 launch

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

### M1 已知边界

Python 不能安全强杀一个任意且永不返回的同步 backend 线程。M1 会按时终止 Action、忽略迟到结果，并在实际调用结束前维持 `backend_busy`、拒绝新目标；但若第三方 backend 永久不返回，不能承诺该 Python 进程自行干净退出。仓库内置的 scripted、delayed、invalid 和 raising 测试 backend 都是有限执行，以上 launch 与重复运行均已 clean exit。

本记录也不声称：

- hold 已让真实硬件达到物理静止；
- Python/ROS 2 执行器具备硬实时保证；
- 已完成真实机械臂、外部 learned-policy 服务、运动规划或仿真器集成。

### M1 验收结论

在官方 ROS 2 Humble 容器中，M1 已通过干净依赖解析、双包构建、90 项零失败零跳过测试、真实 Action/Topic/Diagnostics launch 集成、故障后复用和连续两次正常运行。结果支持本阶段声明的运行时超时、first-wins 终止、单 worker busy 门控、迟到结果隔离、最近有效状态 hold、无有效状态不伪造命令以及三组件诊断；声明范围不超过上述消息层与测试环境边界。
