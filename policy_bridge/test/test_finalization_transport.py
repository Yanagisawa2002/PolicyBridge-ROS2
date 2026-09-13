"""Read-only audit probes against exact checkout methods; NO ROS integration.

Only ROS imports are replaced by minimal message/handle/transport doubles.
The policy server class and its method bodies are compiled unchanged from source.
Each probe confirms a narrowly stated behavior, not hardware or transport safety.
All blocked workers are explicitly released and joined.
"""
from __future__ import annotations

import ast
import asyncio
import hashlib
import json
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from enum import Enum
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "policy_bridge"))
import numpy as np
from policy_bridge.observation import ObservationSnapshot, validate_image_layout
from policy_bridge.observation_runtime import ObservationStore
from policy_bridge.runtime_state import RuntimeStateMachine
from policy_bridge.scripted_policy import ScriptedPolicy

class Message:
    def __init__(self, **kwargs):
        self.header = NS(stamp=NS(sec=1, nanosec=0), frame_id="camera")
        self.__dict__.update(kwargs)

class DiagnosticStatus(Message):
    OK, WARN, ERROR = 0, 1, 2

class Response(Enum):
    REJECT = 1
    ACCEPT = 2

class Logger:
    def __getattr__(self, name):
        return lambda *args, **kwargs: None

class Node:
    def get_logger(self):
        return Logger()
    def get_clock(self):
        return NS(now=lambda: NS(to_msg=lambda: NS(sec=1, nanosec=0)))

class Publisher:
    def __init__(self):
        self.messages = []
        self.calls = 0
        self.fail_when = lambda: False
    def publish(self, message):
        self.calls += 1
        if self.fail_when():
            raise RuntimeError("injected diagnostics transport failure")
        self.messages.append(message)

class Handle:
    def __init__(self, timeout=1.0, number=1):
        self.request = NS(timeout_seconds=timeout, max_steps=2, instruction="move to home")
        self.goal_id = NS(uuid=bytes([number]) * 16)
        self.is_active = True
        self.is_cancel_requested = False
        self.state = "EXECUTING"
        self.feedback = []
    def succeed(self):
        self.is_active, self.state = False, "SUCCEEDED"
    def abort(self):
        self.is_active, self.state = False, "ABORTED"
    def canceled(self):
        self.is_active, self.state = False, "CANCELED"
    def publish_feedback(self, feedback):
        self.feedback.append(feedback)

def load_server():
    source = ROOT / "policy_bridge/policy_bridge/policy_server.py"
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    excluded = {
        "rclpy", "message_filters", "diagnostic_msgs.msg", "sensor_msgs.msg",
        "std_msgs.msg", "policy_bridge_interfaces.action"
    }
    tree.body = [
        item for item in tree.body
        if not (
            isinstance(item, ast.Import)
            and any(alias.name in excluded for alias in item.names)
        )
        and not (
            isinstance(item, ast.ImportFrom)
            and item.level == 0
            and (item.module in excluded or (item.module or "").startswith("rclpy."))
        )
    ]
    module = ModuleType("policy_bridge._audit_server")
    module.__package__ = "policy_bridge"
    module.__dict__.update(
        Node=Node, DiagnosticArray=Message, DiagnosticStatus=DiagnosticStatus,
        KeyValue=Message, Float64MultiArray=Message, GoalResponse=Response,
        CancelResponse=Response,
        ExecutePolicy=NS(Goal=Message, Result=Message, Feedback=Message)
    )
    sys.modules[module.__name__] = module
    exec(compile(tree, str(source), "exec"), module.__dict__)
    return module

server = load_server()

def new_server(mode="joint_only", clock=time.monotonic):
    node = object.__new__(server.PolicyActionServer)
    node._runtime = RuntimeStateMachine(
        backend_name="scripted", inference_timeout_seconds=1.0,
        joint_state_timeout_seconds=1.0, clock=clock
    )
    node._observations = ObservationStore(
        observation_mode=mode, image_timeout_seconds=1.0,
        synchronized_observation_timeout_seconds=1.0,
        sync_queue_size=10, sync_slop_seconds=0.05, now=clock
    )
    node._joint_names = tuple(f"joint_{i}" for i in range(1, 7))
    node._latest_joint_positions = np.zeros(6)
    node._joint_state_lock = threading.Lock()
    node._command_gate_lock = threading.RLock()
    node._finalization_lock = threading.RLock()
    node._diagnostic_event_lock = threading.Lock()
    node._active_goal_observation_epoch = None
    node._finalized_episode_id = ""
    node._finalized_result = None
    node._diagnostic_event_level = 0
    node._diagnostic_event_message = "idle"
    node._shutdown_requested = threading.Event()
    node._execution_wake_event = threading.Event()
    node._command_publisher = Publisher()
    node._diagnostics_publisher = Publisher()
    node._control_rate_hz = 10.0
    node._goal_tolerance = 0.005
    node._policy = ScriptedPolicy()
    node._policy_executor = ThreadPoolExecutor(max_workers=1)
    return node

def add_snapshot(node, sequence=1, stamp=None, positions=None):
    if stamp is None:
        stamp = time.monotonic_ns()
    rgb = np.zeros((1, 1, 3), dtype=np.uint8) if node._observations.image_required else None
    value = ObservationSnapshot(
        sequence_id=sequence,
        joint_positions=np.zeros(6) if positions is None else positions,
        joint_names=node._joint_names, rgb=rgb,
        joint_stamp_ns=1_000_000_000, image_stamp_ns=1_000_000_000 if rgb is not None else None,
        received_monotonic_ns=stamp,
        synchronization_skew_ms=0.0 if rgb is not None else None,
        image_frame_id="camera" if rgb is not None else None
    )
    node._runtime.record_observation(valid=True)
    node._observations.commit_snapshot(value)
    return value

def accept(node, handle):
    assert node._goal_callback(handle.request) is Response.ACCEPT

import pytest

@pytest.mark.parametrize("failure_phase", ["startup", "terminal", "cleanup", "always"])
def test_diagnostic_failure_preserves_result_and_releases_admission(failure_phase):
    node, handle = new_server(), Handle()
    try:
        add_snapshot(node)
        accept(node, handle)
        def should_fail():
            if failure_phase == "startup":
                return node._diagnostics_publisher.calls == 2
            if failure_phase == "terminal":
                return node._runtime.termination_decision(node._episode_id(handle)) is not None
            if failure_phase == "cleanup":
                return node._finalized_result is not None and not node._runtime.snapshot().active_goal
            return True
        node._diagnostics_publisher.fail_when = should_fail
        result = asyncio.run(node._execute_callback(handle))
        assert result.success and handle.state == "SUCCEEDED"
        assert not node._runtime.snapshot().active_goal
        assert node._goal_callback(Handle(number=2).request) is Response.ACCEPT
        assert node._runtime.abandon_reservation()
    finally:
        node._policy_executor.shutdown(wait=True)
