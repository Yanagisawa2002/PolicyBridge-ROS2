"""One-shot ExecutePolicy action client used by the M0 launch demo."""

from __future__ import annotations

from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future

from policy_bridge_interfaces.action import ExecutePolicy


class DemoClient(Node):
    """Send one parameterized policy goal and print feedback and result."""

    def __init__(self) -> None:
        """Declare goal parameters and construct the action client."""

        super().__init__("demo_client")
        self.declare_parameter("action_name", "execute_policy")
        self.declare_parameter("instruction", "move to home")
        self.declare_parameter("max_steps", 200)
        self.declare_parameter("timeout_seconds", 0.0)

        self._action_name = str(self.get_parameter("action_name").value)
        self._instruction = str(self.get_parameter("instruction").value)
        self._max_steps = int(self.get_parameter("max_steps").value)
        self._timeout_seconds = float(self.get_parameter("timeout_seconds").value)
        self._action_client = ActionClient(self, ExecutePolicy, self._action_name)

    def send_goal(self) -> None:
        """Wait for the server and asynchronously send the configured goal."""

        self.get_logger().info(f"Waiting for ExecutePolicy action server '{self._action_name}'")
        while rclpy.ok() and not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Action server not ready; still waiting")
        if not rclpy.ok():
            return

        goal = ExecutePolicy.Goal()
        goal.instruction = self._instruction
        goal.max_steps = self._max_steps
        goal.timeout_seconds = self._timeout_seconds

        self.get_logger().info(
            f"Sending instruction {self._instruction!r} with max_steps={self._max_steps}"
        )
        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _feedback_callback(self, feedback_message: Any) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            f"Feedback: step={feedback.current_step} "
            f"progress={feedback.progress:.3f} "
            f"inference_latency_ms={feedback.inference_latency_ms:.3f}"
        )

    def _goal_response_callback(self, future: Future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("ExecutePolicy goal was rejected")
            rclpy.shutdown()
            return

        self.get_logger().info("ExecutePolicy goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future: Future) -> None:
        response = future.result()
        result = response.result
        status = response.status
        summary = (
            f"success={result.success} "
            f"termination_reason={result.termination_reason} "
            f"episode_id={result.episode_id}"
        )
        if status == GoalStatus.STATUS_SUCCEEDED and result.success:
            self.get_logger().info(f"Result: {summary}")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"Canceled result: {summary}")
        else:
            self.get_logger().error(f"Unsuccessful result (status={status}): {summary}")
        rclpy.shutdown()


def main(args: list[str] | None = None) -> None:
    """Run the one-shot demo client until its result arrives."""

    rclpy.init(args=args)
    node = DemoClient()
    node.send_goal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
