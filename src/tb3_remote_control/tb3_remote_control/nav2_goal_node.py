#!/usr/bin/env python3

# To activate virtual environment: source .venv/bin/activate

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class Nav2GoalClient(Node):
    def __init__(self):
        super().__init__('nav2_goal_client')

        # Matches what ros2 action list shows:
        # /navigate_to_pose [nav2_msgs/action/NavigateToPose]
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal_and_wait(self, x: float, y: float, yaw_deg: float):
        """Send one NavigateToPose goal and block until result."""

        # 1) Wait for server
        self.get_logger().info("Waiting for Nav2 '/navigate_to_pose' action server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 '/navigate_to_pose' action server NOT available.")
            return

        # 2) Build goal (same structure as your working CLI command)
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # same as CLI: frame_id: 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        # orientation: CLI used z=0.0, w=1.0 (yaw = 0)
        yaw = math.radians(yaw_deg)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal.pose = pose

        self.get_logger().info(
            f"Sending Nav2 goal: x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}° in 'map' frame"
        )

        # 3) Send goal async, but block on the future so we’re effectively synchronous
        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb
        )

        self.get_logger().info("Waiting for goal handle...")
        rclpy.spin_until_future_complete(self, send_future)

        if not send_future.done():
            self.get_logger().error("Goal send_future did not complete.")
            return

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal was REJECTED by server.")
            return

        self.get_logger().info("Nav2 goal ACCEPTED, waiting for result...")

        # 4) Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        if not result_future.done():
            self.get_logger().error("Result future did not complete.")
            return

        result = result_future.result()
        status = result.status
        self.get_logger().info(f"Nav2 goal finished with status: {status}")
        # result.result is the NavigateToPose result message if you want to inspect it further

    def _feedback_cb(self, feedback_msg):
        """Print distance remaining feedback."""
        fb = feedback_msg.feedback
        # NavigateToPoseFeedback has distance_remaining
        if hasattr(fb, 'distance_remaining') and fb.distance_remaining is not None:
            self.get_logger().info(
                f"[Feedback] Distance remaining: {fb.distance_remaining:.2f} m"
            )


def main():
    rclpy.init()
    node = Nav2GoalClient()

    try:
        # Pick a goal you know is reachable (same as the CLI example)
        node.send_goal_and_wait(0.3, 0.3, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
