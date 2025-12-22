#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger


class Nav2GoalNode(Node):
    def __init__(self):
        super().__init__('nav2_goal_node')

        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Service: when called, send a fixed Nav2 goal
        self._srv = self.create_service(
            Trigger,
            '/go_to_goal_1',            # service name
            self._go_to_goal_1_cb       # callback
        )

        self.get_logger().info("Nav2GoalNode ready (service /go_to_goal_1).")

    # ---------- Service callback ----------

    def _go_to_goal_1_cb(self, request, response):
        """Trigger: send goal (1.30, 0.0, 40 deg) and wait for completion."""
        self.get_logger().info("Service /go_to_goal_1 called, sending Nav2 goal...")
        ok = self.send_goal_and_wait(0.069, -0.07, 47.440)
        #ok = self.send_goal_and_wait(-1.066, -1.180, -135.0)
        response.success = ok
        response.message = "Goal sent" if ok else "Failed to send / complete goal"
        return response

    # ---------- Action helper (same logic as before, slightly refactored) ----------

    def send_goal_and_wait(self, x: float, y: float, yaw_deg: float) -> bool:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 '/navigate_to_pose' action server NOT available.")
            return False

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        yaw = math.radians(yaw_deg)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal.pose = pose

        self.get_logger().info(
            f"Sending Nav2 goal: x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}° in 'map' frame"
        )

        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb
        )

        self.get_logger().info("Waiting for goal handle...")
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal was REJECTED by server.")
            return False

        self.get_logger().info("Nav2 goal ACCEPTED, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        status = result.status
        self.get_logger().info(f"Nav2 goal finished with status: {status}")
        return True

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        if hasattr(fb, 'distance_remaining') and fb.distance_remaining is not None:
            self.get_logger().info(
                f"[Feedback] Distance remaining: {fb.distance_remaining:.2f} m"
            )


def main():
    rclpy.init()
    node = Nav2GoalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
