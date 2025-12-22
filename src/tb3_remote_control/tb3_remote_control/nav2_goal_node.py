#!/usr/bin/env python3

# To activate virtual environment: source .venv/bin/activate

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

import tf2_ros
from tf2_ros import TransformException



class Nav2GoalClient(Node):
    def __init__(self):
        super().__init__('nav2_goal_client')

        # Matches what ros2 action list shows:
        # /navigate_to_pose [nav2_msgs/action/NavigateToPose]
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # TF2 listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Storage for trajectory
        self.trajectory = []

    
    def sample_robot_pose(self):
        """Read map -> base_link transform and store it."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = tf.transform.translation.x
            y = tf.transform.translation.y

            q = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

            t = time.perf_counter()
            self.trajectory.append((t, x, y, yaw))

        except TransformException:
            # Normal during startup
            pass

    def save_trajectory_csv(self, filename="trajectory.csv"):
        with open(filename, "w") as f:
            f.write("time,x,y,yaw\n")
            for t, x, y, yaw in self.trajectory:
                f.write(f"{t},{x},{y},{yaw}\n")

        self.get_logger().info(f"Trajectory written to {filename}")


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

        nav_start_time = time.perf_counter()

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
        #result_future = goal_handle.get_result_async()
        #rclpy.spin_until_future_complete(self, result_future)

        result_future = goal_handle.get_result_async()

        # -------- SAMPLE TRAJECTORY WHILE NAVIGATING --------
        while rclpy.ok() and not result_future.done():
            self.sample_robot_pose()
            rclpy.spin_once(self, timeout_sec=0.1)

        if not result_future.done():
            self.get_logger().error("Result future did not complete.")
            return

        nav_end_time = time.perf_counter()
        nav_duration = nav_end_time - nav_start_time

        result = result_future.result()
        status = result.status
        self.get_logger().info(f"Nav2 goal finished with status: {status}")
        # result.result is the NavigateToPose result message if you want to inspect it further
        self.get_logger().info(f"Navigation took {nav_duration:.2f} seconds.")
    
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
        #node.send_goal_and_wait(0.069, -0.07, 47.440)
        node.send_goal_and_wait(-1.066, -1.180, -135.0) 
        node.save_trajectory_csv()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
