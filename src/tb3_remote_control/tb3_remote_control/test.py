# To activate virtual environment: source .venv/bin/activate
# We do this because Ubuntu 24.04 intentionally forbids installing Python packages globally with pip, because it can break system tools that depend on Python.
# The folder tb3_devspace contains ROS packages, we do not wish to mix system-level python packages with ROS, venv allows us to:
# Isolate python dependencis thereby enabling different projects to have different dependencies.

"""
To create map

Report: What are Global costmap vs Local costmap in navigation2?

To launch navigation2 with custom map and parameters:

ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  map:=$HOME/Desktop/tb3_devspace/maps/my_room_map.yaml \
  params_file:=$HOME/tb3_devspace/burger_custom.yaml

"""

from pynput import keyboard

"""

ros2 topic echo /initialpose
ros2 topic echo /amcl_pose
ros2 run tf2_ros tf2_echo map base_link


ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  map:=$HOME/tb3_devspace/homemap.yaml \
  params_file:=$HOME/tb3_devspace/burger_custom.yaml

  ANDERS:
ros2 launch turtlebot3_navigation2 navigation2.launch.py
  map:=$HOME/Workspaces/sem3/map.yaml
  params_file:=$HOME/Workspaces/sem3/burger_custom.yaml

  to get coordinates run:
  ros2 run tf2_ros tf2_echo map base_link
 """

"""
amcl_pose publishes AMCL's best estimate of the robot's position and outputs PoseWithCovarianceStamped
PoseWithCovarianceStamped is
This differs from the TF transform due to:
 - localization still converging
 - covariance is large...

tf2_echo map base_link is the transform tree that connects all robot frames: map -> odom -> base_link
This is the pose actually used by the navigation system.
/amcl_pose -> Nav2 interprets it and updates a TF transform called map -> odom, now TF knows where base_link is in map

When these two converge within given tolerances, we are ready to navigate
"""

# To build and source
# cd ~/tb3_devspace
# colcon build --packages-select tb3_remote_control
# source ~/tb3_devspace/install/setup.bash

#export TURTLEBOT3_MODEL=burger
#ros2 launch turtlebot3_bringup robot.launch.py

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, PoseWithCovarianceStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, # Sends messages of type PoseWithCovarianceStamped
                                          '/initialpose',           # Publishes on topic /initialpose
                                          10)                       # QoS

        self.sub = self.create_subscription(PoseWithCovarianceStamped,
                                             '/amcl_pose', 
                                             self.amcl_pose,
                                             10)

        # Give  Nav2/AMCL a moment to start
        self.timer = self.create_timer(5.0, self.publish_initial_pose)

        self.current_pose = None

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()                           # Creates an empty PoseWithCovarianceStamped message.
        msg.header.frame_id = 'map'                                 # Initial pose must be given in map frame, same as RViz selects "map" when you click 2D Pose Estimate
        msg.header.stamp = self.get_clock().now().to_msg()          # Provides the current ROS timestamp, this is required for Nav2 modules

        "Position components"
        # Facing along +x
        msg.pose.pose.position.x = -0.038
        msg.pose.pose.position.y = -0.124
        msg.pose.pose.position.z = 0.0

        # Facing forward (yaw = 9.363) -> quarternion (0, 0, 0, 1)
        yaw_deg = 2.829
        yaw = math.radians(yaw_deg)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)  # z
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)  # w

        # Minimal covariance, these are set to estimate the confidence in the initial position
        # If you set the covariance too high, the robot will “wander” and AMCL will struggle.
        # If too low, AMCL may refuse to update because it thinks the pose is perfect.
        # How to relate this to an actual unit?
        msg.pose.covariance[0]  = 0.1 # x
        msg.pose.covariance[7]  = 0.1 # y
        msg.pose.covariance[35] = 0.1 # yaw

        self.get_logger().info("Publishing initial pose to /initialpose")
        self.get_logger().info("")
        self.pub.publish(msg)

        # Such that initial pose is only shared once and not every 2 seconds.
        self.timer.cancel()

    def amcl_pose(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        self.current_pose = (x,y,z,w)

        self.get_logger().info(f"Current pose: x={x:.2f}, y={y:.2f}, w={w:.2f}")

#--------------------------------------------------------------------------------------------------------------------

class Nav2GoalNode(Node):
    def __init__(self):
        super().__init__('nav2_goal_node')

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x: float, y: float, yaw_deg: float):
        """Send a Nav2 NavigateToPose goal in the 'map' frame."""
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 'navigate_to_pose' action server not available!")
            return
        
        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id    = 'map'
        pose.header.stamp       = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        # yaw in degrees -> quarternion (z,w)
        yaw = math.radians(yaw_deg)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal_msg.pose = pose

        self.get_logger().info(
            f"Sending Nav2 goal: x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f}°"
        )

        send_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb
        )
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal was rejected.")
            return

        self.get_logger().info("Nav2 goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        # result.status, result.result
        self.get_logger().info(f"Nav2 goal finished with status: {result.status}")

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        # Nav2 feedback usually has distance_remaining
        if hasattr(fb, 'distance_remaining') and fb.distance_remaining is not None:
            self.get_logger().info(
                f"Nav2 feedback: distance remaining {fb.distance_remaining:.2f} m"
            )


#--------------------------------------------------------------------------------------------------------------------
class ManualDrive(Node):
    def __init__(self):
        super().__init__('manual_drive')

        # Parameters set in ROS2 manner
        self.declare_parameter('linear_acc', 0.025)
        self.declare_parameter('angular_acc', 0.25)                     # Test value
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')                      # Standard name to publish velocity commands to for TB3

        self.lin_acc = float(self.get_parameter('linear_acc').value)
        self.ang_acc = float(self.get_parameter('angular_acc').value)
        self.vel_cmd_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.max_lin = 0.20  # m/s
        self.max_ang = 2.5  # rad/s
        self.current_lin = 0.0
        self.current_ang = 0.0

        # Publisher
        self.pub = self.create_publisher(TwistStamped, self.vel_cmd_topic, 10)  # Creates the ROS2 publisher

        # Timer to regularly publish velocity states otherwise TB3 stops, this is a safety function
        self.timer = self.create_timer(0.05, self._vel) #20 hz

    def _vel(self):
        """Set velocities"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.current_lin
        msg.twist.angular.z = self.current_ang
        self.pub.publish(msg)

    def _stop(self):
        """Set velocities to zero and publish"""
        self.current_lin = 0.0
        self.current_ang = 0.0
        self._vel()
        self.get_logger().info('Stop command has been issued')

    def handle_key(self, key):
        """To interpret keyboard commands"""
        if key == keyboard.Key.up:
            if (self.current_lin + self.lin_acc) > self.max_lin:
                self.current_lin = self.max_lin
                self.get_logger().info(f"Up pressed, max speed reached: {self.current_lin:.2f} m/s")    
            else:
                self.current_lin += self.lin_acc
                self.get_logger().info(f"Up pressed, new speed: {self.current_lin:.2f} m/s")
        
        if key == keyboard.Key.down:
            if (self.current_lin - self.lin_acc) < -self.max_lin:
                self.current_lin = -self.max_lin
                self.get_logger().info(f"Down pressed, max speed reached: {self.current_lin:.2f} m/s")
            else:
                self.current_lin -= self.lin_acc
                self.get_logger().info(f"Down pressed, new speed: {self.current_lin:.2f} m/s")
        
        if key == keyboard.Key.left:
            if (self.current_ang + self.ang_acc) > self.max_ang:
                self.current_ang = self.max_ang
                self.get_logger().info(f"left pressed, max speed reached: {self.current_ang:.2f} m/s")
            else:
                self.current_ang += self.ang_acc
                self.get_logger().info(f"left pressed, new speed: {self.current_ang:.2f} m/s")
        
        if key == keyboard.Key.right:
            if (self.current_ang - self.ang_acc) < -self.max_ang:
                self.current_ang = -self.max_ang
                self.get_logger().info(f"right pressed, max speed reached: {self.current_ang:.2f} m/s")
            else:
                self.current_ang -= self.ang_acc
                self.get_logger().info(f"right pressed, new speed: {self.current_ang:.2f} m/s")
        if key.char == 'w':  #keyboard.Key.up:
            self.current_lin += self.lin_acc
            self.get_logger().info(f"Up pressed, new speed: {self.current_lin:.2f} m/s")
        if key.char == 's':  #keyboard.Key.down:
            self.current_lin -= self.lin_acc
            self.get_logger().info(f"Down pressed, new speed: {self.current_lin:.2f} m/s")
        if key.char == 'a':  #keyboard.Key.left:
            self.current_ang += self.ang_acc
            self.get_logger().info(f"left pressed, new speed: {self.current_ang:.2f} m/s")
        if key.char == 'd':  #keyboard.Key.right:
            self.current_ang -= self.ang_acc
            self.get_logger().info(f"left pressed, new speed: {self.current_ang:.2f} m/s")
        elif key == keyboard.Key.space:
            self._stop()
"""
class Calibrate(Node):
    def __init__(self):
        super().__init__('calibrate')

        # Declare parameters
        self.declare_parameter('angular_speed', 0.5)  # rad/s
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        # Create TwistStamped publisher
        self.pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        # Rotation bookkeeping
        self.total_rotated = 0.0      # accumulated rotation (radians)
        self.last_time = self.get_clock().now()

        # Timer → 20 Hz update rate
        self.timer = self.create_timer(0.05, self._spin_once)

        self.get_logger().info("Calibrate node initialized. Rotating 360°...")

    def _spin_once(self):
        Rotate the robot until 2π radians is reached.
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Integrate angular velocity
        self.total_rotated += abs(self.angular_speed) * dt

        # Stop if we reached 360 degrees
        if self.total_rotated >= 2 * math.pi:  # 6.283185…
            self._stop()
            self.get_logger().info("Rotation complete: 360° reached.")
            self.timer.cancel()
            return

        # Publish angular velocity
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.angular.z = self.angular_speed  # positive rotation
        self.pub.publish(msg)

        self.get_logger().info(f"Rotating… {math.degrees(self.total_rotated):.1f}° completed")

    def _stop(self):
        #Publish a zero TwistStamped to stop rotation.
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.angular.z = 0.0
        self.pub.publish(msg)
        self.get_logger().info("Stop command published.")


"""
"""
# Learn this #
def main1():
    rclpy.init()
    node = ManualDrive()
    #initial = InitialPosePublisher()

    # Define keyboard callbacks *inside* main so they can capture 'node'
    def on_press(key):
        # Delegate to the node method
        node.handle_key(key)

    def on_release(key):
        if key == keyboard.Key.esc:
            node.get_logger().info("ESC pressed -> shutting down.")
            node.stop()
            return False   # stops the keyboard listener

    # Start keyboard listener in a *background thread*
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C detected, stopping.')
        node.stop()
    finally:
        listener.stop()
        node.destroy_node()
        rclpy.shutdown()
"""
#--------------------------------------------------------------------------------------------------------
"""
def main():
    rclpy.init()

    manual = ManualDrive()
    initial = InitialPosePublisher()
    nav2_goal = Nav2GoalNode()
    cali = Calibrate()

    executor = MultiThreadedExecutor()
    executor.add_node(cali)
    executor.add_node(manual)
    executor.add_node(initial)
    executor.add_node(nav2_goal)

    # Keyboard handling
    def on_press(key):
        # Arrow keys / space -> teleop
        manual.handle_key(key)

        # Letter keys (char attribute) for Nav2 stuff
        if hasattr(key, 'char'):
            if key.char == 'g':
                # Example: send a goal at (1.0, 0.0, yaw=0°) in map frame
                nav2_goal.get_logger().info("Key 'g' pressed → sending Nav2 goal")
                nav2_goal.send_goal(0.5, 0.5, 0.0)

            # You can add more keys, e.g. 'h' for another goal

    def on_release(key):
        if key == keyboard.Key.esc:
            manual.get_logger().info("ESC pressed -> shutting down.")
            manual._stop()
            return False  # Stop listener

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        manual.get_logger().info("Ctrl+C detected. Shutting down.")
    finally:
        listener.stop()
        manual.destroy_node()
        initial.destroy_node()
        nav2_goal.destroy_node()
        rclpy.shutdown()
"""

def main():
    rclpy.init()
    #nav2_goal = Nav2GoalNode()
    node = InitialPosePublisher()   # <-- create an instance

    try:
        rclpy.spin(node)
        #rclpy.spin(nav2_goal)
        #nav2_goal.send_goal(0.5, 0.5, 0.0)
    finally:
        node.destroy_node()
        #nav2_goal.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    
