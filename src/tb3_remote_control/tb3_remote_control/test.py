# To activate virtual environment: source .venv/bin/activate
# We do this because Ubuntu 24.04 intentionally forbids installing Python packages globally with pip, because it can break system tools that depend on Python.
# The folder tb3_devspace contains ROS packages, we do not wish to mix system-level python packages with ROS, venv allows us to:
# Isolate python dependencis thereby enabling different projects to have different dependencies.

from pynput import keyboard

"""

ros2 launch turtlebot3_navigation2 navigation2.launch.py
  map:=$HOME/Desktop/tb3_devspace/maps/my_room_map.yaml
  params_file:=$HOME/Desktop/tb3_devspace/burger_custom.yaml

  ANDERS:
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  map:=$HOME/Workspaces/sem3/map.yaml \
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
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0

        # Facing forward (yaw = 0) -> quarternion (0, 0, 0, 1)
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

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

        self.current_lin = 0.0
        self.current_ang = 0.0

        # Publisher
        self.pub = self.create_publisher(TwistStamped, self.vel_cmd_topic, 10)  # Creates the ROS2 publisher

        # Timer to regularly publish velocity states otherwise TB3 stops, this is a safety function
        self.timer = self.create_timer(0.05, self._vel)

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
            self.current_lin += self.lin_acc
            self.get_logger().info(f"Up pressed, new speed: {self.current_lin:.2f} m/s")
        if key == keyboard.Key.down:
            self.current_lin -= self.lin_acc
            self.get_logger().info(f"Down pressed, new speed: {self.current_lin:.2f} m/s")
        if key == keyboard.Key.left:
            self.current_ang += self.ang_acc
            self.get_logger().info(f"left pressed, new speed: {self.current_ang:.2f} m/s")
        if key == keyboard.Key.right:
            self.current_ang -= self.ang_acc
            self.get_logger().info(f"left pressed, new speed: {self.current_ang:.2f} m/s")
        elif key == keyboard.Key.space:
            self._stop()

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

#--------------------------------------------------------------------------------------------------------

def main():
    rclpy.init()

    # Create BOTH nodes
    manual = ManualDrive()
    initial = InitialPosePublisher()

    # Use multithreaded executor so both nodes run concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(manual)
    executor.add_node(initial)

    # Start keyboard listener and bind it to manual drive
    def on_press(key):
        manual.handle_key(key)

    def on_release(key):
        if key == keyboard.Key.esc:
            manual.get_logger().info("ESC pressed -> shutting down.")
            manual._stop()
            return False  # Stop listener

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        executor.spin()  # Run both nodes
    except KeyboardInterrupt:
        manual.get_logger().info("Ctrl+C detected. Shutting down.")
    finally:
        listener.stop()
        manual.destroy_node()
        initial.destroy_node()
        rclpy.shutdown()



def main1():
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    