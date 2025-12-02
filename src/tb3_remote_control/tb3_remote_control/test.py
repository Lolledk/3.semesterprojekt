# To activate virtual environment: source .venv/bin/activate
# We do this because Ubuntu 24.04 intentionally forbids installing Python packages globally with pip, because it can break system tools that depend on Python.
# The folder tb3_devspace contains ROS packages, we do not wish to mix system-level python packages with ROS, venv allows us to:
# Isolate python dependencis thereby enabling different projects to have different dependencies.

from pynput import keyboard

# To build and source
# cd ~/tb3_devspace
# colcon build --packages-select tb3_remote_control
# source ~/tb3_devspace/install/setup.bash


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped  # <-- add TwistStamped

"""
This is where the magic actually starts.

keyboard.Listener(...)

Creates a Listener object that:

Listens to global keyboard events.
Calls on_press when a key is pressed.
Calls on_release when a key is released.

on_press=on_press

Tells the Listener:
"Whenever a key is pressed, call this function."

So the on_press function you defined earlier is passed in as a callback.
on_release=on_release
Same idea for key releases.

with ... as listener:

This uses a context manager (the with block) to:

Start the listener automatically when entering the block.
Stop and clean it up automatically when exiting (after return False or an exception).
Inside this block, the listener is active.

listener.join()

.join() blocks the main thread and waits until the listener stops.

In practice, this means:
The script will not exit immediately.
It will keep running, listening to keys, until:

You press ESC (which triggers on_release, which returns False, which stops the listener).
Or the program is otherwise terminated.
"""

"""
print("Listening for keyboard inputs...")

def on_press(key):
    try:
        print(f"Pressed: {key.char}")
    except AttributeError:
        print(f"Special key: {key}")

def on_release(key):
    if key == keyboard.Key.esc:
        print("Exiting...")
        return False
def hi(key):
    if key == keyboard.Key.up:
        print("hi")
    else:
        print("not up")

with keyboard.Listener(on_press=hi, on_release=on_release) as listener:
    listener.join()
"""


class ManualDrive(Node):
    def __init__(self):
        super().__init__('manual_drive')

        # Parameters set in ROS2 manner
        self.declare_parameter('linear_acc', 0.05)
        self.declare_parameter('angular_acc', 0.5)                     # Test value
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
def main():
    rclpy.init()
    node = ManualDrive()

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


if __name__ == '__main__':
    main()
    