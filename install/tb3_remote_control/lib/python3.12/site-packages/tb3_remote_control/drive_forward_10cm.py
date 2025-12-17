# To build and source
# cd ~/tb3_devspace
# colcon build --packages-select tb3_remote_control
# source ~/tb3_devspace/install/setup.bash


#!/usr/bin/env python3 
# shebang tells the OS to use Python3 from current env to run this file
import rclpy                                        # Imports the main ROS2 Python client library
from rclpy.node import Node                         # Imports the Node base class which enables us to publish, subscribe, create timers, etc...
from geometry_msgs.msg import Twist, TwistStamped   # Imports messages types: Twist: velocity command without timestamp, TwistStamped: same as Twist but wrapped with a std_msgs/Header(includes stamp and frame_id)

class DriveForward(Node):                                                                                   # Inherits from node
    def __init__(self):                                                                                     # Constructor
        super().__init__('drive_forward_10cm')                                                              # Calls the base class Node and registers this node with ROS2 using the node name "drive_forward_10cm", it can be seen in: ros2 node list

        # Parameters                                                                                        # Makes a ROS parameter available on the node, gives it a name and default
        # By settings parameters as such instead of self.distance = x we get the oppurtinity to edit this values at runtime:    ros2 run tb3_remote_control drive_forward_10cm \
        self.declare_parameter('distance_m', 0.20)                                                                             #--ros-args -p distance_m:=0.5 -p speed_mps:=0.2
        self.declare_parameter('speed_mps', 0.1)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')      # relative name, respects namespaces        # topic name to publish to, default 'cmd_vel'
        self.declare_parameter('use_stamped', True)                                                         # whether to publish TwistStamped instead of Twist

        self.distance = float(self.get_parameter('distance_m').value)
        self.speed = float(self.get_parameter('speed_mps').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.use_stamped = bool(self.get_parameter('use_stamped').value)

        # Publisher: pick type based on param
        if self.use_stamped:
            self.pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)                          # Creates a ROS2 publisher QoS queue size of 10 means we save up to 10 values for processing, We choose this as the tb3 doesn't respond to Twist, reason not know??
        else:
            self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Timing                                                                                            # Used to calculate the duration in order to meet the specified travel distance
        if self.speed <= 0.0:
            self.get_logger().warn('speed_mps <= 0, clamping to 0.01')
            self.speed = 0.01
        self.duration = self.distance / self.speed
        self.end_time = self.get_clock().now() + rclpy.time.Duration(seconds=self.duration)                 # rclpy.time.Duration(seconds=self.duration) creates a time duration object representing duration many seconds

        self.timer = self.create_timer(0.05, self._tick)  # 20 Hz                                           # Calls callback every period seconds. _tick() runs every 0.05 seconds.
        self._stopped = False                                                                               # Flag to track whether or not stop command has been sent

        self.get_logger().info(                                                                             # Uses the node's logger to print the following:
            f'Driving {self.distance*100:.1f} cm @ {self.speed*100:.1f} cm/s (~{self.duration:.2f}s); '
            f'publishing {"TwistStamped" if self.use_stamped else "Twist"} on "{self.cmd_vel_topic}"'
        )

    def _tick(self):
        now = self.get_clock().now()
        if now < self.end_time:
            if self.use_stamped:
                msg = TwistStamped()
                msg.header.stamp = now.to_msg()
                msg.header.frame_id = 'base_link'  # typical; not strictly required for velocity
                msg.twist.linear.x = self.speed                                                             # If we wish to turn then: angular.z, drive in arc: msg.linear.x = 0.1, msg.angular.z = 0.5
                self.pub.publish(msg)
            else:
                msg = Twist()
                msg.linear.x = self.speed
                self.pub.publish(msg)
        else:
            if not self._stopped:
                self._stop()
                self._stopped = True
                self.get_logger().info('Done. Sent stop command.')
                self.create_timer(0.2, self._shutdown_once)

    def _stop(self):
        if self.use_stamped:
            stop = TwistStamped()                                                                           # TwisStamped() object has all fields initialized to 0 be default
            stop.header.stamp = self.get_clock().now().to_msg()
            stop.header.frame_id = 'base_link'
            # zero twist is default
            self.pub.publish(stop)
        else:
            self.pub.publish(Twist())

    def _shutdown_once(self):                                                                               # Shuts down the ROS2 client library: Stops all nodes, stops spinning
        self.destroy_timer(self.timer)
        rclpy.shutdown()

def main():                                                                                                 # 
    rclpy.init()                                                                                            # Initializes the ROS2 client library, must be called before creating nodes.
    node = DriveForward()                                                                                   # Constructs our node
    try:
        rclpy.spin(node)                                                                                    # Enters ROS2 event loop: Processes timers, subscription, services, etc...
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted. Stopping.')
        node._stop()                                                                                        # Calls rclpy.shutdown()
    finally:
        node.destroy_node()

if __name__ == '__main__':                                                                                  # calls the above main() function
    main()
