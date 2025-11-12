# To build and source
# cd ~/tb3_devspace
# colcon build --packages-select tb3_remote_control
# source ~/tb3_devspace/install/setup.bash


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped  # <-- add TwistStamped

class DriveForward(Node):
    def __init__(self):
        super().__init__('drive_forward_10cm')

        # Parameters
        self.declare_parameter('distance_m', 2.0)
        self.declare_parameter('speed_mps', 0.2)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')      # relative name, respects namespaces
        self.declare_parameter('use_stamped', True)             # <-- new

        self.distance = float(self.get_parameter('distance_m').value)
        self.speed = float(self.get_parameter('speed_mps').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.use_stamped = bool(self.get_parameter('use_stamped').value)

        # Publisher: pick type based on param
        if self.use_stamped:
            self.pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        else:
            self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Timing
        if self.speed <= 0.0:
            self.get_logger().warn('speed_mps <= 0, clamping to 0.01')
            self.speed = 0.01
        self.duration = self.distance / self.speed
        self.end_time = self.get_clock().now() + rclpy.time.Duration(seconds=self.duration)

        self.timer = self.create_timer(0.05, self._tick)  # 20 Hz
        self._stopped = False

        self.get_logger().info(
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
                msg.twist.linear.x = self.speed
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
            stop = TwistStamped()
            stop.header.stamp = self.get_clock().now().to_msg()
            stop.header.frame_id = 'base_link'
            # zero twist is default
            self.pub.publish(stop)
        else:
            self.pub.publish(Twist())

    def _shutdown_once(self):
        self.destroy_timer(self.timer)
        rclpy.shutdown()

def main():
    rclpy.init()
    node = DriveForward()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted. Stopping.')
        node._stop()
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
