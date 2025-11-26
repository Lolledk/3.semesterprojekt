#!/usr/bin/env python3
import subprocess

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Tb3Driver(Node):
    def __init__(self):
        super().__init__("tb3_driver")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def handle_command(self, text: str):
        text = text.lower().strip()
        self.get_logger().info(f"ASR text: {text!r}")
        msg = Twist()
        if "forward" in text:
            msg.linear.x = 0.1
        elif "back" in text:
            msg.linear.x = -0.1
        elif "left" in text:
            msg.angular.z = 0.5
        elif "right" in text:
            msg.angular.z = -0.5
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = Tb3Driver()

    # Start the ASR process (streamAudio.py should print one command per line)
    proc = subprocess.Popen(
        ["python3", "streamAudio.py"],
        stdout=subprocess.PIPE,
        text=True,
        bufsize=1
    )

    for line in proc.stdout:
        if not rclpy.ok():
            break
        node.handle_command(line)

    proc.terminate()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
