#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import Trigger

TOPIC = "/audio_wav"
SERVICE = "/record_wav"
OUT = "capture.wav"
TIMEOUT_S = 30.0

class RecordClientSaver(Node):
    def __init__(self):
        super().__init__("record_client_saver")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.got = False
        self.sub = self.create_subscription(UInt8MultiArray, TOPIC, self.on_wav, qos)
        self.cli = self.create_client(Trigger, SERVICE)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /record_wav...")
        self.get_logger().info("Calling /record_wav...")
        self.future = self.cli.call_async(Trigger.Request())
        self.start = time.time()

    def on_wav(self, msg: UInt8MultiArray):
        if self.got: return
        data = bytes(msg.data)
        with open(OUT, "wb") as f:
            f.write(data)
        self.get_logger().info(f"Saved {len(data)} bytes to {OUT}")
        self.got = True
        rclpy.shutdown()

rclpy.init()
node = RecordClientSaver()
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.1)
    if node.got: break
    if time.time() - node.start > TIMEOUT_S:
        node.get_logger().error("Timed out waiting for WAV")
        rclpy.shutdown()
        break
