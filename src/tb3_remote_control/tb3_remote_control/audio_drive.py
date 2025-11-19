#!/usr/bin/env python3
import rclpy, time, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, TwistStamped

from faster_whisper import WhisperModel

TOPIC = "/audio_wav"
SERVICE = "/record_wav"
OUT = "capture.wav"
TIMEOUT_S = 30.0

FORWARD_DISTANCE = 0.10      # 10 cm
BACKWARD_DISTANCE = 0.10     # 10 cm
LIN_SPEED = 0.05             # 5 cm/s

TURN_ANGLE_DEG = 90.0        # degrees
ANG_SPEED = math.radians(30) # 30 deg/s

class AudioDrive(Node):
    def __init__(self):
        super().__init__("audio_drive")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Subscribe to audio from TB3
        self.sub = self.create_subscription(UInt8MultiArray, TOPIC, self.on_wav, qos)

        # Service to trigger recording on TB3
        self.cli = self.create_client(Trigger, SERVICE)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /record_wav service...")

        # PUBLISHER: match working script → TwistStamped on /cmd_vel
        self.cmd_vel_topic = "/cmd_vel"
        self.pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        # Trigger recording immediately
        self.get_logger().info("Calling /record_wav...")
        self.future = self.cli.call_async(Trigger.Request())
        self.start = time.time()

        # Load faster-whisper model once
        self.get_logger().info("Loading faster-whisper model (base/int8)…")
        self.model = WhisperModel("base", device="cpu", compute_type="int8")
        self.get_logger().info("Model loaded.")

        self.got_audio = False

    def on_wav(self, msg: UInt8MultiArray):
        if self.got_audio:
            return

        wav_bytes = bytes(msg.data)
        with open(OUT, "wb") as f:
            f.write(wav_bytes)
        self.get_logger().info(f"Saved WAV ({len(wav_bytes)} bytes). Running ASR...")

        # Transcribe
        segments, info = self.model.transcribe(OUT, language="en")
        text = " ".join(seg.text for seg in segments).strip().lower()
        self.get_logger().info(f"ASR result: '{text}'")
        print(f"ASR: {text}")

        # -------- COMMAND DISPATCH --------
        if any(cmd in text for cmd in ("move forward", "forward", "forward")):
            self.get_logger().info("Recognized command: MOVE FORWARD")
            self.move_forward()

        elif any(cmd in text for cmd in ("move backward", "go back", "backward")):
            self.get_logger().info("Recognized command: MOVE BACKWARD")
            self.move_backward()

        elif "turn left" in text:
            self.get_logger().info("Recognized command: TURN LEFT")
            self.turn_left()

        elif "turn right" in text:
            self.get_logger().info("Recognized command: TURN RIGHT")
            self.turn_right()

        elif "stop" in text:
            self.get_logger().info("Recognized command: STOP")
            self.stop_robot()
        else:
            self.get_logger().info("No known command detected in ASR text.")

        self.get_logger().info("Done.")
        self.got_audio = True
        rclpy.shutdown()

    # ---- Motion helpers (TwistStamped) ----
    def execute_twist(self, linear_x: float = 0.0, angular_z: float = 0.0, duration: float = 0.0):
        """
        Replicates the behavior of your working DriveForward node:
        publishes TwistStamped on /cmd_vel at ~20 Hz for 'duration' seconds.
        """
        start = time.time()
        while time.time() - start < duration and rclpy.ok():
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = linear_x
            msg.twist.angular.z = angular_z
            self.pub.publish(msg)
            time.sleep(0.05)  # 20 Hz

        # send one final stop
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = "base_link"
        self.pub.publish(stop)

    def move_forward(self):
        self.get_logger().info("Command: MOVE FORWARD 10 cm")
        duration = FORWARD_DISTANCE / LIN_SPEED
        self.execute_twist(linear_x=LIN_SPEED, angular_z=0.0, duration=duration)

    def move_backward(self):
        self.get_logger().info("Command: MOVE BACKWARD 10 cm")
        duration = BACKWARD_DISTANCE / LIN_SPEED
        self.execute_twist(linear_x=-LIN_SPEED, angular_z=0.0, duration=duration)

    def turn_left(self):
        self.get_logger().info(f"Command: TURN LEFT {TURN_ANGLE_DEG} deg")
        duration = math.radians(TURN_ANGLE_DEG) / ANG_SPEED
        self.execute_twist(linear_x=0.0, angular_z=ANG_SPEED, duration=duration)

    def turn_right(self):
        self.get_logger().info(f"Command: TURN RIGHT {TURN_ANGLE_DEG} deg")
        duration = math.radians(TURN_ANGLE_DEG) / ANG_SPEED
        self.execute_twist(linear_x=0.0, angular_z=-ANG_SPEED, duration=duration)

    def stop_robot(self):
        self.get_logger().info("Command: STOP")
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = "base_link"
        self.pub.publish(stop)

def main():
    rclpy.init()
    node = AudioDrive()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.got_audio:
            break

if __name__ == "__main__":
    main()