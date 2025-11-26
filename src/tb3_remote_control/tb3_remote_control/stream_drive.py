#!/usr/bin/env python3
import rclpy, time, math
import io, wave
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, TwistStamped
import time
from faster_whisper import WhisperModel
from collections import deque

TOPIC = "/audio_wav"
SERVICE = "/record_wav"
OUT = "capture.wav"
TIMEOUT_S = 30.0
INTERVAL = 0.1

FORWARD_DISTANCE = 0.10      # 10 cm
BACKWARD_DISTANCE = 0.10     # 10 cm
LIN_SPEED = 0.05             # 5 cm/s
FAST_SPEED = 0.1

TURN_ANGLE_DEG = 90.0        # degrees
ANG_SPEED = math.radians(30) # 30 deg/s

def wav_bytes_to_float_mono(wav_bytes: bytes) -> np.ndarray:
    """
    Take a full WAV file as bytes and return a 1D float32 numpy array in [-1, 1].
    Assumes 16-bit PCM mono; resamples not handled here.
    """
    buf = io.BytesIO(wav_bytes)
    with wave.open(buf, "rb") as wf:
        n_channels = wf.getnchannels()
        sampwidth = wf.getsampwidth()
        framerate = wf.getframerate()
        n_frames = wf.getnframes()

        # Basic sanity checks (optional but nice)
        if n_channels != 1:
            raise ValueError(f"Expected mono audio, got {n_channels} channels")
        if sampwidth != 2:
            raise ValueError(f"Expected 16-bit audio, got sampwidth={sampwidth}")
        if framerate != 16000:
            # You can either raise or just log/accept; for now we enforce 16k.
            raise ValueError(f"Expected 16000Hz, got {framerate}Hz")

        pcm = wf.readframes(n_frames)

    # Convert 16-bit PCM → float32 in [-1, 1]
    audio = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
    return audio


class AudioDrive(Node):
    def __init__(self):
        super().__init__("audio_drive")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
         # Buffer of incoming WAV chunks (bytes). Oldest first.
        self.buffer = deque()

        # Subscribe to audio from TB3
        self.sub = self.create_subscription(UInt8MultiArray, TOPIC, self.on_wav, qos)

        #self.timer = create_timer(INTERVAL, self.timer_callback)

        # Service to trigger recording on TB3
        #self.cli = self.create_client(Trigger, SERVICE)
        #while not self.cli.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info("Waiting for /record_wav service...")

        # PUBLISHER: match working script → TwistStamped on /cmd_vel
        self.cmd_vel_topic = "/cmd_vel"
        self.pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        # Trigger recording immediately
        #self.get_logger().info("Calling /record_wav...")
        #self.future = self.cli.call_async(Trigger.Request())
        #self.start = time.time()

        # Load faster-whisper model once
        startmodel = time.perf_counter()
        self.get_logger().info("Loading faster-whisper model (base/int8)…")
        self.model = WhisperModel("base", device="cpu", compute_type="int8")
        self.get_logger().info("Model loaded.")
        endmodel = time.perf_counter()
        self.get_logger().info(f"Model load time: {endmodel - startmodel:.5f} seconds")

        self.processing = False

        # Timer that periodically checks the buffer and processes the oldest chunk
        self.process_timer = self.create_timer(0.1, self.process_next_chunk)
        # 0.1 s is just a polling interval; ASR time will dominate anyway

    def on_wav(self, msg: UInt8MultiArray):
        """
        Callback for incoming audio chunks.
        We ONLY store them in a FIFO buffer here.
        """
        wav_bytes = bytes(msg.data)
        self.buffer.append(wav_bytes)
        self.get_logger().info(f"Buffered WAV chunk ({len(wav_bytes)} bytes). "
                            f"Buffer size: {len(self.buffer)}")

    def process_next_chunk(self):
        """
        Called regularly by a timer.
        If not already processing and buffer not empty:
        - take the oldest WAV from the buffer
        - run Whisper on it
        - send robot command if applicable
        """
        if self.processing:
            # still working on previous chunk
            return

        if not self.buffer:
            # nothing to do
            return

        # Now we are starting to process one chunk
        self.processing = True
        wav_bytes = self.buffer.popleft()
        start_total = time.perf_counter()

        try:
            self.get_logger().info(f"Processing buffered WAV ({len(wav_bytes)} bytes). "
                                   f"Remaining in buffer: {len(self.buffer)}")

            # Decode to mono float32
            audio = wav_bytes_to_float_mono(wav_bytes)

            # Run ASR
            start_asr = time.perf_counter()
            segments, info = self.model.transcribe(
                audio,
                language="en"
            )
            end_asr = time.perf_counter()

            text = " ".join(seg.text for seg in segments).strip().lower()
            self.get_logger().info(f"ASR result: '{text}'")
            self.get_logger().info(f"ASR time: {end_asr - start_asr:.5f} seconds")
            print(f"ASR: {text}")


            # -------- COMMAND DISPATCH --------
            if any(cmd in text for cmd in ("move forward", "forward", "go")):
                self.get_logger().info("Recognized command: MOVE FORWARD")
                self.move_forward()

            elif any(cmd in text for cmd in ("move backward", "go back", "backward")):
                self.get_logger().info("Recognized command: MOVE BACKWARD")
                self.move_backward()

            elif any(cmd in text for cmd in ("turn left", "left")):
                self.get_logger().info("Recognized command: TURN LEFT")
                self.turn_left()

            elif any(cmd in text for cmd in ("turn right", "right")):
                self.get_logger().info("Recognized command: TURN RIGHT")
                self.turn_right()
            
            elif any(cmd in text for cmd in ("penis", "Penis")):
                self.get_logger().info("Recognized command: TURN RIGHT")
                self.turn_right()

            elif "stop" in text:
                self.get_logger().info("Recognized command: STOP")
                self.stop_robot()
            else:
                self.get_logger().info("No known command detected in ASR text.")

        except Exception as e:
                    self.get_logger().error(f"Error while processing chunk: {e}")

        finally:
            end_total = time.perf_counter()
            self.get_logger().info(f"Total processing time for chunk: {end_total - start_total:.5f} seconds")
            # Let Python free audio & wav_bytes when out of scope
            self.processing = False


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

    def penis_move(self):
        self.get_logger().info("Command: Penis")
        duration = 0.5 / FAST_SPEED
        self.execute_twist(linear_x=FAST_SPEED, angular_z=0.0, duration=duration)


    def stop_robot(self):
        self.get_logger().info("Command: STOP")
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = "base_link"
        self.pub.publish(stop)

def main():
    rclpy.init()
    node = AudioDrive()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()