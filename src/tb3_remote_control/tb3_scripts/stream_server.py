#!/usr/bin/env python3
import rclpy, subprocess, io, wave 
#rclpy - ROS2 Python library, subprocess - run external Linux commands(arecord), io - used for in-memory byte buffer, .wav not saved to storage, wave - python module to create WAV files
from rclpy.node import Node
# server node inherits from this.
# A Node in ROS 2 is an independent program that: can publish/subscribe to topics, offer/call services, have parameters and run timers like a camera driver node, a movement controller node, a microphone recorder node
# A node can be thought of as a base class and our class RecordWavServer can be thought of as a custom version of a Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# QoS = Quality of Service, ROS 2 communicates using DDS (Data Distribution Service)
# DDS is a networking system that controls: Message transfer reliability, message history storage, size of storage in terms of messages
# Controls how DDS handles published audio messages??
from std_msgs.msg import UInt8MultiArray
#The WAV file is sent as a list of bytes
from std_srvs.srv import Trigger
# used for the service: /record_wav, Trigger = empty request, boolean response
from gpio_led import LEDController

# fixed settings; change here if needed
DEVICE = "plughw:1,0"   # is found by using arecord -l
RATE   = 16000          # sample rate
CH     = 1              # mono
DUR    = 5              # seconds
TOPIC  = "/audio_wav"   # where the WAV is published, the ROS topic name to publish on
INTERVAL = 0.1

led_pin = 12
led = LEDController(led_pin)

def capture_raw():
    # Use arecord to grab PCM audio from usb microphone
    #led.blink(on_time=0.1, off_time=0.1, n=3)
    # returns raw PCM bytes (S16_LE)
    cmd = ["arecord", "-D", DEVICE, "-f", "S16_LE", "-c", str(CH),
           "-r", str(RATE), "-d", str(DUR), "-t", "raw"]
    # arecord       : arecord is a Linux command-line tool used to record audio from microphones
    # -D DEVICE     : audio device
    # -f S16_LE     : 16-bit little endian format
    # -c 1          : mono
    # -r 1600       : 16k sample rate
    # -d str(DUR)   : record 5 seconds
    # -t raw        : output raw PCM(no WAV header)
    return subprocess.check_output(cmd)
    # Runs arecord and returns the raw PCM audio bytes

def pcm_to_wav_bytes(pcm: bytes) -> bytes:
    # Adds a WAV header so the host can use the file
    b = io.BytesIO() ##creates a memory buffer to store the WAV file
    with wave.open(b, "wb") as w:  # opens a WAV writer
        w.setnchannels(CH)
        w.setsampwidth(2)           # 16-bit
        w.setframerate(RATE)
        w.writeframes(pcm)          # write all PCM bytes inside the WAV file
    return b.getvalue()             # returns the finished WAV file bytes, nothing is written to disk, everything is in memory


class ContinuousRecorder(Node):
    def __init__(self):
        super().__init__("continuous_audio_stream")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pub = self.create_publisher(UInt8MultiArray, TOPIC, qos)

        # Call `self.timer_callback` every INTERVAL seconds
        self.timer = self.create_timer(INTERVAL, self.timer_callback)

        self.get_logger().info(
            f"Starting continuous 5s recording every {INTERVAL}s..."
        )

    def timer_callback(self):
        """Runs every INTERVAL seconds."""
        try:
            self.get_logger().info("Recording 5s chunk...")
            pcm = capture_raw()

            wav_bytes = pcm_to_wav_bytes(pcm)
            msg = UInt8MultiArray()
            msg.data = list(wav_bytes)  # convert bytes → list[uint8]

            self.pub.publish(msg)

            self.get_logger().info(f"Published WAV ({len(wav_bytes)} bytes).")

        except Exception as e:
            self.get_logger().error(f"Error during recording: {e}")

def main():
    rclpy.init()
    node = ContinuousRecorder()
    rclpy.spin(node)

if __name__ == "__main__":
    main()