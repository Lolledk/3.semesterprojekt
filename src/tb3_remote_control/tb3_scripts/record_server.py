"""
To run this file on TB#:
python3 ~/ros_scripts/record_server.py
"""

#!/usr/bin/env python3
import rclpy, subprocess, io, wave
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import Trigger

# fixed settings; change here if needed
DEVICE = "plughw:1,0"   # use the one that worked with arecord -l / your test
RATE   = 16000
CH     = 1
DUR    = 5              # seconds
TOPIC  = "/audio_wav"

def capture_raw():
    # returns raw PCM bytes (S16_LE)
    cmd = ["arecord", "-D", DEVICE, "-f", "S16_LE", "-c", str(CH),
           "-r", str(RATE), "-d", str(DUR), "-t", "raw"]
    return subprocess.check_output(cmd)

def pcm_to_wav_bytes(pcm: bytes) -> bytes:
    b = io.BytesIO()
    with wave.open(b, "wb") as w:
        w.setnchannels(CH)
        w.setsampwidth(2)     # 16-bit
        w.setframerate(RATE)
        w.writeframes(pcm)
    return b.getvalue()

class RecordWavServer(Node):
    def __init__(self):
        super().__init__("record_wav_server")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # keep last WAV for late subscribers
        )
        self.pub = self.create_publisher(UInt8MultiArray, TOPIC, qos)
        self.srv = self.create_service(Trigger, "/record_wav", self.on_trigger)

    def on_trigger(self, req, res):
        try:
            self.get_logger().info(f"Recording {DUR}s @ {RATE}Hz from {DEVICE}...")
            pcm = capture_raw()
            wav_bytes = pcm_to_wav_bytes(pcm)
            msg = UInt8MultiArray()
            msg.data = list(wav_bytes)   # ✅ list of uint8
            self.pub.publish(msg)
            self.get_logger().info(f"Published WAV ({len(wav_bytes)} bytes) on {TOPIC}.")
            res.success, res.message = True, f"ok ({len(wav_bytes)} bytes)"
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"arecord failed: {e}")
            res.success, res.message = False, f"arecord failed: {e}"
        except Exception as e:
            self.get_logger().error(f"error: {e}")
            res.success, res.message = False, f"error: {e}"
        return res

def main():
    rclpy.init()
    rclpy.spin(RecordWavServer())


if __name__ == "__main__":
    main()