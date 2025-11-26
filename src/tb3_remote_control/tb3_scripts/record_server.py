"""
To run this file on TB#:
python3 ~/ros_scripts/record_server.py
"""
"""
Nodes produce topics and services, but their names are independent.

🧠 ROS 2 Has Two Main Communication Types
✔ 1. Topics (publish–subscribe)
✔ 2. Services (request–response)

They solve different problems.

Let’s break them down in a clean, intuitive way.

📡 1. TOPICS — “Continuous data streams”

Topics are for ongoing, asynchronous, one-way communication.

Think of them like:
radio broadcast
live video stream
continuous sensor readings
control command stream

✔ A node publishes messages
✔ Other nodes subscribe to receive them
✔ No guarantee anyone is listening
✔ No reply is sent back
✔ Fast & continuous

⭐ Examples of topics
Topic Name	        Type	        Meaning
/cmd_vel	        Twist	        Robot velocity commands
/scan	            LaserScan	    LIDAR stream
/odom	            Odometry	    Robot motion estimates
/image_raw	        Image	        Camera images
/audio_wav	        UInt8MultiArray	Your published WAV file

🚪 2. SERVICES — “Ask and wait for a reply”

A Service is:

blocking
synchronous
two-way
one-time action

Think of a service as calling a function remotely:

“Hey, do X and tell me when you’re done!”

✔ Exactly one request → one response
✔ Client waits for server
✔ Used for commands, actions, queries
⭐ Examples of services
Service Name	    Type	            What it does
/record_wav	        Trigger	            Start recording now, reply when done
/reset_odometry	    Empty	            Reset odometry
/save_map	        SaveMap	            Write map to disk
/get_parameters	    Parameter service	Ask node for settings


"""
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


class RecordWavServer(Node): # The class inherits from Node
    def __init__(self):                                     # The constructor
        super().__init__("record_wav_server")               # Calls the parent class (Node) constructor, "record_wav_server" is the node name in ROS 2, this is where is registers with ROS graph
        # now we have created a ROS node
        qos = QoSProfile(
            depth=1,                                        # Only store the last WAV message
            reliability=ReliabilityPolicy.RELIABLE,         # Makes sure every /audio_wav message arrives
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    # keep last WAV for late subscribers
        )
        self.pub = self.create_publisher(UInt8MultiArray, TOPIC, qos)               # This returns a Publisher object used to self.pub.publish(msg), message type: an array of uint8, qos: so the publisher uses the QoS setup above   
        self.srv = self.create_service(Trigger, "/record_wav", self.on_trigger)     # This returns a Service object stored as self.srv, it is rarely used. Trigger is the service type, self.on_trigger: callback function that will run whenever /record_wav is called

    def on_trigger(self, req, res):                                                         # This method is called whenerver host does: ros2 service call /record_wav std_srvs/srv/Trigger "{}"
        try:
            self.get_logger().info(f"Recording {DUR}s @ {RATE}Hz from {DEVICE}...")         # self.get_logger(): creates a ROS logger, .info() logs an info message in terminal to see
            pcm = capture_raw()                                                             # Calls capture_raw() function
            wav_bytes = pcm_to_wav_bytes(pcm)                                               # Converts PCM bytes to WAV file as bytes and wraps in a WAV header
            msg = UInt8MultiArray()                                                         # Creates an empty message of type std_msgs/msg/UInt8MultiArray
            msg.data = list(wav_bytes)   # ✅ list of uint8                                 # wav_bytes is a bytes object, like b'\x52\x49\x46\x46...', list(wav_bytes) converts this into a list of integers: [82, 73, 70, 70, ...], UInt8MultiArray.data expects a sequence of uint8 numbers (0–255), not a raw Python bytes object, so this conversion is needed
            self.pub.publish(msg)                                                           # Send the WAV file over ROS
            self.get_logger().info(f"Published WAV ({len(wav_bytes)} bytes) on {TOPIC}.")   # Logs the size and topic name
            res.success, res.message = True, f"ok ({len(wav_bytes)} bytes)"                 # Sets the service response
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"arecord failed: {e}")
            res.success, res.message = False, f"arecord failed: {e}"
        except Exception as e:
            self.get_logger().error(f"error: {e}")
            res.success, res.message = False, f"error: {e}"
        return res

def main():
    rclpy.init()                    # initializes the ROS 2 client library
    rclpy.spin(RecordWavServer())   # RecordWavServer() creates an instance of the node class, rclpy.spin(...) enters a loop and: waits for service calls, handles callbacks (on_trigger), keeps the node alive


if __name__ == "__main__":
    main()                          # Runs the node