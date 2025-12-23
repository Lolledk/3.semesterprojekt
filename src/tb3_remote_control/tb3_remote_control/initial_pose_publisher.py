#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped


def yaw_from_quat_z_w(z: float, w: float) -> float:
    """Planar yaw from quaternion (x=y=0 assumed): yaw = 2*atan2(z,w)."""
    return 2.0 * math.atan2(z, w)


def angle_wrap(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class InitialPosePublisher(Node):
    """
    Publishes an initial pose, verifies AMCL convergence, and if unsuccessful
    runs a calibration motion (turn 45°, drive forward 0.1m, turn 360°),
    then publishes the initial pose again.
    """

    def __init__(self):
        super().__init__("initial_pose_publisher")

        # ---- Parameters you may want to tune ----
        self.declare_parameter("initial_x", -0.038)
        self.declare_parameter("initial_y", -0.124)
        self.declare_parameter("initial_yaw_deg", 2.829)

        self.declare_parameter("cov_xy", 0.1)
        self.declare_parameter("cov_yaw", 0.1)

        self.declare_parameter("amcl_timeout_s", 8.0)       # wait time for AMCL to match pose
        self.declare_parameter("pos_tolerance_m", 0.30)     # how close AMCL must be in x/y
        self.declare_parameter("yaw_tolerance_deg", 25.0)   # how close AMCL must be in yaw

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("calib_lin_speed", 0.05)     # m/s
        self.declare_parameter("calib_ang_speed", 0.5)      # rad/s
        self.declare_parameter("calib_distance_m", 0.10)    # 0.1 m forward

        # New: calibration turn angle
        self.declare_parameter("calib_turn_deg", 45.0)      # degrees

        # ---- IO ----
        self.pub_initpose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl_pose, 10
        )
        self.pub_cmd = self.create_publisher(
            TwistStamped, str(self.get_parameter("cmd_vel_topic").value), 10
        )

        # ---- State ----
        self.current_pose = None  # (x, y, yaw)
        self._attempt = 0

        self._state = "BOOT_WAIT"
        self._state_started = self.get_clock().now()
        self._rotated = 0.0
        self._last_time = self.get_clock().now()

        # Start: give Nav2/AMCL some time
        self._boot_timer = self.create_timer(5.0, self._start_initialization)

        # Control loop for calibration motions (20 Hz)
        self._loop_timer = self.create_timer(0.05, self._loop)

    # ---------------- Initial pose publishing ----------------

    def _start_initialization(self):
        self._boot_timer.cancel()
        self._attempt = 1
        self._publish_initial_pose()
        self._set_state("WAIT_AMCL")

    def _publish_initial_pose(self):
        x0 = float(self.get_parameter("initial_x").value)
        y0 = float(self.get_parameter("initial_y").value)
        yaw_deg = float(self.get_parameter("initial_yaw_deg").value)
        yaw = math.radians(yaw_deg)

        cov_xy = float(self.get_parameter("cov_xy").value)
        cov_yaw = float(self.get_parameter("cov_yaw").value)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x0
        msg.pose.pose.position.y = y0
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        msg.pose.covariance[0] = cov_xy       # x
        msg.pose.covariance[7] = cov_xy       # y
        msg.pose.covariance[35] = cov_yaw     # yaw

        self.get_logger().info(f"[Attempt {self._attempt}] Publishing initial pose to /initialpose")
        self.pub_initpose.publish(msg)

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        yaw = yaw_from_quat_z_w(z, w)
        self.current_pose = (x, y, yaw)

    def _amcl_is_close_enough(self) -> bool:
        if self.current_pose is None:
            return False

        x, y, yaw = self.current_pose

        x0 = float(self.get_parameter("initial_x").value)
        y0 = float(self.get_parameter("initial_y").value)
        yaw0 = math.radians(float(self.get_parameter("initial_yaw_deg").value))

        pos_tol = float(self.get_parameter("pos_tolerance_m").value)
        yaw_tol = math.radians(float(self.get_parameter("yaw_tolerance_deg").value))

        dx = x - x0
        dy = y - y0
        dist = math.hypot(dx, dy)
        dyaw = abs(angle_wrap(yaw - yaw0))

        return (dist <= pos_tol) and (dyaw <= yaw_tol)

    # ---------------- Calibration routine (state machine) ----------------

    def _set_state(self, new_state: str):
        self._state = new_state
        self._state_started = self.get_clock().now()
        self._last_time = self.get_clock().now()

        # reset rotation accumulator when entering any rotation state
        if new_state in ("CALIB_TURN_45", "CALIB_TURN_360"):
            self._rotated = 0.0

        self.get_logger().info(f"STATE -> {new_state}")

    def _elapsed_s(self) -> float:
        return (self.get_clock().now() - self._state_started).nanoseconds / 1e9

    def _publish_twist(self, lin_x: float = 0.0, ang_z: float = 0.0):
        msg = TwistStamped()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(lin_x)
        msg.twist.angular.z = float(ang_z)
        self.pub_cmd.publish(msg)

    def _stop(self):
        self._publish_twist(0.0, 0.0)

    def _loop(self):
        # Main state machine tick (runs at ~20 Hz)

        # 1) Wait for AMCL to accept/align
        if self._state == "WAIT_AMCL":
            timeout_s = float(self.get_parameter("amcl_timeout_s").value)

            if self._amcl_is_close_enough():
                self.get_logger().info("AMCL pose accepted / converged sufficiently.")
                self._stop()
                self._set_state("DONE")
                return

            if self._elapsed_s() >= timeout_s:
                self.get_logger().warn(
                    "Initial pose did not converge within timeout -> starting calibration routine."
                )
                self._set_state("CALIB_TURN_45")
                return

        # 2) Calibration: turn 45°
        if self._state == "CALIB_TURN_45":
            w = float(self.get_parameter("calib_ang_speed").value)
            target_deg = float(self.get_parameter("calib_turn_deg").value)
            target_rad = math.radians(target_deg)

            now = self.get_clock().now()
            dt = (now - self._last_time).nanoseconds / 1e9
            self._last_time = now
            self._rotated += abs(w) * dt

            if self._rotated < target_rad:
                self._publish_twist(lin_x=0.0, ang_z=w)
            else:
                self._stop()
                self._set_state("CALIB_FORWARD")
            return

        # 3) Calibration: forward 0.1 m
        if self._state == "CALIB_FORWARD":
            v = float(self.get_parameter("calib_lin_speed").value)
            d = float(self.get_parameter("calib_distance_m").value)
            t_needed = d / max(v, 1e-6)

            if self._elapsed_s() < t_needed:
                self._publish_twist(lin_x=v, ang_z=0.0)
            else:
                self._stop()
                self._set_state("CALIB_TURN_360")
            return

        # 4) Calibration: rotate 360°
        if self._state == "CALIB_TURN_360":
            w = float(self.get_parameter("calib_ang_speed").value)

            now = self.get_clock().now()
            dt = (now - self._last_time).nanoseconds / 1e9
            self._last_time = now
            self._rotated += abs(w) * dt

            if self._rotated < 2.0 * math.pi:
                self._publish_twist(lin_x=0.0, ang_z=w)
            else:
                self._stop()
                self.get_logger().info("Calibration rotation complete (360°).")

                # Re-publish initial pose after calibration
                self._attempt += 1
                self._publish_initial_pose()
                self._set_state("WAIT_AMCL_AGAIN")
            return

        # 5) Re-check AMCL after calibration
        if self._state == "WAIT_AMCL_AGAIN":
            timeout_s = float(self.get_parameter("amcl_timeout_s").value)

            if self._amcl_is_close_enough():
                self.get_logger().info("AMCL pose accepted after calibration routine.")
                self._stop()
                self._set_state("DONE")
                return

            if self._elapsed_s() >= timeout_s:
                self.get_logger().error(
                    "AMCL still did not converge after calibration. "
                    "Consider increasing tolerances or checking map/TF."
                )
                self._stop()
                self._set_state("DONE")
            return

        # Terminal state
        if self._state == "DONE":
            return


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()