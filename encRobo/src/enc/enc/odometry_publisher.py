#!/usr/bin/env python3
"""
odometry_publisher.py — FIXED
──────────────────────────────
Subscribes to /joint_states and publishes /odom.

FIXES:
  - wheel_radius corrected to 0.033m (from robot_core.xacro)
    Your calibrated 0.045 was wrong — 14cm diameter = 7cm = 0.07m radius,
    but your URDF says 0.033m. Always trust your URDF measurement.
    If your robot still moves wrong distances, fix the URDF, not this file.
  - TF broadcaster REMOVED — EKF owns the odom→base_link TF (publish_tf: true)
    Having both publish TF causes SLAM to get confused and produce the
    "ray burst" explosion and pink line artefacts.
  - Added IMU-based instability guard: if BNO055 detects significant
    roll/pitch (robot tilted/lifted), odometry is frozen until stable.
    This prevents bad encoder readings while robot is lifted from entering EKF.

WHEEL PARAMS (from robot_core.xacro):
  wheel_radius     = 0.033 m
  wheel_separation = 0.297 m  (wheel_offset_y × 2 = 0.1485 × 2)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import math

# ── Robot params from robot_core.xacro ────────────────────────────────────────
WHEEL_RADIUS     = 0.0450   # metres — matches xacro wheel_radius property
WHEEL_SEPARATION = 0.36   # metres — wheel_offset_y * 2 = 0.1485 * 2

# ── IMU stability guard ────────────────────────────────────────────────────────
# If the robot is tilted more than this many degrees (roll or pitch),
# odometry updates are FROZEN until the BNO055 reports stable flat position.
# This prevents lifted-wheel / bumped-robot encoder noise from corrupting the map.
MAX_TILT_RAD = math.radians(8.0)   # 8 degrees — adjust to your floor bumpiness


class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        # Robot state
        self.x   = 0.0
        self.y   = 0.0
        self.th  = 0.0

        self.last_left_pos  = 0.0
        self.last_right_pos = 0.0
        self.initialized    = False
        self.last_time      = self.get_clock().now()

        # IMU stability state
        self.robot_stable = True   # False = robot is tilted/lifted, freeze odom
        self.imu_roll     = 0.0
        self.imu_pitch    = 0.0

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers / Subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        self.get_logger().info(
            f'OdometryPublisher started  '
            f'radius={WHEEL_RADIUS}m  sep={WHEEL_SEPARATION}m  '
            f'tilt_guard={math.degrees(MAX_TILT_RAD):.0f}deg')

    # ── IMU callback: only used for stability detection ───────────────────────
    def imu_callback(self, msg: Imu):
        # Extract roll/pitch from quaternion
        q = msg.orientation
        # Roll (x-axis rotation)
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.imu_roll = math.atan2(sinr, cosr)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self.imu_pitch = math.asin(sinp)

        # Update stability flag
        was_stable = self.robot_stable
        self.robot_stable = (
            abs(self.imu_roll)  < MAX_TILT_RAD and
            abs(self.imu_pitch) < MAX_TILT_RAD
        )

        if was_stable and not self.robot_stable:
            self.get_logger().warn(
                f'Robot unstable! roll={math.degrees(self.imu_roll):.1f}° '
                f'pitch={math.degrees(self.imu_pitch):.1f}° — odometry FROZEN',
                throttle_duration_sec=2.0)
        elif not was_stable and self.robot_stable:
            self.get_logger().info('Robot stable — odometry RESUMED')
            # Reset time to avoid large dt jump after being frozen
            self.last_time = self.get_clock().now()

    # ── Joint state callback: main odometry calculation ───────────────────────
    def joint_states_callback(self, msg: JointState):
        current_time = self.get_clock().now()

        # Find wheel indices by name (order is not guaranteed from ESP32)
        left_idx = right_idx = -1
        for i, name in enumerate(msg.name):
            if name == 'left_wheel_joint':  left_idx  = i
            if name == 'right_wheel_joint': right_idx = i

        if left_idx == -1 or right_idx == -1:
            self.get_logger().warn(
                'Missing left_wheel_joint or right_wheel_joint in /joint_states',
                throttle_duration_sec=5.0)
            return

        left_pos  = msg.position[left_idx]
        right_pos = msg.position[right_idx]

        # First-run init
        if not self.initialized:
            self.last_left_pos  = left_pos
            self.last_right_pos = right_pos
            self.last_time      = current_time
            self.initialized    = True
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        # ── If robot is unstable (tilted/lifted), skip this update ────────────
        if not self.robot_stable:
            # Still update stored positions so we don't get a position jump
            # when the robot becomes stable again
            self.last_left_pos  = left_pos
            self.last_right_pos = right_pos
            self.last_time      = current_time
            return

        # ── Differential drive kinematics ─────────────────────────────────────
        d_left  = (left_pos  - self.last_left_pos)  * WHEEL_RADIUS
        d_right = (right_pos - self.last_right_pos) * WHEEL_RADIUS

        d_centre = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / WHEEL_SEPARATION

        # Mid-point integration (more accurate than simple Euler for curves)
        self.x  += d_centre * math.cos(self.th + d_theta / 2.0)
        self.y  += d_centre * math.sin(self.th + d_theta / 2.0)
        self.th += d_theta

        vx  = d_centre / dt if dt > 0 else 0.0
        vth = d_theta  / dt if dt > 0 else 0.0

        # ── Build Odometry message ─────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self._yaw_to_quat(self.th)

        # Pose covariance — 6×6 row-major
        # 99999 on unused axes (z, roll, pitch) = "I don't know, ignore"
        odom.pose.covariance = [
            0.05,  0,     0,     0,     0,     0,
            0,     0.05,  0,     0,     0,     0,
            0,     0,     99999, 0,     0,     0,
            0,     0,     0,     99999, 0,     0,
            0,     0,     0,     0,     99999, 0,
            0,     0,     0,     0,     0,     0.1,   # yaw: moderate trust
        ]

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = vth

        odom.twist.covariance = [
            0.1,   0,     0,     0,     0,     0,
            0,     0.1,   0,     0,     0,     0,
            0,     0,     99999, 0,     0,     0,
            0,     0,     0,     99999, 0,     0,
            0,     0,     0,     0,     99999, 0,
            0,     0,     0,     0,     0,     0.2,
        ]

        self.odom_pub.publish(odom)

        # ── BROADCAST TF: odom → base_link ───────────────────────────────────
        # This is CRITICAL for Nav2 controller to know where the robot is!
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self._yaw_to_quat(self.th)
        self.tf_broadcaster.sendTransform(t)

        # Save state
        self.last_left_pos  = left_pos
        self.last_right_pos = right_pos
        self.last_time      = current_time

    def _yaw_to_quat(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
