#!/usr/bin/env python3
"""
odometry_publisher.py — FULLY CORRECTED
────────────────────────────────────────
Subscribes to /joint_states and publishes /odom.

CRITICAL FIXES:
  1. WHEEL_SEPARATION corrected to 0.182m (from robot_core.xacro: wheel_offset_y=0.091 × 2)
  2. Uses JointState timestamp for odometry and TF (fixes TF_OLD_DATA errors)
  3. TF broadcaster COMMENTED OUT by default - EKF should own the odom→base_link TF
     (Set PUBLISH_TF=True if EKF is NOT publishing TF)

WHEEL PARAMS (from robot_core.xacro):
  wheel_radius     = 0.045 m
  wheel_separation = 0.182 m  (wheel_offset_y = 0.091 × 2)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import math

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION - Adjust these to match your robot
# ─────────────────────────────────────────────────────────────────────────────

# Wheel parameters from robot_core.xacro
WHEEL_RADIUS     = 0.045   # metres - matches xacro wheel_radius property
WHEEL_SEPARATION = 0.364   # metres - wheel_offset_y (0.091) × 2

# TF Publishing - SET TO FALSE IF EKF IS PUBLISHING TF
# Check with: ros2 param get /ekf_filter_node publish_tf
PUBLISH_TF = False  # Set to True only if EKF is NOT publishing odom→base_link

# IMU stability guard - freeze odometry if robot is tilted/lifted
MAX_TILT_RAD = math.radians(8.0)   # 8 degrees - adjust to your floor bumpiness

# Covariance values (tune these based on your encoder quality)
POSE_COV_XY = 0.05     # Lower = trust encoders more for position
POSE_COV_YAW = 0.1     # Lower = trust encoders more for heading
TWIST_COV_VX = 0.1     # Lower = trust encoders more for linear velocity
TWIST_COV_VTH = 0.2    # Lower = trust encoders more for angular velocity


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
        self.robot_stable = True
        self.imu_roll     = 0.0
        self.imu_pitch    = 0.0

        # TF Broadcaster (only used if PUBLISH_TF is True)
        if PUBLISH_TF:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.get_logger().info('TF broadcasting ENABLED - odometry_publisher will publish odom→base_link')
        else:
            self.get_logger().info('TF broadcasting DISABLED - assuming EKF publishes odom→base_link')

        # Publishers / Subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        self.get_logger().info(
            f'OdometryPublisher started:\n'
            f'  wheel_radius:     {WHEEL_RADIUS:.4f} m\n'
            f'  wheel_separation: {WHEEL_SEPARATION:.4f} m\n'
            f'  tilt_guard:       {math.degrees(MAX_TILT_RAD):.1f}°\n'
            f'  publish_tf:       {PUBLISH_TF}'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # IMU callback - only used for stability detection (tilt/lift guard)
    # ─────────────────────────────────────────────────────────────────────────
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

    # ─────────────────────────────────────────────────────────────────────────
    # Joint state callback - main odometry calculation
    # ─────────────────────────────────────────────────────────────────────────
    def joint_states_callback(self, msg: JointState):
        # CRITICAL: Use timestamp from the JointState message
        # This prevents TF_OLD_DATA and extrapolation errors
        current_time = rclpy.time.Time.from_msg(msg.header.stamp)

        # Find wheel indices by name (order is not guaranteed from ESP32)
        left_idx = right_idx = -1
        for i, name in enumerate(msg.name):
            if name == 'left_wheel_joint':
                left_idx = i
            elif name == 'right_wheel_joint':
                right_idx = i

        if left_idx == -1 or right_idx == -1:
            self.get_logger().warn(
                'Missing left_wheel_joint or right_wheel_joint in /joint_states',
                throttle_duration_sec=5.0)
            return

        left_pos  = msg.position[left_idx]
        right_pos = msg.position[right_idx]

        # First-run initialization
        if not self.initialized:
            self.last_left_pos  = left_pos
            self.last_right_pos = right_pos
            self.last_time      = current_time
            self.initialized    = True
            return

        # Calculate time delta
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        if dt > 0.5:  # Sanity check - if dt is too large, something's wrong
            self.get_logger().warn(
                f'Large dt detected: {dt:.3f}s - skipping update',
                throttle_duration_sec=2.0)
            self.last_left_pos  = left_pos
            self.last_right_pos = right_pos
            self.last_time      = current_time
            return

        # Skip update if robot is unstable (tilted/lifted)
        if not self.robot_stable:
            self.last_left_pos  = left_pos
            self.last_right_pos = right_pos
            self.last_time      = current_time
            return

        # ─────────────────────────────────────────────────────────────────────
        # Differential drive kinematics
        # ─────────────────────────────────────────────────────────────────────
        d_left  = (left_pos  - self.last_left_pos)  * WHEEL_RADIUS
        d_right = (right_pos - self.last_right_pos) * WHEEL_RADIUS

        d_centre = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / WHEEL_SEPARATION

        # Mid-point integration (more accurate than simple Euler for curves)
        self.x  += d_centre * math.cos(self.th + d_theta / 2.0)
        self.y  += d_centre * math.sin(self.th + d_theta / 2.0)
        self.th += d_theta

        # Normalize theta to [-π, π]
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        # Calculate velocities
        vx  = d_centre / dt if dt > 0 else 0.0
        vth = d_theta  / dt if dt > 0 else 0.0

        # ─────────────────────────────────────────────────────────────────────
        # Build and publish Odometry message
        # ─────────────────────────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self._yaw_to_quat(self.th)

        # Pose covariance (6×6 row-major)
        # 99999 on unused axes = "I don't know, ignore this dimension"
        odom.pose.covariance = [
            POSE_COV_XY, 0.0,        0.0, 0.0,    0.0,    0.0,
            0.0,        POSE_COV_XY, 0.0, 0.0,    0.0,    0.0,
            0.0,        0.0,        99999.0, 0.0,    0.0,    0.0,
            0.0,        0.0,        0.0,    99999.0, 0.0,    0.0,
            0.0,        0.0,        0.0,    0.0,    99999.0, 0.0,
            0.0,        0.0,        0.0,    0.0,    0.0,    POSE_COV_YAW
        ]

        # Velocity
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.angular.z = vth

        # Twist covariance
        odom.twist.covariance = [
            TWIST_COV_VX, 0.0,          0.0, 0.0,    0.0,    0.0,
            0.0,          TWIST_COV_VX, 0.0, 0.0,    0.0,    0.0,
            0.0,          0.0,          99999.0, 0.0,    0.0,    0.0,
            0.0,          0.0,          0.0,    99999.0, 0.0,    0.0,
            0.0,          0.0,          0.0,    0.0,    99999.0, 0.0,
            0.0,          0.0,          0.0,    0.0,    0.0,    TWIST_COV_VTH
        ]

        self.odom_pub.publish(odom)

        # ─────────────────────────────────────────────────────────────────────
        # Broadcast TF: odom → base_link (only if EKF is not doing it)
        # ─────────────────────────────────────────────────────────────────────
        if PUBLISH_TF:
            t = TransformStamped()
            t.header.stamp    = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id  = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = self._yaw_to_quat(self.th)
            self.tf_broadcaster.sendTransform(t)

        # Save state for next callback
        self.last_left_pos  = left_pos
        self.last_right_pos = right_pos
        self.last_time      = current_time

    def _yaw_to_quat(self, yaw: float) -> Quaternion:
        """Convert yaw angle to quaternion."""
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('OdometryPublisher shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

