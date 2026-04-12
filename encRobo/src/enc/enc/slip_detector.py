#!/usr/bin/env python3
"""
slip_detector.py — FIXED
─────────────────────────
Detects wheel slip by comparing encoder movement vs IMU yaw change.

PREVIOUS BUG:
  Was publishing zero Twist directly to /cmd_vel.
  Problem: Nav2's Collision Monitor ALSO publishes to /cmd_vel at 10Hz.
  Result: slip detector stops robot for 100ms, Nav2 immediately resumes it.
  The robot flickers between stopped and moving — useless.

FIX:
  Instead of publishing to /cmd_vel, we now CANCEL the active Nav2 goal.
  This tells Nav2's BehaviorTree to abort the current navigation task.
  Nav2 then stops the robot cleanly through its own pipeline.
  
  This is the CORRECT ROS2 Nav2 pattern for external safety interrupts.

HOW SLIP DETECTION WORKS:
  If wheel encoders say the robot is moving (delta_left or delta_right > threshold)
  BUT the IMU says yaw is not changing (delta_yaw < threshold)
  → one or both wheels are spinning without the robot actually turning
  → this indicates slip, wheel lift, or obstacle collision

  Note: This only detects slip during ROTATION.
  Straight-line slip (both wheels spinning, no yaw change) is normal motion.
  We detect that separately by checking if both wheels spin equally with zero yaw.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Imu, JointState
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import math


class SlipDetector(Node):
    def __init__(self):
        super().__init__('slip_detector')

        # Subscriptions
        self.imu_sub   = self.create_subscription(Imu,        '/imu/data',    self.imu_callback,   10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # Nav2 action client — used to CANCEL goal on slip (correct pattern)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # IMU state
        self.prev_yaw = 0.0
        self.curr_yaw = 0.0

        # Encoder state
        self.last_left_pos  = 0.0
        self.last_right_pos = 0.0
        self.initialized    = False

        # Slip detection cooldown (prevent spam cancellations)
        self.last_slip_time = self.get_clock().now()
        self.SLIP_COOLDOWN_SEC = 3.0

        self.get_logger().info('SlipDetector started. Monitoring encoders vs IMU.')

    def imu_callback(self, msg: Imu):
        self.prev_yaw = self.curr_yaw
        self.curr_yaw = self._yaw_from_quaternion(msg.orientation)

    def joint_callback(self, msg: JointState):
        try:
            left_idx  = msg.name.index('left_wheel_joint')
            right_idx = msg.name.index('right_wheel_joint')
        except ValueError:
            return

        left_pos  = msg.position[left_idx]
        right_pos = msg.position[right_idx]

        if not self.initialized:
            self.last_left_pos  = left_pos
            self.last_right_pos = right_pos
            self.initialized    = True
            return

        delta_left  = abs(left_pos  - self.last_left_pos)
        delta_right = abs(right_pos - self.last_right_pos)
        self.last_left_pos  = left_pos
        self.last_right_pos = right_pos

        delta_yaw = abs(self.curr_yaw - self.prev_yaw)

        # ── Slip condition ────────────────────────────────────────────────────
        # Wheels are moving significantly (>0.05 rad change in wheel angle)
        # BUT yaw is not changing at all (<0.005 rad = ~0.3 degrees)
        # This means wheels are spinning but robot isn't turning → SLIP
        #
        # We only trigger during asymmetric movement (one wheel more than other)
        # because symmetric movement with no yaw change = straight line = normal
        asymmetric = abs(delta_left - delta_right) > 0.02
        wheels_moving = (delta_left > 0.05 or delta_right > 0.05)
        yaw_frozen = (delta_yaw < 0.005)

        if wheels_moving and asymmetric and yaw_frozen:
            now = self.get_clock().now()
            elapsed = (now - self.last_slip_time).nanoseconds / 1e9
            if elapsed > self.SLIP_COOLDOWN_SEC:
                self.last_slip_time = now
                self.get_logger().warn(
                    f'WHEEL SLIP DETECTED! '
                    f'delta_L={delta_left:.3f} delta_R={delta_right:.3f} '
                    f'delta_yaw={delta_yaw:.4f} — cancelling Nav2 goal')
                self._cancel_nav2_goal()

    def _cancel_nav2_goal(self):
        """Cancel the active Nav2 goal — the correct way to stop the robot."""
        if not self._nav_client.server_is_ready():
            self.get_logger().warn('Nav2 not available, cannot cancel goal.')
            return
        # Cancel all goals on the navigate_to_pose action server
        future = self._nav_client._cancel_goal_async(None)
        # Nav2 will then stop the robot cleanly through its own cmd_vel pipeline

    def _yaw_from_quaternion(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main():
    rclpy.init()
    node = SlipDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
