#!/usr/bin/env python3
"""
slip_detector.py — FULLY FIXED AND CLEANED
───────────────────────────────────────────
CRASH FIX:
  Previous code called _cancel_goal_async(None) which crashes because
  it needs a ClientGoalHandle object, not None.

  Correct approach for an external observer that wants to cancel ALL
  active Nav2 goals: call the action server's cancel service directly.
  Every ROS2 action server exposes:
    /navigate_to_pose/_action/cancel_goal  (action_msgs/srv/CancelGoal)
  Sending a request with empty GoalInfo = "cancel all active goals".
  This is the official ROS2 pattern for external goal cancellation.

SLIP LOGIC FIX:
  Only flag asymmetric wheel motion with no yaw change as slip.
  Symmetric motion + no yaw change = straight line driving = NOT slip.
  
SENSITIVITY TUNING:
  - Increased thresholds to prevent false positives during normal turns
  - Added confirmation counter requiring multiple consecutive detections
  - Properly tuned for differential drive robots
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
import math


class SlipDetector(Node):
    def __init__(self):
        super().__init__('slip_detector')

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        # Direct service client to cancel all Nav2 goals
        self._cancel_client = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal'
        )

        # ─────────────────────────────────────────────────────────────────
        # TUNABLE PARAMETERS - Adjust these based on your robot's behavior
        # ─────────────────────────────────────────────────────────────────
        self.MIN_WHEEL_MOVEMENT = 0.10      # Minimum position change (radians)
        self.ASYMMETRY_THRESHOLD = 0.15     # Difference between wheels (radians)
        self.YAW_THRESHOLD = 0.01           # Minimum yaw change (radians)
        self.SLIP_COOLDOWN_SEC = 3.0        # Seconds between slip detections
        self.SLIP_CONFIRMATION_COUNT = 3    # Consecutive detections required

        # IMU state
        self.prev_yaw = 0.0
        self.curr_yaw = 0.0

        # Encoder state
        self.last_left_pos = 0.0
        self.last_right_pos = 0.0
        self.initialized = False

        # Cooldown state
        self.last_slip_time = self.get_clock().now()
        self.slip_counter = 0

        self.get_logger().info(
            f'SlipDetector ready with thresholds:\n'
            f'  MIN_WHEEL_MOVEMENT: {self.MIN_WHEEL_MOVEMENT:.3f} rad\n'
            f'  ASYMMETRY_THRESHOLD: {self.ASYMMETRY_THRESHOLD:.3f} rad\n'
            f'  YAW_THRESHOLD: {self.YAW_THRESHOLD:.4f} rad\n'
            f'  CONFIRMATION_COUNT: {self.SLIP_CONFIRMATION_COUNT}\n'
            f'  COOLDOWN: {self.SLIP_COOLDOWN_SEC:.1f} sec'
        )

    def imu_callback(self, msg: Imu):
        """Store previous and current yaw for delta calculation."""
        self.prev_yaw = self.curr_yaw
        self.curr_yaw = self._yaw_from_quat(msg.orientation)

    def joint_callback(self, msg: JointState):
        """Main slip detection logic using wheel encoders and IMU yaw."""
        # Find wheel indices by name
        try:
            li = msg.name.index('left_wheel_joint')
            ri = msg.name.index('right_wheel_joint')
        except ValueError:
            self.get_logger().warn(
                'Missing left_wheel_joint or right_wheel_joint in /joint_states',
                throttle_duration_sec=5.0
            )
            return

        left_pos = msg.position[li]
        right_pos = msg.position[ri]

        # Initialize on first message
        if not self.initialized:
            self.last_left_pos = left_pos
            self.last_right_pos = right_pos
            self.initialized = True
            return

        # Calculate deltas
        delta_left = abs(left_pos - self.last_left_pos)
        delta_right = abs(right_pos - self.last_right_pos)
        delta_yaw = abs(self.curr_yaw - self.prev_yaw)

        # Update stored positions for next callback
        self.last_left_pos = left_pos
        self.last_right_pos = right_pos

        # ─────────────────────────────────────────────────────────────────
        # SLIP DETECTION CONDITIONS
        # ─────────────────────────────────────────────────────────────────
        # Slip = wheels are moving asymmetrically BUT robot is NOT turning
        # Normal turn = wheels moving asymmetrically AND robot IS turning
        # Straight driving = wheels moving symmetrically AND robot NOT turning

        wheels_moving = (delta_left > self.MIN_WHEEL_MOVEMENT or
                        delta_right > self.MIN_WHEEL_MOVEMENT)
        asymmetric = abs(delta_left - delta_right) > self.ASYMMETRY_THRESHOLD
        yaw_frozen = (delta_yaw < self.YAW_THRESHOLD)

        if wheels_moving and asymmetric and yaw_frozen:
            self.slip_counter += 1
            self.get_logger().debug(
                f'Potential slip #{self.slip_counter}: '
                f'dL={delta_left:.3f} dR={delta_right:.3f} dyaw={delta_yaw:.4f}'
            )

            if self.slip_counter >= self.SLIP_CONFIRMATION_COUNT:
                now = self.get_clock().now()
                elapsed = (now - self.last_slip_time).nanoseconds / 1e9

                if elapsed > self.SLIP_COOLDOWN_SEC:
                    self.last_slip_time = now
                    self.get_logger().warn(
                        f'⚠️  SLIP CONFIRMED! ⚠️\n'
                        f'   dL={delta_left:.3f} rad, dR={delta_right:.3f} rad\n'
                        f'   Asymmetry: {abs(delta_left - delta_right):.3f} rad\n'
                        f'   dyaw: {delta_yaw:.4f} rad (frozen)\n'
                        f'   Canceling all navigation goals...'
                    )
                    self._cancel_all_goals()

                # Reset counter after triggering
                self.slip_counter = 0
        else:
            # Reset counter if conditions are not met
            if self.slip_counter > 0:
                self.get_logger().debug(
                    f'Slip conditions cleared (counter was {self.slip_counter})'
                )
            self.slip_counter = 0

    def _cancel_all_goals(self):
        """Cancel all active Nav2 goals via the action server's cancel service."""
        if not self._cancel_client.service_is_ready():
            self.get_logger().warn(
                'Nav2 cancel service not ready. Cannot cancel goals.',
                throttle_duration_sec=2.0
            )
            return

        # Empty GoalInfo = cancel ALL active goals on this action server
        request = CancelGoal.Request()
        request.goal_info = GoalInfo()

        future = self._cancel_client.call_async(request)
        future.add_done_callback(self._cancel_response_callback)

    def _cancel_response_callback(self, future):
        """Handle the response from the cancel service."""
        try:
            response = future.result()
            self.get_logger().info(
                f'Cancel request completed. '
                f'Return code: {response.return_code}'
            )
            # return_code meanings:
            # 0 = CANCEL_NONE (no goals canceled)
            # 1 = CANCEL_REQUESTED (request accepted)
            # 2 = CANCEL_ALL (all goals canceled)
        except Exception as e:
            self.get_logger().error(f'Cancel request failed: {e}')

    def _yaw_from_quat(self, q) -> float:
        """Extract yaw angle from quaternion."""
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)


def main():
    rclpy.init()
    node = SlipDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('SlipDetector shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
