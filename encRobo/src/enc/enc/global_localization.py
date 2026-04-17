#!/usr/bin/env python3
"""
global_localization.py
───────────────────────
Handles initial robot localization when placed anywhere on the field.
Uses a combination of particle dispersion and controlled rotation.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import Spin
from nav2_msgs.srv import SetInitialPose
from geometry_msgs.msg import Twist
import math
import time


class GlobalLocalizer(Node):
    """
    TUNING GUIDE:
    - ROTATION_SPEED: How fast the robot spins during localization (rad/s)
    - ROTATION_ANGLE: How much to rotate (radians) - full 360° = 6.28
    - CONVERGENCE_TIME: Seconds to wait after rotation for particles to converge
    - PARTICLE_COUNT: Higher = more accurate but slower
    """
    
    # ─── TUNABLE PARAMETERS ──────────────────────────────────────────────────
    ROTATION_SPEED = 0.5          # rad/s (slow for safety)
    ROTATION_ANGLE = 2 * math.pi  # full 360° rotation
    CONVERGENCE_TIME = 2.0        # seconds to wait after rotation
    # ──────────────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('global_localizer')
        
        # Service client for AMCL global localization
        self.global_loc_client = self.create_client(
            SetInitialPose, 
            '/amcl/global_localization'
        )
        
        # Publisher for manual rotation (safer than Spin action with no obstacles)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('GlobalLocalizer ready. Waiting for AMCL...')
        
        # Wait for services to be ready
        self.global_loc_client.wait_for_service(timeout_sec=10.0)
        
    def localize(self):
        """
        Main localization routine:
        1. Disperse particles across the entire map
        2. Rotate in place (with lidar active)
        3. Wait for particle convergence
        """
        self.get_logger().info('Starting global localization...')
        
        # Step 1: Trigger global localization (disperse particles)
        self._disperse_particles()
        
        # Step 2: Rotate to gather lidar data for convergence
        self._rotate_for_localization()
        
        # Step 3: Wait for convergence
        self._wait_for_convergence()
        
        self.get_logger().info('Localization complete!')
        
    def _disperse_particles(self):
        """Call AMCL's global_localization service"""
        request = SetInitialPose.Request()
        # Empty request = disperse particles randomly
        future = self.global_loc_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Particles dispersed across map')
        else:
            self.get_logger().error('Failed to disperse particles')
            
    def _rotate_for_localization(self):
        """Rotate robot in place to gather lidar scans for convergence"""
        self.get_logger().info(f'Rotating {math.degrees(self.ROTATION_ANGLE):.0f}° for scan matching...')
        
        twist = Twist()
        twist.angular.z = self.ROTATION_SPEED
        
        duration = self.ROTATION_ANGLE / abs(self.ROTATION_SPEED)
        end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=duration)
        
        # Publish rotation commands
        rate = self.create_rate(20)  # 20Hz
        while self.get_clock().now() < end_time and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
            
        # Stop
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Rotation complete')
        
    def _wait_for_convergence(self):
        """Give AMCL time to converge particles"""
        self.get_logger().info(f'Waiting {self.CONVERGENCE_TIME}s for particle convergence...')
        time.sleep(self.CONVERGENCE_TIME)


def main(args=None):
    rclpy.init(args=args)
    localizer = GlobalLocalizer()
    
    try:
        localizer.localize()
    except Exception as e:
        localizer.get_logger().error(f'Localization failed: {e}')
    finally:
        localizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
