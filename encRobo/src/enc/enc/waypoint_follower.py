#!/usr/bin/env python3
"""
waypoint_follower.py
─────────────────────
Sends your robot to a list of waypoints autonomously using Nav2.

HOW IT WORKS:
  1. You define waypoints as (x, y, yaw) in map coordinates
  2. This script sends them one-by-one to Nav2's NavigateToPose action
  3. Nav2 plans a path, avoids obstacles, and drives the robot there
  4. Robot moves using /cmd_vel which your motor driver subscribes to

HOW TO GET WAYPOINT COORDINATES:
  Run RViz, click "2D Pose Estimate" and look at the pose in the
  /initialpose topic, or use the "Publish Point" tool and note the
  coordinates shown in the terminal.

Usage (after starting navigation.launch.py):
  ros2 run enc waypoint_follower.py

  Or with custom waypoints:
  Edit the WAYPOINTS list below then run.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import time


# ── DEFINE YOUR WAYPOINTS HERE ─────────────────────────────────────────────────
# Format: (x_metres, y_metres, yaw_radians)
# x, y are in MAP frame coordinates (match your my_map.yaml origin)
# yaw: 0=facing right (+X), pi/2=facing up (+Y), pi=facing left (-X)
#
# TIP: In RViz use "Nav2 Goal" button to find coordinates by clicking on map
# TIP: math.radians(90) = pi/2, math.radians(180) = pi, etc.

WAYPOINTS = [
    (1.0,  0.0,  0.0),               # 1m forward, facing right
    (1.0,  1.0,  math.radians(90)),  # 1m forward 1m left, facing up
    (0.0,  1.0,  math.radians(180)), # back to side, facing left
    (0.0,  0.0,  0.0),               # home, facing right
]

LOOP = False   # set True to repeat waypoints forever
# ───────────────────────────────────────────────────────────────────────────────


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._current_wp = 0
        self._active = False

    def send_waypoint(self, x, y, yaw):
        self.get_logger().info(
            f'→ Waypoint {self._current_wp + 1}/{len(WAYPOINTS)}: '
            f'x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}°')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        # Yaw → quaternion
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by Nav2!')
            return
        self.get_logger().info('Goal accepted — robot moving...')
        self._result_future = handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        self.get_logger().info(
            f'  Distance remaining: {dist:.2f}m',
            throttle_duration_sec=3.0)

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info(
                f'Reached waypoint {self._current_wp + 1}!')
        else:
            self.get_logger().warn(
                f'Waypoint {self._current_wp + 1} failed (status={status}). Moving to next.')

        self._current_wp += 1

        if self._current_wp >= len(WAYPOINTS):
            if LOOP:
                self.get_logger().info('All waypoints done — looping!')
                self._current_wp = 0
            else:
                self.get_logger().info('All waypoints completed!')
                rclpy.shutdown()
                return

        # Small pause before next waypoint
        time.sleep(1.0)
        wp = WAYPOINTS[self._current_wp]
        self.send_waypoint(*wp)

    def run(self):
        self.get_logger().info(
            f'WaypointFollower ready — {len(WAYPOINTS)} waypoints, loop={LOOP}')
        self.get_logger().info('Waiting for Nav2...')
        self._client.wait_for_server()
        self.get_logger().info('Nav2 connected! Starting navigation.')
        self.send_waypoint(*WAYPOINTS[0])


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    node.run()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
