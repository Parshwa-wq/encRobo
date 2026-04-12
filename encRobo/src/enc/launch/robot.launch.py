"""
robot.launch.py
───────────────
Launches the full robot stack:
  1. Robot State Publisher     (URDF → TF)
  2. micro-ROS agent           (ESP32 serial bridge)
  3. Odometry Publisher        (/joint_states → /odom)
  4. EKF Node                  (/odom + /imu/data → /odometry/filtered)
  5. SLAM Toolbox              (uses /odometry/filtered)
  6. RViz2

Usage:
  ros2 launch <your_package> robot.launch.py

Dependencies (install if missing):
  sudo apt install ros-$ROS_DISTRO-robot-localization
  sudo apt install ros-$ROS_DISTRO-slam-toolbox
  pip install pyserial
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('enc')  # ← CHANGE THIS

    # ── 1. Robot State Publisher (URDF / xacro) ───────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'rsp.launch.py')
        )
    )

    # ── 2. micro-ROS Agent (serial bridge to ESP32) ──────────────────────-----
    # ── 3. Odometry Publisher (/joint_states → /odom) ─────────────────────────
    odometry_publisher = Node(
        package='enc',   # ← CHANGE THIS
        executable='odometry_publisher.py',
        name='odometry_publisher',
        output='screen',
    )

    # ── 4. EKF Node (robot_localization) ─────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'ekf.yaml')],
        remappings=[
            # EKF publishes to /odometry/filtered by default
            # SLAM / Nav2 should subscribe to /odometry/filtered
        ]
    )

    # ── 5. Lidar (sllidar_ros2) ───────────────────────────────────────────────
    #  Adjust serial_port and serial_baudrate for your LIDAR model.
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',   # ← check with: ls /dev/ttyUSB*
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen',
    )

    # ── 6. SLAM Toolbox ───────────────────────────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'online_async_launch.py')
        )
    )

    slip_detetctor_node=Node(
        package='enc',
        executable='slip_detector.py',
        output='screen'
    )

    # ── 7. RViz2 ─────────────────────────────────────────────────────────────
    # ── Launch order: slight delays so micro-ROS agent starts before nodes ────
    return LaunchDescription([
        rsp,
        TimerAction(period=2.0, actions=[odometry_publisher]),  # wait for agent
        TimerAction(period=2.5, actions=[ekf_node]),
        TimerAction(period=3.0, actions=[lidar_node]),
        TimerAction(period=3.5, actions=[slam]),
    ])
