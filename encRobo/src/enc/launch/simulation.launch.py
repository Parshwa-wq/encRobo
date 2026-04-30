"""
simulation_launch.py — Gazebo Harmonic (ros_gz) — FIXED
────────────────────────────────────────────────────────────────
BUG FIXES vs original:
  1. rsp_sim.launch.py → rsp_sim_launch.py  (FileNotFoundError)
  2. Added /tf bridge — WITHOUT THIS odom→base_link TF never
     reaches ROS 2, so SLAM and Nav2 are completely broken
  3. Fixed bridge direction markers:
       @ = bidirectional (avoid unless needed)
       [ = gz → ros2 only  (sensors, odom, clock, tf)
       ] = ros2 → gz only  (cmd_vel)
  4. Fixed /imu/data message type: gz.msgs.IMU (not gz.msgs.Imu)
  5. FIX (this patch): GZ_SIM_RESOURCE_PATH passed via additional_env
     directly on the ExecuteProcess for gz sim — NOT via
     SetEnvironmentVariable, which only sets it in the launch process
     and is not reliably inherited by child processes on all systems.
     Also sets IGN_GAZEBO_RESOURCE_PATH (legacy name, some Harmonic
     builds still read this) and GZ_SIM_RESOURCE_PATH colon-appended
     so any existing system path is preserved.
  6. Removed /joint_states bridge (diff_drive in Harmonic doesn't
     publish gz.msgs.Model — the RSP handles joint states via TF)

Usage:
  ros2 launch enc simulation_launch.py
  ros2 launch enc simulation_launch.py world:=/abs/path/to/field.world

PREREQUISITE — CMakeLists.txt must install the meshes folder:
  install(DIRECTORY meshes/
          DESTINATION share/${PROJECT_NAME}/meshes)
  Then rebuild:  colcon build --packages-select enc
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('enc')

    # Absolute path to the meshes directory inside the installed package.
    # structure.dae must live here after colcon build.
    meshes_path = os.path.join(pkg, 'meshes')

    # Preserve any existing system resource path and append ours.
    # This avoids breaking Gazebo's own built-in model paths.
    system_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    full_resource_path = (
        meshes_path if not system_resource_path
        else f"{meshes_path}:{system_resource_path}"
    )

    # ── Arguments ─────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg, 'worlds', 'field.world'),
        description='Absolute path to the SDF world file'
    )

    # ── Start Gazebo Harmonic with the field world ─────────────────
    # KEY FIX: additional_env injects env vars directly into the gz sim
    # child process. SetEnvironmentVariable (used previously) only sets
    # the var in the launch orchestrator process — gz sim never sees it,
    # so it falls back to Fuel download → hangs → SIGKILL.
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', LaunchConfiguration('world')],
        additional_env={
            # Primary env var for Gazebo Harmonic (gz-sim 8+)
            'GZ_SIM_RESOURCE_PATH': full_resource_path,
            # Legacy name — some Harmonic point releases still read this
            'IGN_GAZEBO_RESOURCE_PATH': full_resource_path,
        },
        output='screen',
    )

    # ── Robot State Publisher ─────────────────────────────────────
    # BUG FIX 1: was 'rsp_sim.launch.py' — file is rsp_sim_launch.py
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'rsp_sim_launch.py')
        ),
    )

    # ── Spawn robot into Gazebo ────────────────────────────────────
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
        ],
        output='screen',
    )

    # ── GZ → ROS 2 Bridge ──────────────────────────────────────────
    # BUG FIX 2: Added /tf bridge (odom→base_link TF from diff_drive plugin)
    # BUG FIX 3: Fixed direction markers ([ = gz→ros2, ] = ros2→gz)
    # BUG FIX 4: /imu/data type is gz.msgs.IMU (capital U)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # cmd_vel: ROS 2 → Gazebo only  (] = ros2 → gz)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

            # odom: Gazebo → ROS 2 only  ([ = gz → ros2)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # scan: Gazebo lidar → ROS 2
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # IMU: Gazebo → ROS 2  (gz.msgs.IMU — capital IMU)
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',

            # BUG FIX 2: TF from diff_drive plugin → ROS 2
            # Without this, odom→base_link TF is missing and SLAM breaks
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            # clock: Gazebo → ROS 2 (required for use_sim_time=true)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    # ── EKF Node ──────────────────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'ekf_sim.yaml')],
    )

    # ── SLAM Toolbox ─────────────────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(
                pkg, 'config', 'mapper_params_online_async_sim.yaml'
            ),
        }.items(),
    )

    # ── RViz ──────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'nav2_rviz.rviz')],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        rsp,
        TimerAction(period=5.0,  actions=[spawn_robot]),
        TimerAction(period=6.0,  actions=[bridge]),
        TimerAction(period=7.0,  actions=[ekf_node]),
        TimerAction(period=8.0,  actions=[slam]),
        TimerAction(period=10.0, actions=[rviz]),
    ])
