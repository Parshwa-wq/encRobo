"""
navigation_sim_launch.py — Nav2 full stack for Gazebo Harmonic simulation
──────────────────────────────────────────────────────────────────────────
Run AFTER you have built a map with simulation_launch.py and saved it:
    ros2 run nav2_map_server map_saver_cli -f ~/my_sim_map

Workflow:
  1. ros2 launch enc simulation_launch.py          ← build the map
  2. Save map when done:  ros2 run nav2_map_server map_saver_cli -f ~/my_sim_map
  3. Ctrl+C the SLAM launch
  4. ros2 launch enc navigation_sim_launch.py map:=~/my_sim_map.yaml

This launch file starts:
  - Gazebo Harmonic + field world
  - Robot State Publisher
  - Bridge (gz ↔ ROS 2)
  - EKF (odometry fusion)
  - Map Server + AMCL (localization using saved map)
  - Full Nav2 stack (planner, controller, behaviors, BT navigator)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, ExecuteProcess, SetEnvironmentVariable, OpaqueFunction
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('enc')
    params_file = os.path.join(pkg, 'config', 'nav2_params_sim.yaml')
    map_path = LaunchConfiguration('map').perform(context)

    if not os.path.isfile(map_path):
        raise FileNotFoundError(
            f"\n\n  MAP FILE NOT FOUND: {map_path}\n"
            f"  Build a map first with simulation_launch.py, then save it:\n"
            f"    ros2 run nav2_map_server map_saver_cli -f ~/my_sim_map\n"
        )

    # ── Set GZ mesh resource path ──────────────────────────────────
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg, 'meshes')
    )

    # ── Gazebo ────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r',
             os.path.join(pkg, 'worlds', 'field.world')],
        output='screen',
    )

    # ── Robot State Publisher ─────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'rsp_sim_launch.py')
        ),
    )

    # ── Spawn robot ───────────────────────────────────────────────
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ],
        output='screen',
    )

    # ── Bridge ────────────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    # ── EKF ───────────────────────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'ekf_sim.yaml')],
    )

    # ── Map Server ────────────────────────────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_path}],
    )

    # ── AMCL (particle-filter localization) ──────────────────────
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
    )

    # ── Lifecycle Manager: Localization ───────────────────────────
    lifecycle_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # ── Nav2 Stack ────────────────────────────────────────────────
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel_smoothed'),
        ],
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[params_file],
    )

    # ── Lifecycle Manager: Navigation ─────────────────────────────
    lifecycle_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 60.0,
            'attempt_respawn_reconnection': True,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'collision_monitor',
            ],
        }],
    )

    # ── RViz ──────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'nav2_rviz.rviz')],
        output='screen',
    )

    return [
        set_resource_path,
        gazebo,
        rsp,
        TimerAction(period=5.0,  actions=[spawn_robot]),
        TimerAction(period=6.0,  actions=[bridge]),
        TimerAction(period=7.0,  actions=[ekf_node, map_server, amcl, lifecycle_loc]),
        TimerAction(period=12.0, actions=[
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            collision_monitor,
        ]),
        TimerAction(period=18.0, actions=[lifecycle_nav]),
        TimerAction(period=20.0, actions=[rviz]),
    ]


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(os.path.expanduser('~'), 'my_sim_map.yaml'),
        description='Full path to saved map .yaml file'
    )
    return LaunchDescription([
        map_arg,
        OpaqueFunction(function=launch_setup),
    ])
