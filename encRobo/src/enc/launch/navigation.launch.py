"""
navigation.launch.py — WORKING VERSION
──────────────────────────────────────
Fixed: removed global_localizer (will add back later)
Fixed: increased bond_timeout for lifecycle_nav
Fixed: removed rclpy.shutdown() from Python scripts (done separately)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('enc')
    params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')
    map_path = LaunchConfiguration('map').perform(context)

    if not os.path.isfile(map_path):
        raise FileNotFoundError(f"\n\n  MAP FILE NOT FOUND: {map_path}\n")

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'rsp.launch.py')
        )
    )

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_path}],
    )

    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
    )

    # Lifecycle Manager - Localization
    lifecycle_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # Odometry Publisher
    odometry_publisher = Node(
        package='enc',
        executable='odometry_publisher.py',
        name='odometry_publisher',
        output='screen',
    )

    # EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'ekf.yaml')],
    )

    # Nav2 Nodes
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

    # Lifecycle Manager - Navigation
    lifecycle_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
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

    # Lidar
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen',
    )

    # Slip Detector
    slip_detector = Node(
        package='enc',
        executable='slip_detector.py',
        name='slip_detector',
        output='screen',
    )

    return [
        rsp,
        map_server,
        amcl,
        lifecycle_loc,
        TimerAction(period=2.0, actions=[odometry_publisher]),
        TimerAction(period=2.5, actions=[ekf_node]),
        TimerAction(period=3.0, actions=[lidar_node]),
        TimerAction(period=5.0, actions=[
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            collision_monitor,
        ]),
        TimerAction(period=12.0, actions=[lifecycle_nav]),
        TimerAction(period=13.0, actions=[slip_detector]),
    ]


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(os.path.expanduser('~'), 'my_map.yaml'),
        description='Full path to map yaml file'
    )
    return LaunchDescription([map_arg, OpaqueFunction(function=launch_setup)])
