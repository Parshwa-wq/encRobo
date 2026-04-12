"""
navigation.launch.py — FIXED
──────────────────────────────
BUG FIXED: yaml_filename parameter was passed as an unresolved
LaunchConfiguration substitution object. map_server tried to read
it at configure time before it was resolved → crash.

FIX: Use OnProcessStart / EnvironmentVariable approach is too complex.
     Simplest reliable fix: use LaunchConfiguration with a proper
     OpaqueFunction so the map path is a real Python string by the
     time we build the Node() object.

     This way {'yaml_filename': map_path_str} contains an actual
     resolved string, not a substitution object.
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
    """
    OpaqueFunction resolves all LaunchConfiguration values into real
    Python strings BEFORE we build Node() objects.
    This guarantees yaml_filename is a real string, not a substitution.
    """

    pkg         = get_package_share_directory('enc')
    params_file = os.path.join(pkg, 'config', 'nav2_params.yaml')

    # ── Resolve map path to a real string ─────────────────────────────────────
    # LaunchConfiguration('map').perform(context) → actual string like
    # '/home/pasu/my_map.yaml'  — this is what map_server needs
    map_path = LaunchConfiguration('map').perform(context)

    # Sanity check — tell user immediately if map file doesn't exist
    if not os.path.isfile(map_path):
        raise FileNotFoundError(
            f"\n\n  MAP FILE NOT FOUND: {map_path}\n"
            f"  Run:  ros2 launch enc navigation.launch.py "
            f"map:=/full/path/to/your/my_map.yaml\n"
            f"  Example: map:=/home/pasu/my_map.yaml\n"
        )

    # ── 1. Robot State Publisher ───────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'rsp.launch.py')
        )
    )

    # ── 2. Map Server ──────────────────────────────────────────────────────────
    # FIXED: yaml_filename is now a real resolved string, not LaunchConfiguration
    # map_server reads this at configure time → works correctly
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': map_path}   # ← real string now, not substitution
        ],
    )

    # ── 3. AMCL ───────────────────────────────────────────────────────────────
    # AMCL publishes map→odom TF. Without this, nothing in Nav2 works.
    # It only activates AFTER map_server is active (lifecycle manager handles order).
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
    )

    # ── 4. Lifecycle Manager — Localization ────────────────────────────────────
    # Configures and activates map_server FIRST, then amcl.
    # This order matters: amcl needs /map to be published before it activates.
    lifecycle_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart':    True,
            'node_names':   ['map_server', 'amcl'],
        }],
    )

    # ── 5. Odometry Publisher ─────────────────────────────────────────────────
    odometry_publisher = Node(
        package='enc',
        executable='odometry_publisher.py',
        name='odometry_publisher',
        output='screen',
    )

    # ── 6. EKF ────────────────────────────────────────────────────────────────
    # Publishes odom→base_link TF and /odometry/filtered.
    # Nav2 needs both this AND AMCL's map→odom TF for the full chain:
    # map → odom → base_link → laser_frame
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'ekf.yaml')],
    )

    # ── 7. Controller Server ───────────────────────────────────────────────────
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    # ── 8. Smoother Server ────────────────────────────────────────────────────
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file],
    )

    # ── 9. Planner Server ─────────────────────────────────────────────────────
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    # ── 10. Behavior Server ───────────────────────────────────────────────────
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    # ── 11. BT Navigator ──────────────────────────────────────────────────────
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
    )

    # ── 12. Waypoint Follower ─────────────────────────────────────────────────
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file],
    )

    # ── 13. Velocity Smoother ─────────────────────────────────────────────────
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('cmd_vel',          'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel_smoothed'),
        ],
    )

    # ── 14. Collision Monitor ─────────────────────────────────────────────────
    # FINAL publisher of /cmd_vel.
    # /cmd_vel_smoothed → emergency stop check → /cmd_vel → your ESP32
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[params_file],
    )

    # ── 15. Lifecycle Manager — Navigation ────────────────────────────────────
    # docking_server intentionally NOT in this list.
    #
    # bond_timeout: how long lifecycle_manager waits for each node to respond
    # to a configure/activate call. Default is 4.0s — too short for
    # controller_server which must load the Pure Pursuit plugin from disk.
    # Set to 30.0s so slow plugin loads never cause silent activation failures.
    #
    # attempt_respawn_reconnection: if a managed node dies and restarts,
    # lifecycle_manager will try to re-bond with it automatically.
    lifecycle_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time':                   False,
            'autostart':                      True,
            'bond_timeout':                   30.0,   # ← was default 4.0s, too short
            'attempt_respawn_reconnection':   True,
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

    # ── 16. Lidar ─────────────────────────────────────────────────────────────
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port':      '/dev/ttyUSB0',
            'serial_baudrate':  115200,
            'frame_id':         'laser_frame',
            'inverted':         False,
            'angle_compensate': True,
        }],
        output='screen',
    )

    # ── 17. Slip Detector ─────────────────────────────────────────────────────
    slip_detector = Node(
        package='enc',
        executable='slip_detector.py',
        name='slip_detector',
        output='screen',
    )

    # ── Launch sequence ────────────────────────────────────────────────────────
    # Order matters:
    #   Localization (map_server + amcl) must be ACTIVE before Nav2 starts.
    #   Nav2 costmaps need map→odom→base_link TF chain to exist.
    #   EKF must be running before Nav2 tries to read /odometry/filtered.
    return [
        # t=0: localization stack starts (map_server → amcl → map frame appears)
        rsp,
        map_server,
        amcl,
        lifecycle_loc,

        # t=2: sensor/odometry (waits for micro-ROS Docker topics)
        TimerAction(period=2.0, actions=[odometry_publisher]),
        TimerAction(period=2.5, actions=[ekf_node]),
        TimerAction(period=3.0, actions=[lidar_node]),

        # t=5: Nav2 nodes spawn — they begin loading plugins from disk
        # controller_server loads Pure Pursuit plugin (can take 2-4 seconds)
        # 7-second gap before lifecycle knocks (was 0.5s — that was the bug)
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

        # t=12: lifecycle activates nav nodes
        # bond_timeout:30.0 in lifecycle params handles slow plugin loads
        TimerAction(period=12.0, actions=[lifecycle_nav]),

        # t=13: safety nodes
        TimerAction(period=13.0, actions=[slip_detector]),
    ]


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        'map',
        # ── DEFAULT MAP PATH ───────────────────────────────────────────────────
        # Change this to wherever your my_map.yaml actually is.
        # Or always pass it explicitly:
        #   ros2 launch enc navigation.launch.py map:=/home/pasu/my_map.yaml
        default_value=os.path.join(os.path.expanduser('~'), 'my_map.yaml'),
        description='Full path to map yaml file (e.g. /home/pasu/my_map.yaml)'
    )

    return LaunchDescription([
        map_arg,
        OpaqueFunction(function=launch_setup),
    ])
