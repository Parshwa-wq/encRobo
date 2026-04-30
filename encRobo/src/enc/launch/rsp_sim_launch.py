# rsp_sim_launch.py — Simulation-only Robot State Publisher
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('enc')

    # BUG FIX: was 'robot.urdf.xacro' which matched the old wrong filename.
    # Now correctly references robot.urdf.xacro (the file has been renamed).
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # Process xacro with simulation flags:
    #   sim_mode=true  → includes gazebo_control.xacro (Harmonic diff-drive)
    #   sim_mode=false → includes ros2_control.xacro   (Arduino hardware)
    doc = xacro.process_file(
        xacro_file,
        mappings={
            'sim_mode': 'true',
            'use_ros2_control': 'false',
        }
    )
    robot_description_config = doc.toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_config,
                'use_sim_time': True,
            }]
        )
    ])
