#!/usr/bin/env python3
"""V550 in an empty arena; use world:=/absolute/path/my_map.world to reload edits."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('ego_ground_sim')
    defaults = {
        'world': os.path.join(share, 'worlds', 'v550_empty.world'),
        'gui': 'true', 'rviz': 'true', 'focus_robot': 'true',
        'camera_view': 'top', 'camera_height': '12.0',
        'camera_center_x': '0.0', 'camera_center_y': '0.0', 'lidar_visualize': 'true',
        'spawn_x': '0.0', 'spawn_y': '0.0', 'spawn_z': '0.03', 'spawn_yaw': '0.0',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'sim_v550_mec_2d.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items()),
    ])
