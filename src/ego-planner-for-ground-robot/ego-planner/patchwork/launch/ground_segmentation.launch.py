#!/usr/bin/env python3
"""Starts the patchwork ground segmentation node.

    ros2 launch patchwork ground_segmentation.launch.py pointcloud_topic:=/registered_scan

Subscribes : /pointcloud_scan  (PointCloud2)
Publishes  : /benchmark/cloud, /benchmark/P (ground), /benchmark/N (non-ground)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    patchwork_share = get_package_share_directory('patchwork')
    param_file = PathJoinSubstitution([patchwork_share, 'config', 'ground_segmentation.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('pointcloud_topic', default_value='/registered_scan'),
        Node(
            package='patchwork',
            executable='ground_segmentation',
            name='ground_segmentation',
            output='screen',
            parameters=[param_file],
            remappings=[('/pointcloud_scan', LaunchConfiguration('pointcloud_topic'))],
        ),
    ])
