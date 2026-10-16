#!/usr/bin/env python3
"""Starts RViz2 with the ego-planner layout (ROS 2 replacement of rviz.launch)."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    ego_planner_share = get_package_share_directory('ego_planner')
    rviz_config = PathJoinSubstitution([ego_planner_share, 'launch', 'ego_planner.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='log',
        ),
    ])
