#!/usr/bin/env python3
"""One-click start in the CMU indoor corridor scene.

    ros2 launch ego_ground_sim sim_cmu_indoor.launch.py
    ros2 launch ego_ground_sim sim_cmu_indoor.launch.py gui:=false rviz:=false

Uses worlds/cmu_indoor.world (the mesh that ships with this repository) and
config/ego_planner_indoor.yaml (140 x 110 x 2.5 m map, 8 m local update range).

The default spawn point (-31.3, -23.4) is near the red robot in imag/ego_example.gif,
in the passage above the rectangular enclosure. This is (31.5, -3.0) in the
original CMU frame, with about 1.3 m clearance to the walls. Floor support under
the robot and 2 m along the passage was checked against the collision mesh.
The robot faces along -Y; the Gazebo and RViz cameras look at this area.
The CMU scene's own origin (0, 0) is at (-62.8, -20.4) after the mesh shift of
worlds/cmu_indoor.world; it can still be selected with spawn_x/spawn_y.

Send a goal with RViz2's "2D Goal Pose"; nearby examples on the same open floor
are (-31.3, -25.4) and (-31.3, -21.4). Use scripts/indoor_free_space.py to find
more candidate goals.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    sim_share = get_package_share_directory('ego_ground_sim')

    declared = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'spawn_x', default_value='-31.3',
            description='Passage near the red robot in imag/ego_example.gif; '
                        'CMU original coordinate (31.5, -3.0)'),
        DeclareLaunchArgument('spawn_y', default_value='-23.4'),
        DeclareLaunchArgument('spawn_z', default_value='0.03'),
        DeclareLaunchArgument('spawn_yaw', default_value='-1.57079632679'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([sim_share, 'config', 'cmu_indoor.rviz']),
            description='RViz2 layout with a 160 x 160 m grid and the camera on the robot'),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'sim_ego_planner.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': PathJoinSubstitution([sim_share, 'worlds', 'cmu_indoor.world']),
            'planner_config': PathJoinSubstitution([sim_share, 'config', 'ego_planner_indoor.yaml']),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
    )

    return LaunchDescription(declared + [simulation])
