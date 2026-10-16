#!/usr/bin/env python3
"""ROS 2 (Humble) replacement of `run_in_sim.launch`.

Starts the ego-planner FSM, its MPC trajectory server, the patchwork ground
segmentation node and RViz2.

Example:
    ros2 launch ego_planner run_in_sim.launch.py \
        odom_topic:=/state_estimation pointcloud_topic:=/registered_scan
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    ego_planner_share = get_package_share_directory('ego_planner')
    patchwork_share = get_package_share_directory('patchwork')

    param_file = PathJoinSubstitution([ego_planner_share, 'config', 'ego_planner.yaml'])
    patchwork_param_file = PathJoinSubstitution([patchwork_share, 'config', 'ground_segmentation.yaml'])
    rviz_config = PathJoinSubstitution([ego_planner_share, 'launch', 'ego_planner.rviz'])

    # ------------------------------------------------------------------ args
    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')
    map_size_z = LaunchConfiguration('map_size_z')
    odom_topic = LaunchConfiguration('odom_topic')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    goal_topic = LaunchConfiguration('goal_topic')
    max_vel = LaunchConfiguration('max_vel')
    max_acc = LaunchConfiguration('max_acc')
    planning_horizon = LaunchConfiguration('planning_horizon')
    flight_type = LaunchConfiguration('flight_type')
    start_rviz = LaunchConfiguration('rviz')

    declared_args = [
        DeclareLaunchArgument('map_size_x', default_value='300.0',
                              description='Map size in x [m] (reduce it for small robots)'),
        DeclareLaunchArgument('map_size_y', default_value='300.0'),
        DeclareLaunchArgument('map_size_z', default_value='4.0'),
        DeclareLaunchArgument('odom_topic', default_value='/state_estimation',
                              description='Odometry topic, e.g. from your SLAM'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/registered_scan',
                              description='PointCloud2 topic fed to mapping/segmentation'),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose',
                              description='"2D Goal Pose" of RViz2 publishes here'),
        DeclareLaunchArgument('max_vel', default_value='1.5'),
        DeclareLaunchArgument('max_acc', default_value='3.0'),
        DeclareLaunchArgument('planning_horizon', default_value='10.5'),
        DeclareLaunchArgument('flight_type', default_value='1',
                              description='1: goal from RViz, 2: preset waypoints'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    # parameters that mirror the former <arg>/<param> overrides
    shared_overrides = {
        'grid_map.map_size_x': map_size_x,
        'grid_map.map_size_y': map_size_y,
        'grid_map.map_size_z': map_size_z,
        'fsm.flight_type': flight_type,
        'fsm.planning_horizon': planning_horizon,
        'manager.planning_horizon': planning_horizon,
        'manager.max_vel': max_vel,
        'manager.max_acc': max_acc,
        'optimization.max_vel': max_vel,
        'optimization.max_acc': max_acc,
        'bspline.limit_vel': max_vel,
        'bspline.limit_acc': max_acc,
    }

    # -------------------------------------------------------------- planner
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner_node',
        output='screen',
        parameters=[param_file, shared_overrides],
        remappings=[
            ('/odom_map', odom_topic),
            ('/way_point', goal_topic),
            ('/grid_map/odom', odom_topic),
            ('/grid_map/cloud', pointcloud_topic),
        ],
    )

    traj_server = Node(
        package='ego_planner',
        executable='traj_server',
        name='traj_server',
        output='screen',
        parameters=[param_file],
        remappings=[
            ('/state_estimation', odom_topic),
        ],
    )

    # ------------------------------------------------- ground segmentation
    ground_segmentation = Node(
        package='patchwork',
        executable='ground_segmentation',
        name='ground_segmentation',
        output='screen',
        parameters=[patchwork_param_file],
        remappings=[
            ('/pointcloud_scan', pointcloud_topic),
        ],
    )

    # ----------------------------------------------------------------- rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(start_rviz),
        output='log',
    )

    return LaunchDescription(declared_args + [
        ego_planner_node,
        traj_server,
        ground_segmentation,
        rviz_node,
    ])
