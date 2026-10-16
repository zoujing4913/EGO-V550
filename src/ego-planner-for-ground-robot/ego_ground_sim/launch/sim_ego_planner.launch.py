#!/usr/bin/env python3
"""One-click closed loop: Gazebo simulation + patchwork + ego-planner + MPC + RViz2.

    ros2 launch ego_ground_sim sim_ego_planner.launch.py
    ros2 launch ego_ground_sim sim_ego_planner.launch.py gui:=false rviz:=false

Then use RViz2's "2D Goal Pose" tool (topic /goal_pose) to send the robot to a
target. Everything runs on simulation time; the velocity chain is
    ego_planner/traj_server -> /cmd_vel -> cmd_vel_safety -> /cmd_vel_safe -> Gazebo
and the perception chain is
    Gazebo 3D LiDAR -> /scan_3d -> point_cloud_transformer -> /registered_scan
                                                     -> patchwork -> /benchmark/N
                                                     -> ego-planner local map.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('ego_ground_sim')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    odom_topic = LaunchConfiguration('odom_topic')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    mapping_cloud_topic = LaunchConfiguration('mapping_cloud_topic')
    goal_topic = LaunchConfiguration('goal_topic')
    planner_config = LaunchConfiguration('planner_config')

    patchwork_config = PathJoinSubstitution([sim_share, 'config', 'ground_segmentation_sim.yaml'])
    rviz_config = LaunchConfiguration('rviz_config')

    declared = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world',
                              default_value=PathJoinSubstitution([sim_share, 'worlds', 'obstacles.world']),
                              description='Gazebo world file, i.e. the scenario/map that is simulated'),
        DeclareLaunchArgument('spawn_x', default_value='-8.0'),
        DeclareLaunchArgument('spawn_y', default_value='-6.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.03'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.64'),
        DeclareLaunchArgument('odom_topic', default_value='/state_estimation'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/registered_scan'),
        DeclareLaunchArgument(
            'mapping_cloud_topic', default_value='/benchmark/N',
            description='Cloud used for the occupancy map. The ground-segmented cloud '
                        '(non-ground points only) is used by default: the mapper marks every '
                        'measured point as occupied, so feeding it the raw scan would put the '
                        'ground itself - and therefore the robot - inside an obstacle.'),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([sim_share, 'config', 'ego_ground_sim.rviz']),
            description='RViz2 layout file'),
        DeclareLaunchArgument(
            'planner_config',
            default_value=PathJoinSubstitution([sim_share, 'config', 'ego_planner_sim.yaml']),
            description='planner/traj_server parameter file, e.g. config/ego_planner_indoor.yaml '
                        'for the CMU indoor scene (larger map, wider local update range)'),
    ]

    # ---------------- simulation (Gazebo + chassis + LiDAR + TF) ----------------
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'ego_ground_sim.launch.py')),
        launch_arguments={
            'gui': gui,
            'use_sim_time': use_sim_time,
            'world': world,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'spawn_yaw': spawn_yaw,
        }.items(),
    )

    # ------------------------- ground segmentation -----------------------------
    ground_segmentation = Node(
        package='patchwork',
        executable='ground_segmentation',
        name='ground_segmentation',
        output='screen',
        parameters=[patchwork_config],
        remappings=[('/pointcloud_scan', pointcloud_topic)],
    )

    # ------------------------------ planner + MPC -------------------------------
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner_node',
        output='screen',
        parameters=[planner_config],
        remappings=[
            ('/odom_map', odom_topic),
            ('/way_point', goal_topic),
            ('/grid_map/odom', odom_topic),
            ('/grid_map/cloud', mapping_cloud_topic),
        ],
    )

    traj_server = Node(
        package='ego_planner',
        executable='traj_server',
        name='traj_server',
        output='screen',
        parameters=[planner_config],
        remappings=[('/state_estimation', odom_topic)],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='log',
    )

    return LaunchDescription(declared + [
        simulation,
        ground_segmentation,
        ego_planner_node,
        traj_server,
        rviz_node,
    ])
