#!/usr/bin/env python3
"""Gazebo simulation only (chassis, LiDAR, TF and the planner interfaces).

    ros2 launch ego_ground_sim ego_ground_sim.launch.py

Provides (everything stamped in simulation time):
    /state_estimation   nav_msgs/Odometry      (differential drive odometry)
    /registered_scan    sensor_msgs/PointCloud2 (LiDAR scan in the 'map' frame)
    /scan_3d            sensor_msgs/PointCloud2 (raw scan in the 'lidar_link' frame)
    TF: map -> odom -> base_footprint -> base_link -> {wheels, lidar_link}
Consumes:
    /cmd_vel            geometry_msgs/Twist    (watchdog -> /cmd_vel_safe -> wheels)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _gazebo_environment(sim_share):
    """Environment for gzserver/gzclient.

    `gazebo_ros`'s own launch files export only the paths that *ROS packages*
    declare via <gazebo_ros> exports plus whatever GAZEBO_RESOURCE_PATH the shell
    already had. If the shell never sourced /usr/share/gazebo/setup.sh - which is
    the normal case - GAZEBO_RESOURCE_PATH ends up without Gazebo's own media
    directory. OGRE then cannot find 'Gazebo/shadow_caster', the scene fails to
    initialise ("Failed to initialize scene" in gzclient's log) and gzclient exits
    immediately, while gzserver (no rendering) and RViz2 (own OGRE resources) keep
    working. So the paths from /usr/share/gazebo-11/setup.sh are added here
    explicitly; an existing value is kept and a launch argument can override it.
    """
    def first_dir(*candidates):
        for candidate in candidates:
            if candidate and os.path.isdir(candidate):
                return candidate
        return ''

    def join(entries):
        return os.pathsep.join([e for e in entries if e])

    share = first_dir('/usr/share/gazebo-11', '/usr/share/gazebo')
    plugin_dir = first_dir('/usr/lib/x86_64-linux-gnu/gazebo-11/plugins',
                           '/usr/lib/gazebo-11/plugins')
    ogre_dir = first_dir('/usr/lib/x86_64-linux-gnu/OGRE-1.9.0')

    # drop the useless ".../share/<pkg>/.." entries some packages export: Gazebo
    # would scan every ROS package as a model and spam "Missing model.config"
    def clean_path(value):
        entries = [e for e in value.split(os.pathsep) if e]
        return [e for e in entries if os.path.basename(os.path.normpath(e)) != '..']

    resource = [share, os.path.join(share, 'media') if share else '']
    resource += clean_path(os.environ.get('GAZEBO_RESOURCE_PATH', ''))
    # models installed with this package (e.g. the CMU indoor scene) come first
    model = [os.path.join(sim_share, 'models')]
    model += [os.path.join(share, 'models') if share else '']
    model += clean_path(os.environ.get('GAZEBO_MODEL_PATH', ''))
    plugin = [plugin_dir] + clean_path(os.environ.get('GAZEBO_PLUGIN_PATH', ''))

    env = {
        'GAZEBO_RESOURCE_PATH': join(resource),
        'GAZEBO_MODEL_PATH': join(model),
        'GAZEBO_PLUGIN_PATH': join(plugin),
    }
    if ogre_dir:
        env['OGRE_RESOURCE_PATH'] = ogre_dir
    return env


def generate_launch_description():
    sim_share = get_package_share_directory('ego_ground_sim')

    urdf_file = os.path.join(sim_share, 'urdf', 'ego_ground_robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    gazebo_env = _gazebo_environment(sim_share)

    declared = [
        DeclareLaunchArgument('world',
                              default_value=PathJoinSubstitution([sim_share, 'worlds', 'obstacles.world']),
                              description='Gazebo world file'),
        DeclareLaunchArgument('gui', default_value='true', description='start gzclient'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('spawn_x', default_value='-8.0'),
        DeclareLaunchArgument('spawn_y', default_value='-6.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.03'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.64'),
        DeclareLaunchArgument('gazebo_resource_path', default_value=gazebo_env['GAZEBO_RESOURCE_PATH'],
                              description='overrides GAZEBO_RESOURCE_PATH of gzserver/gzclient'),
    ]

    # the same environment for both, with an overridable resource path
    gazebo_env['GAZEBO_RESOURCE_PATH'] = LaunchConfiguration('gazebo_resource_path')

    # gzserver: physics + ROS init (/clock) and factory (/spawn_entity) plugins
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        additional_env=gazebo_env,
        output='screen',
    )
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        additional_env=gazebo_env,
        output='screen',
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_ground_robot',
        output='screen',
        arguments=['-file', urdf_file,
                   '-entity', 'ego_ground_robot',
                   '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
                   '-Y', spawn_yaw,
                   '-timeout', '120.0'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # odom frame == map frame (no SLAM in this simulation)
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    cloud_transformer = Node(
        package='ego_ground_sim',
        executable='point_cloud_transformer',
        name='point_cloud_transformer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'target_frame': 'map',
                     'input_topic': '/scan_3d',
                     'output_topic': '/registered_scan',
                     'min_range': 0.2,
                     'max_range': 12.0,
                     'odom_topic': '/state_estimation'}],
    )

    cmd_vel_safety = Node(
        package='ego_ground_sim',
        executable='cmd_vel_safety',
        name='cmd_vel_safety',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'cmd_timeout': 0.5,
                     'odom_timeout': 1.0,
                     'max_linear': 1.5,
                     'max_angular': 2.0}],
    )

    return LaunchDescription(declared + [
        gzserver,
        gzclient,
        robot_state_publisher,
        map_to_odom,
        cloud_transformer,
        cmd_vel_safety,
        spawn_entity,
    ])
