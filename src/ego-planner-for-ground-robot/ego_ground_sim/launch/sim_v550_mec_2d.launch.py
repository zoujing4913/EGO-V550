#!/usr/bin/env python3
"""Vendor V550_mec visuals + 2D scan + ground EGO/MPC in the CMU indoor scene.

No hardware driver is launched. All velocity commands use /v550_sim topics.
Gazebo planar motion is an ideal chassis model, not mecanum roller dynamics.
"""
import importlib.util
import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _clean_world(context, directory):
    directory.cleanup()
    return []


def _start_server(context, environment):
    path = os.path.abspath(os.path.expanduser(LaunchConfiguration('world').perform(context)))
    tree = ET.parse(path)
    world = tree.getroot().find('world')
    if world is None:
        raise RuntimeError(f'No SDF world found in {path}')
    # Gazebo's Save World As also saves the currently spawned robot. Remove
    # that saved copy in a temporary file, so the launch can spawn a fresh one.
    saved_robots = [m for m in world.findall('model') if m.get('name') == 'v550_mec']
    actions = []
    if saved_robots:
        for model in saved_robots:
            world.remove(model)
        for state in world.findall('state'):
            for model in list(state.findall('model')):
                if model.get('name') == 'v550_mec':
                    state.remove(model)
        directory = tempfile.TemporaryDirectory(prefix='v550_world_')
        cleaned = os.path.join(directory.name, 'scene.world')
        tree.write(cleaned, encoding='utf-8', xml_declaration=True)
        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=_clean_world, kwargs={'directory': directory})])))
        actions.append(LogInfo(msg='Saved V550 copy removed from temporary world; source file preserved.'))
        path = cleaned
    config = os.path.join(get_package_share_directory('ego_ground_sim'), 'config', 'gazebo_v550.yaml')
    actions.append(ExecuteProcess(cmd=['gzserver', '--verbose', path,
                                      '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
                                      '--ros-args', '--params-file', config, '--'],
                                  additional_env=environment, output='screen'))
    return actions


def generate_launch_description():
    share = get_package_share_directory('ego_ground_sim')
    spec = importlib.util.spec_from_file_location(
        'ground_sim_environment', os.path.join(share, 'launch', 'ego_ground_sim.launch.py'))
    base_sim = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(base_sim)
    environment = base_sim._gazebo_environment(share)
    with open(os.path.join(share, 'urdf', 'v550_mec_sim.urdf')) as urdf:
        description = urdf.read()
    description = description.replace('package://ego_ground_sim/', 'file://' + share + '/')
    before_laser, after_laser = description.split('<visualize>false</visualize>', 1)
    description = [before_laser + '<visualize>', LaunchConfiguration('lidar_visualize'),
                   '</visualize>' + after_laser]
    clock = {'use_sim_time': True}
    planner_config = os.path.join(share, 'config', 'ego_planner_v550_sim.yaml')
    arguments = [
        ('gui', 'true', 'Show Gazebo'),
        ('rviz', 'true', 'Show RViz with the V550 model and planning displays'),
        ('focus_robot', 'true', 'Initially focus Gazebo camera on the V550'),
        ('camera_view', 'top', 'Gazebo camera: top (overhead) or follow'),
        ('camera_height', '8.0', 'Height of the overhead camera in metres'),
        ('camera_center_x', LaunchConfiguration('spawn_x'), 'Overhead view centre X'),
        ('camera_center_y', LaunchConfiguration('spawn_y'), 'Overhead view centre Y'),
        ('lidar_visualize', 'true', 'Show the simulated laser rays in Gazebo'),
        ('world', os.path.join(share, 'worlds', 'v550_cmu_indoor.world'), 'Gazebo world'),
        ('spawn_x', '-31.3', 'Open passage in the CMU indoor scene'),
        ('spawn_y', '-23.4', 'Spawn Y in the world/odom_combined frame'),
        ('spawn_z', '0.03', 'Spawn height above the floor'),
        ('spawn_yaw', '-1.57079632679', 'Initial heading along the passage'),
    ]
    spawner = Node(
        package='gazebo_ros', executable='spawn_entity.py', name='spawn_v550_mec',
        arguments=['-topic', '/robot_description', '-entity', 'v550_mec',
                   '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'),
                   '-z', LaunchConfiguration('spawn_z'), '-Y', LaunchConfiguration('spawn_yaw'),
                   '-timeout', '120.0'], parameters=[clock], output='screen')
    actions = [SetEnvironmentVariable('CYCLONEDDS_URI', os.environ.get(
        'CYCLONEDDS_URI', 'file://' + os.path.join(share, 'config', 'cyclonedds_sim.xml')))]
    actions.extend(DeclareLaunchArgument(n, default_value=v, description=d) for n, v, d in arguments)
    actions.extend([
        OpaqueFunction(function=_start_server, kwargs={'environment': environment}),
        ExecuteProcess(cmd=['gzclient', '--verbose'], additional_env=environment,
                       condition=IfCondition(LaunchConfiguration('gui')), output='log'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[clock, {'robot_description': ParameterValue(description, value_type=str)}],
             output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=[
            ExecuteProcess(cmd=['python3', os.path.join(share, 'scripts', 'focus_v550.py'),
                                '--view', LaunchConfiguration('camera_view'),
                                '--height', LaunchConfiguration('camera_height'),
                                '--center-x', LaunchConfiguration('camera_center_x'),
                                '--center-y', LaunchConfiguration('camera_center_y')],
                           additional_env=environment,
                           condition=IfCondition(LaunchConfiguration('focus_robot')),
                           output='screen')]), condition=IfCondition(LaunchConfiguration('gui'))),
        spawner,
        Node(package='ego_ground_sim', executable='trajectory_visualizer',
             parameters=[clock], output='log'),
        Node(package='ego_wheeltec', executable='sensor_bridge', parameters=[clock, {
            'sensor_mode': 'scan', 'input_topic': '/scan', 'planning_frame': 'odom_combined',
            'base_frame': 'base_footprint', 'min_range': 0.15, 'max_range': 8.0,
            'max_height': 0.4}], output='screen'),
        Node(package='ego_planner', executable='ego_planner_node', name='ego_planner_node',
             parameters=[planner_config, clock], output='log', remappings=[
                 ('/odom_map', '/odom_combined'), ('/grid_map/odom', '/odom_combined'),
                 ('/grid_map/cloud', '/ego/obstacles'), ('/benchmark/N', '/ego/obstacles'),
                 ('/way_point', '/ego/goal'), ('/cmd_vel', '/v550_sim/cmd_vel_raw')]),
        Node(package='ego_planner', executable='traj_server', name='traj_server',
             parameters=[planner_config, clock], output='log', remappings=[
                 ('/state_estimation', '/odom_combined'), ('/cmd_vel', '/v550_sim/cmd_vel_raw')]),
        Node(package='ego_ground_sim', executable='cmd_vel_safety', name='v550_sim_watchdog',
             parameters=[clock, {'max_linear': 0.22, 'max_angular': 0.8,
                                 'cmd_timeout': 0.5, 'odom_timeout': 0.4,
                                 'use_scan_guard': True, 'robot_radius': 0.18,
                                 'forward_only': True,
                                 'collision_margin': 0.12, 'scan_timeout': 0.35}], remappings=[
                 ('/cmd_vel', '/v550_sim/cmd_vel_raw'), ('/cmd_vel_safe', '/v550_sim/cmd_vel'),
                 ('/collision_guard/stopped', '/v550_sim/collision_stop'),
                 ('/state_estimation', '/odom_combined')], output='log'),
        Node(package='rviz2', executable='rviz2', parameters=[clock],
             arguments=['-d', os.path.join(share, 'config', 'v550_sim.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='log'),
        LogInfo(msg='V550_mec + 2D LiDAR, simulated clock. Send RViz 2D Goal Pose; '
                    'nearby goal: x=-31.3, y=-25.4. Simulation commands only on /v550_sim/ topics.'),
    ])
    return LaunchDescription(actions)
