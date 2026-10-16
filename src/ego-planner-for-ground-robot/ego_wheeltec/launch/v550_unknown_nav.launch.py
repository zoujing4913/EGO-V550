"""V550_mec + N10p: online SLAM, unknown-goal routing, EGO/MPC and chassis guard."""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory('ego_wheeltec')
    radius, margin = float(value('robot_radius')), float(value('collision_margin'))
    if not math.isfinite(radius) or radius <= 0 or not math.isfinite(margin) or margin < 0:
        raise ValueError('robot_radius must be positive; collision_margin must be non-negative')
    arguments = {name: value(name) for name in
                 ('start_base', 'start_lidar', 'controller_port', 'lidar_port', 'imu_mode',
                  'drive', 'rviz', 'robot_radius', 'collision_margin', 'planner_config')}
    arguments.update(car_mode='V550_mec', sensor_mode='scan', lidar_type='ls_N10Plus_uart',
                     planning_frame='odom_combined', base_frame='base_footprint',
                     odom_topic='/odom_combined', scan_topic='/scan', goal_topic='/goal_pose',
                     planner_goal_topic='/ego/local_goal',
                     guard_config=os.path.join(share, 'config', 'guard_unknown.yaml'),
                     obstacles_inflation=str(radius + margin),
                     rviz_config=os.path.join(share, 'config', 'v550_unknown.rviz'), rviz_frame='map')
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'v550_ego.launch.py')), launch_arguments=arguments.items()),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             parameters=[value('slam_config'), {'use_sim_time': False}],
             condition=IfCondition(value('start_slam')), output='screen'),
        Node(package='ego_wheeltec', executable='goal_router.py', name='ego_goal_router',
             parameters=[{'use_sim_time': False, 'robot_radius': radius, 'collision_margin': margin,
                          'tracking_margin': 0.10, 'lookahead': 1.2,
                          'local_map_half_size': 40.0}], output='screen'),
        Node(package='ego_wheeltec', executable='trajectory_visualizer',
             parameters=[{'use_sim_time': False, 'frame_id': 'odom_combined'}], output='screen'),
        LogInfo(msg='Unknown-goal navigation: build SLAM online; route unknown optimistically, '
                    'execute ONLY observed free space. Guard starts disabled: enable, then set a NEW RViz goal.'),
    ]


def generate_launch_description():
    share = get_package_share_directory('ego_wheeltec')
    arguments = [
        ('start_base', 'true', 'Start bundled chassis/EKF/TF; false if already running'),
        ('start_lidar', 'true', 'Start bundled N10p serial driver; false if already running'),
        ('start_slam', 'true', 'Start online SLAM; false only if an equivalent mapper is running'),
        ('controller_port', '/dev/wheeltec_controller', 'Actual chassis serial device'),
        ('lidar_port', '/dev/wheeltec_lidar', 'Actual N10p serial device'),
        ('imu_mode', 'stm32', 'Chassis onboard IMU; external IMUs need their own drivers'),
        ('drive', 'true', 'Connect guarded output to chassis; guard still starts disabled'),
        ('rviz', 'true', 'Start RViz2; false on headless vehicle'),
        ('robot_radius', '0.30', 'PROVISIONAL: replace with measured circumscribed footprint radius in metres'),
        ('collision_margin', '0.10', 'Additional clearance beyond the measured footprint'),
        ('planner_config', os.path.join(share, 'config', 'ego_v550_unknown.yaml'), 'EGO/MPC parameters'),
        ('slam_config', os.path.join(share, 'config', 'slam_v550.yaml'), 'Online SLAM parameters'),
    ]
    return LaunchDescription([DeclareLaunchArgument(name, default_value=default, description=description)
                              for name, default, description in arguments] + [OpaqueFunction(function=start)])
