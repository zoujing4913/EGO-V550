"""Confirmed hardware: V550_mec + single-plane 2D LiDAR, forward/yaw MPC."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('ego_wheeltec')
    arguments = [
        ('start_base', 'false', 'Start factory V550_mec base, EKF and robot TF if not already running'),
        ('start_lidar', 'false', 'Start factory configured 2D LiDAR driver if not already running'),
        ('controller_port', '/dev/wheeltec_controller', 'Actual chassis serial device'),
        ('imu_mode', 'stm32', 'Chassis IMU; external IMU drivers are separate'),
        ('lidar_type', 'ls_N10Plus_uart', 'N10p USB/serial model in the bundled factory driver'),
        ('lidar_port', '/dev/wheeltec_lidar', 'Actual LiDAR serial device'),
        ('drive', 'false', 'false: observe; true: connect guard output to the chassis'),
        ('rviz', 'true', 'Start RViz; set false on a headless onboard computer'),
        ('odom_topic', '/odom_combined', 'Factory EKF odometry topic'),
        ('scan_topic', '/scan', 'Actual sensor_msgs/LaserScan topic'),
        ('planning_frame', 'odom_combined', 'Must match odometry header.frame_id'),
        ('base_frame', 'base_footprint', 'Must match odometry child_frame_id'),
        ('self_filter_radius', '0.0', 'Measured self-return exclusion radius; do not guess'),
        ('goal_topic', '/goal_pose', 'RViz target pose topic'),
        ('cmd_vel_topic', '/cmd_vel', 'Factory chassis velocity input'),
        ('planner_config', os.path.join(share, 'config', 'ego_v550.yaml'),
         'Planner/MPC commissioning parameters; measure footprint before driving'),
    ]
    forwarded = {name: LaunchConfiguration(name) for name, _, _ in arguments}
    forwarded.update(car_mode='V550_mec', sensor_mode='scan')
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in arguments],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'v550_ego.launch.py')),
            launch_arguments=forwarded.items()),
    ])
