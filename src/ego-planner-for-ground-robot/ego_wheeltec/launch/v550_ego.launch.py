"""V550 real robot integration. Observe first; explicit drive + enable + NEW goal to move."""
import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def boolean(name):
        answer = value(name).lower()
        if answer not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return answer == 'true'

    car = value('car_mode')
    mode = value('sensor_mode')
    drive = boolean('drive')
    if car not in ('V550_4wd', 'V550_mec', 'V550_akm'):
        raise ValueError('Set car_mode:=V550_4wd, V550_mec or V550_akm to match the REAL chassis')
    if mode not in ('scan', 'cloud'):
        raise ValueError('Set sensor_mode:=scan for 2D LiDAR, or cloud for 3D LiDAR')
    if car == 'V550_akm' and drive:
        raise ValueError('V550_akm cannot execute this unicycle MPC/rotate-in-place FSM. '
                         'Observe only until steering/curvature constraints are implemented.')

    share = get_package_share_directory('ego_wheeltec')
    frame, base, odom = value('planning_frame'), value('base_frame'), value('odom_topic')
    planner_file = value('planner_config')
    robot_height = float(value('robot_height'))
    height_margin = float(value('height_margin'))
    if not math.isfinite(height_margin) or height_margin < 0:
        raise ValueError('height_margin must be a finite non-negative distance')
    if mode == 'cloud' and (not math.isfinite(robot_height) or robot_height <= 0):
        raise ValueError('3D projection requires robot_height:=... measured from the floor '
                         'to the highest part of the vehicle, including equipment')
    obstacle_height = robot_height + height_margin if robot_height > 0 else 3.0
    common = {'use_sim_time': False}
    actions = []
    if boolean('start_base'):
        factory = get_package_share_directory('turn_on_wheeltec_robot')
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(factory, 'launch', 'turn_on_wheeltec_robot.launch.py')),
            launch_arguments={'car_mode': car, 'imu_mode': value('imu_mode'),
                              'controller_port': value('controller_port')}.items()))
    if boolean('start_lidar'):
        factory = get_package_share_directory('turn_on_wheeltec_robot')
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(factory, 'launch', 'wheeltec_lidar.launch.py')),
            launch_arguments={'lidar_type': value('lidar_type'),
                              'lidar_port': value('lidar_port')}.items()))
    if mode == 'cloud':
        height = float(value('sensor_height'))
        if not math.isfinite(height) or height <= 0 or not value('lidar_frame'):
            raise ValueError('3D segmentation requires measured sensor_height:=... '
                             '(metres above the floor) and lidar_frame:=...')
        actions.append(Node(package='patchwork', executable='ground_segmentation',
            name='ground_segmentation', output='screen', parameters=[
                os.path.join(share, 'config', 'ground_segmentation.yaml'), common,
                {'sensor_height': height, 'sensor_frame': value('lidar_frame'),
                 'output_frame': value('lidar_frame'), 'allow_latest_tf': False}],
            remappings=[('/pointcloud_scan', value('cloud_topic')),
                        ('/benchmark/N', '/ego/non_ground'),
                        ('/benchmark/P', '/ego/ground'),
                        ('/benchmark/cloud', '/ego/raw_cloud')]))
    planner_overrides = {'grid_map.frame_id': frame,
                         'grid_map.projection_max_height': obstacle_height,
                         'odom_twist_in_body_frame': True}
    if value('obstacles_inflation'):
        planner_overrides['grid_map.obstacles_inflation'] = float(value('obstacles_inflation'))
    guard_parameters = [common]
    if value('guard_config'):
        guard_parameters.append(value('guard_config'))
    guard_parameters.append({'planning_frame': frame, 'base_frame': base, 'odom_topic': odom,
                             'robot_radius': float(value('robot_radius')),
                             'collision_margin': float(value('collision_margin')),
                             'output_topic': value('cmd_vel_topic') if drive else '/ego/cmd_vel_checked'})
    actions.extend([
        Node(package='ego_wheeltec', executable='sensor_bridge', output='screen', parameters=[
            common, {'planning_frame': frame, 'base_frame': base, 'sensor_mode': mode,
                     'input_topic': value('scan_topic') if mode == 'scan' else '/ego/non_ground',
                     'max_height': obstacle_height,
                     'self_filter_radius': float(value('self_filter_radius'))}],
            remappings=[('/goal_pose', value('goal_topic'))]),
        Node(package='ego_planner', executable='ego_planner_node', name='ego_planner_node',
            output='screen', parameters=[planner_file, common, planner_overrides],
            remappings=[('/odom_map', odom), ('/grid_map/odom', odom),
                        ('/grid_map/cloud', '/ego/obstacles'), ('/benchmark/N', '/ego/obstacles'),
                        ('/way_point', value('planner_goal_topic')), ('/cmd_vel', '/ego/cmd_vel_raw')]),
        Node(package='ego_planner', executable='traj_server', name='traj_server',
            output='screen', parameters=[planner_file, common, {'odom_twist_in_body_frame': True}],
            remappings=[('/state_estimation', odom), ('/cmd_vel', '/ego/cmd_vel_raw')]),
        Node(package='ego_wheeltec', executable='cmd_vel_guard', output='screen', parameters=[
            *guard_parameters]),
        LogInfo(msg=f'V550 {car}, sensor={mode}, planning frame={frame}, drive={drive}. '
                    'Guard starts DISABLED; enable and send a NEW goal to execute.'),
    ])
    if boolean('rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', parameters=[common],
            arguments=['-d', value('rviz_config'), '-f', value('rviz_frame') or frame]))
    return actions


def generate_launch_description():
    share = get_package_share_directory('ego_wheeltec')
    args = [
        ('car_mode', '', 'REQUIRED: V550_4wd / V550_mec / V550_akm'),
        ('sensor_mode', '', 'REQUIRED: scan (2D) / cloud (3D + Patchwork)'),
        ('start_base', 'false', 'Start factory base + EKF + robot TF; do not duplicate existing drivers'),
        ('start_lidar', 'false', 'Start factory selected LiDAR driver; independent of start_base'),
        ('controller_port', '/dev/wheeltec_controller', 'Actual chassis serial device'),
        ('imu_mode', 'stm32', 'Bundled profile uses the chassis IMU; external IMUs need their drivers'),
        ('lidar_type', 'ls_N10Plus_uart', 'Bundled N10p USB/serial driver selection'),
        ('lidar_port', '/dev/wheeltec_lidar', 'Actual LiDAR serial device'),
        ('drive', 'false', 'false: isolated checked-command topic; true: allow chassis command output'),
        ('rviz', 'true', 'Start RViz; use false on a headless onboard computer'),
        ('planning_frame', 'odom_combined', 'MUST equal odometry header.frame_id'),
        ('base_frame', 'base_footprint', 'MUST equal odometry child_frame_id'),
        ('odom_topic', '/odom_combined', 'Factory EKF odometry, not raw encoder odom'),
        ('scan_topic', '/scan', 'sensor_msgs/LaserScan input in scan mode'),
        ('cloud_topic', '/points_raw', 'sensor_msgs/PointCloud2 input in cloud mode; set actual topic'),
        ('lidar_frame', '', '3D sensor frame; TF must exist and its Z axis must point up'),
        ('sensor_height', '-1.0', 'Measured 3D LiDAR height above floor (metres)'),
        ('robot_height', '-1.0', 'REQUIRED in cloud mode: measured total vehicle/equipment height (metres)'),
        ('height_margin', '0.1', 'Vertical clearance added above robot_height (metres)'),
        ('self_filter_radius', '0.0', 'Optional measured self-return exclusion radius about base frame'),
        ('goal_topic', '/goal_pose', 'RViz PoseStamped goal; converted into planning_frame'),
        ('planner_goal_topic', '/ego/goal', 'Local EGO target; unknown navigation uses its goal router'),
        ('guard_config', '', 'Optional real command-guard YAML'),
        ('robot_radius', '0.30', 'Measured circumscribed footprint radius, metres'),
        ('collision_margin', '0.10', 'Additional footprint clearance, metres'),
        ('obstacles_inflation', '', 'Optional footprint-based map inflation override in metres'),
        ('rviz_config', os.path.join(share, 'config', 'v550.rviz'), 'RViz configuration'),
        ('rviz_frame', '', 'RViz fixed frame; empty uses planning_frame'),
        ('cmd_vel_topic', '/cmd_vel', 'Factory chassis Twist input; used only when drive=true'),
        ('planner_config', os.path.join(share, 'config', 'ego_v550.yaml'), 'Real robot planner/MPC parameters'),
    ]
    return LaunchDescription([DeclareLaunchArgument(n, default_value=v, description=d)
                              for n, v, d in args] + [OpaqueFunction(function=start_nodes)])
