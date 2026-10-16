"""Bundled resources and complete hardware launch expansion; never run driver nodes."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import LifecycleNode, Node
import yaml


PROJECT = Path(__file__).resolve().parents[2]
WORKSPACE = PROJECT.parent.parent
PACKAGES = ('turn_on_wheeltec_robot', 'wheeltec_robot_msg', 'serial',
            'lslidar_driver', 'lslidar_msgs', 'wheeltec_robot_urdf', 'wheeltec_slam_toolbox')


def test_all_vendor_packages_resolve_inside_this_workspace():
    for package in PACKAGES:
        prefix = Path(get_package_prefix(package)).resolve()
        assert prefix.is_relative_to(WORKSPACE / 'install'), (package, prefix)
        source = PROJECT / 'wheeltec_vendor' / package
        assert source.is_dir()
        for item in source.rglob('*'):
            if item.is_symlink():
                assert item.resolve().is_relative_to(PROJECT)


def test_installed_v550_models_have_every_mesh():
    for model in ('V550_mec', 'V550_4wd', 'V550_akm'):
        share = Path(get_package_share_directory('wheeltec_robot_urdf'))
        root = ET.parse(share / 'urdf' / f'{model}_robot.urdf').getroot()
        for mesh in root.iter('mesh'):
            uri = mesh.attrib['filename']
            assert uri.startswith('package://')
            package, relative = uri[len('package://'):].split('/', 1)
            assert (Path(get_package_share_directory(package)) / relative).is_file(), uri


def inspect_launch(monkeypatch, filename, arguments):
    seen = []

    def capture(action, context):
        # Expand the same parameters, xacro and remaps as launch, without executing a node.
        action._perform_substitutions(context)

        def expand(value):
            return perform_substitutions(context, normalize_to_list_of_substitutions(value))

        parameters = {}
        for filename, is_file in action._Node__expanded_parameter_arguments or []:
            if is_file:
                # launch emits trusted parameter arrays using YAML tuple tags.
                document = yaml.full_load(Path(filename).read_text()) or {}
                for block in document.values():
                    if isinstance(block, dict):
                        parameters.update(block.get('ros__parameters', {}))
        seen.append((expand(action.node_package), expand(action.node_executable), parameters))
        return []

    def forbidden(*args, **kwargs):
        raise AssertionError('The launch inspection must never execute a driver process')

    monkeypatch.setattr(Node, 'execute', capture)
    monkeypatch.setattr(LifecycleNode, 'execute', capture)
    monkeypatch.setattr(ExecuteProcess, 'execute', forbidden)
    share = Path(get_package_share_directory('ego_wheeltec'))
    include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(share / 'launch' / filename)), launch_arguments=arguments.items())
    service = LaunchService()
    service.include_launch_description(LaunchDescription([include]))
    assert service.run() == 0
    return seen


def test_complete_hardware_launch_keeps_ports_model_and_observation_mode(monkeypatch):
    seen = inspect_launch(monkeypatch, 'v550_mec_2d.launch.py', {
            'start_base': 'true', 'start_lidar': 'true', 'rviz': 'false',
            'drive': 'false', 'controller_port': '/dev/test_v550_base',
            'lidar_port': '/dev/test_v550_lidar', 'lidar_type': 'ls_N10Plus_uart',
        })
    by_executable = {executable: parameters for _, executable, parameters in seen}
    base = by_executable['wheeltec_robot_node']
    assert base['car_mode'] == 'V550_mec'
    assert base['usart_port_name'] == '/dev/test_v550_base'
    lidar = by_executable['lslidar_driver_node']
    assert lidar['lidar_model'] == 'N10Plus'
    assert lidar['serial_port'] == '/dev/test_v550_lidar'
    assert by_executable['cmd_vel_guard']['output_topic'] == '/ego/cmd_vel_checked'
    assert 'ekf_node' in by_executable
    assert 'robot_description' in by_executable['robot_state_publisher']


def test_unknown_navigation_launch_has_slam_router_guard_and_only_ego_controller(monkeypatch):
    seen = inspect_launch(monkeypatch, 'v550_unknown_nav.launch.py', {
        'rviz': 'false', 'controller_port': '/dev/test_base', 'lidar_port': '/dev/test_scan',
        'robot_radius': '0.31', 'collision_margin': '0.09',
    })
    by_executable = {executable: parameters for _, executable, parameters in seen}
    assert 'async_slam_toolbox_node' in by_executable
    assert 'goal_router.py' in by_executable
    assert 'trajectory_visualizer' in by_executable
    assert by_executable['goal_router.py']['robot_radius'] == 0.31
    guard = by_executable['cmd_vel_guard']
    assert guard['obstacle_guard'] and guard['require_navigation_ready'] and guard['forward_only']
    assert guard['robot_radius'] == 0.31 and guard['collision_margin'] == 0.09
    assert guard['output_topic'] == '/cmd_vel'
    assert by_executable['async_slam_toolbox_node']['use_sim_time'] is False
    assert 'controller_server' not in by_executable
