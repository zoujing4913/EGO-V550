"""ROS interface regression tests: synthetic inputs, isolated domain, NO chassis driver.

Run after building: ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q this_file
"""
import math
import os
from pathlib import Path
import subprocess
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, UInt8
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker


@pytest.fixture
def node():
    assert os.environ.get('ROS_DOMAIN_ID') == '217', 'Use isolated ROS_DOMAIN_ID=217'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Use ROS_LOCALHOST_ONLY=1'
    rclpy.init()
    n = rclpy.create_node('ego_interface_test')
    yield n
    n.destroy_node()
    rclpy.shutdown()


def launch(executable, *arguments):
    path = Path(get_package_prefix('ego_wheeltec'))/'lib/ego_wheeltec'/executable
    return subprocess.Popen([str(path), *arguments], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)


def finish(process):
    process.terminate()
    try:
        _, stderr = process.communicate(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        _, stderr = process.communicate()
    assert process.returncode in (0, -15), stderr.decode()


def test_guard_requires_new_goal_stops_and_latches_on_data_loss(node):
    process = launch('cmd_vel_guard')
    outputs = []
    subscriptions = [node.create_subscription(Twist, '/ego/cmd_vel_checked',
                                               lambda m: outputs.append(m), 10)]
    odom_pub = node.create_publisher(Odometry, '/odom_combined', 5)
    cloud_pub = node.create_publisher(PointCloud2, '/ego/obstacles', 5)
    map_pub = node.create_publisher(OccupancyGrid, '/map_2d', 5)
    cmd_pub = node.create_publisher(Twist, '/ego/cmd_vel_raw', 5)
    goal_pub = node.create_publisher(PoseStamped, '/ego/goal', 5)
    client = node.create_client(SetBool, '/ego/set_enabled')

    def pump(duration, cloud=True, cmd=True, invalid=False, stale=False):
        until = time.monotonic()+duration
        while time.monotonic() < until:
            stamp = node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.stamp = stamp; odom.header.frame_id = 'odom_combined'
            odom.child_frame_id = 'base_footprint'; odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            if cloud:
                m = PointCloud2(); m.header.stamp = stamp; m.header.frame_id = 'odom_combined'
                if stale:
                    m.header.stamp.sec -= 3
                cloud_pub.publish(m)
            m = OccupancyGrid(); m.header.stamp = stamp; m.header.frame_id = 'odom_combined'
            m.info.width = m.info.height = 1; m.info.resolution = 0.1; m.data = [0]
            map_pub.publish(m)
            if cmd:
                c = Twist(); c.linear.x = math.nan if invalid else 0.6; c.angular.z = 1.0
                cmd_pub.publish(c)
            rclpy.spin_once(node, timeout_sec=0.02)
        rclpy.spin_once(node, timeout_sec=0.02)

    def enable():
        assert client.wait_for_service(timeout_sec=5)
        req = SetBool.Request(); req.data = True
        future = client.call_async(req)
        deadline = time.monotonic()+3
        while not future.done() and time.monotonic() < deadline:
            pump(0.03)
        assert future.done() and future.result().success

    def goal():
        m = PoseStamped(); m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = 'odom_combined'; m.pose.orientation.w = 1.0; m.pose.position.x = 1.0
        goal_pub.publish(m)

    try:
        pump(1.5)
        assert outputs and outputs[-1].linear.x == 0
        enable()
        pump(0.3)
        assert outputs[-1].linear.x == 0, 'Old commands must not execute after enabling'
        goal(); pump(0.5)
        assert outputs[-1].linear.x == pytest.approx(0.15)
        assert outputs[-1].angular.z == pytest.approx(0.4)
        pump(0.2, invalid=True)
        assert outputs[-1].linear.x == 0, 'NaN must disarm'
        pump(0.2)
        assert outputs[-1].linear.x == 0, 'Valid data must not auto-rearm'
        enable(); goal(); pump(0.3)
        assert outputs[-1].linear.x > 0
        pump(1.0, cloud=False)
        assert outputs[-1].linear.x == 0, 'Perception loss must stop'
        pump(0.3)
        assert outputs[-1].linear.x == 0, 'Perception recovery must not auto-rearm'
        enable(); goal(); pump(0.3)
        pump(1.0, stale=True)
        assert outputs[-1].linear.x == 0, 'Repeated stale packets must not keep motion alive'
        pump(0.3); enable(); goal(); pump(0.3)
        pump(0.5, cmd=False)
        assert outputs[-1].linear.x == 0, 'Planner command loss must stop'
    finally:
        finish(process)


def test_unknown_guard_stops_at_obstacle_or_missing_route_and_allows_only_observation_turn(node):
    process = launch('cmd_vel_guard', '--ros-args', '-p', 'obstacle_guard:=true',
                     '-p', 'require_navigation_ready:=true', '-p', 'forward_only:=true',
                     '-p', 'robot_radius:=0.30', '-p', 'collision_margin:=0.10')
    outputs = []
    subscription = node.create_subscription(Twist, '/ego/cmd_vel_checked', outputs.append, 10)
    odom_pub = node.create_publisher(Odometry, '/odom_combined', 5)
    cloud_pub = node.create_publisher(PointCloud2, '/ego/obstacles', 5)
    map_pub = node.create_publisher(OccupancyGrid, '/map_2d', 5)
    nav_pub = node.create_publisher(Bool, '/ego/navigation_ready', 1)
    turn_pub = node.create_publisher(Twist, '/ego/exploration_turn', 1)
    cmd_pub = node.create_publisher(Twist, '/ego/cmd_vel_raw', 1)
    goal_pub = node.create_publisher(PoseStamped, '/ego/goal', 5)
    client = node.create_client(SetBool, '/ego/set_enabled')

    def pump(duration, ready=True, obstacle=2.0, heartbeat=True, turn=0.0, speed=0.1):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            stamp = node.get_clock().now().to_msg()
            odom = Odometry(); odom.header.stamp = stamp
            odom.header.frame_id, odom.child_frame_id = 'odom_combined', 'base_footprint'
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            cloud_pub.publish(create_cloud_xyz32(Header(stamp=stamp, frame_id='odom_combined'),
                                                 [(obstacle, 0.0, 0.2)]))
            grid = OccupancyGrid(); grid.header.stamp = stamp; grid.header.frame_id = 'odom_combined'
            grid.info.width = grid.info.height = 1; grid.data = [0]
            map_pub.publish(grid)
            if heartbeat:
                nav_pub.publish(Bool(data=ready))
            rotation = Twist(); rotation.angular.z = turn; turn_pub.publish(rotation)
            command = Twist(); command.linear.x = speed; cmd_pub.publish(command)
            rclpy.spin_once(node, timeout_sec=0.015)

    try:
        pump(1.0)
        assert client.wait_for_service(timeout_sec=3)
        request = SetBool.Request(); request.data = True
        future = client.call_async(request)
        while not future.done():
            pump(0.03)
        assert future.result().success
        goal = PoseStamped(); goal.header.frame_id = 'odom_combined'
        goal.header.stamp = node.get_clock().now().to_msg(); goal.pose.orientation.w = 1.0
        goal_pub.publish(goal); pump(0.3)
        assert outputs[-1].linear.x > 0
        pump(0.3, obstacle=0.45)
        assert outputs[-1].linear.x == 0, 'Stop before entering the braking envelope'
        pump(0.3, ready=False)
        assert outputs[-1].linear.x == outputs[-1].angular.z == 0
        pump(0.3, ready=False, turn=0.3)
        assert outputs[-1].linear.x == 0 and outputs[-1].angular.z > 0
        pump(0.3, ready=False, turn=0.3, obstacle=0.2)
        assert outputs[-1].linear.x == outputs[-1].angular.z == 0
        pump(0.3, speed=-0.1)
        assert outputs[-1].linear.x == 0, 'Negative raw input must not reverse'
        pump(1.0, heartbeat=False)
        assert outputs[-1].linear.x == 0
        pump(0.3)
        assert outputs[-1].linear.x == 0, 'Router recovery must not re-enable a disarmed guard'
    finally:
        finish(process)


def test_scan_is_really_rotated_and_goal_uses_planning_frame(node):
    process = launch('sensor_bridge')
    clouds, goals = [], []
    subscriptions = [
        node.create_subscription(PointCloud2, '/ego/obstacles', lambda m: clouds.append(m), 10),
        node.create_subscription(PoseStamped, '/ego/goal', lambda m: goals.append(m), 10),
    ]
    scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 5)
    static = StaticTransformBroadcaster(node)
    dynamic = TransformBroadcaster(node)
    mount = TransformStamped(); mount.header.frame_id = 'base_footprint'
    mount.child_frame_id = 'laser'; mount.header.stamp = node.get_clock().now().to_msg()
    mount.transform.translation.z = 0.1; mount.transform.rotation.w = 1.0
    static.sendTransform(mount)
    try:
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            stamp = node.get_clock().now().to_msg()
            tf = TransformStamped(); tf.header.stamp = stamp; tf.header.frame_id = 'odom_combined'
            tf.child_frame_id = 'base_footprint'; tf.transform.translation.x = 1.0
            tf.transform.translation.y = 2.0
            tf.transform.rotation.z = math.sin(math.pi/4); tf.transform.rotation.w = math.cos(math.pi/4)
            # Sensor stamp trails TF by 20 ms, allowing exact-time interpolation.
            dynamic.sendTransform(tf)
            if stamp.nanosec >= 20000000:
                stamp.nanosec -= 20000000
            else:
                stamp.sec -= 1; stamp.nanosec += 980000000
            scan = LaserScan(); scan.header.stamp = stamp; scan.header.frame_id = 'laser'
            scan.angle_increment = 0.01; scan.range_min = 0.15; scan.range_max = 8.0
            scan.ranges = [1.0, math.inf, math.nan]
            scan_pub.publish(scan)
            goal = PoseStamped(); goal.header.frame_id = 'laser'
            goal.pose.position.x = 1.0; goal.pose.orientation.w = 1.0
            goal_pub.publish(goal)
            rclpy.spin_once(node, timeout_sec=0.02)
        assert clouds and goals
        cloud = clouds[-1]
        points = list(read_points(cloud, field_names=('x', 'y', 'z')))
        assert cloud.header.frame_id == 'odom_combined' and len(points) == 1
        assert float(points[0][0]) == pytest.approx(1.0, abs=1e-4)
        assert float(points[0][1]) == pytest.approx(3.0, abs=1e-4)
        assert goals[-1].header.frame_id == 'odom_combined'
        assert goals[-1].pose.position.y == pytest.approx(3.0, abs=1e-4)
    finally:
        finish(process)


def test_planner_projects_tall_obstacle_and_rotates_body_velocity(node):
    from ego_planner.msg import Bspline
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_prefix('ego_planner'))/'lib/ego_planner/ego_planner_node'
    config = Path(get_package_share_directory('ego_wheeltec'))/'config/ego_v550.yaml'
    process = subprocess.Popen([
        str(path), '--ros-args', '--params-file', str(config),
        '-r', '/grid_map/cloud:=/ego/obstacles', '-r', '/benchmark/N:=/ego/obstacles',
        '-r', '/grid_map/odom:=/odom_combined', '-r', '/odom_map:=/odom_combined',
        '-r', '/way_point:=/ego/goal', '-r', '/cmd_vel:=/ego/test_planner_cmd',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    maps, trajectories, global_paths = [], [], []
    subscriptions = [
        node.create_subscription(OccupancyGrid, '/map_2d', lambda m: maps.append(m), 10),
        node.create_subscription(Bspline, '/planning/bspline', lambda m: trajectories.append(m), 10),
        node.create_subscription(Marker, '/global_list', lambda m: global_paths.append(m), 10),
    ]
    odom_pub = node.create_publisher(Odometry, '/odom_combined', 5)
    cloud_pub = node.create_publisher(PointCloud2, '/ego/obstacles', 5)
    goal_pub = node.create_publisher(PoseStamped, '/ego/goal', 5)

    def pump(duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            h = Header(); h.stamp = node.get_clock().now().to_msg(); h.frame_id = 'odom_combined'
            odom = Odometry(); odom.header = h; odom.child_frame_id = 'base_footprint'
            odom.pose.pose.orientation.z = math.sin(math.pi/4)
            odom.pose.pose.orientation.w = math.cos(math.pi/4)
            odom.twist.twist.linear.x = 0.1  # Forward in the body frame -> +Y in odometry.
            odom_pub.publish(odom)
            cloud_pub.publish(create_cloud_xyz32(h, [(1.5, 0.0, 1.0)]))
            rclpy.spin_once(node, timeout_sec=0.03)

    try:
        pump(2.5)
        assert maps
        m = maps[-1]
        assert m.header.frame_id == 'odom_combined' and m.info.origin.orientation.w == 1.0
        x = int((1.5-m.info.origin.position.x)/m.info.resolution)
        y = int((0.0-m.info.origin.position.y)/m.info.resolution)
        assert m.data[y*m.info.width+x] == 100, 'One-metre-high obstacle must project into the 2D map'
        goal = PoseStamped(); goal.header.frame_id = 'odom_combined'
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.y = 1.5; goal.pose.orientation.w = 1.0
        goal_pub.publish(goal)
        pump(3.0)
        assert global_paths
        path = next(m for m in global_paths if len(m.points) >= 2)
        assert path.header.frame_id == 'odom_combined'
        # The global polynomial uses measured odometry velocity; samples are
        # 0.1 s apart. GEN_NEW_TRAJ deliberately starts the LOCAL spline at rest,
        # so its derivative cannot test measured-velocity frame conversion.
        vx = (path.points[1].x-path.points[0].x)/0.1
        vy = (path.points[1].y-path.points[0].y)/0.1
        assert abs(vx) < 0.002 and vy == pytest.approx(0.1, abs=0.002), (vx, vy)
        assert trajectories, 'Aligned body heading and world start velocity should produce a trajectory'
    finally:
        finish(process)


def test_3d_ground_segmentation_and_obstacle_transform(node):
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_prefix('patchwork'))/'lib/patchwork/ground_segmentation'
    config = Path(get_package_share_directory('ego_wheeltec'))/'config/ground_segmentation.yaml'
    segmentation = subprocess.Popen([
        str(path), '--ros-args', '--params-file', str(config),
        '-p', 'sensor_frame:=laser', '-p', 'output_frame:=laser', '-p', 'sensor_height:=0.45',
        '-p', 'allow_latest_tf:=false', '-r', '/pointcloud_scan:=/ego/test_points',
        '-r', '/benchmark/P:=/ego/ground', '-r', '/benchmark/N:=/ego/non_ground',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    bridge = launch('sensor_bridge', '--ros-args', '-p', 'sensor_mode:=cloud',
                    '-p', 'input_topic:=/ego/non_ground', '-p', 'max_height:=0.9')
    ground, obstacles = [], []
    subscriptions = [
        node.create_subscription(PointCloud2, '/ego/ground', lambda m: ground.append(m), 10),
        node.create_subscription(PointCloud2, '/ego/obstacles', lambda m: obstacles.append(m), 10),
    ]
    publisher = node.create_publisher(PointCloud2, '/ego/test_points', qos_profile_sensor_data)
    static = StaticTransformBroadcaster(node)
    dynamic = TransformBroadcaster(node)
    mount = TransformStamped(); mount.header.frame_id = 'base_footprint'
    mount.child_frame_id = 'laser'; mount.header.stamp = node.get_clock().now().to_msg()
    mount.transform.translation.z = 0.45; mount.transform.rotation.w = 1.0
    static.sendTransform(mount)
    floor = [(x/10, y/10, -0.45) for x in range(-40, 41) for y in range(-40, 41)]
    box = [(x/10, y/10, z/10) for x in range(18, 23) for y in range(-2, 3) for z in range(7)]
    ceiling = [(x/10, y/10, 2.05) for x in range(-40, 41) for y in range(-40, 41)]
    header = Header(); header.frame_id = 'laser'
    cloud = create_cloud_xyz32(header, floor+box+ceiling)
    try:
        deadline = time.monotonic()+4.0
        while time.monotonic() < deadline:
            stamp = node.get_clock().now().to_msg()
            tf = TransformStamped(); tf.header.stamp = stamp; tf.header.frame_id = 'odom_combined'
            tf.child_frame_id = 'base_footprint'; tf.transform.translation.x = 1.0
            tf.transform.translation.y = 2.0
            tf.transform.rotation.z = math.sin(math.pi/4); tf.transform.rotation.w = math.cos(math.pi/4)
            dynamic.sendTransform(tf)
            cloud.header.stamp = stamp
            publisher.publish(cloud)
            rclpy.spin_once(node, timeout_sec=0.03)
            time.sleep(0.02)
        assert ground and obstacles
        ground_points = list(read_points(ground[-1], field_names=('x', 'y', 'z')))
        assert len(ground_points) > 100, 'The synthetic flat floor must be classified as ground'
        assert all(abs(float(p[2])+0.45) < 0.02 for p in ground_points)
        assert obstacles[-1].header.frame_id == 'odom_combined'
        points = list(read_points(obstacles[-1], field_names=('x', 'y', 'z')))
        assert points, 'The elevated box must survive ground removal'
        assert all(0.75 <= float(p[0]) <= 1.25 and 3.75 <= float(p[1]) <= 4.25 and
                   0.44 <= float(p[2]) <= 0.91 for p in points), points[:5]
    finally:
        finish(bridge)
        finish(segmentation)


def test_mpc_yields_to_fsm_rotation_and_keeps_emergency_stop(node):
    from ego_planner.msg import Bspline
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_prefix('ego_planner'))/'lib/ego_planner/traj_server'
    config = Path(get_package_share_directory('ego_wheeltec'))/'config/ego_v550.yaml'
    process = subprocess.Popen([
        str(path), '--ros-args', '--params-file', str(config),
        '-r', '/state_estimation:=/ego/test_mpc_odom', '-r', '/cmd_vel:=/ego/test_mpc_cmd',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    commands = []
    subscription = node.create_subscription(Twist, '/ego/test_mpc_cmd', lambda m: commands.append(m), 10)
    odom_pub = node.create_publisher(Odometry, '/ego/test_mpc_odom', 5)
    traj_pub = node.create_publisher(Bspline, '/planning/bspline', 5)
    adjust_pub = node.create_publisher(UInt8, '/is_adjust_yaw', 5)
    stop_pub = node.create_publisher(UInt8, '/emergency_stop', 5)
    trajectory = Bspline(); trajectory.order = 3
    trajectory.knots = [(i-3)*0.5 for i in range(10)]
    trajectory.pos_pts = [Point(x=(i-1)*0.04) for i in range(6)]

    def pump(duration, adjust=0, stop=0):
        deadline = time.monotonic()+duration
        next_publish = 0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                stamp = node.get_clock().now().to_msg()
                odom = Odometry(); odom.header.stamp = stamp
                odom.header.frame_id = 'odom_combined'; odom.child_frame_id = 'base_footprint'
                odom.pose.pose.orientation.w = 1.0; odom_pub.publish(odom)
                trajectory.start_time = stamp; traj_pub.publish(trajectory)
                adjust_pub.publish(UInt8(data=adjust)); stop_pub.publish(UInt8(data=stop))
                next_publish = time.monotonic()+0.1
            rclpy.spin_once(node, timeout_sec=0.01)

    try:
        pump(2.5)
        assert commands and any(c.linear.x > 0.001 for c in commands)
        assert all(math.isfinite(c.linear.x) and abs(c.linear.x) <= 0.151 and
                   math.isfinite(c.angular.z) and abs(c.angular.z) <= 0.401 for c in commands)
        pump(0.3, adjust=1); commands.clear(); pump(0.3, adjust=1)
        assert not commands, 'MPC must yield the velocity topic to the FSM during rotation'
        pump(0.4)
        assert commands and any(c.linear.x > 0.001 for c in commands), 'MPC must resume afterwards'
        pump(0.3, adjust=1, stop=1); commands.clear(); pump(0.3, adjust=1, stop=1)
        assert commands and all(c.linear.x == 0 and c.angular.z == 0 for c in commands)
    finally:
        finish(process)
