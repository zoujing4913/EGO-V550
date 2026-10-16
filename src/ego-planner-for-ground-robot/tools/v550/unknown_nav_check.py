#!/usr/bin/env python3
"""Software-only closed loop: synthetic 2D rays/odometry, actual SLAM/EGO/guard.

No chassis driver is started; output is /ego/cmd_vel_checked in an isolated domain.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '225'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    log_path = Path('/tmp/v550_unknown_nav_check.log')
    with log_path.open('w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'ego_wheeltec', 'v550_unknown_nav.launch.py',
            'start_base:=false', 'start_lidar:=false', 'rviz:=false', 'drive:=false'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = rclpy.create_node('synthetic_unknown_nav_check')
        scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        odom_pub = node.create_publisher(Odometry, '/odom_combined', 10)
        goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 5)
        transforms, static = TransformBroadcaster(node), StaticTransformBroadcaster(node)
        mount = TransformStamped()
        mount.header.frame_id, mount.child_frame_id = 'base_footprint', 'laser'
        mount.transform.translation.z, mount.transform.rotation.w = 0.2, 1.0
        mount.header.stamp = node.get_clock().now().to_msg()
        static.sendTransform(mount)
        command, maps, planner_maps, states = [], [], [], []
        node.create_subscription(Twist, '/ego/cmd_vel_checked', lambda m: command.append((time.monotonic(), m)), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, '/map', lambda m: maps.append(m), latched)
        node.create_subscription(OccupancyGrid, '/map_2d', lambda m: planner_maps.append(m), 1)
        node.create_subscription(String, '/ego/navigation_state', lambda m: states.append(m.data), 1)
        client = node.create_client(SetBool, '/ego/set_enabled')
        x = y = heading = max_speed = 0.0
        enabled, sent, future, unknown_goal = False, False, None, False
        began = previous = last_report = time.monotonic()
        goal_time = 0.0
        report = {}
        try:
            while time.monotonic() - began < 160:
                rclpy.spin_once(node, timeout_sec=0.005)
                now = time.monotonic()
                dt, previous = min(0.05, now - previous), now
                v = w = 0.0
                if command and now - command[-1][0] < 0.3:
                    v, w = command[-1][1].linear.x, command[-1][1].angular.z
                assert v >= -1e-6, 'Reverse command in forward-only profile'
                max_speed = max(max_speed, abs(v))
                x += v * math.cos(heading) * dt
                y += v * math.sin(heading) * dt
                heading += w * dt
                assert min(x + 2, 5 - x, 2.0 - abs(y)) > 0.30, 'Synthetic body touched a wall'
                if abs(x - 1.5) < 0.30:
                    assert 0.5 < y < 1.5, 'Synthetic body touched the doorway wall'
                stamp = node.get_clock().now().to_msg()
                odom = Odometry()
                odom.header.frame_id, odom.child_frame_id, odom.header.stamp = 'odom_combined', 'base_footprint', stamp
                odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
                odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
                odom_pub.publish(odom)
                tf = TransformStamped()
                tf.header, tf.child_frame_id = odom.header, 'base_footprint'
                tf.transform.translation.x, tf.transform.translation.y = x, y
                tf.transform.rotation = odom.pose.pose.orientation
                transforms.sendTransform(tf)
                angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
                dx, dy = np.cos(angles + heading), np.sin(angles + heading)
                candidates = []
                for delta, direction in ((5-x, dx), (-2-x, dx), (2.0-y, dy), (-2.0-y, dy)):
                    distances = np.divide(delta, direction, out=np.full_like(direction, np.inf), where=np.abs(direction) > 1e-6)
                    candidates.append(np.where(distances > 0, distances, np.inf))
                wall = np.divide(1.5-x, dx, out=np.full_like(dx, np.inf), where=np.abs(dx) > 1e-6)
                crossing_y = y + wall * dy
                candidates.append(np.where((wall > 0) & ((crossing_y < 0.2) | (crossing_y > 1.8)), wall, np.inf))
                ranges = np.minimum.reduce(candidates)
                scan = LaserScan()
                scan.header.frame_id, scan.header.stamp = 'laser', stamp
                scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, 2 * math.pi / 360
                scan.range_min, scan.range_max, scan.scan_time = 0.1, 8.0, 0.02
                scan.ranges = ranges.tolist()
                scan_pub.publish(scan)
                if not enabled and maps and planner_maps and client.service_is_ready():
                    if future is None:
                        request = SetBool.Request(); request.data = True
                        future = client.call_async(request)
                    elif future.done():
                        enabled = future.result().success
                        if not enabled:
                            future = None
                if enabled and not sent:
                    assert not node.get_publishers_info_by_topic('/cmd_vel'), 'Unexpected real chassis output'
                    grid = maps[-1]
                    gx = int((3.0 - grid.info.origin.position.x) / grid.info.resolution)
                    gy = int((0.0 - grid.info.origin.position.y) / grid.info.resolution)
                    unknown_goal = not (0 <= gx < grid.info.width and 0 <= gy < grid.info.height)
                    if not unknown_goal:
                        unknown_goal = grid.data[gy * grid.info.width + gx] < 0
                    assert unknown_goal, 'Test goal must start in UNKNOWN space'
                    goal = PoseStamped()
                    goal.header.frame_id, goal.header.stamp = 'map', stamp
                    goal.pose.position.x, goal.pose.orientation.w = 3.0, 1.0
                    goal_pub.publish(goal)
                    sent, goal_time = True, now
                if now - last_report > 10:
                    print(json.dumps({'elapsed':round(now-began, 1),'x':round(x, 3),'y':round(y, 3),
                                      'state':states[-1] if states else 'starting'}), flush=True)
                    last_report = now
                if sent and now - goal_time > 3 and math.hypot(x - 3, y) < 0.30:
                    break
                if process.poll() is not None:
                    raise RuntimeError(f'Launch exited; see {log_path}')
                time.sleep(0.015)
            report = {'goal_initially_unknown':unknown_goal,'final_xy':[x, y],
                      'position_error':math.hypot(x-3, y),'max_linear':max_speed,
                      'slam_updates':len(maps),'state':states[-1] if states else None,
                      'hardware_tested':False}
            print(json.dumps(report), flush=True)
            assert sent and report['position_error'] < 0.30, 'Unknown-goal closed loop did not reach target'
            assert max_speed <= 0.10001
        finally:
            Path('/tmp/v550_unknown_nav_check.json').write_text(json.dumps(report, indent=2)+'\n')
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
