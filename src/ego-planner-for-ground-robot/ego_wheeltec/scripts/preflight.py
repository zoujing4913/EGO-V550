#!/usr/bin/env python3
"""Read-only odometry/sensor/TF check. Never publishes a motor command."""
import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--odom-topic', default='/odom_combined')
    parser.add_argument('--sensor-topic', default='/scan')
    parser.add_argument('--sensor-type', choices=['scan', 'cloud'], default='scan')
    parser.add_argument('--planning-frame', default='odom_combined')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--timeout', type=float, default=8.0)
    parser.add_argument('--min-rate', type=float, default=5.0)
    args = parser.parse_args()
    if args.timeout <= 0 or args.min_rate <= 0:
        parser.error('timeout/min-rate must be positive')
    rclpy.init()
    node = rclpy.create_node('ego_v550_preflight')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    seen = {'odom': [], 'sensor': []}
    latest = {}

    def receive(key, msg):
        latest[key] = msg
        seen[key].append(time.monotonic())

    subscriptions = [
        node.create_subscription(Odometry, args.odom_topic,
                                 lambda m: receive('odom', m), qos_profile_sensor_data),
        node.create_subscription(LaserScan if args.sensor_type == 'scan' else PointCloud2,
                                 args.sensor_topic, lambda m: receive('sensor', m),
                                 qos_profile_sensor_data),
    ]
    stop = time.monotonic() + args.timeout
    while time.monotonic() < stop:
        rclpy.spin_once(node, timeout_sec=0.05)
    errors = []
    for key in seen:
        ts = seen[key]
        rate = (len(ts)-1)/(ts[-1]-ts[0]) if len(ts) > 1 and ts[-1] > ts[0] else 0.0
        print(f'{key}: {len(ts)} messages, {rate:.1f} Hz')
        if rate < args.min_rate:
            errors.append(f'{key}: missing or below {args.min_rate} Hz')
        if key in latest:
            age = (node.get_clock().now()-Time.from_msg(latest[key].header.stamp)).nanoseconds/1e9
            print(f'  frame={latest[key].header.frame_id}, stamp age={age:.3f}s')
            if not -0.05 <= age <= 0.7:
                errors.append(f'{key}: stale/future timestamp; check clocks/use_sim_time')
    if 'odom' in latest:
        m = latest['odom']
        if m.header.frame_id != args.planning_frame or m.child_frame_id != args.base_frame:
            errors.append(f'odom frames must be {args.planning_frame} -> {args.base_frame}, '
                          f'got {m.header.frame_id} -> {m.child_frame_id}')
        p, q = m.pose.pose.position, m.pose.pose.orientation
        vals = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in vals) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > 0.05:
            errors.append('invalid odometry position/quaternion')
    if 'sensor' in latest:
        m = latest['sensor']
        if args.sensor_type == 'scan':
            if not any(math.isfinite(v) and m.range_min <= v <= m.range_max for v in m.ranges):
                errors.append('scan has no valid return')
        elif m.width * m.height == 0:
            errors.append('cloud is empty')
        for target in (args.base_frame, args.planning_frame):
            try:
                buffer.lookup_transform(target, m.header.frame_id, Time.from_msg(m.header.stamp))
                print(f'TF {target} <- {m.header.frame_id}: OK at measurement time')
            except Exception as exc:
                errors.append(f'TF {target} <- {m.header.frame_id}: {exc}')
    publishers = node.get_publishers_info_by_topic('/cmd_vel')
    print('/cmd_vel publishers:', [p.node_name for p in publishers])
    if publishers:
        print('Before drive=true: stop other navigation/teleop command sources; retain only ego_cmd_vel_guard.')
    for error in errors:
        print('FAIL:', error)
    print('PASS: input interfaces ready (not a driving test)' if not errors else 'NOT READY')
    node.destroy_node()
    rclpy.shutdown()
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
