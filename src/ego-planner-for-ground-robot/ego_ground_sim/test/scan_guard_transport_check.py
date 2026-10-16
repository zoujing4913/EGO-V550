#!/usr/bin/env python3
"""ROS transport check. Run with only cmd_vel_safety in a separate ROS domain.

Start the guard with use_scan_guard:=true and max_linear:=0.22. The synthetic
scan is already in odom_combined; real simulation additionally exercises TF.
"""
import json
import argparse
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, default=Path('/tmp/v550_scan_guard_check.json'))
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('scan_guard_transport_check')
    commands = node.create_publisher(Twist, '/cmd_vel', 10)
    odometry = node.create_publisher(Odometry, '/state_estimation', 10)
    scans = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    outputs = []
    subscription = node.create_subscription(Twist, '/cmd_vel_safe',
                                            lambda msg: outputs.append(msg.linear.x), 10)
    deadline = time.monotonic()+10
    while not commands.get_subscription_count() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    results = {}
    for name, distance, scan_on, should_move in [
            ('clear', 0.8, True, True), ('obstacle', 0.4, True, False),
            ('stale_scan', 0.8, False, False), ('recovered', 0.8, True, True)]:
        outputs.clear()
        deadline = time.monotonic()+1.2
        while time.monotonic() < deadline:
            stamp = node.get_clock().now().to_msg()
            command = Twist(); command.linear.x = 0.22; commands.publish(command)
            odom = Odometry(); odom.header.frame_id = 'odom_combined'
            odom.header.stamp = stamp; odom.pose.pose.orientation.w = 1.0
            odometry.publish(odom)
            if scan_on:
                scan = LaserScan(); scan.header.frame_id = 'odom_combined'
                scan.header.stamp = stamp; scan.angle_min = -math.pi
                scan.angle_increment = 2*math.pi/720; scan.range_min = 0.15; scan.range_max = 12.0
                scan.ranges = [math.inf]*720; scan.ranges[360] = distance
                scans.publish(scan)
            rclpy.spin_once(node, timeout_sec=0.02)
        tail = outputs[-10:]
        passed = bool(tail) and (all(v > 0.2 for v in tail) if should_move
                                else all(abs(v) < 1e-6 for v in tail))
        results[name] = {'passed': passed, 'last_velocities': tail}
        assert passed, results
    # Leave this isolated test guard with a zero command.
    commands.publish(Twist())
    target = args.report
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps(results, indent=2))
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
