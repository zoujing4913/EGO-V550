#!/usr/bin/env python3
"""Send a nearby simulation goal and record the V550 + 2D LiDAR closed loop.

Use only with sim_v550_mec_2d.launch.py, in the same isolated ROS domain.
The script publishes a goal; it never publishes chassis velocity commands.
"""
import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from ego_planner.msg import Bspline
from visualization_msgs.msg import Marker


def world_clearances(world_path, positions, radius):
    """Ground truth planar clearance against static primitive collision geometry.

    This test uses the robot's circumscribed circle, not the planner's own map.
    Floor and door lintels outside the chassis height are excluded.
    """
    shapes = []
    def pose(node):
        return [float(v) for v in node.findtext('pose', '0 0 0 0 0 0').split()]
    def compose(a, b):
        c, s = math.cos(a[5]), math.sin(a[5])
        return [a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], a[2]+b[2],
                0, 0, a[5]+b[5]]
    for model in ET.parse(world_path).getroot().find('world').findall('model'):
        if model.findtext('static') not in ('true', '1'): continue
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                p = compose(compose(pose(model), pose(link)), pose(collision))
                geometry = collision.find('geometry')
                box, cylinder = geometry.find('box'), geometry.find('cylinder')
                if box is not None:
                    sx, sy, height = map(float, box.findtext('size').split())
                    shape = ('box', p, sx/2, sy/2)
                elif cylinder is not None:
                    height = float(cylinder.findtext('length'))
                    shape = ('circle', p, float(cylinder.findtext('radius')), 0)
                else:
                    raise ValueError('Ground truth check supports boxes and vertical cylinders only')
                if p[2]+height/2 > 0.025 and p[2]-height/2 < 0.22:
                    shapes.append(shape)
    if not shapes: raise ValueError('No static obstacle geometry to check')
    distances = []
    for x, y in positions:
        nearest = math.inf
        for kind, p, a, b in shapes:
            dx, dy = x-p[0], y-p[1]
            if kind == 'circle':
                distance = math.hypot(dx, dy)-a
            else:
                c, s = math.cos(p[5]), math.sin(p[5])
                qx, qy = abs(c*dx+s*dy)-a, abs(-s*dx+c*dy)-b
                distance = math.hypot(max(qx, 0), max(qy, 0))+min(max(qx, qy), 0)
            nearest = min(nearest, distance-radius)
        distances.append(nearest)
    return min(distances), sum(d < -0.005 for d in distances)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal-x', type=float, default=-31.3)
    parser.add_argument('--goal-y', type=float, default=-25.4)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--report', default='/tmp/v550_sim_check.json')
    parser.add_argument('--trace', help='Optional JSON with actual positions and planned curves')
    parser.add_argument('--world', help='Check actual path against static boxes/cylinders in this world')
    parser.add_argument('--robot-radius', type=float, default=0.18)
    parser.add_argument('--expect-forward-only', action='store_true')
    parser.add_argument('--require-escape', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('v550_sim_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
    data = {'odom': None, 'scan': None, 'map': None, 'cmd': None, 'description': '', 'planned': []}
    path, trajectories = [], []
    tracking_errors = []
    planned_history = []
    commands, body_speeds, headings, escape_markers = [], [], [], []
    last_plan_stamp = None

    def planned(msg):
        nonlocal last_plan_stamp
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        data['planned'] = points
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if points and stamp != last_plan_stamp:
            planned_history.append(points)
            last_plan_stamp = stamp

    def stopped():
        command, odom = data['cmd'], data['odom']
        return (command is not None and odom is not None and
                abs(command.linear.x) < 0.001 and abs(command.angular.z) < 0.001 and
                math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y) < 0.02 and
                abs(odom.twist.twist.angular.z) < 0.02)

    def odometry(msg):
        data['odom'] = msg
        p = msg.pose.pose.position
        path.append((p.x, p.y))
        body_speeds.append(msg.twist.twist.linear.x)
        q = msg.pose.pose.orientation
        headings.append(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        if data['planned']:
            tracking_errors.append(min(math.hypot(p.x-x, p.y-y) for x, y in data['planned']))

    def command(msg):
        data['cmd'] = msg
        commands.append(msg.linear.x)

    subscriptions = [
        node.create_subscription(Odometry, '/odom_combined', odometry, 10),
        node.create_subscription(LaserScan, '/scan', lambda m: data.update(scan=m), qos_profile_sensor_data),
        node.create_subscription(OccupancyGrid, '/map_2d', lambda m: data.update(map=m), 10),
        node.create_subscription(Twist, '/v550_sim/cmd_vel', command, 10),
        node.create_subscription(Marker, '/planning/escape_direction',
                                 lambda m: escape_markers.append(m) if m.action == Marker.ADD else None, 10),
        node.create_subscription(Bspline, '/planning/bspline', lambda m: trajectories.append(m), 10),
        node.create_subscription(String, '/robot_description', lambda m: data.update(description=m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(RosPath, '/planning/bspline_path', planned,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]
    publisher = node.create_publisher(PoseStamped, '/goal_pose', 5)
    deadline = time.monotonic()+45
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if all(data[k] is not None for k in ('odom', 'scan', 'map')) and data['description'] and publisher.get_subscription_count():
            break
    if not all(data[k] is not None for k in ('odom', 'scan', 'map')) or not data['description']:
        missing = [k for k in ('odom', 'scan', 'map') if data[k] is None]
        if not data['description']: missing.append('description')
        print(f'FAIL: simulation inputs/model not ready; missing={missing}', flush=True)
        node.destroy_node(); rclpy.shutdown()
        return 1
    if 'meshes/v550_mec/' not in data['description']:
        raise RuntimeError('Expected the bundled manufacturer V550 model')
    start = path[-1]
    goal = PoseStamped(); goal.header.frame_id = 'odom_combined'
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x = args.goal_x; goal.pose.position.y = args.goal_y
    goal.pose.orientation.w = 1.0
    publisher.publish(goal)
    tracking_errors.clear()
    data['planned'] = []
    print(f'Simulation goal sent: ({args.goal_x}, {args.goal_y}), start={start}', flush=True)
    deadline = time.monotonic()+args.timeout
    reached_since = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.03)
        error = math.hypot(path[-1][0]-args.goal_x, path[-1][1]-args.goal_y)
        if error < 0.25 and stopped():
            reached_since = reached_since or time.monotonic()
            if time.monotonic()-reached_since > 2.0:
                break
        else:
            reached_since = None
    end = path[-1]
    scan, grid, odom = data['scan'], data['map'], data['odom']
    error = math.hypot(end[0]-args.goal_x, end[1]-args.goal_y)
    report = {
        'model': 'manufacturer V550_mec STL + ideal Gazebo planar motion',
        'start_xy': start, 'final_xy': end, 'goal_xy': [args.goal_x, args.goal_y],
        'goal_error_m': error,
        'travelled_m': sum(math.dist(a, b) for a, b in zip(path, path[1:])),
        'bspline_count': len(trajectories),
        'cross_track_rmse_m': math.sqrt(sum(e*e for e in tracking_errors)/len(tracking_errors)) if tracking_errors else None,
        'cross_track_max_m': max(tracking_errors) if tracking_errors else None,
        'actual_y_range': [min(p[1] for p in path), max(p[1] for p in path)],
        'scan_samples': len(scan.ranges),
        'scan_valid_returns': sum(math.isfinite(r) and scan.range_min <= r <= scan.range_max for r in scan.ranges),
        'scan_frame': scan.header.frame_id, 'odom_frame': odom.header.frame_id,
        'base_frame': odom.child_frame_id, 'map_frame': grid.header.frame_id,
        'map_occupied_cells': sum(v == 100 for v in grid.data),
        'last_command': [data['cmd'].linear.x, data['cmd'].linear.y, data['cmd'].angular.z] if data['cmd'] else None,
        'reached': error < 0.25 and bool(trajectories) and stopped(),
        'minimum_forward_command_mps': min(commands) if commands else None,
        'negative_command_count': sum(v < -1e-6 for v in commands),
        'reverse_motion_samples': sum(v < -0.005 for v in body_speeds),
        'escape_marker_samples': len(escape_markers),
    }
    if args.world:
        clearance, collisions = world_clearances(args.world, path, args.robot_radius)
        report.update(minimum_body_clearance_m=clearance, body_overlap_samples=collisions)
    target = Path(args.report); target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    if args.trace:
        trace = Path(args.trace); trace.parent.mkdir(parents=True, exist_ok=True)
        trace.write_text(json.dumps({'actual': path, 'planned': planned_history,
                                    'headings': headings, 'forward_commands': commands})+'\n')
    print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
    node.destroy_node(); rclpy.shutdown()
    passed = report['reached'] and report.get('body_overlap_samples', 0) == 0
    if args.expect_forward_only:
        passed = passed and report['negative_command_count'] == 0 and report['reverse_motion_samples'] == 0
    if args.require_escape: passed = passed and report['escape_marker_samples'] > 0
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
