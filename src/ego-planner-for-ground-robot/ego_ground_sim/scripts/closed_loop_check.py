#!/usr/bin/env python3
"""Closed loop check for the ego_ground_sim simulation.

Run it while `ros2 launch ego_ground_sim sim_ego_planner.launch.py` is running:

    python3 scripts/closed_loop_check.py 6.0 4.5 [timeout_s]

It sends a goal, records pose/velocity/trajectory messages and prints a report:
travelled distance, max speed, number of B-spline trajectories, distance to the
known obstacles of worlds/obstacles.world, goal error and the stop behaviour.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from ego_planner.msg import Bspline


class Obstacle:
    """Axis aligned (optionally rotated) box/cylinder footprint of the world file."""

    def __init__(self, name, x, y, half_x, half_y, yaw=0.0):
        self.name, self.x, self.y = name, x, y
        self.hx, self.hy, self.yaw = half_x, half_y, yaw

    def distance(self, px, py):
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        dx, dy = px - self.x, py - self.y
        lx, ly = c * dx - s * dy, s * dx + c * dy
        ox = max(abs(lx) - self.hx, 0.0)
        oy = max(abs(ly) - self.hy, 0.0)
        outside = math.hypot(ox, oy)
        inside = min(max(abs(lx) - self.hx, abs(ly) - self.hy), 0.0)
        return outside + inside


OBSTACLES = [
    Obstacle('box_1', -1.0, -1.0, 0.6, 0.6),
    Obstacle('box_2', 0.8, 0.6, 0.5, 0.5, 0.3),
    Obstacle('box_3', -3.2, -3.0, 0.5, 0.5, 0.6),
    Obstacle('cylinder_1', 2.6, 2.2, 0.5, 0.5),
    Obstacle('box_4', 4.6, 4.0, 0.5, 0.5, -0.4),
]


class Recorder(Node):
    def __init__(self, goal):
        super().__init__('closed_loop_check')
        self.goal = goal
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        best_effort = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, '/state_estimation', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel_safe', self.on_cmd_safe, 10)
        self.create_subscription(Bspline, '/planning/bspline', self.on_bspline, 10)
        self.create_subscription(PointCloud2, '/benchmark/N', self.on_nonground, best_effort)
        self.create_subscription(OccupancyGrid, '/map_2d', self.on_map2d, 10)
        self.path = []
        self.bsplines = 0
        self.nonground_points = 0
        self.map2d_count = 0
        self.cmd_safe = None
        self.timeline = []
        self.last_sample = 0.0

    def on_odom(self, msg):
        self.path.append((msg.pose.pose.position.x, msg.pose.pose.position.y,
                          msg.twist.twist.linear.x))
        import time as _time
        now = _time.time()
        if now - self.last_sample > 2.0:
            self.last_sample = now
            cmd = self.cmd_safe
            self.timeline.append(
                (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x,
                 cmd.linear.x if cmd else float('nan'), cmd.angular.z if cmd else float('nan')))

    def on_cmd(self, _msg):
        pass

    def on_cmd_safe(self, msg):
        self.cmd_safe = msg

    def on_bspline(self, _msg):
        self.bsplines += 1

    def on_nonground(self, msg):
        self.nonground_points = msg.width * msg.height

    def on_map2d(self, _msg):
        self.map2d_count += 1

    def send_goal(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.goal[0]
        msg.pose.position.y = self.goal[1]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)


def main():
    goal = (float(sys.argv[1]), float(sys.argv[2])) if len(sys.argv) > 2 else (6.0, 4.5)
    timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 90.0
    rclpy.init()
    node = Recorder(goal)
    print(f'goal = {goal}', flush=True)

    start = time.time()
    while time.time() - start < 30.0 and len(node.path) < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(node.path) < 5:
        print('FAIL: no odometry received')
        return

    start = time.time()
    goal_sent = 0
    moving_since = None
    last_motion = None
    stuck_since = None
    while time.time() - start < timeout:
        if goal_sent < 5 and time.time() - start < 1.0:
            node.send_goal()
            goal_sent += 1
        rclpy.spin_once(node, timeout_sec=0.05)
        if len(node.path) > 1:
            speed = abs(node.path[-1][2])
            if speed > 0.05:
                moving_since = moving_since or time.time()
                last_motion = time.time()
                stuck_since = None
            elif moving_since and last_motion and time.time() - last_motion > 3.0:
                at_goal = math.hypot(node.path[-1][0] - goal[0], node.path[-1][1] - goal[1]) < 1.0
                if at_goal:
                    break
                # a pause can also be a replanning / heading-adjust phase, so only
                # give up when there is no motion at all for a longer time
                stuck_since = stuck_since or time.time()
                if time.time() - stuck_since > 30.0:
                    print('NOTE: no motion for 30 s -> stopped recording (robot is stuck or idle)')
                    break

    path = node.path
    dist = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
    goal_err = math.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1])
    straight = math.hypot(path[-1][0] - path[0][0], path[-1][1] - path[0][1])
    print('---------------- report ----------------')
    print(f'start -> final        : ({path[0][0]:.2f}, {path[0][1]:.2f}) -> '
          f'({path[-1][0]:.2f}, {path[-1][1]:.2f})')
    print(f'travelled / straight  : {dist:.2f} m / {straight:.2f} m (detour = avoidance)')
    print(f'max speed             : {max(abs(p[2]) for p in path):.2f} m/s')
    print(f'B-spline trajectories : {node.bsplines}')
    print(f'non-ground points     : {node.nonground_points}, /map_2d msgs: {node.map2d_count}')
    print(f'goal error            : {goal_err:.2f} m  -> {"REACHED" if goal_err < 1.0 else "NOT REACHED"}')
    hits = []
    for obs in OBSTACLES:
        d = min(obs.distance(p[0], p[1]) for p in path)
        print(f'min distance to {obs.name:11s}: {d:+.2f} m')
        if d < 0.0:
            hits.append(obs.name)
    print(f'obstacle hits         : {hits if hits else "none"}')
    if node.cmd_safe is not None:
        print(f'final /cmd_vel_safe   : v={node.cmd_safe.linear.x:+.3f} '
              f'w={node.cmd_safe.angular.z:+.3f}')
    print('timeline (every 2 s):      x,      y,   v_odom,   v_cmd,   w_cmd,  dist_to_goal')
    for x, y, v, vc, wc in node.timeline:
        print(f'                    {x:8.2f}, {y:8.2f}, {v:7.2f}, {vc:7.2f}, {wc:7.2f}, '
              f'{math.hypot(x - goal[0], y - goal[1]):7.2f}')
    print('----------------------------------------')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
