#!/usr/bin/env python3
"""Route a map-frame RViz goal to observed local EGO targets while SLAM grows."""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from navigation_grid import NavigationGrid


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def transform_xy(point, tf):
    angle = yaw(tf.transform.rotation)
    c, s = math.cos(angle), math.sin(angle)
    t = tf.transform.translation
    return (t.x + c * point[0] - s * point[1], t.y + s * point[0] + c * point[1])


class GoalRouter(Node):
    def __init__(self):
        super().__init__('ego_goal_router')
        self.frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom_combined').value
        self.radius = self.declare_parameter('robot_radius', 0.30).value
        self.margin = self.declare_parameter('collision_margin', 0.10).value
        self.tracking_margin = self.declare_parameter('tracking_margin', 0.10).value
        self.lookahead = self.declare_parameter('lookahead', 1.2).value
        self.map_timeout = self.declare_parameter('map_timeout', 5.0).value
        self.local_map_half_size = self.declare_parameter('local_map_half_size', 40.0).value
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.radius, self.lookahead, self.map_timeout, self.local_map_half_size)) or self.margin < 0:
            raise ValueError('Invalid goal routing distances/timeouts')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=True)
        self.grid, self.goal, self.odom, self.target, self.clear = None, None, None, None, None
        self.map_received = self.odom_received = self.last_plan = self.sent = 0.0
        self.command = Twist()
        self.command_received = 0.0
        self.scan = None
        self.local_map = None
        self.local_map_received = 0.0
        self.changed = True
        self.observe_direction = None
        self.sent_local = None
        self.status = 'waiting for SLAM map and RViz goal'
        self.ready_pub = self.create_publisher(Bool, '/ego/navigation_ready', 1)
        self.state_pub = self.create_publisher(String, '/ego/navigation_state', 1)
        self.target_pub = self.create_publisher(PoseStamped, '/ego/local_goal', 5)
        self.turn_pub = self.create_publisher(Twist, '/ego/exploration_turn', 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/ego/global_path', latched)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, latched)
        self.create_subscription(OccupancyGrid, '/map_2d', self.local_map_callback, 1)
        self.create_subscription(Odometry, '/odom_combined', self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 5)
        self.create_subscription(Twist, '/ego/cmd_vel_raw', self.command_callback, 1)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_service(Trigger, '/ego/cancel_goal', self.cancel)
        self.create_timer(0.1, self.tick)

    def map_callback(self, msg):
        width, height, resolution = msg.info.width, msg.info.height, msg.info.resolution
        if msg.header.frame_id != self.frame or not width or not height or resolution <= 0:
            return
        if len(msg.data) != width * height or width * height > 4000000:
            return
        p = msg.info.origin
        self.grid = NavigationGrid(np.asarray(msg.data, dtype=np.int8).reshape(height, width),
                                   resolution, (p.position.x, p.position.y, yaw(p.orientation)),
                                   self.radius + self.margin + self.tracking_margin,
                                   observed_radius=self.radius + self.margin)
        self.map_received, self.changed = time.monotonic(), True

    def odom_callback(self, msg):
        if msg.header.frame_id != self.odom_frame:
            return
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not -0.05 <= age <= 0.7 or not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            return
        self.odom, self.odom_received = msg, time.monotonic()

    def local_map_callback(self, msg):
        if (msg.header.frame_id == self.odom_frame and msg.info.resolution > 0 and
                msg.info.width and msg.info.height and
                len(msg.data) == msg.info.width * msg.info.height):
            self.local_map, self.local_map_received = msg, time.monotonic()

    def goal_callback(self, msg):
        if not msg.header.frame_id or not all(math.isfinite(v) for v in
                                             (msg.pose.position.x, msg.pose.position.y)):
            return
        try:
            point = msg.pose.position.x, msg.pose.position.y
            if msg.header.frame_id != self.frame:
                point = transform_xy(point, self.buffer.lookup_transform(self.frame, msg.header.frame_id, Time()))
            self.goal, self.target, self.changed = point, None, True
            self.ready_pub.publish(Bool(data=False))
            self.last_plan = 0.0
        except TransformException as error:
            self.get_logger().warning(f'Goal TF unavailable: {error}')

    def command_callback(self, msg):
        if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
            self.command, self.command_received = msg, time.monotonic()

    def scan_callback(self, msg):
        if msg.header.frame_id and math.isfinite(msg.angle_min) and math.isfinite(msg.angle_increment):
            self.scan = msg

    def cancel(self, request, response):
        self.goal, self.target = None, None
        self.ready_pub.publish(Bool(data=False))
        self.status = 'goal cancelled'
        response.success, response.message = True, self.status
        return response

    def tick(self):
        ready = False
        turn_command = Twist()
        now = time.monotonic()
        try:
            if self.grid is None or now - self.map_received > self.map_timeout:
                self.status = 'waiting for fresh SLAM map'
            elif self.odom is None or now - self.odom_received > 0.7:
                self.status = 'waiting for fresh odometry'
            elif self.local_map is None or now - self.local_map_received > 0.7:
                self.status = 'waiting for EGO local occupancy map'
            elif self.goal is None:
                self.status = 'waiting for RViz goal'
            else:
                map_odom = self.buffer.lookup_transform(self.frame, self.odom_frame, Time())
                odom_map = self.buffer.lookup_transform(self.odom_frame, self.frame, Time())
                p = self.odom.pose.pose.position
                position = transform_xy((p.x, p.y), map_odom)
                heading = yaw(self.odom.pose.pose.orientation) + yaw(map_odom.transform.rotation)
                local_map = self.local_map
                lp = local_map.info.origin
                t = odom_map.transform.translation
                self.grid.overlay_local_obstacles(
                    np.asarray(local_map.data).reshape(local_map.info.height, local_map.info.width),
                    local_map.info.resolution, (lp.position.x, lp.position.y, yaw(lp.orientation)),
                    (t.x, t.y, yaw(odom_map.transform.rotation)))
                if self.scan is not None:
                    age = (self.get_clock().now() - Time.from_msg(self.scan.header.stamp)).nanoseconds / 1e9
                    if -0.05 <= age <= 0.7:
                        try:
                            tf = self.buffer.lookup_transform(self.frame, self.scan.header.frame_id,
                                                              Time.from_msg(self.scan.header.stamp))
                            t = tf.transform.translation
                            self.grid.observe_scan((t.x, t.y, yaw(tf.transform.rotation)),
                                self.scan.angle_min, self.scan.angle_increment, self.scan.ranges,
                                self.scan.range_min, self.scan.range_max)
                        except TransformException:
                            pass  # Keep SLAM observations; never substitute a newer sensor TF.
                if max(abs(p.x), abs(p.y)) > self.local_map_half_size - 2.0:
                    self.status = 'EGO local map boundary reached; increase map_size_x/y before continuing'
                elif math.dist(position, self.goal) <= 0.25:
                    self.goal, self.target = None, None
                    self.status = 'goal reached (position tolerance 0.25 m)'
                else:
                    reached = self.target is not None and math.dist(position, self.target) < 0.35
                    if now - self.last_plan > 1.0 and (self.changed or reached or self.target is None):
                        path, target, self.status = self.grid.route(position, self.goal, self.lookahead)
                        clear = self.grid.observed_clearance(position)
                        # A map refresh must not continuously move an already
                        # reachable waypoint and restart EGO's yaw adjustment.
                        if (self.target is not None and math.dist(position, self.target) > 0.25 and
                                (not reached or target is None) and
                                self.grid.segment_known(position, self.target, clear)):
                            target = self.target
                        self.observe_direction = path[min(8, len(path) - 1)] if path else None
                        self.last_plan, self.changed = now, False
                        message = Path()
                        message.header.frame_id, message.header.stamp = self.frame, self.get_clock().now().to_msg()
                        for x, y in path:
                            pose = PoseStamped()
                            pose.header = message.header
                            pose.pose.position.x, pose.pose.position.y = x, y
                            pose.pose.orientation.w = 1.0
                            message.poses.append(pose)
                        self.path_pub.publish(message)
                        local = transform_xy(target, odom_map) if target is not None else None
                        if target is not None and (self.target is None or reached or
                                self.sent_local is None or math.dist(self.sent_local, local) > 0.10):
                            message = PoseStamped()
                            message.header.frame_id, message.header.stamp = self.odom_frame, self.get_clock().now().to_msg()
                            message.pose.position.x, message.pose.position.y = local
                            message.pose.orientation.w = 1.0
                            self.target_pub.publish(message)
                            self.sent, self.sent_local = now, local
                        self.target = target
                    self.clear = self.grid.observed_clearance(position)
                    if self.target is not None and now - self.sent > 0.2:
                        speed = self.command.linear.x if now - self.command_received < 0.7 else 0.0
                        turn = self.command.angular.z if now - self.command_received < 0.7 else 0.0
                        ready = self.grid.motion_known(position, heading, speed, turn, self.clear)
                        ready = ready and self.grid.motion_known(position, heading,
                            self.odom.twist.twist.linear.x, self.odom.twist.twist.angular.z, self.clear)
                        if not ready:
                            self.status = 'stopping before unobserved/occupied space'
                    elif self.target is None and self.observe_direction is not None:
                        direction = math.atan2(self.observe_direction[1] - position[1],
                                               self.observe_direction[0] - position[0])
                        error = math.remainder(direction - heading, 2 * math.pi)
                        if abs(error) > 0.20:
                            turn_command.angular.z = max(-0.3, min(0.3, error))
                            self.status = 'rotating in place to observe the route direction'
        except (TransformException, ValueError, IndexError) as error:
            self.status = f'waiting for valid map/TF: {error}'
        self.ready_pub.publish(Bool(data=bool(ready)))
        self.turn_pub.publish(turn_command)
        self.state_pub.publish(String(data=self.status))


def main():
    rclpy.init()
    node = GoalRouter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.ready_pub.publish(Bool(data=False))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
