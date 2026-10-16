#!/usr/bin/env python3
"""Tells apart the three reasons why "the robot does not move in Gazebo".

Run it while the simulation is running (any launch of this package):

    source /opt/ros/humble/setup.bash && source <ws>/install/setup.bash
    python3 install/ego_ground_sim/share/ego_ground_sim/scripts/robot_motion_check.py

It prints, once per second
    * the pose in the Gazebo *server*   (`gz model -m ego_ground_robot -p`)
    * the wheel joint angle            (`gz model -m ego_ground_robot -i`)
    * the published odometry           (/state_estimation)
    * the velocity command             (/cmd_vel_safe)
and a verdict:
    * world pose changing                    -> the physics moves; if the Gazebo window
                                                looks static, it is the camera / a stale
                                                window (see the README FAQ)
    * world pose static, |v_cmd| > 0          -> the wheels are commanded but the body does
                                                not move: the robot is physically blocked
                                                (spawn point / step); try another spawn_x/y
    * world pose static, v_cmd == 0           -> the planner is pausing (replanning or it
                                                reached the (local) goal); nothing moves
"""

import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def gazebo_pose():
    try:
        out = subprocess.run(['gz', 'model', '-m', 'ego_ground_robot', '-p'],
                             capture_output=True, text=True, timeout=3).stdout.split()
        return float(out[0]), float(out[1])
    except Exception:
        return None


def wheel_angle():
    try:
        out = subprocess.run(['gz', 'model', '-m', 'ego_ground_robot', '-i'],
                             capture_output=True, text=True, timeout=3).stdout
        lines = out.splitlines()
        for i, line in enumerate(lines):
            if 'left_wheel_joint' in line:
                for follow in lines[i:i + 6]:
                    if 'angle:' in follow:
                        return float(follow.split()[-1])
        return None
    except Exception:
        return None


class Checker(Node):
    def __init__(self):
        super().__init__('robot_motion_check')
        self.odom = None
        self.cmd = None
        self.create_subscription(Odometry, '/state_estimation', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel_safe', self.on_cmd, 10)
        self.previous_world = None
        self.previous_odom = None
        self.static_since = None

    def on_odom(self, msg):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_cmd(self, msg):
        self.cmd = msg.linear.x

    def report(self):
        world = gazebo_pose()
        angle = wheel_angle()
        odom = self.odom
        cmd = self.cmd
        world_s = f'({world[0]:7.2f},{world[1]:7.2f})' if world else '   gz 无响应  '
        odom_s = f'({odom[0]:7.2f},{odom[1]:7.2f})' if odom else '   等待里程计  '
        cmd_s = f'{cmd:6.3f}' if cmd is not None else '   --  '
        angle_s = f'{angle:8.2f}' if angle is not None else '    --  '
        print(f'{time.strftime("%H:%M:%S")} | world {world_s} | odom {odom_s} | v_cmd {cmd_s} '
              f'| 轮角 {angle_s}', flush=True)

        moved = 0.0
        rolled = 0.0
        if self.previous_world is not None and world is not None:
            moved = ((world[0] - self.previous_world[0]) ** 2
                     + (world[1] - self.previous_world[1]) ** 2) ** 0.5
        if self.previous_angle is not None and angle is not None:
            rolled = abs(angle - self.previous_angle) * 0.06   # wheel radius 0.06 m
        moving = moved > 0.02
        self.previous_world = world
        self.previous_angle = angle
        if moving:
            self.static_since = None
            return
        self.static_since = self.static_since or time.time()
        if time.time() - self.static_since < 2.0:
            return
        # 轮子转了很多而车身没动 = 打滑/被挡住
        if rolled > 0.15 and moved < 0.05:
            print(f'    ↳ 轮子转了约 {rolled:.2f} m 而车身只动了 {moved:.3f} m：**打滑/被挡住**'
                  '（多半是低矮台阶、门槛之类本规划器看不到的负障碍）——换出生点/目标，别硬顶', flush=True)
        elif cmd is not None and abs(cmd) > 0.05:
            print('    ↳ 有速度指令但车身几乎不动：可能被挡住', flush=True)
        else:
            print('    ↳ 世界位姿不变且没有速度指令：规划在暂停/已到点，'
                  '这时 Gazebo 和 RViz 的车都不会动（RViz 里只有轨迹线在刷新）', flush=True)
        self.static_since = time.time()


def main():
    rclpy.init()
    node = Checker()
    print('时间     | Gazebo 服务器真实位姿 | 里程计             | 指令        | 左轮角度')
    print('---------|----------------------|--------------------|-------------|---------')
    next_report = 0.0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() >= next_report:
            try:
                node.report()
            except BrokenPipeError:   # 例如被 `| head` 提前关闭
                break
            next_report = time.time() + 1.0
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
