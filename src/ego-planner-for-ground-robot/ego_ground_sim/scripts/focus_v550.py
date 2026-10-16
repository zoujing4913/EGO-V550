#!/usr/bin/env python3
"""Position Gazebo Classic's GUI camera near the spawned V550 once at startup."""
import math
import argparse
import subprocess
import time


def gz(*args):
    return subprocess.check_output(['gz', *args], text=True,
                                   stderr=subprocess.DEVNULL, timeout=8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--view', choices=['top', 'follow'], default='top')
    parser.add_argument('--height', type=float, default=8.0)
    parser.add_argument('--center-x', type=float)
    parser.add_argument('--center-y', type=float)
    args = parser.parse_args()
    if not math.isfinite(args.height) or args.height <= 0:
        parser.error('--height must be positive and finite')
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        try:
            if 'gzclient_camera' not in gz('camera', '-l'):
                time.sleep(1)
                continue
            pose = [float(v) for v in gz('model', '-m', 'v550_mec', '-p').split()]
            gui_topic = next(t for t in gz('topic', '-l').splitlines()
                             if t.startswith('/gazebo/') and t.endswith('/gui'))
            if len(pose) < 3:
                raise ValueError('Model pose not ready')
            x, y, z = pose[:3]
            dx, dy, dz = 0.55, 1.1, 1.2
            yaw = math.atan2(-dy, -dx)
            pitch = math.atan2(dz - 0.10, math.hypot(dx, dy))
            if args.view == 'top':
                x = args.center_x if args.center_x is not None else x
                y = args.center_y if args.center_y is not None else y
                dx, dy, dz = 0.0, 0.0, args.height
                # Align world X with the horizontal screen axis for wide buildings.
                yaw, pitch = math.pi/2, math.pi/2
            qx = -math.sin(pitch / 2) * math.sin(yaw / 2)
            qy = math.sin(pitch / 2) * math.cos(yaw / 2)
            qz = math.cos(pitch / 2) * math.sin(yaw / 2)
            qw = math.cos(pitch / 2) * math.cos(yaw / 2)
            track = ('track { name: "" }' if args.view == 'top' else
                     'track { name: "v550_mec" inherit_yaw: false '
                     'inherit_orientation: false min_dist: 1.0 max_dist: 2.0 '
                     'static: false use_model_frame: false '
                     f'xyz {{ x: {dx} y: {dy} z: {dz} }} }}')
            message = (
                'camera { name: "user_camera" '
                f'pose {{ position {{ x: {x+dx} y: {y+dy} z: {z+dz} }} '
                f'orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }} }} '
                f'{track} }}')
            gz('topic', '-p', gui_topic, '-m', message)
            print(f'Gazebo camera: {args.view} view.', flush=True)
            return
        except (subprocess.SubprocessError, ValueError, StopIteration):
            time.sleep(1)
    print('Gazebo camera was not ready. Select v550_mec in the World panel to focus it.',
          flush=True)


if __name__ == '__main__':
    main()
