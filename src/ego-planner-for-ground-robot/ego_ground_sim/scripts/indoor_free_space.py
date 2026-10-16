#!/usr/bin/env python3
"""Pick reachable goals inside the CMU indoor scene.

Reads the preview point cloud that ships with the scene
(autonomous_exploration_development_environment/src/vehicle_simulator/mesh/indoor/preview/pointcloud.ply),
builds a 2D occupancy grid at robot height and reports free cells that are
*connected* to the spawn point on observed floor. These are candidate goals;
the preview cloud is an approximation of the collision mesh.

    python3 scripts/indoor_free_space.py [spawn_x spawn_y] [min_dist] [max_dist]

Default spawn is the demo screenshot's passage at (-31.3, -23.4), matching
launch/sim_cmu_indoor.launch.py.
"""

import sys
from collections import deque

import numpy as np

PLY = ('/home/zoujing/ego_ground_ws/src/ego-planner-for-ground-robot/'
       'autonomous_exploration_development_environment/src/vehicle_simulator/'
       'mesh/indoor/preview/pointcloud.ply')
SHIFT = (-62.8, -20.4)          # must match worlds/cmu_indoor.world
BOUND = 70.0                    # grid covers [-BOUND, BOUND] in x and y
RES = 0.25                      # cell size of the analysis grid
ROBOT_RADIUS = 0.45             # inflation used for the occupancy grid


def load_points(path):
    with open(path, 'rb') as f:
        header = b''
        while b'end_header' not in header:
            header += f.readline()
        n = 0
        for line in header.decode('ascii', 'ignore').splitlines():
            if line.startswith('element vertex'):
                n = int(line.split()[-1])
        return np.frombuffer(f.read(n * 12), dtype='<f4').reshape(-1, 3).astype(np.float64)


def main():
    if len(sys.argv) == 2 or len(sys.argv) > 5:
        sys.exit('usage: indoor_free_space.py [spawn_x spawn_y] [min_dist] [max_dist]')
    spawn = (float(sys.argv[1]), float(sys.argv[2])) if len(sys.argv) >= 3 else (-31.3, -23.4)
    min_dist = float(sys.argv[3]) if len(sys.argv) > 3 else 8.0
    max_dist = float(sys.argv[4]) if len(sys.argv) > 4 else 30.0

    pts = load_points(PLY)
    pts[:, 0] += SHIFT[0]
    pts[:, 1] += SHIFT[1]

    # floor: the point cloud must contain a surface near z = 0 at that place, otherwise
    # the cell is a hole / atrium and the robot would fall through it
    floor = pts[(pts[:, 2] > -0.20) & (pts[:, 2] < 0.05)]

    n = int(2 * BOUND / RES)
    occ = np.zeros((n, n), dtype=bool)
    # everything between 0.15 m and 1.6 m above the floor blocks the robot
    structure = pts[(pts[:, 2] > -0.05 + 0.15) & (pts[:, 2] < -0.05 + 1.6)]
    idx = ((structure[:, :2] + BOUND) / RES).astype(int)
    idx = idx[(idx[:, 0] >= 0) & (idx[:, 0] < n) & (idx[:, 1] >= 0) & (idx[:, 1] < n)]
    occ[idx[:, 0], idx[:, 1]] = True

    # inflate by the robot radius
    r = int(np.ceil(ROBOT_RADIUS / RES))
    offs = [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)
            if dx * dx + dy * dy <= r * r]
    inflated = occ.copy()
    for dx, dy in offs:
        inflated |= np.roll(np.roll(occ, dx, axis=0), dy, axis=1)
    inflated[:r, :] = inflated[-r:, :] = True
    inflated[:, :r] = inflated[:, -r:] = True

    def cell(p):
        return int((p[0] + BOUND) / RES), int((p[1] + BOUND) / RES)

    sx, sy = cell(spawn)
    if not (0 <= sx < n and 0 <= sy < n):
        sys.exit('spawn is outside the analysis grid')
    floor_cells = np.zeros((n, n), dtype=bool)
    fi = ((floor[:, :2] + BOUND) / RES).astype(int)
    fi = fi[(fi[:, 0] >= 0) & (fi[:, 0] < n) & (fi[:, 1] >= 0) & (fi[:, 1] < n)]
    floor_cells[fi[:, 0], fi[:, 1]] = True
    # Fill sampling gaps of one cell, but keep atriums and off-map space blocked.
    floor_support = floor_cells.copy()
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            floor_support |= np.roll(np.roll(floor_cells, dx, axis=0), dy, axis=1)
    walkable = ~inflated & floor_support
    has_floor = floor_support[sx, sy]
    print(f'spawn ({spawn[0]:.1f}, {spawn[1]:.1f}) -> cell ({sx}, {sy}), '
          f'free: {not inflated[sx, sy]}, floor: {has_floor}')
    if inflated[sx, sy] or not has_floor:
        sys.exit('this spawn is blocked or has no floor - pick another one')

    # BFS over reachable free cells
    seen = np.zeros_like(inflated)
    q = deque([(sx, sy)])
    seen[sx, sy] = True
    while q:
        x, y = q.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < n and 0 <= ny < n and not seen[nx, ny] and walkable[nx, ny]:
                seen[nx, ny] = True
                q.append((nx, ny))

    # Arrays are indexed [x, y], as when inserting the cloud above.
    xs, ys = np.nonzero(seen)
    wx = (xs + 0.5) * RES - BOUND
    wy = (ys + 0.5) * RES - BOUND
    dist = np.hypot(wx - spawn[0], wy - spawn[1])
    area = len(xs) * RES * RES
    print(f'reachable free area from the spawn: {area:.0f} m^2 ({len(xs)} cells)')

    # candidate goals: reachable, inside the requested distance band, and as open
    # as possible (clearance = distance to the nearest blocked cell)
    # a goal must be free *and* on the floor
    reachable_floor = floor_cells[xs, ys]
    cand = (dist >= min_dist) & (dist <= max_dist) & reachable_floor
    if not cand.any():
        print('no reachable goal in the requested distance band')
        return
    blocked = np.argwhere(~walkable)
    bx = (blocked[:, 0] + 0.5) * RES - BOUND
    by = (blocked[:, 1] + 0.5) * RES - BOUND
    idx_cand = np.nonzero(cand)[0]
    clearance = np.array([np.hypot(bx - wx[i], by - wy[i]).min() for i in idx_cand])
    order = np.argsort(-clearance)
    print(f'reachable goals between {min_dist:.0f} and {max_dist:.0f} m '
          f'(sorted by clearance to obstacles / unsupported floor):')
    printed = 0
    for i in idx_cand[order]:
        c = np.hypot(bx - wx[i], by - wy[i]).min()
        if c < 1.5:
            continue
        print(f'  ({wx[i]:7.2f}, {wy[i]:7.2f})  dist {dist[i]:5.2f} m  clearance {c:.2f} m')
        printed += 1
        if printed >= 10:
            break
    if printed == 0:
        print('  none with more than 1.5 m clearance - widen the distance band')


if __name__ == '__main__':
    main()
