"""Goal-directed optimistic routing, with observed-space execution constraints."""
import heapq
import math
import time

import numpy as np
from scipy.ndimage import distance_transform_edt, maximum_filter


class NavigationGrid:
    def __init__(self, data, resolution, origin, radius, routing_resolution=0.1, observed_radius=None):
        self.origin = origin  # x, y, yaw of OccupancyGrid origin
        factor = max(1, int(math.ceil(routing_resolution / resolution)))
        height, width = data.shape
        padded = np.pad(data, ((0, (-height) % factor), (0, (-width) % factor)),
                        constant_values=-1)
        blocks = padded.reshape(padded.shape[0] // factor, factor,
                                padded.shape[1] // factor, factor)
        occupied = (blocks >= 50).any(axis=(1, 3))
        unknown = (blocks < 0).any(axis=(1, 3))
        self.data = np.where(occupied, 100, np.where(unknown, -1, 0)).astype(np.int8)
        self.resolution = resolution * factor
        self.radius = radius
        self.observed_radius = radius if observed_radius is None else observed_radius
        self.blocked = self.inflate(self.data >= 50, radius)

    def inflate(self, occupied, radius):
        if not occupied.any():
            return np.zeros_like(occupied, dtype=bool)
        # EGO expands x/y separately: its footprint in the grid is a square.
        # A circular global inflation would accept corners the local map rejects.
        cells = int(math.ceil(radius / self.resolution))
        return maximum_filter(occupied, size=2 * cells + 1, mode='constant', cval=0).astype(bool)

    def overlay_local_obstacles(self, data, resolution, origin, map_to_local):
        rows, cols = np.indices(self.data.shape)
        x, y = (cols + 0.5) * self.resolution, (rows + 0.5) * self.resolution
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        wx, wy = self.origin[0] + c*x-s*y, self.origin[1]+s*x+c*y
        c, s = math.cos(map_to_local[2]), math.sin(map_to_local[2])
        lx, ly = map_to_local[0] + c*wx-s*wy-origin[0], map_to_local[1]+s*wx+c*wy-origin[1]
        c, s = math.cos(origin[2]), math.sin(origin[2])
        ix = np.floor((c*lx+s*ly)/resolution).astype(int)
        iy = np.floor((-s*lx+c*ly)/resolution).astype(int)
        inside = (ix >= 0) & (iy >= 0) & (ix < data.shape[1]) & (iy < data.shape[0])
        occupied = data[np.clip(iy, 0, data.shape[0]-1), np.clip(ix, 0, data.shape[1]-1)] >= 50
        self.blocked |= inside & occupied

    def index(self, point):
        dx, dy = point[0] - self.origin[0], point[1] - self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return (int(math.floor((c * dx + s * dy) / self.resolution)),
                int(math.floor((-s * dx + c * dy) / self.resolution)))

    def point(self, index):
        x, y = (index[0] + 0.5) * self.resolution, (index[1] + 0.5) * self.resolution
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return (self.origin[0] + c * x - s * y, self.origin[1] + s * x + c * y)

    def observed_clearance(self, start):
        free = self.data == 0
        sx, sy = self.index(start)
        # The current footprint is already physically occupied by this vehicle.
        # Only its unknown cells may be assumed free, never measured obstacles.
        cells = int(math.ceil(self.observed_radius / self.resolution))
        for y in range(max(0, sy - cells), min(free.shape[0], sy + cells + 1)):
            for x in range(max(0, sx - cells), min(free.shape[1], sx + cells + 1)):
                if math.hypot(x - sx, y - sy) * self.resolution <= self.observed_radius:
                    if self.data[y, x] < 0:
                        free[y, x] = True
        distance = distance_transform_edt(np.pad(free, 1, constant_values=False))[1:-1, 1:-1]
        clear = distance * self.resolution > self.observed_radius
        if 0 <= sy < free.shape[0] and 0 <= sx < free.shape[1] and not self.blocked[sy, sx]:
            clear[sy, sx] = True
        return clear

    def observe_scan(self, sensor_pose, angle_min, increment, ranges, minimum, maximum):
        """Fill SLAM raster holes only in front of VALID FINITE returns.

        N10p's cloud-to-scan conversion uses +inf both for empty bins and masked
        angles. Therefore +inf/NaN/zero cannot independently certify free space.
        """
        if increment <= 0 or len(ranges) < 2:
            return
        rows, cols = np.indices(self.data.shape)
        x, y = (cols + 0.5) * self.resolution, (rows + 0.5) * self.resolution
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        dx = self.origin[0] + c * x - s * y - sensor_pose[0]
        dy = self.origin[1] + s * x + c * y - sensor_pose[1]
        angle = (np.arctan2(dy, dx) - sensor_pose[2] - angle_min) % (2 * math.pi)
        indices = np.floor(angle / increment + 0.5).astype(int)
        valid_index = indices < len(ranges)
        values = np.asarray(ranges)[np.clip(indices, 0, len(ranges) - 1)]
        valid = valid_index & np.isfinite(values) & (values >= minimum) & (values <= maximum)
        free = valid & (np.hypot(dx, dy) + self.resolution * math.sqrt(2) < values)
        self.data[(self.data < 0) & free] = 0

    def known(self, point, clear):
        x, y = self.index(point)
        return (0 <= y < clear.shape[0] and 0 <= x < clear.shape[1] and
                bool(clear[y, x]) and not self.blocked[y, x])

    def segment_known(self, start, end, clear):
        steps = max(1, math.ceil(math.dist(start, end) / (self.resolution * 0.4)))
        return all(self.known((start[0] + (end[0] - start[0]) * i / steps,
                               start[1] + (end[1] - start[1]) * i / steps), clear)
                   for i in range(steps + 1))

    def route(self, start, goal, lookahead=1.0, unknown_cost=4.0, max_cells=300000):
        begin, end = self.index(start), self.index(goal)
        # Extend the routing grid with UNKNOWN cells, not free cells. This lets
        # a user point outside the current SLAM map without commanding that area.
        pad = int(math.ceil(10.0 / self.resolution))
        low_x, low_y = min(begin[0], end[0]) - pad, min(begin[1], end[1]) - pad
        high_x, high_y = max(begin[0], end[0]) + pad + 1, max(begin[1], end[1]) + pad + 1
        width, height = high_x - low_x, high_y - low_y
        if width * height > max_cells:
            return [], None, 'goal/map too large; choose a nearer goal'
        grid = np.full((height, width), -1, dtype=np.int8)
        blocked = np.zeros((height, width), dtype=bool)
        x0, y0 = max(0, low_x), max(0, low_y)
        x1, y1 = min(self.data.shape[1], high_x), min(self.data.shape[0], high_y)
        if x1 > x0 and y1 > y0:
            grid[y0-low_y:y1-low_y, x0-low_x:x1-low_x] = self.data[y0:y1, x0:x1]
            blocked[y0-low_y:y1-low_y, x0-low_x:x1-low_x] = self.blocked[y0:y1, x0:x1]
        first = begin[0] - low_x, begin[1] - low_y
        last = end[0] - low_x, end[1] - low_y
        if blocked[first[1], first[0]] or blocked[last[1], last[0]]:
            return [], None, 'start/goal inside obstacle clearance'
        queue = [(0.0, first)]
        costs, parent = {first: 0.0}, {}
        deadline = time.monotonic() + 0.30
        expanded = set()
        directions = ((1, 0), (-1, 0), (0, 1), (0, -1),
                      (1, 1), (-1, 1), (1, -1), (-1, -1))
        while queue:
            _, current = heapq.heappop(queue)
            if current in expanded:
                continue
            if current == last:
                break
            expanded.add(current)
            if len(expanded) > 60000 or (len(expanded) % 128 == 0 and time.monotonic() > deadline):
                return [], None, 'routing budget exceeded; choose a nearer goal'
            for dx, dy in directions:
                nxt = current[0] + dx, current[1] + dy
                x, y = nxt
                if not (0 <= x < width and 0 <= y < height) or blocked[y, x]:
                    continue
                if dx and dy and (blocked[current[1], x] or blocked[y, current[0]]):
                    continue
                penalty = unknown_cost if grid[y, x] < 0 else 1.0
                value = costs[current] + math.hypot(dx, dy) * penalty
                if value >= costs.get(nxt, math.inf):
                    continue
                costs[nxt], parent[nxt] = value, current
                heuristic = math.hypot(last[0] - x, last[1] - y)
                heapq.heappush(queue, (value + heuristic, nxt))
        if last != first and last not in parent:
            return [], None, 'no optimistic route to goal'
        indices, cell = [last], last
        while cell != first:
            cell = parent[cell]
            indices.append(cell)
        path = [self.point((x + low_x, y + low_y)) for x, y in reversed(indices)]
        clear, target, travelled = self.observed_clearance(start), None, 0.0
        previous = start
        for point in path[1:]:
            travelled += math.dist(previous, point)
            previous = point
            if travelled > lookahead or not self.known(point, clear):
                break
            if not self.segment_known(start, point, clear):
                break
            target = point
        if target is not None and math.dist(start, target) < 0.24:
            target = None
        if target is None and len(path) > 1:
            # At a grid corner the shortest path may turn after only 10 cm,
            # shorter than EGO's 20 cm minimum planning distance. Extend a safe
            # straight leg into observed space before taking the next turn.
            preferred = path[min(10, len(path) - 1)]
            direction = np.asarray(preferred) - start
            best = math.inf
            for distance in (0.35, 0.5, 0.7, 0.9):
                if distance > lookahead:
                    continue
                for angle in np.linspace(-math.pi, math.pi, 32, endpoint=False):
                    delta = np.array((distance * math.cos(angle), distance * math.sin(angle)))
                    if float(delta @ direction) < 0.05:
                        continue
                    candidate = start[0] + delta[0], start[1] + delta[1]
                    if not self.segment_known(start, candidate, clear):
                        continue
                    score = math.dist(candidate, preferred) + 0.05 * distance
                    if score < best:
                        best, target = score, candidate
        return path, target, 'advancing through observed space' if target else 'waiting for new observed free space'

    def motion_known(self, position, yaw, speed, turn, clear, reaction=0.5, deceleration=0.2):
        horizon = reaction + abs(speed) / deceleration
        steps = max(1, math.ceil(horizon / 0.02))
        for i in range(steps + 1):
            t = horizon * i / steps
            dx = speed * t if abs(turn) < 1e-6 else speed * math.sin(turn * t) / turn
            dy = 0.0 if abs(turn) < 1e-6 else speed * (1 - math.cos(turn * t)) / turn
            point = (position[0] + math.cos(yaw) * dx - math.sin(yaw) * dy,
                     position[1] + math.sin(yaw) * dx + math.cos(yaw) * dy)
            if not self.known(point, clear):
                return False
        return True
