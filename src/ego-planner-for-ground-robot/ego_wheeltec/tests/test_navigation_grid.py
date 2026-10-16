"""Unknown may guide routing, but must never become a commanded local target."""
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from navigation_grid import NavigationGrid


def test_outside_map_goal_gets_only_an_observed_local_target():
    data = np.full((100, 100), -1, dtype=np.int8)
    data[20:80, 10:70] = 0
    grid = NavigationGrid(data, 0.1, (0, 0, 0), 0.3)
    path, target, _ = grid.route((3.0, 5.0), (13.0, 5.0), lookahead=2.0)
    assert path and target is not None
    assert abs(path[-1][0] - 13.0) < 0.1
    assert grid.known(target, grid.observed_clearance((3.0, 5.0)))
    assert target[0] < 6.7


def test_no_translation_is_planned_through_a_completely_unknown_map():
    grid = NavigationGrid(np.full((100, 100), -1, dtype=np.int8), 0.1, (0, 0, 0), 0.3)
    path, target, _ = grid.route((4.0, 5.0), (8.0, 5.0))
    assert path and target is None


def test_known_islands_do_not_allow_jumping_over_unknown_space():
    data = np.full((100, 160), -1, dtype=np.int8)
    data[20:80, 10:60] = 0
    data[20:80, 90:150] = 0
    grid = NavigationGrid(data, 0.1, (0, 0, 0), 0.25)
    _, target, _ = grid.route((4.5, 5), (12, 5), lookahead=8.0)
    assert target is not None and target[0] < 5.8


def test_known_wall_routes_through_door_and_local_target_keeps_line_of_sight():
    data = np.zeros((120, 160), dtype=np.int8)
    data[:, 79:82] = 100
    data[70:90, 79:82] = 0
    grid = NavigationGrid(data, 0.1, (0, 0, 0), 0.25)
    path, target, _ = grid.route((3, 3), (12, 3), lookahead=1.2)
    assert path and target is not None
    assert all(7.2 <= y <= 8.8 for x, y in path if 7.9 <= x <= 8.2)
    assert grid.segment_known((3, 3), target, grid.observed_clearance((3, 3)))


def test_motion_stops_before_unknown_even_when_reference_route_extends_there():
    data = np.full((100, 100), -1, dtype=np.int8)
    data[20:80, 10:60] = 0
    grid = NavigationGrid(data, 0.1, (0, 0, 0), 0.3)
    clear = grid.observed_clearance((5.5, 5.0))
    assert grid.motion_known((5.5, 5.0), 0.0, 0.0, 0.4, clear)
    assert not grid.motion_known((5.5, 5.0), 0.0, 0.5, 0.0, clear)


def test_rotated_grid_coordinates_and_occupied_goal():
    data = np.zeros((50, 50), dtype=np.int8)
    data[25, 25] = 100
    grid = NavigationGrid(data, 0.1, (10.0, -3.0, 0.7), 0.2)
    assert grid.index(grid.point((12, 18))) == (12, 18)
    path, target, status = grid.route(grid.point((10, 10)), grid.point((25, 25)))
    assert not path and target is None and 'obstacle' in status


def test_finite_scan_fills_raster_holes_but_inf_masked_sectors_stay_unknown():
    grid = NavigationGrid(np.full((80, 80), -1, dtype=np.int8), 0.1, (-4, -4, 0), 0.3)
    rays = np.full(360, np.inf)
    rays[150:211] = 3.0  # forward +/-30 degrees; everything else is uncertified
    grid.observe_scan((0, 0, 0), -np.pi, 2 * np.pi / 360, rays, 0.1, 8.0)
    x, y = grid.index((1.0, 0.0))
    assert grid.data[y, x] == 0
    x, y = grid.index((-1.0, 0.0))
    assert grid.data[y, x] == -1
    x, y = grid.index((3.4, 0.0))
    assert grid.data[y, x] == -1


def test_global_inflation_matches_egos_square_corner_expansion():
    data = np.zeros((100, 100), dtype=np.int8)
    data[50, 50] = 100
    grid = NavigationGrid(data, 0.1, (0, 0, 0), 0.3)
    assert grid.blocked[53, 53], 'EGO also occupies the square inflation corner'


def test_local_ego_obstacles_are_respected_after_frame_conversion():
    grid = NavigationGrid(np.zeros((100, 100), dtype=np.int8), 0.1, (0, 0, 0), 0.3)
    clear = grid.observed_clearance((1.0, 1.0))
    assert grid.known((2.05, 2.05), clear)
    local = np.zeros((100, 100), dtype=np.int8)
    local[20, 30] = 100
    grid.overlay_local_obstacles(local, 0.1, (0, 0, 0), (1.0, 0.0, 0.0))
    assert not grid.known((2.05, 2.05), clear)


def test_grid_corner_gets_a_safe_leg_long_enough_for_ego():
    data = np.zeros((100, 100), dtype=np.int8)
    data[:22, 15] = 100
    grid = NavigationGrid(data, 0.1, (0, 0, 0), 0.5, observed_radius=0.3)
    start = (2.04, 2.71)
    path, target, _ = grid.route(start, (4.0, 1.0))
    assert path and target is not None
    assert np.linalg.norm(np.asarray(target) - start) >= 0.24
    assert grid.segment_known(start, target, grid.observed_clearance(start))
