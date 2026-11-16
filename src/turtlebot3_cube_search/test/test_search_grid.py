import math

import numpy as np
import pytest

from turtlebot3_cube_search.search_grid import GridGeometry, reachable_grid


def test_rotated_origin_and_cell_roundtrip():
    geometry = GridGeometry(.1, -3, 8, math.pi / 2)
    assert geometry.to_world(0, 0) == pytest.approx((-3.05, 8.05))
    for row, col in [(0, 0), (12, 30), (42, 5)]:
        assert geometry.to_cell(*geometry.to_world(row, col)) == (row, col)


def test_obstacles_unknown_disconnected_and_border_clearance():
    occupancy = np.zeros((40, 60), dtype=np.int8)
    occupancy[:, 30] = 100  # Disconnected right half.
    occupancy[15:20, 10:15] = -1
    geometry = GridGeometry(.1, 0, 0)
    points = reachable_grid(occupancy, geometry, (1, 1), .5, .25)
    assert len(points) > 1
    blocked = np.argwhere((occupancy < 0) | (occupancy > 20))
    for x, y in points:
        assert .25 <= x <= 2.75
        assert .25 <= y <= 3.75
        assert all(math.dist((x, y), geometry.to_world(r, c)) >= .25 for r, c in blocked)


def test_narrow_area_fallback_and_invalid_robot():
    geometry = GridGeometry(.1, 0, 0)
    occupancy = np.zeros((9, 9), np.int8)
    assert reachable_grid(occupancy, geometry, (.45, .45), 10., .15) == [(.45, .45)]
    with pytest.raises(ValueError, match='Robot pose'):
        reachable_grid(occupancy, geometry, (-1, -1), 1., .15)
    with pytest.raises(ValueError, match='Robot pose'):
        reachable_grid(np.full((9, 9), -1), geometry, (.45, .45), 1., .15)


def test_diagonal_corner_is_not_a_path():
    occupancy = np.full((6, 6), 100, np.int8)
    occupancy[1:3, 1:3] = 0
    occupancy[3:5, 3:5] = 0
    geometry = GridGeometry(1., 0, 0)
    points = reachable_grid(occupancy, geometry, (1.5, 1.5), 1., .1)
    assert all(x < 3 and y < 3 for x, y in points)
