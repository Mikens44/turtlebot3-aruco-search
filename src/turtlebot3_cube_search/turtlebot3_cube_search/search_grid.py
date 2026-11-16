"""Conservative observation grid in an OccupancyGrid, without ROS imports."""

from collections import deque
from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass
class GridGeometry:
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def to_world(self, row, col):
        x, y = (col + 0.5) * self.resolution, (row + 0.5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (self.origin_x + c * x - s * y, self.origin_y + s * x + c * y)

    def to_cell(self, x, y):
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (math.floor((-s * dx + c * dy) / self.resolution),
                math.floor((c * dx + s * dy) / self.resolution))


def reachable_grid(occupancy, geometry, robot_xy, spacing, clearance, free_threshold=20):
    """Return regularly spaced reachable cell centres in world coordinates.

    Inflate unknown, occupied and outside-map cells. Four-connected flood fill
    avoids diagonal shortcuts through corners. Nav2 still validates each path.
    """
    occupancy = np.asarray(occupancy)
    if occupancy.ndim != 2 or not occupancy.size:
        raise ValueError('Map must be a nonempty 2D array')
    if not all(math.isfinite(v) and v > 0
               for v in (geometry.resolution, spacing, clearance)):
        raise ValueError('Resolution, spacing and clearance must be positive')
    if not 0 <= free_threshold <= 100:
        raise ValueError('Invalid free threshold')
    free = ((occupancy >= 0) & (occupancy <= free_threshold)).astype(np.uint8)
    padded = np.pad(free, 1, constant_values=0)
    distance = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
    # Distance transform measures cell centres; subtract half the cell diagonal
    # to bound distance to the actual occupied square conservatively.
    safe = (distance * geometry.resolution - geometry.resolution / math.sqrt(2)) >= clearance
    start = geometry.to_cell(*robot_xy)
    height, width = safe.shape
    if not (0 <= start[0] < height and 0 <= start[1] < width and safe[start]):
        raise ValueError('Robot pose is outside conservatively inflated free space')
    reached = np.zeros_like(safe)
    reached[start] = True
    queue = deque([start])
    while queue:
        row, col = queue.popleft()
        for nr, nc in ((row-1, col), (row+1, col), (row, col-1), (row, col+1)):
            if 0 <= nr < height and 0 <= nc < width and safe[nr, nc] and not reached[nr, nc]:
                reached[nr, nc] = True
                queue.append((nr, nc))
    step = max(1, math.ceil(spacing / geometry.resolution))
    cells = [(r, c) for r in range(step // 2, height, step)
             for c in range(step // 2, width, step) if reached[r, c]]
    # Small or narrow reachable areas may contain no regular-grid intersection.
    if not cells:
        cells = [start]
    return [geometry.to_world(r, c) for r, c in cells]
