"""Small robust per-ID estimator; no ROS dependencies."""

from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np


@dataclass
class Track:
    samples: deque = field(default_factory=deque)
    last_stamp: float = -math.inf
    position: object = None
    observations: int = 0


class CubeEstimator:
    def __init__(self, min_observations=5, window_size=30, inlier_radius=0.15,
                 max_spread=0.08, min_interval=0.15, min_duration=0.8,
                 max_sample_age=20.0):
        if min_observations < 2 or window_size < min_observations:
            raise ValueError('Need 2 <= min_observations <= window_size')
        if not all(math.isfinite(v) and v > 0 for v in (
                inlier_radius, max_spread, min_interval, min_duration, max_sample_age)):
            raise ValueError('Estimator thresholds must be finite and positive')
        self.min_observations = min_observations
        self.window_size = window_size
        self.radius = inlier_radius
        self.max_spread = max_spread
        self.min_interval = min_interval
        self.min_duration = min_duration
        self.max_sample_age = max_sample_age
        self.tracks = {}

    def add(self, marker_id, position, stamp):
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.isfinite(position).all() or not math.isfinite(stamp):
            return False
        track = self.tracks.setdefault(marker_id, Track())
        if stamp - track.last_stamp < self.min_interval:
            return False  # Duplicate, out-of-order or overly frequent observation.
        track.last_stamp = stamp
        track.samples.append((stamp, position))
        while (len(track.samples) > self.window_size or
               stamp - track.samples[0][0] > self.max_sample_age):
            track.samples.popleft()
        if len(track.samples) < self.min_observations:
            return False
        points = np.array([p for _, p in track.samples])
        # Largest neighbourhood also recovers when the very first reading was an outlier.
        distances = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)
        seed = int(np.argmax(np.sum(distances <= self.radius, axis=1)))
        centre = np.median(points[distances[seed] <= self.radius], axis=0)
        mask = np.linalg.norm(points - centre, axis=1) <= self.radius
        inliers = points[mask]
        if len(inliers) < self.min_observations:
            return False
        stamps = np.array([t for t, _ in track.samples])[mask]
        if stamps[-1] - stamps[0] < self.min_duration:
            return False
        estimate = np.median(inliers, axis=0)
        spread = float(np.sqrt(np.mean(np.sum((inliers - estimate) ** 2, axis=1))))
        if spread > self.max_spread:
            return False
        # A confirmed static cube must not jump to a new cluster with the same ID.
        if track.position is not None and np.linalg.norm(estimate - track.position) > self.radius:
            return False
        track.position = estimate
        track.observations = len(inliers)
        return True

    def confirmed(self):
        return {i: (t.position.copy(), t.observations)
                for i, t in self.tracks.items() if t.position is not None}

    def reset(self):
        self.tracks.clear()
