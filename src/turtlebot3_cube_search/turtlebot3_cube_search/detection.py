"""OpenCV-only detection and pose estimation, independent of ROS."""

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass
class Detection:
    marker_id: int
    rotation: np.ndarray
    translation: np.ndarray
    reprojection_error: float


def rotation_to_quaternion(rotation):
    """Convert a Rodrigues vector to an (x, y, z, w) unit quaternion."""
    vector = np.asarray(rotation, dtype=float).reshape(3)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    xyz = vector * (math.sin(angle / 2.0) / angle)
    return (*map(float, xyz), math.cos(angle / 2.0))


class MarkerDetector:
    def __init__(self, marker_size, max_error=2.5, min_range=0.1, max_range=5.0):
        values = (marker_size, max_error, min_range, max_range)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Marker size, error and ranges must be finite and positive')
        if max_range <= min_range:
            raise ValueError('max_range must exceed min_range')
        self.max_error = max_error
        self.min_range = min_range
        self.max_range = max_range
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        # Ubuntu 24.04 has OpenCV 4.6; ArucoDetector appeared in newer OpenCV.
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
            self.detect_corners = self.detector.detectMarkers
        else:
            parameters = cv2.aruco.DetectorParameters_create()
            self.detect_corners = lambda image: cv2.aruco.detectMarkers(
                image, dictionary, parameters=parameters)
        half = marker_size / 2.0
        # Required corner order for SOLVEPNP_IPPE_SQUARE: TL, TR, BR, BL.
        self.object_points = np.array([
            [-half, half, 0], [half, half, 0],
            [half, -half, 0], [-half, -half, 0],
        ], dtype=np.float64)

    def detect(self, image, camera_matrix, distortion):
        corners, ids, _ = self.detect_corners(image)
        if ids is None:
            return []
        output = []
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            image_points = marker_corners.reshape(4, 2).astype(np.float64)
            ok, rotation, translation = cv2.solvePnP(
                self.object_points, image_points, camera_matrix, distortion,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok or not np.isfinite(translation).all() or not np.isfinite(rotation).all():
                continue
            distance = float(np.linalg.norm(translation))
            if translation[2, 0] <= 0 or not self.min_range <= distance <= self.max_range:
                continue
            projected, _ = cv2.projectPoints(
                self.object_points, rotation, translation, camera_matrix, distortion)
            error = float(np.sqrt(np.mean(np.sum(
                (projected.reshape(4, 2) - image_points) ** 2, axis=1))))
            if error <= self.max_error:
                output.append(Detection(int(marker_id), rotation, translation.reshape(3), error))
        return output
