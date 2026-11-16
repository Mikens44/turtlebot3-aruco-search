import math
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from turtlebot3_cube_search.detection import MarkerDetector, rotation_to_quaternion


def marker_image(marker_id=17, pixels=180):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, pixels)
    else:
        marker = cv2.aruco.drawMarker(dictionary, marker_id, pixels)
    image = np.full((480, 640), 255, dtype=np.uint8)
    image[150:150 + pixels, 230:230 + pixels] = marker
    return image


def test_synthetic_marker_id_and_metric_pose():
    matrix = np.array([[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]])
    detections = MarkerDetector(0.12).detect(marker_image(), matrix, np.zeros(5))
    assert len(detections) == 1
    detection = detections[0]
    assert detection.marker_id == 17
    assert detection.translation[2] == pytest.approx(600 * .12 / 179, abs=.005)
    assert np.linalg.norm(detection.translation[:2]) < .003
    assert detection.reprojection_error < 1.0
    assert np.linalg.norm(rotation_to_quaternion(detection.rotation)) == pytest.approx(1.0)


def test_blank_and_out_of_range():
    matrix = np.array([[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]])
    detector = MarkerDetector(.12, max_range=.3)
    assert detector.detect(marker_image(), matrix, np.zeros(5)) == []
    assert detector.detect(np.full((480, 640), 255, np.uint8), matrix, np.zeros(5)) == []


def test_configured_scale_matches_arm05_marker_instead_of_cube():
    # arm05_sim: a 9 cm cube face maps a 755 px texture containing a 588 px marker.
    # Generate ideal image corners independently of the configured PnP size.
    half = (.09 * 588 / 755) / 2
    points = np.array([[-half, half, 0], [half, half, 0],
                       [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
    matrix = np.array([[1696.80279, 0, 960], [0, 1696.80267, 540], [0, 0, 1.]])
    truth = np.array([.1, .1, 3.8])
    corners, _ = cv2.projectPoints(points, np.array([3., .1, .05]), truth, matrix, np.zeros(5))
    config_path = Path(__file__).resolve().parents[1] / 'config' / 'search.yaml'
    configured_size = yaml.safe_load(config_path.read_text())['aruco_detector']['ros__parameters'][
        'marker_size']
    detector = MarkerDetector(configured_size)
    detector.detect_corners = lambda _: ([corners.reshape(1, 4, 2)], np.array([[0]]), [])
    result = detector.detect(np.zeros((1, 1), np.uint8), matrix, np.zeros(5))[0]
    assert np.linalg.norm(result.translation - truth) < .01
    # The historical 9 cm assumption yields ~1.08 m error even with ideal corners.
    wrong = MarkerDetector(.09)
    wrong.detect_corners = detector.detect_corners
    biased = wrong.detect(np.zeros((1, 1), np.uint8), matrix, np.zeros(5))[0]
    assert np.linalg.norm(biased.translation - truth) > 1.0


def test_quaternion_identity_and_half_turn():
    assert rotation_to_quaternion([0, 0, 0]) == (0, 0, 0, 1)
    assert rotation_to_quaternion([math.pi, 0, 0]) == pytest.approx((1, 0, 0, 0))


@pytest.mark.parametrize('size', [0, -1, math.nan, math.inf])
def test_bad_size(size):
    with pytest.raises(ValueError):
        MarkerDetector(size)
