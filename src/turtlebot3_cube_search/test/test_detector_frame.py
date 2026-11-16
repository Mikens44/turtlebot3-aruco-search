"""OpenCV poses must retain optical axes regardless of camera driver headers."""

import math
from types import SimpleNamespace

from cv_bridge import CvBridge
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo

from turtlebot3_cube_search.aruco_detector import ArucoDetector
from turtlebot3_cube_search.detection import Detection


@pytest.mark.parametrize('source_frame,optical_frame', [
    ('camera_rgb_frame', 'camera_rgb_optical_frame'),
    ('camera_rgb_optical_frame', 'camera_rgb_optical_frame'),
    ('front_camera_link', 'front_camera_optical_frame'),
])
def test_output_frame_timestamp_and_pose_without_extra_rotation(source_frame, optical_frame):
    rclpy.init()
    parameters = [Parameter('marker_size', value=.12)]
    if optical_frame != 'camera_rgb_optical_frame':
        parameters.append(Parameter('optical_frame', value=optical_frame))
    node = ArucoDetector(parameter_overrides=parameters)
    try:
        published = []
        node.publisher = SimpleNamespace(publish=published.append)
        # Fix solvePnP output to test the ROS boundary independently of detection.
        node.detector.detect = lambda *_: [Detection(
            17, np.array([math.pi, 0., 0.]),
            np.array([-.23184, .10913, 2.77726]), .1)]
        info = CameraInfo()
        info.header.frame_id = source_frame
        info.width, info.height = 640, 480
        info.distortion_model = 'plumb_bob'
        info.k = [600., 0., 320., 0., 600., 240., 0., 0., 1.]
        info.d = [0.] * 5
        node.on_info(info)
        image = CvBridge().cv2_to_imgmsg(np.zeros((480, 640), np.uint8), encoding='mono8')
        image.header.frame_id = source_frame
        image.header.stamp.sec, image.header.stamp.nanosec = 123, 456789012
        node.on_image(image)
        assert len(published) == 1
        message = published[0]
        assert message.header.frame_id == optical_frame
        assert message.header.stamp == image.header.stamp
        assert image.header.frame_id == source_frame  # No aliased header mutation.
        assert info.header.frame_id == source_frame
        detection = message.detections[0]
        position, q = detection.pose.position, detection.pose.orientation
        assert [position.x, position.y, position.z] == pytest.approx([-.23184, .10913, 2.77726])
        assert [q.x, q.y, q.z, q.w] == pytest.approx([1., 0., 0., 0.])
    finally:
        node.destroy_node()
        rclpy.shutdown()
