"""Camera subscriber and DICT_ARUCO_ORIGINAL pose publisher."""

from copy import deepcopy
import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cube_search_interfaces.msg import ArucoDetection, ArucoDetectionArray
from sensor_msgs.msg import CameraInfo, Image

from .detection import MarkerDetector, rotation_to_quaternion
from .runtime import run_node


class ArucoDetector(Node):
    def __init__(self, **kwargs):
        super().__init__('aruco_detector', **kwargs)
        self.declare_parameter('marker_size', 0.0)
        self.declare_parameter('optical_frame', 'camera_rgb_optical_frame')
        self.declare_parameter('max_reprojection_error', 2.5)
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('allowed_ids', [-1])
        self.optical_frame = self.get_parameter('optical_frame').value
        if not self.optical_frame or self.optical_frame != self.optical_frame.strip():
            raise ValueError('optical_frame must be a nonempty TF frame without surrounding whitespace')
        self.detector = MarkerDetector(
            self.get_parameter('marker_size').value,
            self.get_parameter('max_reprojection_error').value,
            self.get_parameter('min_range').value,
            self.get_parameter('max_range').value)
        allowed = self.get_parameter('allowed_ids').value
        self.allowed_ids = None if allowed == [-1] else set(allowed)
        if self.allowed_ids is not None and any(i < 0 or i >= 1024 for i in self.allowed_ids):
            raise ValueError('DICT_ARUCO_ORIGINAL IDs must be between 0 and 1023')
        self.bridge = CvBridge()
        self.calibration = None
        self.publisher = self.create_publisher(ArucoDetectionArray, '/aruco/detections', 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.on_image,
                                 qos_profile_sensor_data)

    def on_info(self, message):
        matrix = np.asarray(message.k, dtype=float).reshape(3, 3)
        distortion = np.asarray(message.d, dtype=float)
        if (message.distortion_model not in ('plumb_bob', 'rational_polynomial') or
                not np.isfinite(matrix).all() or not np.isfinite(distortion).all() or
                matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or
                len(distortion) not in (0, 4, 5, 8, 12, 14) or
                message.binning_x > 1 or message.binning_y > 1 or
                message.roi.x_offset or message.roi.y_offset or
                message.roi.width not in (0, message.width) or
                message.roi.height not in (0, message.height)):
            self.calibration = None
            self.get_logger().warning(
                'Unsupported/invalid calibration: use full-resolution pinhole images, '
                'without ROI/binning, and matching CameraInfo.', throttle_duration_sec=5.0)
            return
        self.calibration = (message, matrix, distortion)

    def on_image(self, message):
        if self.calibration is None:
            self.get_logger().warning('Waiting for valid CameraInfo', throttle_duration_sec=5.0)
            return
        info, matrix, distortion = self.calibration
        if (not message.header.frame_id or message.header.frame_id != info.header.frame_id or
                message.width != info.width or message.height != info.height or
                (message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0)):
            self.get_logger().warning(
                'Image must have a nonzero stamp and match CameraInfo size/source frame',
                throttle_duration_sec=5.0)
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='mono8')
            detections = self.detector.detect(image, matrix, distortion)
        except (CvBridgeError, cv2.error, ValueError) as error:
            self.get_logger().warning(f'Image processing failed: {error}', throttle_duration_sec=5.0)
            return
        output = ArucoDetectionArray()
        # solvePnP returns both translation and rotation in optical axes, even
        # when the driver labels the image/CameraInfo with a mechanical frame.
        # Keep the measurement time; TF2 applies the existing optical transform.
        output.header = deepcopy(message.header)
        output.header.frame_id = self.optical_frame
        for detection in detections:
            if self.allowed_ids is not None and detection.marker_id not in self.allowed_ids:
                continue
            marker = ArucoDetection()
            marker.id = detection.marker_id
            p = detection.translation
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, p)
            q = rotation_to_quaternion(detection.rotation)
            (marker.pose.orientation.x, marker.pose.orientation.y,
             marker.pose.orientation.z, marker.pose.orientation.w) = q
            marker.reprojection_error = detection.reprojection_error
            if math.isfinite(marker.reprojection_error):
                output.detections.append(marker)
        self.publisher.publish(output)


def main(args=None):
    run_node(ArucoDetector, args)
