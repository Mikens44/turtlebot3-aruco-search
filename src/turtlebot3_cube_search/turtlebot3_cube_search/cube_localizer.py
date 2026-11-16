"""Timestamped TF2 transforms and robust per-ID position confirmation."""

from collections import deque
from copy import deepcopy
import math
import time

import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener

from cube_search_interfaces.msg import ArucoDetectionArray, Cube, CubeArray
from .estimation import CubeEstimator
from .runtime import run_node


def durable_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CubeLocalizer(Node):
    def __init__(self, **kwargs):
        super().__init__('cube_localizer', **kwargs)
        defaults = {
            'map_frame': 'map', 'min_observations': 5, 'window_size': 30,
            'inlier_radius': 0.15, 'max_spread': 0.08, 'min_interval': 0.15,
            'min_duration': 0.8, 'max_sample_age': 20.0,
            'tf_wait_timeout': 0.5, 'max_measurement_age': 2.0,
            'max_reprojection_error': 2.5, 'pending_queue_size': 100,
            'marker_to_cube_offset': [0.0, 0.0, 0.0],
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda name: self.get_parameter(name).value
        self.map_frame = p('map_frame')
        self.tf_wait = p('tf_wait_timeout')
        self.max_age = p('max_measurement_age')
        self.max_error = p('max_reprojection_error')
        self.offset = np.asarray(p('marker_to_cube_offset'), dtype=float)
        if self.offset.shape != (3,) or not np.isfinite(self.offset).all():
            raise ValueError('marker_to_cube_offset must contain three finite values')
        if (not self.map_frame or not all(math.isfinite(v) and v > 0
                                        for v in (self.tf_wait, self.max_age, self.max_error)) or
                p('pending_queue_size') < 1):
            raise ValueError('Invalid localizer parameters')
        self.estimator = CubeEstimator(**{name: p(name) for name in (
            'min_observations', 'window_size', 'inlier_radius', 'max_spread',
            'min_interval', 'min_duration', 'max_sample_age')})
        self.pending = deque(maxlen=p('pending_queue_size'))
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(CubeArray, '/cubes', durable_qos())
        self.create_subscription(ArucoDetectionArray, '/aruco/detections', self.on_detections, 10)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.create_timer(0.05, self.process_pending)
        self.create_timer(1.0, self.publish_cubes)
        self.last_clock = self.get_clock().now().nanoseconds
        self.publish_cubes()

    def on_reset(self, request, response):
        self.pending.clear()
        self.estimator.reset()
        self.publish_cubes()
        response.success = True
        response.message = 'Cleared all cube observations; use only while search is idle.'
        return response

    def on_detections(self, message):
        stamp = message.header.stamp
        if not message.header.frame_id or (stamp.sec == 0 and stamp.nanosec == 0):
            return
        self.pending.append((time.monotonic() + self.tf_wait, message))
        self.process_pending()

    def process_pending(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self.last_clock:
            # A simulator reset invalidates observations from the previous timeline.
            self.pending.clear()
            self.estimator.reset()
            self.publish_cubes()
        self.last_clock = now_ns
        remaining = deque(maxlen=self.pending.maxlen)
        changed = False
        while self.pending:
            expiry, message = self.pending.popleft()
            stamp = Time.from_msg(message.header.stamp)
            age = (now_ns - stamp.nanoseconds) / 1e9
            if age > self.max_age or age < -0.1:
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, message.header.frame_id, stamp)
            except TransformException:
                if time.monotonic() < expiry:
                    remaining.append((expiry, message))
                else:
                    self.get_logger().warning(
                        'TF unavailable at image stamp; measurement dropped', throttle_duration_sec=5.0)
                continue
            seen = set()
            for detection in message.detections:
                if detection.id in seen or not 0 <= detection.id < 1024:
                    continue
                seen.add(detection.id)
                if (not math.isfinite(detection.reprojection_error) or
                        not 0 <= detection.reprojection_error <= self.max_error):
                    continue
                pose = detection.pose
                q = pose.orientation
                quaternion = np.array([q.x, q.y, q.z, q.w])
                if (not np.isfinite(quaternion).all() or
                        abs(np.linalg.norm(quaternion) - 1.0) > 0.01):
                    continue
                # Offset is expressed in marker axes (marker +Z points out of its face).
                xyz, w = quaternion[:3], quaternion[3]
                rotated_offset = self.offset + 2 * np.cross(
                    xyz, np.cross(xyz, self.offset) + w * self.offset)
                adjusted = deepcopy(pose)
                adjusted.position.x += float(rotated_offset[0])
                adjusted.position.y += float(rotated_offset[1])
                adjusted.position.z += float(rotated_offset[2])
                mapped = do_transform_pose(adjusted, transform)
                point = mapped.position
                changed |= self.estimator.add(detection.id, [point.x, point.y, point.z],
                                              stamp.nanoseconds / 1e9)
        self.pending = remaining
        if changed:
            self.publish_cubes()

    def publish_cubes(self):
        output = CubeArray()
        output.header.frame_id = self.map_frame
        output.header.stamp = self.get_clock().now().to_msg()
        for marker_id, (position, count) in sorted(self.estimator.confirmed().items()):
            cube = Cube()
            cube.id = marker_id
            cube.position.x, cube.position.y, cube.position.z = map(float, position)
            cube.observations = count
            output.cubes.append(cube)
        self.publisher.publish(output)


def main(args=None):
    run_node(CubeLocalizer, args)
