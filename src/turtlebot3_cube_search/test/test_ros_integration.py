"""Real rclpy/TF/messages, synthetic camera and fake Nav2 action servers.

No Gazebo or robot is used here. These tests verify contracts and cancellation,
not navigation quality or physical stopping.
"""

from copy import deepcopy
from itertools import count
import math
import threading
import time

from action_msgs.msg import GoalStatus
from cube_search_interfaces.action import FindCubes
from cube_search_interfaces.msg import ArucoDetection, ArucoDetectionArray, Cube, CubeArray
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
import numpy as np
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from turtlebot3_cube_search.aruco_detector import ArucoDetector
from turtlebot3_cube_search.cube_localizer import CubeLocalizer, durable_qos
from turtlebot3_cube_search.cube_search_server import CubeSearchServer
from test_detection import marker_image


_test_domains = count(100)


def wait_until(predicate, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('Timed out waiting for ROS condition')


def future_result(future, timeout=5.0):
    wait_until(future.done, timeout)
    return future.result()


@pytest.fixture
def runtime():
    # Each case recreates identically named Nav2 actions. Isolate DDS discovery
    # so a disappearing server from the previous case cannot satisfy readiness.
    rclpy.init(domain_id=next(_test_domains))
    executor = MultiThreadedExecutor(num_threads=8)
    spin_stop = threading.Event()
    spin_errors = []

    def spin():
        try:
            while not spin_stop.is_set():
                executor.spin_once(timeout_sec=.05)
        except Exception as error:
            spin_errors.append(error)

    thread = threading.Thread(target=spin, daemon=True)
    nodes = []

    def add(node):
        nodes.append(node)
        executor.add_node(node)
        return node

    thread.start()
    yield add
    for node in nodes:
        if isinstance(node, CubeSearchServer):
            node.stop()
    # Let cancellation/result callbacks complete before shutting the executor down.
    limit = time.monotonic() + 2
    while any(getattr(node, 'busy', False) for node in nodes) and time.monotonic() < limit:
        time.sleep(.02)
    # Join the spin loop before shutdown destroys its guard condition. Otherwise
    # a concurrent spin_once may touch an already destroyed rclpy handle.
    spin_stop.set()
    thread.join(timeout=3)
    executor.shutdown(timeout_sec=3)
    for node in reversed(nodes):
        node.destroy_node()
    rclpy.shutdown()
    assert not thread.is_alive()
    assert not spin_errors, spin_errors


class FakeNav(Node):
    def __init__(self, blocked='', fail_plan=False, publish_map=True, zero_clock=False):
        super().__init__('fake_nav')
        self.blocked = blocked
        self.fail_plan = fail_plan
        self.ignore_cancel = False
        self.started = {name: threading.Event() for name in ('planning', 'navigating', 'spinning')}
        self.canceled = {name: threading.Event() for name in self.started}
        self.active = set()
        self.counts = {name: 0 for name in self.started}
        self.group = ReentrantCallbackGroup()
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', durable_qos())
        self.cubes_pub = self.create_publisher(CubeArray, '/cubes', durable_qos())
        self.tf = StaticTransformBroadcaster(self)
        transforms = []
        self.dynamic_tf = TransformBroadcaster(self)
        for frame in ['base_footprint', 'camera_optical_frame']:
            t = TransformStamped()
            t.header.frame_id = 'map'
            t.child_frame_id = frame
            t.header.stamp = self.get_clock().now().to_msg()
            t.transform.translation.x = 2.0 if frame == 'base_footprint' else 1.0
            t.transform.translation.y = 2.0
            t.transform.rotation.w = 1.0
            if frame == 'base_footprint' and not zero_clock:
                self.base_tf = t
                self.create_timer(.05, self.publish_base_tf)
            else:
                transforms.append(t)
        self.tf.sendTransform(transforms)
        self.map = OccupancyGrid()
        self.map.header.frame_id = 'map'
        self.map.info.width = self.map.info.height = 8
        self.map.info.resolution = .5
        self.map.info.origin.orientation.w = 1.0
        self.map.data = [0] * 64
        if publish_map:
            self.map_pub.publish(self.map)
        self.publish_cubes(0)
        self.servers = []
        for action_type, name, phase in [
                (ComputePathToPose, '/compute_path_to_pose', 'planning'),
                (NavigateToPose, '/navigate_to_pose', 'navigating'),
                (Spin, '/spin', 'spinning')]:
            self.servers.append(ActionServer(
                self, action_type, name,
                execute_callback=lambda h, t=action_type, p=phase: self.execute(h, t, p),
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=self.group))
        self.find = ActionClient(self, FindCubes, '/find_cubes', callback_group=self.group)

    def publish_base_tf(self):
        self.base_tf.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_tf.sendTransform(self.base_tf)

    def publish_cubes(self, count):
        array = CubeArray()
        array.header.frame_id = 'map'
        array.header.stamp = self.get_clock().now().to_msg()
        for index in range(count):
            cube = Cube()
            cube.id = index + 10
            cube.position.x = float(index)
            cube.position.y = 2.0
            cube.observations = 5
            array.cubes.append(cube)
        self.cubes_pub.publish(array)

    def execute(self, handle, action_type, phase):
        self.active.add(phase)
        self.counts[phase] += 1
        self.started[phase].set()
        try:
            while self.blocked == phase and rclpy.ok():
                if handle.is_cancel_requested and not self.ignore_cancel:
                    self.canceled[phase].set()
                    handle.canceled()
                    return action_type.Result()
                time.sleep(.01)
            result = action_type.Result()
            if action_type is ComputePathToPose:
                if self.fail_plan:
                    handle.abort()
                    return result
                result.path.poses = [deepcopy(handle.request.goal)]
            if action_type is Spin:
                assert handle.request.target_yaw == pytest.approx(2 * math.pi)
            handle.succeed()
            return result
        finally:
            self.active.discard(phase)


def start_search(runtime, blocked='', fail_plan=False, publish_map=True, zero_clock=False, **overrides):
    params = dict(startup_timeout=1.0, stop_timeout=.6, planning_timeout=2.0,
                  navigation_timeout=2.0, spin_timeout=2.0, pose_max_age=600.0,
                  grid_spacing=2.0, default_timeout=5.0)
    params.update(overrides)
    search = runtime(CubeSearchServer(parameter_overrides=[Parameter(k, value=v)
                                                          for k, v in params.items()]))
    nav = runtime(FakeNav(blocked, fail_plan, publish_map, zero_clock))
    assert nav.find.wait_for_server(timeout_sec=3)
    wait_until(lambda: search.map_message is not None if publish_map else True)
    return search, nav


def send_goal(nav, timeout=4.0):
    goal = FindCubes.Goal()
    goal.timeout_sec = timeout
    handle = future_result(nav.find.send_goal_async(goal))
    assert handle.accepted
    return handle


@pytest.mark.parametrize('phase', ['planning', 'navigating', 'spinning'])
def test_five_ids_during_each_phase_cancel_active_nav2_goal(runtime, phase):
    search, nav = start_search(runtime, blocked=phase)
    nav.publish_cubes(4)
    wait_until(lambda: len(search.cubes) == 4)
    handle = send_goal(nav)
    assert nav.started[phase].wait(3)
    nav.publish_cubes(5)
    result = future_result(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.code == FindCubes.Result.SUCCESS
    assert len({c.id for c in result.result.cubes.cubes}) == 5
    assert result.result.cubes.header.frame_id == 'map'
    assert nav.canceled[phase].is_set()
    assert not nav.active


@pytest.mark.parametrize('phase', ['planning', 'navigating', 'spinning'])
def test_cancel_during_each_phase_and_reject_concurrent_goal(runtime, phase):
    search, nav = start_search(runtime, blocked=phase)
    handle = send_goal(nav)
    assert nav.started[phase].wait(3)
    second = future_result(nav.find.send_goal_async(FindCubes.Goal()))
    assert not second.accepted
    response = future_result(handle.cancel_goal_async())
    assert response.goals_canceling
    result = future_result(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED
    assert result.result.code == FindCubes.Result.CANCELED
    assert nav.canceled[phase].is_set()
    assert not nav.active
    wait_until(lambda: not search.busy)


def test_mission_timeout_returns_partial_and_cancels_navigation(runtime):
    search, nav = start_search(runtime, blocked='navigating')
    nav.publish_cubes(2)
    wait_until(lambda: len(search.cubes) == 2)
    handle = send_goal(nav, timeout=.7)
    result = future_result(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.code == FindCubes.Result.TIMEOUT
    assert len(result.result.cubes.cubes) == 2
    assert nav.canceled['navigating'].is_set()
    assert not nav.active


def test_unreachable_points_are_exhausted_without_navigation(runtime):
    _, nav = start_search(runtime, fail_plan=True)
    handle = send_goal(nav)
    result = future_result(handle.get_result_async())
    assert result.result.code == FindCubes.Result.EXHAUSTED
    assert nav.counts['planning'] > 0
    assert nav.counts['navigating'] == 0


def test_successful_visits_and_spins_but_missing_cubes_exhaust_grid(runtime):
    _, nav = start_search(runtime)
    result = future_result(send_goal(nav).get_result_async())
    assert result.result.code == FindCubes.Result.EXHAUSTED
    assert nav.counts['spinning'] > 0
    assert nav.counts['navigating'] == nav.counts['spinning']


def test_navigation_timeout_cancels_before_next_point(runtime):
    _, nav = start_search(runtime, blocked='navigating', navigation_timeout=.12)
    result = future_result(send_goal(nav).get_result_async())
    assert result.result.code == FindCubes.Result.EXHAUSTED
    assert nav.canceled['navigating'].is_set()
    assert nav.counts['navigating'] > 1
    assert not nav.active


def test_missing_map_and_map_changed(runtime):
    search, nav = start_search(runtime, publish_map=False, blocked='navigating')
    result = future_result(send_goal(nav).get_result_async())
    assert result.result.code == FindCubes.Result.MAP_ERROR
    nav.map_pub.publish(nav.map)
    wait_until(lambda: search.map_message is not None)
    handle = send_goal(nav)
    assert nav.started['navigating'].wait(3)
    nav.map.data[0] = 100
    nav.map_pub.publish(nav.map)
    result = future_result(handle.get_result_async())
    assert result.result.code == FindCubes.Result.MAP_ERROR
    assert nav.canceled['navigating'].is_set()


@pytest.mark.parametrize('timeout', [-1.0, math.nan, math.inf])
def test_invalid_goal_rejected(runtime, timeout):
    _, nav = start_search(runtime)
    goal = FindCubes.Goal()
    goal.timeout_sec = timeout
    assert not future_result(nav.find.send_goal_async(goal)).accepted


def test_detector_and_localizer_with_real_ros_messages_and_reset(runtime):
    detector = runtime(ArucoDetector(parameter_overrides=[Parameter('marker_size', value=.12)]))
    localizer = runtime(CubeLocalizer(parameter_overrides=[
        Parameter('min_observations', value=3), Parameter('min_interval', value=.03),
        Parameter('min_duration', value=.08)]))
    harness = runtime(Node('camera_test'))
    tf = StaticTransformBroadcaster(harness)
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'camera_rgb_optical_frame'
    transform.header.stamp = harness.get_clock().now().to_msg()
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 2.0
    # Model the existing optical TF: optical Z points along robot X.
    transform.transform.rotation.x = -.5
    transform.transform.rotation.y = .5
    transform.transform.rotation.z = -.5
    transform.transform.rotation.w = .5
    tf.sendTransform(transform)
    images = harness.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
    infos = harness.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data)
    output = []
    detections = []
    harness.create_subscription(CubeArray, '/cubes', output.append, durable_qos())
    harness.create_subscription(ArucoDetectionArray, '/aruco/detections', detections.append, 10)
    info = CameraInfo()
    # The driver labels both messages with the mechanical camera frame.
    info.header.frame_id = 'camera_rgb_frame'
    info.width, info.height = 640, 480
    info.distortion_model = 'plumb_bob'
    info.k = [600., 0., 320., 0., 600., 240., 0., 0., 1.]
    info.d = [0.] * 5
    image = CvBridge().cv2_to_imgmsg(marker_image(), encoding='mono8')
    image.header.frame_id = info.header.frame_id
    wait_until(lambda: images.get_subscription_count() > 0 and infos.get_subscription_count() > 0)
    image_stamps = set()
    for _ in range(12):
        infos.publish(info)
        image.header.stamp = harness.get_clock().now().to_msg()
        image_stamps.add((image.header.stamp.sec, image.header.stamp.nanosec))
        images.publish(image)
        time.sleep(.06)
    wait_until(lambda: any(msg.cubes for msg in output))
    wait_until(lambda: bool(detections))
    for message in detections:
        assert message.header.frame_id == 'camera_rgb_optical_frame'
        assert (message.header.stamp.sec, message.header.stamp.nanosec) in image_stamps
    cube = [msg for msg in output if msg.cubes][-1].cubes[0]
    assert cube.id == 17
    assert cube.position.x == pytest.approx(1.402, abs=.01)
    assert cube.position.y == pytest.approx(2.0, abs=.01)
    assert cube.position.z == pytest.approx(0.0, abs=.01)
    assert cube.observations >= 3
    reset = harness.create_client(Trigger, '/cube_localizer/reset')
    assert reset.wait_for_service(timeout_sec=2)
    assert future_result(reset.call_async(Trigger.Request())).success
    wait_until(lambda: output and not output[-1].cubes)
    assert not localizer.estimator.confirmed()
    assert detector.calibration is not None


def test_delayed_tf_and_marker_offset(runtime):
    localizer = runtime(CubeLocalizer(parameter_overrides=[
        Parameter('min_observations', value=3), Parameter('min_interval', value=.03),
        Parameter('min_duration', value=.08), Parameter('tf_wait_timeout', value=1.0),
        Parameter('marker_to_cube_offset', value=[0.0, 0.0, -.05])]))
    harness = runtime(Node('delayed_tf_test'))
    pub = harness.create_publisher(ArucoDetectionArray, '/aruco/detections', 10)
    wait_until(lambda: pub.get_subscription_count() > 0)
    for _ in range(3):
        msg = ArucoDetectionArray()
        msg.header.frame_id = 'delayed_camera'
        msg.header.stamp = harness.get_clock().now().to_msg()
        detection = ArucoDetection()
        detection.id = 4
        detection.pose.position.z = 1.0
        detection.pose.orientation.w = 1.0
        msg.detections = [detection, deepcopy(detection)]  # Same-frame duplicate.
        pub.publish(msg)
        time.sleep(.06)
    assert not localizer.estimator.confirmed()
    tf = StaticTransformBroadcaster(harness)
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'delayed_camera'
    transform.transform.rotation.z = math.sin(math.pi / 4)
    transform.transform.rotation.w = math.cos(math.pi / 4)
    transform.transform.translation.x = 2.0
    tf.sendTransform(transform)
    wait_until(lambda: 4 in localizer.estimator.confirmed())
    position, count = localizer.estimator.confirmed()[4]
    assert position == pytest.approx([2.0, 0.0, .95])
    assert count == 3


def test_stop_failure_disables_new_missions(runtime):
    search, nav = start_search(runtime, blocked='navigating', stop_timeout=.15)
    nav.ignore_cancel = True
    handle = send_goal(nav, timeout=.5)
    assert nav.started['navigating'].wait(3)
    result = future_result(handle.get_result_async())
    assert result.result.code == FindCubes.Result.STOP_FAILED
    assert search.motion_unresolved
    assert not future_result(nav.find.send_goal_async(FindCubes.Goal())).accepted
    nav.blocked = ''  # Let the uncooperative fake server terminate during teardown.
    wait_until(lambda: not nav.active)


def test_wall_timeout_with_paused_simulation_clock(runtime):
    search, nav = start_search(runtime, blocked='navigating', zero_clock=True, use_sim_time=True)
    started = time.monotonic()
    result = future_result(send_goal(nav, timeout=.6).get_result_async())
    assert search.get_clock().now().nanoseconds == 0
    assert result.result.code == FindCubes.Result.TIMEOUT
    assert time.monotonic() - started < 2.5
    assert nav.canceled['navigating'].is_set()


def test_transform_uses_measurement_time_instead_of_latest_tf(runtime):
    localizer = runtime(CubeLocalizer(parameter_overrides=[
        Parameter('min_observations', value=3), Parameter('min_interval', value=.03),
        Parameter('min_duration', value=.08)]))
    harness = runtime(Node('timestamp_test'))
    tf = TransformBroadcaster(harness)
    pub = harness.create_publisher(ArucoDetectionArray, '/aruco/detections', 10)
    wait_until(lambda: pub.get_subscription_count() > 0 and
               tf.pub_tf.get_subscription_count() > 0)
    current = harness.get_clock().now().nanoseconds
    oldest = current - 400_000_000
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'moving_camera'
    transform.transform.rotation.w = 1.0
    for stamp, x in [(oldest, 1.0), (current, 9.0)]:
        transform.header.stamp = Time(nanoseconds=stamp).to_msg()
        transform.transform.translation.x = x
        tf.sendTransform(transform)
    time.sleep(.05)
    for index in range(3):
        stamp = oldest + index * 60_000_000
        camera_x = 1.0 + (stamp - oldest) / (current - oldest) * 8.0
        msg = ArucoDetectionArray()
        msg.header.frame_id = 'moving_camera'
        msg.header.stamp = Time(nanoseconds=stamp).to_msg()
        d = ArucoDetection()
        d.id = 42
        d.pose.position.x = 2.0 - camera_x
        d.pose.position.z = 1.0
        d.pose.orientation.w = 1.0
        msg.detections = [d]
        pub.publish(msg)
    wait_until(lambda: 42 in localizer.estimator.confirmed())
    assert localizer.estimator.confirmed()[42][0] == pytest.approx([2.0, 0.0, 1.0], abs=1e-5)


@pytest.mark.parametrize('map_translation,map_yaw,expected', [
    ((0., 0.), 0., (2.84626, .18484, .00787)),
    ((1., 2.), math.pi / 2, (.81516, 4.84626, .00787)),
])
def test_reported_optical_measurement_maps_to_floor(runtime, map_translation, map_yaw, expected):
    # Keep the production confirmation thresholds: five readings over >= 0.8 s.
    runtime(CubeLocalizer())
    harness = runtime(Node('optical_regression_test'))
    tf = StaticTransformBroadcaster(harness)
    transforms = []
    base = TransformStamped()
    base.header.frame_id, base.child_frame_id = 'map', 'base_footprint'
    base.transform.translation.x, base.transform.translation.y = map_translation
    base.transform.rotation.z = math.sin(map_yaw / 2)
    base.transform.rotation.w = math.cos(map_yaw / 2)
    transforms.append(base)
    camera = TransformStamped()
    camera.header.frame_id, camera.child_frame_id = 'base_footprint', 'camera_rgb_frame'
    camera.transform.translation.x = .069
    camera.transform.translation.y = -.047
    camera.transform.translation.z = .117
    camera.transform.rotation.w = 1.0
    transforms.append(camera)
    optical = TransformStamped()
    optical.header.frame_id, optical.child_frame_id = 'camera_rgb_frame', 'camera_rgb_optical_frame'
    optical.transform.rotation.x = -.5
    optical.transform.rotation.y = .5
    optical.transform.rotation.z = -.5
    optical.transform.rotation.w = .5
    transforms.append(optical)
    # These transforms exist only in the test fixture, standing in for robot TF.
    tf.sendTransform(transforms)
    pub = harness.create_publisher(ArucoDetectionArray, '/aruco/detections', 10)
    output = []
    harness.create_subscription(CubeArray, '/cubes', output.append, durable_qos())
    wait_until(lambda: pub.get_subscription_count() > 0)
    for _ in range(5):
        message = ArucoDetectionArray()
        message.header.frame_id = 'camera_rgb_optical_frame'
        message.header.stamp = harness.get_clock().now().to_msg()
        detection = ArucoDetection()
        detection.id = 17
        detection.pose.position.x = -.23184
        detection.pose.position.y = .10913
        detection.pose.position.z = 2.77726
        detection.pose.orientation.x = 1.0
        detection.pose.orientation.w = 0.0
        message.detections = [detection]
        pub.publish(message)
        time.sleep(.21)
    wait_until(lambda: any(message.cubes for message in output))
    result = [message for message in output if message.cubes][-1]
    assert result.header.frame_id == 'map'
    assert len(result.cubes) == 1
    cube = result.cubes[0]
    assert cube.observations == 5
    assert [cube.position.x, cube.position.y, cube.position.z] == pytest.approx(expected, abs=1e-6)
