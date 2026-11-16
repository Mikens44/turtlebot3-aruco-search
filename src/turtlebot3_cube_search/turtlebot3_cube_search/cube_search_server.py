"""One FindCubes mission at a time: grid, path validation, navigate and spin."""

from copy import deepcopy
import math
import threading
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from cube_search_interfaces.action import FindCubes
from cube_search_interfaces.msg import CubeArray
from .cube_localizer import durable_qos
from .nav_tasks import ActionTask
from .runtime import run_node
from .search_grid import GridGeometry, reachable_grid


class MissionEnd(Exception):
    def __init__(self, code, message, cubes=None):
        super().__init__(message)
        self.code = code
        self.cubes = cubes


class CubeSearchServer(Node):
    TARGET_COUNT = 5

    def __init__(self, **kwargs):
        super().__init__('cube_search_server', **kwargs)
        defaults = {
            'map_frame': 'map', 'base_frame': 'base_footprint',
            'grid_spacing': 1.0, 'robot_radius': 0.30, 'safety_margin': 0.10,
            'free_threshold': 20, 'default_timeout': 600.0,
            'startup_timeout': 15.0, 'planning_timeout': 10.0,
            'navigation_timeout': 90.0, 'spin_timeout': 45.0,
            'stop_timeout': 5.0, 'pose_max_age': 1.0, 'planner_id': '',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        for key, value in self.p.items():
            if isinstance(value, float) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{key} must be finite and positive')
        if not 0 <= self.p['free_threshold'] <= 100:
            raise ValueError('free_threshold must be between 0 and 100')
        if not self.p['map_frame'] or not self.p['base_frame']:
            raise ValueError('TF frames cannot be empty')
        self.group = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.busy = False
        self.stopping = False
        self.motion_unresolved = False
        self.active_task = None
        self.map_message = None
        self.map_revision = 0
        self.cubes = {}
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, durable_qos(),
                                 callback_group=self.group)
        self.create_subscription(CubeArray, '/cubes', self.on_cubes, durable_qos(),
                                 callback_group=self.group)
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose',
                                    callback_group=self.group)
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose',
                                      callback_group=self.group)
        self.spinner = ActionClient(self, Spin, '/spin', callback_group=self.group)
        self.server = ActionServer(
            self, FindCubes, '/find_cubes', execute_callback=self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            callback_group=self.group)

    def on_map(self, message):
        with self.lock:
            previous = self.map_message
            # Repeated publication of the same static map is harmless.
            if (previous is None or previous.info != message.info or
                    previous.header.frame_id != message.header.frame_id or
                    previous.data != message.data):
                self.map_revision += 1
            self.map_message = message

    def on_cubes(self, message):
        if message.header.frame_id != self.p['map_frame']:
            self.get_logger().warning('Ignoring /cubes in wrong frame', throttle_duration_sec=5.0)
            return
        cubes = {}
        for cube in message.cubes:
            p = cube.position
            if (0 <= cube.id < 1024 and cube.observations > 0 and
                    all(math.isfinite(v) for v in (p.x, p.y, p.z))):
                cubes[cube.id] = deepcopy(cube)
        with self.lock:
            self.cubes = cubes

    def on_goal(self, request):
        with self.lock:
            if (self.busy or self.stopping or self.motion_unresolved or
                    not math.isfinite(request.timeout_sec) or request.timeout_sec < 0):
                return GoalResponse.REJECT
            self.busy = True  # Reserve before execute is scheduled.
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def cube_result(self):
        output = CubeArray()
        output.header.frame_id = self.p['map_frame']
        output.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            output.cubes = [deepcopy(self.cubes[i]) for i in sorted(self.cubes)[:self.TARGET_COUNT]]
        return output

    def check_mission(self, goal_handle, deadline, revision=None):
        if goal_handle.is_cancel_requested or self.stopping:
            raise MissionEnd(FindCubes.Result.CANCELED, 'Search canceled')
        with self.lock:
            count = len(self.cubes)
            changed = revision is not None and revision != self.map_revision
            found = self.cube_result() if count >= self.TARGET_COUNT else None
        if changed:
            raise MissionEnd(FindCubes.Result.MAP_ERROR, 'Map changed; start a new mission')
        if time.monotonic() >= deadline:
            raise MissionEnd(FindCubes.Result.TIMEOUT, 'Mission wall-clock limit exceeded')
        if found is not None:
            raise MissionEnd(FindCubes.Result.SUCCESS, 'Confirmed five distinct marker IDs', found)

    def feedback(self, handle, phase, visited=0, total=0):
        feedback = FindCubes.Feedback()
        with self.lock:
            feedback.found_count = min(len(self.cubes), self.TARGET_COUNT)
        feedback.visited_points = visited
        feedback.total_points = total
        feedback.phase = phase
        handle.publish_feedback(feedback)

    def current_xy(self):
        transform = self.tf_buffer.lookup_transform(
            self.p['map_frame'], self.p['base_frame'], Time())
        stamp = Time.from_msg(transform.header.stamp).nanoseconds
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if not -0.1 <= age <= self.p['pose_max_age']:
            raise ValueError('Robot TF is stale or from a future clock')
        t = transform.transform.translation
        if not math.isfinite(t.x) or not math.isfinite(t.y):
            raise ValueError('Invalid robot position')
        return (t.x, t.y)

    def observation_points(self, message, xy):
        info = message.info
        if (message.header.frame_id != self.p['map_frame'] or
                info.width == 0 or info.height == 0 or
                len(message.data) != info.width * info.height):
            raise ValueError('Map frame, dimensions or data length invalid')
        q = info.origin.orientation
        values = [info.resolution, info.origin.position.x, info.origin.position.y,
                  q.x, q.y, q.z, q.w]
        if (not all(math.isfinite(v) for v in values) or
                abs(math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) - 1) > 0.01 or
                abs(q.x) > 1e-6 or abs(q.y) > 1e-6):
            raise ValueError('Map must have a finite planar origin and unit quaternion')
        yaw = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
        geometry = GridGeometry(info.resolution, info.origin.position.x,
                                info.origin.position.y, yaw)
        occupancy = np.asarray(message.data).reshape(info.height, info.width)
        return reachable_grid(occupancy, geometry, xy, self.p['grid_spacing'],
                              self.p['robot_radius'] + self.p['safety_margin'],
                              self.p['free_threshold'])

    def run_task(self, client, request, timeout, handle, deadline, revision, phase, visited, total):
        self.check_mission(handle, deadline, revision)
        if not client.server_is_ready():
            raise MissionEnd(FindCubes.Result.DEPENDENCY_ERROR, 'Nav2 action server disappeared')
        task = ActionTask(client, request)
        with self.lock:
            self.active_task = task
        task_deadline = min(deadline, time.monotonic() + timeout)
        next_feedback = 0.0
        try:
            while not task.done.is_set():
                self.check_mission(handle, deadline, revision)
                if time.monotonic() >= task_deadline:
                    return None
                if time.monotonic() >= next_feedback:
                    self.feedback(handle, phase, visited, total)
                    next_feedback = time.monotonic() + 0.5
                task.done.wait(0.02)
            self.check_mission(handle, deadline, revision)
            if task.succeeded():
                return task.response.result
            self.get_logger().warning(f'{phase}: Nav2 goal rejected or failed ({task.error})')
            return None
        finally:
            if not task.cancel_and_wait(self.p['stop_timeout']):
                self.motion_unresolved = True
                raise MissionEnd(FindCubes.Result.STOP_FAILED,
                                 'Nav2 terminal state unconfirmed; new missions disabled')
            with self.lock:
                self.active_task = None

    def execute(self, handle):
        deadline = time.monotonic() + (handle.request.timeout_sec or self.p['default_timeout'])
        result = FindCubes.Result()
        result.code = FindCubes.Result.INTERNAL_ERROR
        result.message = 'Mission did not finish'
        visited = 0
        total = 0
        try:
            startup_deadline = min(deadline, time.monotonic() + self.p['startup_timeout'])
            while True:
                self.check_mission(handle, deadline)
                with self.lock:
                    message = self.map_message
                    revision = self.map_revision
                ready = all(c.server_is_ready() for c in (self.planner, self.navigator, self.spinner))
                xy = None
                try:
                    xy = self.current_xy()
                except (TransformException, ValueError):
                    pass
                if message is not None and ready and xy is not None:
                    break
                if time.monotonic() >= startup_deadline:
                    if message is None:
                        raise MissionEnd(FindCubes.Result.MAP_ERROR, 'No /map received')
                    if not ready:
                        raise MissionEnd(FindCubes.Result.DEPENDENCY_ERROR,
                                         'Nav2 planner, navigator and Spin must be active')
                    raise MissionEnd(FindCubes.Result.TF_ERROR, 'No fresh map -> base TF')
                self.feedback(handle, 'waiting_for_dependencies')
                time.sleep(0.05)
            self.feedback(handle, 'generating_grid')
            try:
                points = self.observation_points(message, xy)
            except ValueError as error:
                raise MissionEnd(FindCubes.Result.MAP_ERROR, str(error)) from error
            total = len(points)
            while points:
                self.check_mission(handle, deadline, revision)
                try:
                    xy = self.current_xy()
                except (TransformException, ValueError) as error:
                    raise MissionEnd(FindCubes.Result.TF_ERROR, str(error)) from error
                index = min(range(len(points)), key=lambda i: math.dist(xy, points[i]))
                x, y = points.pop(index)
                pose = PoseStamped()
                pose.header.frame_id = self.p['map_frame']
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x, pose.pose.position.y = x, y
                yaw = math.atan2(y - xy[1], x - xy[0])
                pose.pose.orientation.z = math.sin(yaw / 2)
                pose.pose.orientation.w = math.cos(yaw / 2)
                plan = ComputePathToPose.Goal()
                plan.goal = pose
                plan.use_start = False
                plan.planner_id = self.p['planner_id']
                planned = self.run_task(self.planner, plan, self.p['planning_timeout'], handle,
                                        deadline, revision, 'planning', visited, total)
                if planned is not None and planned.path.poses:
                    navigate = NavigateToPose.Goal()
                    navigate.pose = pose
                    arrived = self.run_task(
                        self.navigator, navigate, self.p['navigation_timeout'], handle,
                        deadline, revision, 'navigating', visited, total)
                    if arrived is not None:
                        spin = Spin.Goal()
                        spin.target_yaw = 2.0 * math.pi
                        spin.time_allowance.sec = int(self.p['spin_timeout'])
                        spin.time_allowance.nanosec = int((self.p['spin_timeout'] % 1) * 1e9)
                        self.run_task(self.spinner, spin, self.p['spin_timeout'], handle,
                                      deadline, revision, 'spinning', visited, total)
                visited += 1  # Attempted points, including unreachable/skipped ones.
                self.feedback(handle, 'point_finished', visited, total)
            self.check_mission(handle, deadline, revision)
            raise MissionEnd(FindCubes.Result.EXHAUSTED, 'All observation points attempted')
        except MissionEnd as end:
            result.code = end.code
            result.message = str(end)
            if end.cubes is not None:
                result.cubes = end.cubes
        except Exception as error:
            self.get_logger().error(f'Mission exception: {error!r}')
            result.code = FindCubes.Result.INTERNAL_ERROR
            result.message = str(error)
        finally:
            with self.lock:
                task = self.active_task
            if task is not None:
                if not task.cancel_and_wait(self.p['stop_timeout']):
                    self.motion_unresolved = True
                    result.code = FindCubes.Result.STOP_FAILED
                    result.message = 'Cannot confirm Nav2 stopped; restart server after recovery'
                else:
                    with self.lock:
                        self.active_task = None
            if result.code != FindCubes.Result.SUCCESS:
                result.cubes = self.cube_result()
            # A cancellation received just before result publication still wins.
            if handle.is_cancel_requested and result.code != FindCubes.Result.STOP_FAILED:
                result.code = FindCubes.Result.CANCELED
                result.message = 'Search canceled'
            if handle.is_cancel_requested:
                handle.canceled()
            elif result.code == FindCubes.Result.SUCCESS:
                handle.succeed()
            else:
                handle.abort()
            with self.lock:
                self.busy = False
        return result

    def stop(self):
        self.stopping = True
        with self.lock:
            task = self.active_task
        if task is not None:
            task.cancel()


def main(args=None):
    run_node(CubeSearchServer, args, threads=6)
