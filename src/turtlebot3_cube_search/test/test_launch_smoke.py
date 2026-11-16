"""Exercise the installed executables, YAML, launch and repeated SIGINT cleanup."""

import os
import signal
import subprocess
import time

from cube_search_interfaces.action import FindCubes
import rclpy
import pytest
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient


@pytest.mark.parametrize('optical_frame', ['camera_rgb_optical_frame', 'front_camera_optical_frame'])
def test_launch_discovers_all_nodes_and_shuts_down_cleanly(optical_frame):
    frame_args = ([] if optical_frame == 'camera_rgb_optical_frame'
                  else ['optical_frame:=' + optical_frame])
    process = subprocess.Popen([
        'ros2', 'launch', 'turtlebot3_cube_search', 'cube_search.launch.py',
        'marker_size:=0.08', 'use_sim_time:=false'] + frame_args,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True)
    rclpy.init()
    probe = Node('launch_probe')
    client = ActionClient(probe, FindCubes, '/find_cubes')
    output = ''
    try:
        expected = {'aruco_detector', 'cube_localizer', 'cube_search_server'}
        names = set()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=.1)
            names = set(probe.get_node_names())
            if expected <= names and client.server_is_ready():
                break
            assert process.poll() is None, 'Launch exited before discovery'
        assert expected <= names
        assert client.server_is_ready()
        parameters = AsyncParameterClient(probe, '/aruco_detector')
        assert parameters.wait_for_services(timeout_sec=2)
        future = parameters.get_parameters(['optical_frame'])
        rclpy.spin_until_future_complete(probe, future, timeout_sec=2)
        assert future.done()
        assert future.result().values[0].string_value == optical_frame
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=18)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate()
            raise AssertionError('Launch did not terminate cleanly:\n' + output)
        finally:
            probe.destroy_node()
            rclpy.shutdown()
    assert 'Traceback' not in output, output
    assert 'process has died' not in output, output
    assert output.count('process has finished cleanly') == 3, output
