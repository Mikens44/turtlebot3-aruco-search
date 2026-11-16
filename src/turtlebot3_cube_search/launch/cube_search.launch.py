"""Start visual search alongside an already running robot, localization and Nav2."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('turtlebot3_cube_search'),
                          'config', 'search.yaml')
    arguments = [
        DeclareLaunchArgument('marker_size', description='Measured marker side in metres'),
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('optical_frame', default_value='camera_rgb_optical_frame',
                              description='Existing camera optical TF frame for OpenCV poses'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('navigate_action', default_value='/navigate_to_pose'),
        DeclareLaunchArgument('planner_action', default_value='/compute_path_to_pose'),
        DeclareLaunchArgument('spin_action', default_value='/spin'),
    ]
    common = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    parameters = LaunchConfiguration('params_file')
    return LaunchDescription(arguments + [
        Node(package='turtlebot3_cube_search', executable='aruco_detector', output='screen',
             parameters=[parameters, common, {
                 'marker_size': ParameterValue(LaunchConfiguration('marker_size'), value_type=float),
                 'optical_frame': ParameterValue(LaunchConfiguration('optical_frame'), value_type=str)}],
             remappings=[('/camera/image_raw', LaunchConfiguration('image_topic')),
                         ('/camera/camera_info', LaunchConfiguration('camera_info_topic'))]),
        Node(package='turtlebot3_cube_search', executable='cube_localizer', output='screen',
             parameters=[parameters, common, {'map_frame': LaunchConfiguration('map_frame')}]),
        Node(package='turtlebot3_cube_search', executable='cube_search_server', output='screen',
             sigterm_timeout='15.0',
             parameters=[parameters, common, {'map_frame': LaunchConfiguration('map_frame'),
                                              'base_frame': LaunchConfiguration('base_frame')}],
             remappings=[('/map', LaunchConfiguration('map_topic')),
                         ('/navigate_to_pose', LaunchConfiguration('navigate_action')),
                         ('/compute_path_to_pose', LaunchConfiguration('planner_action')),
                         ('/spin', LaunchConfiguration('spin_action'))]),
    ])
