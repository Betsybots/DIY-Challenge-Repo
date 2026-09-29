"""Launch the RED/GREEN colour detector.

Examples (ROS 2 Humble, after sourcing /opt/ros/humble and the workspace):
  ros2 launch diy_zed_color_detection color_detector.launch.py
  ros2 launch diy_zed_color_detection color_detector.launch.py image_topic:=/camera/image_raw
  ros2 launch diy_zed_color_detection color_detector.launch.py image_topic:=/zed2i/zed_node/rgb/image_rect_color
  ros2 launch diy_zed_color_detection color_detector.launch.py params_file:=/path/to/my_params.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('diy_zed_color_detection'), 'config', 'params.yaml')
    zed_wrapper_launch = os.path.join(
        get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic', default_value='/zed/zed_node/rgb/color/rect/image',
            description='Colour image topic (default: ZED ROS 2 wrapper left rectified image)'),
        DeclareLaunchArgument(
            'qos_reliable', default_value='false',
            description='Subscribe RELIABLE instead of BEST_EFFORT (best effort accepts both)'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML file with detector thresholds'),
        DeclareLaunchArgument(
            'camera_model', default_value='zed2i',
            description='ZED camera model passed to zed_wrapper'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_wrapper_launch),
            launch_arguments={'camera_model': LaunchConfiguration('camera_model')}.items(),
        ),
        Node(
            package='diy_zed_color_detection',
            executable='color_detector_node',
            name='color_detector',
            output='screen',
            # The launch arguments override the same keys in params_file.
            parameters=[LaunchConfiguration('params_file'), {
                'image_topic': LaunchConfiguration('image_topic'),
                'qos_reliable': ParameterValue(LaunchConfiguration('qos_reliable'), value_type=bool),
            }],
        ),
    ])
