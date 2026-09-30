import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The controller starts disabled and publishes zero velocity. After verifying
# /mapless_wall_follower/state and the configured command topic, enable it with:
# ros2 topic pub --once /mapless_wall_follower/enable std_msgs/msg/Bool "{data: true}"
# Disable it and command a controlled stop with:
# ros2 topic pub --once /mapless_wall_follower/enable std_msgs/msg/Bool "{data: false}"
# Use the physical emergency stop when an immediate hardware stop is required.
# ros2 launch mapless_wall_follower \
#   mapless_wall_follower_ackermann.launch.py \
#   cmd_vel_topic:=/cmd_vel_smoothed


def generate_launch_description():
    package_dir = get_package_share_directory('mapless_wall_follower')
    params_file = LaunchConfiguration('params_file')
    cloud_topic = LaunchConfiguration('cloud_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                package_dir,
                'config',
                'mapless_wall_follower_ackermann.yaml',
            ),
            description='Mapless wall-follower parameter file',
        ),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/cloud_registered_body',
            description='PointCloud2 topic in the base_link frame',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel_nav',
            # Production: wall follower -> /cmd_vel_nav -> safety/mux/smoother
            # -> /cmd_vel_smoothed -> Ackermann driver.
            # Direct test without a mux/smoother: the current Ackermann driver
            # listens on /cmd_vel_smoothed, so launch with:
            #   cmd_vel_topic:=/cmd_vel_smoothed
            description='Autonomous Twist output; override for direct driver testing',
        ),
        Node(
            package='mapless_wall_follower',
            executable='mapless_wall_follower_node',
            name='mapless_wall_follower',
            output='screen',
            parameters=[
                params_file,
                {
                    'cloud_topic': cloud_topic,
                    'cmd_vel_topic': cmd_vel_topic,
                },
            ],
        ),
    ])
