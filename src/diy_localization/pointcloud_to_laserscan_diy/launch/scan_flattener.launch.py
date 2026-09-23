import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pointcloud_to_laserscan_diy'),
        'config', 'scan_flattener.yaml')

    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan_diy',
            executable='scan_flattener_node',
            name='scan_flattener_node',
            output='screen',
            parameters=[config],
        ),
    ])
