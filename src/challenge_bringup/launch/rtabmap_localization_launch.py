import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Launches RTAB-Map in localization mode (replaces map_localizer).

    Lidar-only: no camera. Expects FAST-LIO2 (fast_lio_ros2) and the EKF
    (diy_state_estimate) to already be running, publishing
    /cloud_registered_body (frame_id base_link) and /odom (odom->base_footprint),
    same inputs map_localizer used. Requires database_path to already contain a
    map built by a prior rtabmap_mapping_launch.py run -- this node only
    localizes against a saved graph, it never builds one.
    """
    config_path = PathJoinSubstitution(
        [get_package_share_directory('challenge_bringup'), 'config', 'rtabmap_localization.yaml']
    )

    database_path = LaunchConfiguration('database_path')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'database_path',
            # Same default path rtabmap_mapping_launch.py saves to, so an
            # autonomous:=true run after a autonomous:=false mapping run picks
            # up that map automatically.
            default_value=os.path.join(
                get_package_share_directory('challenge_bringup'), 'maps', 'rtabmap.db'
            ),
            description='Path to the map database built by a prior mapping run.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch rtabmap_viz alongside the localization node',
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap',
            output='screen',
            # No '-d': never delete the map being localized against.
            parameters=[config_path, {'database_path': database_path}],
            remappings=[
                ('scan_cloud', '/cloud_registered_body'),
                ('odom', '/odom'),
            ],
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            namespace='rtabmap',
            output='screen',
            condition=IfCondition(use_rviz),
            parameters=[config_path, {'database_path': database_path}],
            remappings=[
                ('scan_cloud', '/cloud_registered_body'),
                ('odom', '/odom'),
            ],
        ),
    ])
