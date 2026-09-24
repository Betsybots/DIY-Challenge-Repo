import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Launches the RTAB-Map mapping backend (replaces loop_pgo).

    Lidar-only: no camera. Expects FAST-LIO2 (fast_lio_ros2) and the EKF
    (diy_state_estimate) to already be running, publishing
    /cloud_registered_body (frame_id base_link) and /odom (odom->base_footprint)
    respectively, same inputs loop_pgo used. RTAB-Map does its own ICP-based
    loop closure + graph optimization on top and publishes the map->odom TF.

    Database is saved to database_path incrementally while running (and
    flushed on clean shutdown); use scripts/export_rtabmap_map.sh to pull a
    plain merged .pcd out of it for the existing pcd_to_pgm.py/
    build_course_map.sh 2D map pipeline.
    """
    config_path = PathJoinSubstitution(
        [get_package_share_directory('challenge_bringup'), 'config', 'rtabmap_mapping.yaml']
    )

    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    database_path = LaunchConfiguration('database_path')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'delete_db_on_start',
            default_value='true',
            description='Start each mapping run from an empty database, same as '
                        'loop_pgo starting a fresh in-memory pose graph per launch.',
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value=os.path.join(
                get_package_share_directory('challenge_bringup'), 'maps', 'rtabmap.db'
            ),
            description='Where RTAB-Map saves its pose graph + point cloud database.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch rtabmap_viz alongside the SLAM node',
        ),
        # Two Node actions (mutually exclusive via condition) since the '-d'
        # CLI argument can't be toggled by a LaunchConfiguration at runtime.
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap',
            output='screen',
            condition=IfCondition(delete_db_on_start),
            parameters=[config_path, {'database_path': database_path}],
            remappings=[
                ('scan_cloud', '/cloud_registered_body'),
                ('odom', '/odom'),
            ],
            arguments=['-d'],
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap',
            output='screen',
            condition=UnlessCondition(delete_db_on_start),
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
