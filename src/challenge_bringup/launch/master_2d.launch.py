#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

"""
hesai lidar driver
pointcloud to laserscan
Robot description
fast lio ros2
nav2 launch

"""
def generate_launch_description():
    pkg_dir = get_package_share_directory('challenge_bringup')
    pcl_to_laserscan_dir = get_package_share_directory('pointcloud_to_laserscan')
    autonomous = LaunchConfiguration('autonomous')
    use_rviz = LaunchConfiguration('use_rviz')
    startup_delay = LaunchConfiguration('startup_delay')
    database_path = LaunchConfiguration('database_path')

    # ── Argument declarations ──────────────────────────────────────────────

    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='false')
    declare_autonomous = DeclareLaunchArgument('autonomous', default_value='true')

    declare_startup_delay = DeclareLaunchArgument(
        'startup_delay',
        default_value='3.0',
        description='Seconds to wait after starting the Hesai driver before '
                    'launching the rest of the stack (lets the sensor come online).',
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_description'),
                'launch',
                'description.launch.py',
            )
        ),
    )

    # ── BLOCK 1: Hesai QT64 lidar driver ──────────────────────────────────────
    hesai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hesai_ros_driver'),
                'launch',
                'start.py',
            )
        ),
    )

    # ── BLOCK 2: Pointcloud to Laserscan ────────────────────────────────
    pcl_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pointcloud_to_laserscan'),
                'launch',
                'sample_pointcloud_to_laserscan.launch.py',
            )
        ),
        condition=IfCondition(autonomous),
    )

    delayed_fast_lio2_launch = TimerAction(
        period=startup_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('fast_lio_ros2'),
                        'launch',
                        'lio_localizer.launch.py',
                    )
                ),
            ),
        ]
    )

    delayed_nav2_launch = TimerAction(
        period=startup_delay*2,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('challenge_bringup'),
                        'launch',
                        'nav2_2d_launch.py',
                    )
                ),
                condition=IfCondition(autonomous),
            ),
        ]
    )
    # ── BLOCK 3: Fast-LIO2 + EKF  ──────────────────────────
    # Starts after the lidar/IMU producers have come online.
    slam_group = GroupAction(
        actions=[
            # FAST-LIO2 in its own scoped group so the /tf remap applies to it
            # alone: it broadcasts odom→base_link unconditionally, and the EKF
            # below must be the sole owner of that transform. /Odometry is
            # NOT remapped — the EKF and RTAB-Map consume it directly.
            GroupAction(
                actions=[
                    SetRemap(src='/tf', dst='/tf_fastlio_unused'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory('fast_lio_ros2'),
                                'launch',
                                'lio_localizer.launch.py',
                            )
                        ),
                    ),
                ]
            ),
            # EKF: /wheel_odom (vx, vyaw) + /imu/data (vyaw, down-weighted)
            # + FAST-LIO2 /Odometry (x, y, yaw) → /odom + odom→base_footprint TF.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('diy_state_estimate'),
                        'launch',
                        'ekf_fusion.launch.py',
                    )
                ),
                launch_arguments={
                    # 'wheel_odom_topic': '/wheel_odom',
                    'imu_topic': '/imu/data',
                    'lidar_odom_topic': '/Odometry',
                    'output_topic': '/odom',
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_dir,
                        'launch',
                        'rtabmap_localization_launch.py',
                    )
                ),
                condition=IfCondition(autonomous),
                launch_arguments={
                    'database_path': database_path,
                    
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_dir,
                        'launch',
                        'rtabmap_mapping_launch.py',
                    )
                ),
                condition=UnlessCondition(autonomous),
                launch_arguments={
                    'database_path': database_path,
                    'use_rviz': True,
                }.items(),
            ),
        ]
    )

    # ── Block 5: map_hba: hierarchical bundle adjustment map refinement ────────────
    delayed_map_hba_launch = TimerAction(
        period=startup_delay,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('map_hba'),
                    'launch',
                    'map_hba_launch.py',
                )
            ),
            condition=UnlessCondition(autonomous),
        )]
    )

    return LaunchDescription([
        declare_startup_delay,
        declare_autonomous,
        declare_use_rviz,
        robot_description_launch,
        hesai_launch,
        pcl_to_laserscan_launch,
        delayed_fast_lio2_launch,
        delayed_nav2_launch,
        delayed_map_hba_launch
    ])
