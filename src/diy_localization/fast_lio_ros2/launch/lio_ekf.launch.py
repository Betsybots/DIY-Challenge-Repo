#!/usr/bin/env python3
"""
lio_ekf.launch.py — standalone robot_localization EKF from config/lio_ekf.yaml

Fuses the raw /zed/zed_node/odom, /Odometry (FAST-LIO2), /wheel_odom and
/zed/zed_node/imu/data topics directly (see config/lio_ekf.yaml for the full
rationale and trust levels). Publishes fused odometry on output_topic and
owns the odom -> base_footprint TF.

DO NOT run this at the same time as diy_state_estimate's ekf_fusion.yaml
(diy_state_estimate/launch/ekf_fusion.launch.py) -- both nodes publish the
same odom -> base_footprint transform.

  ros2 launch fast_lio_ros2 lio_ekf.launch.py
  ros2 launch fast_lio_ros2 lio_ekf.launch.py output_topic:=/odom_lio
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('fast_lio_ros2')

    ekf_config = LaunchConfiguration('ekf_config')
    output_topic = LaunchConfiguration('output_topic')

    declare_args = [
        DeclareLaunchArgument(
            'ekf_config',
            default_value=os.path.join(pkg_dir, 'config', 'lio_ekf.yaml'),
            description='robot_localization EKF parameter file'),
        DeclareLaunchArgument(
            'output_topic', default_value='/odom',
            description='Fused odometry topic'),
    ]

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', output_topic)],
    )

    return LaunchDescription(declare_args + [ekf_node])
