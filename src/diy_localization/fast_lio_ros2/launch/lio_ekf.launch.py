#!/usr/bin/env python3
"""
lio_ekf.launch.py — standalone robot_localization EKF from config/lio_ekf.yaml

LIO-primary fusion by default: /Odometry (FAST-LIO2) is the absolute x/y/yaw
backbone; /zed/zed_node/odom (differential x/y), /wheel_odom (vx/vy) and the
ZED gyro (vyaw) stabilize it and bridge LIO-degenerate stretches (see
config/lio_ekf.yaml for the full rationale and trust levels). Publishes fused
odometry on output_topic and owns the odom -> base_footprint TF.

A/B test variant (ZED visual odometry as the absolute backbone, FAST-LIO2
demoted to differential increments — see config/lio_ekf_zed_primary.yaml):

  ros2 launch fast_lio_ros2 lio_ekf.launch.py ekf_config:=zed

`ekf_config` accepts the shortcuts `lio` (default, lio_ekf.yaml) and `zed`
(lio_ekf_zed_primary.yaml), a bare file name resolved inside this package's
config/ directory, or an absolute path.

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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('fast_lio_ros2')
    config_dir = os.path.join(pkg_dir, 'config')

    ekf_config = LaunchConfiguration('ekf_config')
    output_topic = LaunchConfiguration('output_topic')

    # Shortcut aliases first ('lio'/'zed'); otherwise bare file names are
    # resolved inside this package's config/ directory and absolute paths are
    # used as-is.
    ekf_config_path = PythonExpression(
        ["{'lio': r'", config_dir, "/lio_ekf.yaml', "
         "'zed': r'", config_dir, "/lio_ekf_zed_primary.yaml'}"
         ".get('", ekf_config, "', '", ekf_config, "' if '", ekf_config,
         "'.startswith('/') else r'", config_dir, "' + '/' + '", ekf_config, "')"])

    declare_args = [
        DeclareLaunchArgument(
            'ekf_config',
            default_value='lio',
            description="EKF parameter file: 'lio' (FAST-LIO2-primary, default), "
                        "'zed' (ZED-visual-odometry-primary), a bare file name in "
                        'fast_lio_ros2/config, or an absolute path'),
        DeclareLaunchArgument(
            'output_topic', default_value='/odom',
            description='Fused odometry topic'),
    ]

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', output_topic)],
    )

    return LaunchDescription(declare_args + [ekf_node])
