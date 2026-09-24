#!/usr/bin/env python3
"""
ekf_fusion.launch.py — wheel + IMU + FAST-LIO2 → /odom and odom→base_footprint TF
═══════════════════════════════════════════════════════════════════════════
Starts two nodes:

  sensor_covariance_relay  floors the IMU gyro and FAST-LIO2 pose covariances
                           (see diy_state_estimate/sensor_covariance_relay.py)
  ekf_filter_node          robot_localization EKF, config/ekf_fusion.yaml

Included from challenge_bringup/master.launch.py. Can also be run alone
against a bag or the live robot:

  ros2 launch diy_state_estimate ekf_fusion.launch.py
  ros2 launch diy_state_estimate ekf_fusion.launch.py imu_gyro_cov_floor:=0.01

PRECONDITIONS
  * Nothing else may publish the odom→base_footprint TF. FAST-LIO2
    unconditionally publishes its own odom→base_link TF, so master.launch.py
    remaps its /tf away.
  * /wheel_odom must have child_frame_id base_link (driveStack
    differential-drive diffDrive.yaml base_frame_id) — the EKF drops any
    message whose frame it cannot transform.
  * The URDF must provide the fixed base_footprint → base_link joint and
    imu_link → base_link (robot_description/urdf/robot.urdf.xacro).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('diy_state_estimate')

    ekf_config = LaunchConfiguration('ekf_config')
    wheel_odom_topic = LaunchConfiguration('wheel_odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    lidar_odom_topic = LaunchConfiguration('lidar_odom_topic')
    output_topic = LaunchConfiguration('output_topic')
    imu_gyro_cov_scale = LaunchConfiguration('imu_gyro_cov_scale')
    imu_gyro_cov_floor = LaunchConfiguration('imu_gyro_cov_floor')
    lidar_pose_cov_scale = LaunchConfiguration('lidar_pose_cov_scale')
    watchdog_cmd_vel_topic = LaunchConfiguration('watchdog_cmd_vel_topic')
    watchdog_max_linear_speed = LaunchConfiguration('watchdog_max_linear_speed')
    watchdog_max_angular_speed = LaunchConfiguration('watchdog_max_angular_speed')

    # Internal topics between the relay and the EKF. Not exposed as args on
    # purpose — nothing else should consume them.
    imu_ekf_topic = '/imu/data_ekf'
    lidar_ekf_topic = '/lidar_odom_ekf'

    declare_args = [
        DeclareLaunchArgument(
            'ekf_config',
            default_value=os.path.join(pkg_dir, 'config', 'ekf_fusion.yaml'),
            description='robot_localization EKF parameter file'),
        DeclareLaunchArgument(
            'wheel_odom_topic', default_value='/wheel_odom',
            description='differential-drive encoder odometry (velocities fused)'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/imu/data',
            description='ACEINNA IMU topic (yaw rate fused, down-weighted)'),
        DeclareLaunchArgument(
            'lidar_odom_topic', default_value='/Odometry',
            description='FAST-LIO2 odometry (planar pose fused)'),
        DeclareLaunchArgument(
            'output_topic', default_value='/odom',
            description='Fused odometry topic consumed by Nav2'),
        DeclareLaunchArgument(
            'imu_gyro_cov_scale', default_value='1.0',
            description='Multiplier on the IMU gyro covariance before flooring'),
        DeclareLaunchArgument(
            'imu_gyro_cov_floor', default_value='0.004',
            description='Minimum IMU gyro variance (rad/s)^2; wheel yaw-rate '
                        'covariance is 0.001, so 0.004 ≈ wheels weighted 4x'),
        DeclareLaunchArgument(
            'lidar_pose_cov_scale', default_value='1.0',
            description='Multiplier on the FAST-LIO2 pose covariance before flooring'),
        DeclareLaunchArgument(
            'watchdog_cmd_vel_topic', default_value='/cmd_vel_raw',
            description='localization_watchdog: topic to hold at zero once the '
                        'fused pose diverges (upstream of nav2_collision_monitor, '
                        'if launched -- see nav2_navigation_launch.py)'),
        DeclareLaunchArgument(
            'watchdog_max_linear_speed', default_value='0.6',
            description='localization_watchdog: implied /odom linear speed (m/s) '
                        'considered implausible (2x FollowPath.linear_velocity)'),
        DeclareLaunchArgument(
            'watchdog_max_angular_speed', default_value='2.0',
            description='localization_watchdog: implied /odom angular speed '
                        '(rad/s) considered implausible '
                        '(2x FollowPath.max_angular_velocity)'),
    ]

    relay_node = Node(
        package='diy_state_estimate',
        executable='sensor_covariance_relay',
        name='sensor_covariance_relay',
        output='screen',
        parameters=[{
            'imu_in': imu_topic,
            'imu_out': imu_ekf_topic,
            'imu_gyro_cov_scale': ParameterValue(imu_gyro_cov_scale, value_type=float),
            'imu_gyro_cov_floor': ParameterValue(imu_gyro_cov_floor, value_type=float),
            'lidar_odom_in': lidar_odom_topic,
            'lidar_odom_out': lidar_ekf_topic,
            'lidar_pose_cov_scale': ParameterValue(lidar_pose_cov_scale, value_type=float),
            # lidar_pose_cov_floor stays at the node default [0.09, 0.09, 0.04];
            # edit sensor_covariance_relay.py or pass a params file to change it.
        }],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {
                # 'odom0': wheel_odom_topic,
                'imu0': imu_ekf_topic,
                'odom0': lidar_ekf_topic,
            },
        ],
        remappings=[('odometry/filtered', output_topic)],
    )

    # Watches the EKF's own output for implausible jumps/velocity and holds
    # zero on watchdog_cmd_vel_topic if it ever diverges -- see
    # diy_state_estimate/localization_watchdog.py for the full rationale.
    watchdog_node = Node(
        package='diy_state_estimate',
        executable='localization_watchdog',
        name='localization_watchdog',
        output='screen',
        parameters=[{
            'odom_topic': output_topic,
            'cmd_vel_topic': watchdog_cmd_vel_topic,
            'max_linear_speed': ParameterValue(watchdog_max_linear_speed, value_type=float),
            'max_angular_speed': ParameterValue(watchdog_max_angular_speed, value_type=float),
        }],
    )

    return LaunchDescription(declare_args + [relay_node, ekf_node, watchdog_node])
