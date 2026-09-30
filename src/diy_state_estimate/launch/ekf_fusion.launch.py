#!/usr/bin/env python3
"""
ekf_fusion.launch.py — wheel + FAST-LIO2 + ZED VIO → /odom and
                        odom→base_footprint TF
═══════════════════════════════════════════════════════════════════════════
Starts three nodes:

  sensor_covariance_relay  derives a gated body twist from FAST-LIO2 AND
                           from the ZED VIO node (independently -- neither
                           is checked against the other), each gated against
                           wheels/gyro, removes gyro bias, floors
                           covariances (diy_state_estimate/sensor_covariance_relay.py)
  ekf_filter_node          robot_localization EKF, config/ekf_fusion.yaml:
                           gated FAST-LIO2 + VIO vx/vy/vyaw, wheel vx, and
                           ZED gyro vyaw
  localization_watchdog    holds cmd_vel at zero if /odom diverges

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
  * The ZED wrapper (zed_node) is launched independently, outside this repo
    -- same as the old ACEINNA IMU driver used to be. This launch file just
    waits for /zed/zed_node/odom and /zed/zed_node/imu/data.
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
    vio_odom_topic = LaunchConfiguration('vio_odom_topic')
    output_topic = LaunchConfiguration('output_topic')
    imu_gyro_cov_scale = LaunchConfiguration('imu_gyro_cov_scale')
    imu_gyro_cov_floor = LaunchConfiguration('imu_gyro_cov_floor')
    lidar_pose_cov_scale = LaunchConfiguration('lidar_pose_cov_scale')
    watchdog_cmd_vel_topic = LaunchConfiguration('watchdog_cmd_vel_topic')
    watchdog_max_linear_speed = LaunchConfiguration('watchdog_max_linear_speed')
    watchdog_max_angular_speed = LaunchConfiguration('watchdog_max_angular_speed')
    aceinna_enable = LaunchConfiguration('aceinna_enable')
    aceinna_imu_topic = LaunchConfiguration('aceinna_imu_topic')

    # Internal topics between the relay and the EKF. Not exposed as args on
    # purpose — nothing else should consume them.
    imu_ekf_topic = '/imu/data_ekf'
    lidar_ekf_topic = '/lidar_odom_ekf'
    lidar_twist_ekf_topic = '/lidar_twist_ekf'
    vio_twist_ekf_topic = '/vio_twist_ekf'
    wheel_ekf_topic = '/wheel_odom_ekf'
    aceinna_imu_ekf_topic = '/imu/data_aceinna_ekf'

    declare_args = [
        DeclareLaunchArgument(
            'ekf_config',
            default_value=os.path.join(pkg_dir, 'config', 'ekf_fusion.yaml'),
            description='robot_localization EKF parameter file'),
        DeclareLaunchArgument(
            'wheel_odom_topic', default_value='/wheel_odom',
            description='differential-drive encoder odometry (forward speed vx fused)'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/zed/zed_node/imu/data',
            description='ZED IMU topic (bias-corrected yaw rate fused as the '
                        'primary vyaw source)'),
        DeclareLaunchArgument(
            'lidar_odom_topic', default_value='/Odometry',
            description='FAST-LIO2 odometry (differentiated into a gated body twist by the relay)'),
        DeclareLaunchArgument(
            'vio_odom_topic', default_value='/zed/zed_node/odom',
            description='ZED VIO odometry (differentiated into a gated body '
                        'twist by the relay, independently of FAST-LIO2 -- '
                        'the two are not cross-checked against each other)'),
        DeclareLaunchArgument(
            'output_topic', default_value='/odom',
            description='Fused odometry topic consumed by Nav2'),
        DeclareLaunchArgument(
            'imu_gyro_cov_scale', default_value='1.0',
            description='Multiplier on the IMU gyro covariance before flooring'),
        DeclareLaunchArgument(
            'imu_gyro_cov_floor', default_value='0.004',
            description='Minimum IMU gyro variance (rad/s)^2. At 200 Hz vs the '
                        '10 Hz FAST-LIO2 twist (vyaw var 0.05) the bias-'
                        'corrected gyro dominates yaw rate (~250x)'),
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
        DeclareLaunchArgument(
            'aceinna_enable', default_value='false',
            description='Enable the optional ACEINNA gyro-only (vyaw) EKF '
                        'input (imu1). Off by default -- no ACEINNA driver '
                        'is launched by this repo; see the relay\'s ACEINNA '
                        'docstring section for why only vyaw is fused.'),
        DeclareLaunchArgument(
            'aceinna_imu_topic', default_value='/imu/data',
            description='Raw ACEINNA IMU topic (only used if aceinna_enable '
                        'is true)'),
    ]

    relay_node = Node(
        package='diy_state_estimate',
        executable='sensor_covariance_relay',
        name='sensor_covariance_relay',
        output='screen',
        # ekf_config also carries a sensor_covariance_relay section
        # (e.g. stall_detection); the dict below overrides it for topics and
        # the launch-argument covariance knobs.
        parameters=[ekf_config, {
            'imu_in': imu_topic,
            'imu_out': imu_ekf_topic,
            'imu_gyro_cov_scale': ParameterValue(imu_gyro_cov_scale, value_type=float),
            'imu_gyro_cov_floor': ParameterValue(imu_gyro_cov_floor, value_type=float),
            'lidar_odom_in': lidar_odom_topic,
            'lidar_odom_out': lidar_ekf_topic,
            'lidar_twist_out': lidar_twist_ekf_topic,
            'vio_odom_in': vio_odom_topic,
            'vio_twist_out': vio_twist_ekf_topic,
            'wheel_odom_in': wheel_odom_topic,
            'wheel_odom_out': wheel_ekf_topic,
            'lidar_pose_cov_scale': ParameterValue(lidar_pose_cov_scale, value_type=float),
            # lidar_pose_cov_floor stays at the node default [0.09, 0.09, 0.04];
            # edit sensor_covariance_relay.py or pass a params file to change it.
            'aceinna_enable': ParameterValue(aceinna_enable, value_type=bool),
            'aceinna_imu_in': aceinna_imu_topic,
            'aceinna_imu_out': aceinna_imu_ekf_topic,
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
                # Gated FAST-LIO2 and ZED VIO body twists (each independently
                # gated, not cross-checked against each other), wheel forward
                # speed, and bias-corrected ZED gyro yaw rate, all via the
                # relay. See the header of config/ekf_fusion.yaml.
                'odom0': lidar_twist_ekf_topic,
                'odom1': wheel_ekf_topic,
                'odom2': vio_twist_ekf_topic,
                'imu0': imu_ekf_topic,
                'imu1': aceinna_imu_ekf_topic,
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

