#!/usr/bin/env python3
"""
challenge_master.launch.py — Top-level competition bringup
══════════════════════════════════════════════════════════
Single file that starts the entire robot stack for competition or debug runs.
Every subsystem is gated by a boolean argument so the same file works whether
you are running the full competition stack on the Jetson, a lightweight relay
setup on the Raspberry Pi, or a laptop-only replay / debug session.

Source a profile, then launch:

  ros2 launch challenge_bringup master.launch.py autonomous:=true  (For enabling autonomous mode on the robot)
  ros2 launch challenge_bringup master.launch.py autonomous:=false (For disabling autonomous mode on the robot- only joystick/manual control will be available for mapping)

STARTUP ORDER
─────────────
    1. robot_description — publishes the URDF / TF tree first
    2. Hesai lidar (hesai_ros_driver) — /lidar_points feeds mcl_3dl
    3. Fast LIO2 — CURRENTLY DISABLED (commented out)
    4. EKF (diy_state_estimate) — CURRENTLY DISABLED (commented out).
       zed_base_odom_relay instead republishes ZED VIO /zed/zed_node/odom
       at base_footprint as /odom and owns the odom→base_footprint TF
    5. Autonomous mode: mcl_3dl + Nav2 navigation stack
    6. Manual mode: loop closure / map-refinement stack only
    7. rviz2 — optional debug visualization (use_rviz:=true)

TF OWNERSHIP
────────────
    map  → odom            mcl_3dl
    odom → base_footprint  diy_state_estimate zed_base_odom_relay (publish_tf)
                           — ONLY this node. The ZED wrapper must run with
                           pos_tracking.publish_tf:=false and
                           pos_tracking.publish_map_tf:=false.
    base_footprint → *     robot_state_publisher (URDF): fixed joint
                           base_footprint→base_link, then base_link→sensors/
                           wheels.

WHEEL ODOMETRY / MOTORS
───────────────────────
    /wheel_odom comes from driveStack's differential-drive node, launched
    separately (driveStack bringup). It is NOT started here; the EKF just
    waits for the topic.

NOTE ON THE ZED IMU / VIO:
──────────────────────────
The ZED wrapper (zed_node, publishing /zed/zed_node/imu/data and
/zed/zed_node/odom) is launched INDEPENDENTLY — same as the old ACEINNA IMU
driver used to be — it is not vendored here and not part of this launch
file. FAST-LIO2 itself also consumes /zed/zed_node/imu/data internally
(see diy_localization/fast_lio_ros2/config/qt64.yaml), so it is not an
independent sensor relative to either twist source; see
diy_state_estimate/sensor_covariance_relay.py for how it is still used.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_dir = get_package_share_directory('challenge_bringup')
    autonomous = LaunchConfiguration('autonomous')
    use_rviz = LaunchConfiguration('use_rviz')
    startup_delay = LaunchConfiguration('startup_delay')
    nav2_launch_file = LaunchConfiguration('nav2_launch_file')

    # ── Argument declarations ──────────────────────────────────────────────

    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='false')
    declare_autonomous = DeclareLaunchArgument('autonomous', default_value='true')
    declare_nav2_launch_file = DeclareLaunchArgument(
        'nav2_launch_file',
        default_value='nav2_navigation_launch.py',
        description='Nav2 launch file from challenge_bringup/launch',
    )

    declare_startup_delay = DeclareLaunchArgument(
        'startup_delay',
        default_value='5.0',
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

    # delayed_fast_lio2_launch = TimerAction(
    #     period=2.0,
    #     actions=[
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(
    #                     get_package_share_directory('fast_lio_ros2'),
    #                     'launch',
    #                     'lio_localizer.launch.py',
    #                 )
    #             ),
    #             launch_arguments={'output_topic': '/odom'}.items(),
    #         ),
    #     ]
    # )

    # delayed_ekf_launch = TimerAction(
    #     period=1.0,
    #     actions=[
    #         # EKF: /wheel_odom (vx, vyaw) + /zed/zed_node/imu/data (vyaw) +
    #         # FAST-LIO2 /Odometry + ZED VIO /zed/zed_node/odom (x, y, yaw,
    #         # each independently gated) → /odom + odom→base_footprint TF.
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(
    #                     get_package_share_directory('diy_state_estimate'),
    #                     'launch',
    #                     'ekf_fusion.launch.py',
    #                 )
    #             ),
    #             launch_arguments={
    #                 # 'wheel_odom_topic': '/wheel_odom',
    #                 'imu_topic': '/zed/zed_node/imu/data',
    #                 'lidar_odom_topic': '/Odometry',
    #                 'vio_odom_topic': '/zed/zed_node/odom',
    #                 'output_topic': '/odom',
    #             }.items(),
    #         ),
    #     ]
    # )

    # ZED2i VIO re-expressed at base_footprint → /odom + odom→base_footprint TF (replaces the EKF).
    zed_base_odom_relay_node = Node(
        package='diy_state_estimate',
        executable='zed_base_odom_relay',
        name='zed_base_odom_relay',
        output='screen',
        parameters=[{
            'zed_odom_topic': '/zed/zed_node/odom',
            'output_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'publish_tf': True,
        }],
    )

    delayed_mcl_3dl_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('mcl_3dl'),
                        'launch',
                        'mcl_localizer.launch.py',
                    )
                ),
                # FAST-LIO is off, so /cloud_registered_body is not published; use the raw Hesai cloud.
                # launch_arguments={'cloud_topic': '/cloud_registered_body'}.items(),
                launch_arguments={
                    'cloud_topic': '/cloud_registered_body',
                    # base_footprint odometry from zed_base_odom_relay (not the raw
                    # camera pose on /zed/zed_node/odom) — matches robot_frame.
                    'odom_topic': '/odom',
                }.items(),
                condition=IfCondition(autonomous),
            ),
        ]
    )

    delayed_loop_pgo_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('loop_pgo'),
                        'launch',
                        'loop_pgo_launch.py',
                    )
                ),
                condition=UnlessCondition(autonomous),
            ),
        ]
    )

    delayed_map_hba_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('map_hba'),
                        'launch',
                        'map_hba_launch.py',
                    )
                ),
                condition=UnlessCondition(autonomous),
            ),
        ]
    )

    # ── BLOCK 6: Nav2 autonomous navigation stack ─────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('challenge_bringup'),
                'launch',
                nav2_launch_file,
            ])
        ),
        condition=IfCondition(autonomous),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', "/home/juggernauts/.rviz2/justLocalizer.rviz"],
    )

    delayed_nav2 = TimerAction(
        period=5.0,
        actions=[nav2_launch],
    )


    return LaunchDescription([
        declare_startup_delay,
        declare_autonomous,
        declare_nav2_launch_file,
        declare_use_rviz,
        robot_description_launch,        # always
        hesai_launch,                    # always
        # delayed_fast_lio2_launch,      # always
        # delayed_ekf_launch,            # always
        zed_base_odom_relay_node,        # always
        delayed_mcl_3dl_launch,          # autonomous
        delayed_loop_pgo_launch,         # manual
        delayed_map_hba_launch,          # manual
        delayed_nav2,                    # autonomous
        rviz_node,                       # optional
    ])
