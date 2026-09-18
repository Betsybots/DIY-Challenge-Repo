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
    2. Hesai lidar (hesai_ros_driver) — feeds FAST-LIO2
    3. Fast LIO2 — starts after a short delay for localization
    4. Autonomous mode: map_localizer + Nav2 navigation stack
    5. Manual mode: loop closure / map-refinement stack only
    6. rviz2 — optional debug visualization

NOTE ON THE ACEINNA IMU:
────────────────────────
The IMU driver (imu_can_interface) is launched INDEPENDENTLY on the RPi,
outside this repo entirely — it is not vendored here and not part of this
launch file. 
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    pkg_dir = get_package_share_directory('challenge_bringup')
    autonomous = LaunchConfiguration('autonomous')
    use_rviz = LaunchConfiguration('use_rviz')
    startup_delay = LaunchConfiguration('startup_delay')

    # ── Argument declarations ──────────────────────────────────────────────

    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='false')
    declare_autonomous = DeclareLaunchArgument('autonomous', default_value='true')

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


    # ── BLOCK 3: Fast-LIO2 Launch  ──────────────────────────
    # Starts after the lidar/IMU producers have come online.
    slam_group = GroupAction(
        actions=[
            SetRemap(src='/Odometry', dst='/odom'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('fast_lio_ros2'),
                        'launch',
                        'lio_localizer.launch.py',
                    )
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('map_localizer'),
                        'launch',
                        'map_localizer_launch.py',
                    )
                ),
                condition=IfCondition(autonomous),
                launch_arguments={
                    'use_rviz': 'false',
                }.items(),
            ),
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

    # ── Block 5: map_hba: hierarchical bundle adjustment map refinement ────────────
    map_hba_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('map_hba'),
                'launch',
                'map_hba_launch.py',
            )
        ),
        condition=UnlessCondition(autonomous),
    )

    # ── BLOCK 6: Nav2 autonomous navigation stack ─────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('challenge_bringup'),
                'launch',
                'nav2_navigation_launch.py',
            )
        ),
        condition=IfCondition(autonomous),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'master.rviz')],
    )

    # Everything except the Hesai driver itself waits hesai_startup_delay
    # seconds so the sensor is online before FAST-LIO2 and the rest of the
    # stack start consuming /hesai/points.
    delayed_fast_lio = TimerAction(
        period=startup_delay,
        actions=[slam_group],
    )

    delayed_hba_map = TimerAction(
        period=startup_delay,
        actions=[map_hba_launch],
    )

    delayed_nav2 = TimerAction(
        period=startup_delay,
        actions=[nav2_launch],
    )


    return LaunchDescription([
        declare_startup_delay,
        declare_autonomous,
        declare_use_rviz,
        ## 
        robot_description_launch,
        hesai_launch,
        delayed_fast_lio,
        # delayed_hba_map,
        delayed_nav2,
        rviz_node,
    ])
