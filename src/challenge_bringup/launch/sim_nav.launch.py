"""
sim_nav.launch.py — Nav2 + zone_nav + circuit_runner for Ignition Gazebo sim.

DO NOT use on the real robot — use challenge_master.launch.py instead.
This file exists solely for simulation testing of the navigation pipeline.

Prerequisites (run first in a separate terminal):
    ros2 launch diy_sim sim_competition.launch.py

Then start this launch:
    ros2 launch challenge_bringup sim_nav.launch.py

Fire the start signal (separate terminal after everything is up):
    ros2 topic pub --once /green_light std_msgs/msg/Bool "data: true"

Optional args:
    use_circuit_runner:=false   — skip the lap orchestrator (manual nav goal testing)
    use_rviz:=true              — open RViz2 for visualisation
    use_zone_nav:=false         — skip zone state machine (plain Nav2 only)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _zone_nav_launch(context, use_zone_nav_lc, use_circuit_runner_lc, circuit_waypoints_lc):
    """Conditionally include zone_nav.launch.py at runtime."""
    if context.perform_substitution(use_zone_nav_lc).lower() != 'true':
        return []

    zone_nav_pkg = get_package_share_directory('diy_zone_nav')
    zone_nav_launch = os.path.join(zone_nav_pkg, 'launch', 'zone_nav.launch.py')

    launch_args = {
        'launch_gate':        'false',   # no EKF gate needed in sim
        'use_circuit_runner': context.perform_substitution(use_circuit_runner_lc),
        # Sim overrides for zone_nav_manager_node:
        #   odom_topic        — sim bridge publishes /odom, not /odometry/filtered
        #   mux_param_service — cmd_vel_mux not running in sim; dummy suppresses warns
        # REAL robot: these are not passed (challenge_master.launch.py used instead)
        'odom_topic':        '/odom',
        'mux_param_service': '/nonexistent/dummy',
    }

    circuit_wp = context.perform_substitution(circuit_waypoints_lc).strip()
    if circuit_wp:
        launch_args['circuit_waypoints_file'] = circuit_wp

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zone_nav_launch),
            launch_arguments=launch_args.items(),
        )
    ]


def generate_launch_description():
    pkg_dir          = get_package_share_directory('challenge_bringup')
    nav2_params      = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    keepout_mask_yaml = os.path.join(pkg_dir, 'maps', 'keepout_mask.yaml')
    default_circuit_wp = os.path.join(
        get_package_share_directory('diy_zone_nav'),
        'config', 'sim_circuit_waypoints.yaml')

    use_circuit_runner  = LaunchConfiguration('use_circuit_runner')
    use_zone_nav        = LaunchConfiguration('use_zone_nav')
    use_rviz            = LaunchConfiguration('use_rviz')
    circuit_waypoints   = LaunchConfiguration('circuit_waypoints_file')

    return LaunchDescription([

        # ── Launch arguments ──────────────────────────────────────────────
        DeclareLaunchArgument('use_circuit_runner',  default_value='true'),
        DeclareLaunchArgument('use_zone_nav',        default_value='true'),
        DeclareLaunchArgument('use_rviz',            default_value='false'),
        DeclareLaunchArgument(
            'circuit_waypoints_file',
            default_value=default_circuit_wp,
            description='Path to sim_circuit_waypoints.yaml',
        ),

        # ── Nav2 navigation stack (NO AMCL) ──────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch', 'navigation_launch.py',
                )
            ),
            launch_arguments={
                'params_file':  nav2_params,
                'use_sim_time': 'true',
                'autostart':    'True',
            }.items(),
        ),

        # ── Keepout filter servers ────────────────────────────────────────
        # Serves the keepout mask PNG as a ROS map so the KeepoutFilter
        # costmap plugin blocks planning through inner corner pockets.
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': keepout_mask_yaml,
                'topic_name': '/keepout_filter_mask',
                'frame_id': 'map',
            }],
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'type': 0,
                'filter_info_topic': '/costmap_filter_info',
                'mask_topic': '/keepout_filter_mask',
                'base': 0.0,
                'multiplier': 1.0,
            }],
        ),

        # ── Zone nav + circuit runner ─────────────────────────────────────
        OpaqueFunction(function=_zone_nav_launch,
                       args=[use_zone_nav, use_circuit_runner, circuit_waypoints]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            parameters=[{'use_sim_time': True}],
        ),
    ])
