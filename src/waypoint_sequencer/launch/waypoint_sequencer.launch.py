#!/usr/bin/env python3
"""
waypoint_sequencer.launch.py — standalone launch for the optional
automated /goal_pose sequencer.

Usage:
    ros2 launch diy_waypoint_sequencer waypoint_sequencer.launch.py \
        waypoints_file:=/absolute/path/to/waypoints.yaml

This is deliberately its own launch file, not folded into
pd_navigation.launch.py / pure_pursuit_navigation.launch.py — the A*+PD/
pure-pursuit stack works identically whether or not this is running (see
this package's own README/node docstring). Launch it alongside the
navigation stack when you want automated competition-run sequencing;
leave it out for manual RViz-goal testing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('diy_waypoint_sequencer')
    default_waypoints = os.path.join(pkg_share, 'config', 'waypoints.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=default_waypoints,
            description='Absolute path to the waypoints YAML (see config/waypoints.yaml for the schema).',
        ),
        DeclareLaunchArgument(
            'goal_frame_id',
            default_value='map',
            description='Frame ID stamped on published /goal_pose messages.',
        ),
        DeclareLaunchArgument(
            'wait_for_green_light',
            default_value='true',
            description=(
                'true: wait for /green_light before publishing the first '
                'goal (real competition runs). false: start after '
                'start_delay_s automatically (bench testing without a '
                'green-light publisher).'
            ),
        ),
        DeclareLaunchArgument(
            'start_delay_s',
            default_value='1.0',
            description='Delay before publishing the first goal when wait_for_green_light=false.',
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='false',
            description='Wrap back to the first waypoint after the last one is reached.',
        ),
        Node(
            package='diy_waypoint_sequencer',
            executable='waypoint_sequencer_node',
            name='waypoint_sequencer_node',
            output='screen',
            parameters=[{
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'goal_frame_id': LaunchConfiguration('goal_frame_id'),
                'wait_for_green_light': LaunchConfiguration('wait_for_green_light'),
                'start_delay_s': LaunchConfiguration('start_delay_s'),
                'loop': LaunchConfiguration('loop'),
            }],
        ),
    ])
