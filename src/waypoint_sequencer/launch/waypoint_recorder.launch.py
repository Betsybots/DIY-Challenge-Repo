#!/usr/bin/env python3
"""
waypoint_recorder.launch.py — standalone launch for waypoint_recorder_node.

Run this alongside an offline mapping drive (challenge_master.launch.py /
master.launch.py with autonomous:=false, which brings up FAST-LIO2 +
loop_pgo) to record waypoints live using the joystick's LT/RT triggers.
See waypoint_recorder_node.py's module docstring for the full design.

Usage:
    ros2 launch diy_waypoint_sequencer waypoint_recorder.launch.py
    ros2 launch diy_waypoint_sequencer waypoint_recorder.launch.py \
        output_file:=/absolute/path/to/other_waypoints.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Defaults to maps/recorded_waypoints.yaml under $DIY_ROS_WS -- same
    # DIY_MAP_YAML-style fallback pattern used by pd_navigation.launch.py /
    # pure_pursuit_navigation.launch.py. Deliberately a single fixed
    # filename for now (gets overwritten on the next mapping run) -- rename
    # or copy it elsewhere manually if you need to keep more than one
    # recording session's output.
    default_output_file = (
        os.path.join(
            os.environ.get('DIY_ROS_WS', ''),
            'src', 'DIY-Challenge-Repo', 'maps', 'recorded_waypoints.yaml',
        )
        if os.environ.get('DIY_ROS_WS') else ''
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'output_file',
            default_value=default_output_file,
            description=(
                'Absolute path to write the recorded waypoints YAML to. '
                'Defaults to maps/recorded_waypoints.yaml under $DIY_ROS_WS '
                '(overwritten on each mapping run -- copy elsewhere first '
                'if you need to keep it).'
            ),
        ),
        DeclareLaunchArgument(
            'joy_topic', default_value='/joy',
            description='Topic joy_node publishes raw controller state on.',
        ),
        DeclareLaunchArgument(
            'map_frame', default_value='map',
            description='Frame to record waypoint poses in.',
        ),
        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description='Robot base frame to look up the current pose of.',
        ),
        DeclareLaunchArgument(
            'record_axis_index', default_value='2',
            description='Joy axis index for LT (record) — see joystick.yaml.',
        ),
        DeclareLaunchArgument(
            'remove_axis_index', default_value='5',
            description='Joy axis index for RT (remove last) — see joystick.yaml.',
        ),
        DeclareLaunchArgument(
            'loop', default_value='false',
            description='Set loop: true in the output YAML.',
        ),
        Node(
            package='diy_waypoint_sequencer',
            executable='waypoint_recorder_node',
            name='waypoint_recorder_node',
            output='screen',
            parameters=[{
                'output_file': LaunchConfiguration('output_file'),
                'joy_topic': LaunchConfiguration('joy_topic'),
                'map_frame': LaunchConfiguration('map_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'record_axis_index': LaunchConfiguration('record_axis_index'),
                'remove_axis_index': LaunchConfiguration('remove_axis_index'),
                'loop': LaunchConfiguration('loop'),
            }],
        ),
    ])
