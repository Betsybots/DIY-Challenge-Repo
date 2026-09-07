#!/usr/bin/env python3

"""Static-map navigation stack using the custom A* and pure-pursuit controllers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    robot_clearance = LaunchConfiguration('robot_clearance')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    lookahead_distance = LaunchConfiguration('lookahead_distance')
    linear_velocity = LaunchConfiguration('linear_velocity')
    max_angular_velocity = LaunchConfiguration('max_angular_velocity')
    rotate_in_place_threshold = LaunchConfiguration(
        'rotate_in_place_threshold'
    )
    minimum_turning_velocity = LaunchConfiguration(
        'minimum_turning_velocity'
    )
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    use_rviz = LaunchConfiguration('use_rviz')

    rviz_config = os.path.join(
        get_package_share_directory('diy_motion_planner'),
        'rviz',
        'config.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            description='Absolute path to the saved map YAML file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use ROS simulation clock.',
        ),
        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description='Use base_footprint in simulation and base_link on hardware.',
        ),
        DeclareLaunchArgument(
            'odom_frame', default_value='odom',
            description='Local odometry frame.',
        ),
        DeclareLaunchArgument(
            'robot_clearance', default_value='0.25',
            description='Minimum static-obstacle clearance for A* in meters.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel',
            description='Use /cmd_vel in simulation and /cmd_vel_nav on hardware.',
        ),
        DeclareLaunchArgument(
            'lookahead_distance', default_value='0.2',
            description='Pure-pursuit look-ahead distance in meters.',
        ),
        DeclareLaunchArgument(
            'linear_velocity', default_value='0.3',
            description='Pure-pursuit commanded forward velocity in m/s.',
        ),
        DeclareLaunchArgument(
            'max_angular_velocity', default_value='1.0',
            description='Maximum commanded angular velocity in rad/s.',
        ),
        DeclareLaunchArgument(
            'rotate_in_place_threshold', default_value='1.0',
            description='Heading-error reference for slowing down in radians.',
        ),
        DeclareLaunchArgument(
            'minimum_turning_velocity', default_value='0.05',
            description='Minimum forward velocity while making a large turn in m/s.',
        ),
        DeclareLaunchArgument(
            'goal_tolerance', default_value='0.15',
            description='Final goal stopping tolerance in meters.',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Whether to launch RViz.',
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='map_lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),
        Node(
            package='diy_planning',
            executable='a_star_planner_node',
            name='a_star_planner_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'base_frame': base_frame,
                'robot_clearance': robot_clearance,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='diy_motion_planner',
            executable='pure_pursuit_motion_planner_node',
            name='pure_pursuit_motion_planner_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'cmd_vel_topic': cmd_vel_topic,
                'lookahead_distance': lookahead_distance,
                'linear_velocity': linear_velocity,
                'max_angular_velocity': max_angular_velocity,
                'rotate_in_place_threshold': rotate_in_place_threshold,
                'minimum_turning_velocity': minimum_turning_velocity,
                'goal_tolerance': goal_tolerance,
            }],
        ),
    ])