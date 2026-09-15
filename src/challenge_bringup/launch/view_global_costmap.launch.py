#!/usr/bin/env python3

"""
Standalone Nav2 global costmap viewer for the static, refined_map.pcd-derived
course map. Purely a visualization / sanity-check tool -- NOT part of the
active custom nav stack (a_star_planner_node subscribes directly to the raw
/map OccupancyGrid and does its own internal binary-radius inflation; it
never reads this costmap). Useful for eyeballing wall/corridor geometry and
an approximate inflation margin before/independent of running the full
localizer + controller + waypoint sequencer.

Pipeline:
    course_traced_smooth.yaml (derived from refined_map.pcd)
        -> map_server -> /map
    /map
        -> nav2_costmap_2d (static_layer + inflation_layer)
        -> /costmap/costmap
    RViz subscribes to both /map and /costmap/costmap.

NOTE: the standalone `nav2_costmap_2d` executable self-namespaces to
"/costmap/costmap" regardless of the name= given below (confirmed
empirically -- not a typo). The published costmap topic and the
lifecycle-manager's node_names entry both account for this.

No robot / localizer / lidar required -- a trivial static map->base_link
transform is published so nav2_costmap_2d can resolve a robot pose without
warnings; it is NOT a real localization estimate.

Example:
    ros2 launch challenge_bringup view_global_costmap.launch.py
    ros2 launch challenge_bringup view_global_costmap.launch.py \
        map_yaml:=/path/to/other_map.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    map_yaml = LaunchConfiguration('map_yaml')
    use_rviz = LaunchConfiguration('use_rviz')

    default_map_yaml = os.environ.get('DIY_MAP_YAML') or (
        os.path.join(
            os.environ.get('DIY_ROS_WS', ''),
            'src', 'DIY-Challenge-Repo', 'maps', 'course_traced_smooth.yaml',
        )
        if os.environ.get('DIY_ROS_WS') else ''
    )

    costmap_params_file = os.path.join(
        get_package_share_directory('challenge_bringup'),
        'config', 'global_costmap_static.yaml',
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('challenge_bringup'),
        'rviz', 'global_costmap_view.rviz',
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map_yaml,
            description=(
                'Absolute path to the map YAML to costmap (nav2_map_server '
                'format). Defaults to $DIY_MAP_YAML if set, else '
                'maps/course_traced_smooth.yaml under $DIY_ROS_WS.'
            ),
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz.',
        ),

        # ------------------------------------------------------------
        # A trivial static map->base_link transform so nav2_costmap_2d
        # can resolve a robot pose without a real localizer running.
        # NOT a real pose estimate -- purely so the costmap node doesn't
        # spam "failed to get robot pose" warnings.
        # ------------------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='costmap_view_fake_robot_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        ),

        # ------------------------------------------------------------
        # Map server
        # ------------------------------------------------------------
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': map_yaml,
            }],
        ),

        # ------------------------------------------------------------
        # Global costmap (static_layer + inflation_layer). The
        # standalone nav2_costmap_2d executable self-namespaces to
        # "/costmap/costmap" regardless of name= (confirmed empirically)
        # -- global_costmap_static.yaml's top-level keys match that.
        # ------------------------------------------------------------
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='costmap',
            output='screen',
            parameters=[costmap_params_file],
        ),

        # ------------------------------------------------------------
        # Lifecycle manager -- activates map_server + the costmap node.
        # 'costmap/costmap' matches the costmap's real self-namespaced
        # identity (see note above), not the name= given to the Node.
        # bond_timeout: 0.0 disables the post-activation bond/heartbeat
        # check -- confirmed empirically that the standalone
        # nav2_costmap_2d executable never creates a bond connection
        # (only map_server does), so the default bond_timeout otherwise
        # always fails with "unable to be reached by bond" even though
        # both nodes are genuinely active and publishing correctly.
        # ------------------------------------------------------------
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='costmap_view_lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': ['map_server', 'costmap/costmap'],
            }],
        ),

        # ------------------------------------------------------------
        # RViz
        # ------------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config_file],
        ),
    ])
