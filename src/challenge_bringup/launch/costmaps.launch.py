#!/usr/bin/env python3

"""
costmaps.launch.py — global_costmap + local_costmap for the custom
A*/pure-pursuit stack, gated independently so either can be run alone.

WHY planner_server/controller_server: see costmaps_params.yaml's header
comment. In short, they are the only pre-built Nav2 programs that host a
correctly, distinctly-named costmap (nav2_costmap_2d's bare standalone
executable hardcodes its own node name and cannot run twice). Their own
NavfnPlanner/MPPI planning and control logic is never invoked -- no
bt_navigator is launched, so nothing ever calls their action servers. The
real planner/controller remain diy_planning's a_star_planner_node and
motion_planner's pure_pursuit_motion_planner_node, launched separately
(see navigation.launch.py / master.launch.py).

Pipeline:
    map_yaml (course_traced_smooth.yaml, derived from refined_map.pcd)
        -> map_server -> /map
    /map + live /lidar_points
        -> planner_server's internal global_costmap -> /global_costmap/costmap
    live /lidar_points (rolling window around the robot)
        -> controller_server's internal local_costmap -> /local_costmap/costmap

Requires a real localization stack already running (map_localizer + EKF)
publishing map->odom->base_link -- this file does NOT publish any TF itself.
If global_frame/robot_base_frame can't resolve, the costmaps will log
transform-timeout warnings but keep trying; they do not crash.

Example:
    ros2 launch challenge_bringup costmaps.launch.py
    ros2 launch challenge_bringup costmaps.launch.py use_local_costmap:=false
    ros2 launch challenge_bringup costmaps.launch.py \
        map_yaml:=/path/to/other_map.yaml use_rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _lifecycle_node_names(context, use_global_costmap_lc, use_local_costmap_lc):
    """
    OpaqueFunction: the lifecycle_manager's node_names list depends on which
    of use_global_costmap/use_local_costmap are actually enabled -- a plain
    LaunchConfiguration substitution can't build a conditional list like this.
    """
    node_names = []
    if context.perform_substitution(use_global_costmap_lc).lower() == 'true':
        node_names += ['map_server', 'planner_server']
    if context.perform_substitution(use_local_costmap_lc).lower() == 'true':
        node_names += ['controller_server']

    return [
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='costmaps_lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'bond_timeout': 4.0,
                'node_names': node_names,
            }],
        )
    ]


def generate_launch_description():

    map_yaml = LaunchConfiguration('map_yaml')
    use_global_costmap = LaunchConfiguration('use_global_costmap')
    use_local_costmap = LaunchConfiguration('use_local_costmap')
    use_rviz = LaunchConfiguration('use_rviz')

    default_map_yaml = os.environ.get('DIY_MAP_YAML') or (
        os.path.join(
            os.environ.get('DIY_ROS_WS', ''),
            'src', 'DIY-Challenge-Repo', 'maps', 'course_traced_smooth.yaml',
        )
        if os.environ.get('DIY_ROS_WS') else ''
    )

    costmaps_params = os.path.join(
        get_package_share_directory('challenge_bringup'),
        'config', 'costmaps_params.yaml',
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('challenge_bringup'),
        'rviz', 'costmaps_view.rviz',
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map_yaml,
            description=(
                'Absolute path to the map YAML for global_costmap\'s '
                'static_layer. Defaults to $DIY_MAP_YAML if set, else '
                'maps/course_traced_smooth.yaml under $DIY_ROS_WS. Only '
                'used if use_global_costmap:=true.'
            ),
        ),

        DeclareLaunchArgument(
            'use_global_costmap',
            default_value='true',
            description=(
                'Launch map_server + planner_server (hosting global_costmap).'
            ),
        ),

        DeclareLaunchArgument(
            'use_local_costmap',
            default_value='true',
            description=(
                'Launch controller_server (hosting local_costmap).'
            ),
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz.',
        ),

        # ------------------------------------------------------------
        # Map server -- only needed for global_costmap's static_layer.
        # local_costmap has no static_layer, doesn't need /map at all.
        # ------------------------------------------------------------
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            condition=IfCondition(use_global_costmap),
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': map_yaml,
            }],
        ),

        # ------------------------------------------------------------
        # planner_server -- hosts global_costmap. Its own NavfnPlanner
        # plugin is configured but never invoked (see module docstring).
        # ------------------------------------------------------------
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            condition=IfCondition(use_global_costmap),
            parameters=[costmaps_params],
        ),

        # ------------------------------------------------------------
        # controller_server -- hosts local_costmap. Its own MPPIController
        # plugin is configured but never invoked (see module docstring).
        # ------------------------------------------------------------
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            condition=IfCondition(use_local_costmap),
            parameters=[costmaps_params],
        ),

        # ------------------------------------------------------------
        # Lifecycle manager -- node_names list depends on which of the
        # two costmaps are actually enabled (see _lifecycle_node_names).
        # ------------------------------------------------------------
        OpaqueFunction(
            function=_lifecycle_node_names,
            args=[use_global_costmap, use_local_costmap],
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
