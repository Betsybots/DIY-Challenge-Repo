"""nav2_obs.launch.py — Nav2 bringup for the narrow-terrain obstacle course.

Ackermann robot, MPPI controller, NO AMCL (localization comes from mcl_3dl,
launched separately via mcl_localizer.launch.py, which publishes map->odom
and the /updated_map cloud consumed by the global costmap).

Velocity pipeline:
    controller_server (MPPI)  -> cmd_vel_nav
    velocity_smoother         -> cmd_vel_nav in, /cmd_vel_smoothed out
    behavior_server (BackUp)  -> /cmd_vel_smoothed directly (the smoother is
                                 forward-only, so reversing recoveries must
                                 bypass it)
    ackermann drivetrain      <- /cmd_vel_smoothed

Typical use:
    ros2 launch challenge_bringup nav2_obs.launch.py
    ros2 launch challenge_bringup nav2_obs.launch.py \
        map:=/path/to/course.yaml use_rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('challenge_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    drive_cmd_vel_topic = LaunchConfiguration('drive_cmd_vel_topic')
    use_rviz = LaunchConfiguration('use_rviz')

    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # Map fully qualified names to relative ones so the node's namespace can
    # be prepended (same workaround as nav2_navigation_launch.py).
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'yaml_filename': map_yaml_file,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument('namespace', default_value='',
                              description='Top-level namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Automatically start the Nav2 stack'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config',
                                       'nav2_obs_params.yaml'),
            description='Narrow-terrain Ackermann MPPI Nav2 parameters'),
        DeclareLaunchArgument(
            'map',
            default_value=('/home/juggernauts/DIY-Challenge-Repo/src/'
                           'diy_localization/map/Obstacle_course/'
                           'obstacle_course_clean.yaml'),
            description='pgm+yaml occupancy grid for map_server/static_layer'),
        DeclareLaunchArgument('use_respawn', default_value='False',
                              description='Respawn crashed nodes'),
        DeclareLaunchArgument('log_level', default_value='info',
                              description='Log level'),
        DeclareLaunchArgument(
            'drive_cmd_vel_topic',
            default_value='/cmd_vel_smoothed',
            description='Topic the Ackermann drivetrain subscribes to'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Start RViz with the master view'),

        # NOTE: no AMCL and no lifecycle_manager_localization — mcl_3dl owns
        # map->odom. map_server still runs here to feed the static layer.
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            # BackUp reverses: bypass the forward-only velocity smoother and
            # command the drive directly.
            remappings=remappings + [('cmd_vel', drive_cmd_vel_topic)]),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [
                ('cmd_vel', 'cmd_vel_nav'),
                # ackermann-drive (driveStack) subscribes to /cmd_vel_smoothed.
                ('cmd_vel_smoothed', drive_cmd_vel_topic),
            ]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=namespace,
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', os.path.join(bringup_dir, 'rviz', 'master.rviz')]),
    ])
