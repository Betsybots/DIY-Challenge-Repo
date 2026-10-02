"""
course_supervisor.launch.py -- obstacle course with section-by-section
controller selection. Separate from the existing Nav2 launches, which are not
changed (obstacle_course_jetson.sh / nav2_navigation_mppi_*.launch.py remain
the fallback).

Command chain. course_supervisor is the ONLY publisher on drive_cmd_vel_topic
(/cmd_vel_smoothed = ackermann-drive input):
  controller_server (MPPI) -> /cmd_vel_mppi          -+
  path tracker (built in)                             +-> course_supervisor -> cmd_vel_mppi_raw
                                                      |     -> velocity_smoother -> /cmd_vel_smoother_out
                                                      |     -> course_supervisor -> /cmd_vel_smoothed
  mapless_wall_follower    -> /cmd_vel_wall_follower -+-> course_supervisor -> /cmd_vel_smoothed (direct,
                                                           as when it runs standalone)

Switches (all launch arguments):
  use_nav2            controller_server + FollowPath on the driving line
  use_speed_filter    speed mask on the local costmap (needs mask_yaml_file)
  use_wall_follower   start mapless_wall_follower for wall_follower sections
  use_path_tracker    allow the built-in push-through tracker
  drive_lidar_gate    publish mcl_3dl's mcl_measurement_enabled per section
                      (start mcl_3dl with use_lidar_gate:=true start_lidar_gate_zones:=false)
  use_apriltag_fix    AprilTag sightings -> mcl_measurement pose fixes
  use_tag_triggers    AprilTags advance progress to their section
  start_apriltag      also start zed_apriltag (ZED wrapper must be running); tag_size
                      = printed black-square edge length in metres (default 0.16)
  tag_survey_file     record tag map poses to this file ('' = off)
  laps                laps before stopping (default 2)
A section whose mode is switched off falls back to path_tracker, then nav2.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory('course_supervisor')
    bringup = get_package_share_directory('challenge_bringup')
    L = LaunchConfiguration

    def flag(name):
        return ParameterValue(L(name), value_type=bool)

    def both(a, b):
        return IfCondition(PythonExpression(["'", L(a), "' == 'true' and '", L(b), "' == 'true'"]))

    def first_not_second(a, b):
        return IfCondition(PythonExpression(["'", L(a), "' == 'true' and '", L(b), "' != 'true'"]))

    sim = {'use_sim_time': ParameterValue(L('use_sim_time'), value_type=bool)}
    nav2_files = [L('nav2_params_file'), L('nav2_override_params_file')]

    controller_remaps = [('cmd_vel', '/cmd_vel_mppi')]
    controller = Node(
        package='nav2_controller', executable='controller_server', name='controller_server',
        output='screen', arguments=['--ros-args', '--log-level', L('log_level')],
        parameters=nav2_files + [sim], remappings=controller_remaps,
        condition=first_not_second('use_nav2', 'use_speed_filter'))
    controller_speed_filter = Node(
        package='nav2_controller', executable='controller_server', name='controller_server',
        output='screen', arguments=['--ros-args', '--log-level', L('log_level')],
        parameters=nav2_files + [L('speed_filter_params_file'), sim], remappings=controller_remaps,
        condition=both('use_nav2', 'use_speed_filter'))
    controller_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_course_controller', output='screen',
        parameters=[sim, {'autostart': True, 'node_names': ['controller_server']}],
        condition=IfCondition(L('use_nav2')))

    smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
        output='screen', parameters=nav2_files + [sim],
        remappings=[('cmd_vel', 'cmd_vel_mppi_raw'), ('cmd_vel_smoothed', L('smoother_output_topic'))])
    smoother_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_course_smoother', output='screen',
        parameters=[sim, {'autostart': True, 'node_names': ['velocity_smoother']}])

    mask_server = Node(
        package='nav2_map_server', executable='map_server', name='filter_mask_server', output='screen',
        parameters=[L('speed_filter_params_file'), sim, {'yaml_filename': L('mask_yaml_file')}],
        condition=both('use_nav2', 'use_speed_filter'))
    filter_info_server = Node(
        package='nav2_map_server', executable='costmap_filter_info_server',
        name='costmap_filter_info_server', output='screen',
        parameters=[L('speed_filter_params_file'), sim],
        condition=both('use_nav2', 'use_speed_filter'))
    filter_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_course_speed_filter', output='screen',
        parameters=[sim, {'autostart': True,
                          'node_names': ['filter_mask_server', 'costmap_filter_info_server']}],
        condition=both('use_nav2', 'use_speed_filter'))

    wall_follower = Node(
        package='mapless_wall_follower', executable='mapless_wall_follower_node',
        name='mapless_wall_follower', output='screen',
        parameters=[L('wall_follower_params_file'), L('wall_follower_override_file'), sim],
        condition=IfCondition(L('use_wall_follower')))

    # Resolved only when start_apriltag is true, so zed_apriltag stays optional.
    apriltag = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('zed_apriltag'), 'launch', 'zed_apriltag.launch.py'])),
        launch_arguments={'tag_size': L('tag_size')}.items(),
        condition=IfCondition(L('start_apriltag')))

    supervisor = Node(
        package='course_supervisor', executable='course_supervisor', name='course_supervisor',
        output='screen',
        parameters=[L('supervisor_params_file'), sim, {
            'course_file': L('course_file'),
            'drive_cmd_topic': L('drive_cmd_vel_topic'),
            'smoother_out_topic': L('smoother_output_topic'),
            'use_nav2': flag('use_nav2'),
            'use_wall_follower': flag('use_wall_follower'),
            'use_path_tracker': flag('use_path_tracker'),
            'drive_lidar_gate': flag('drive_lidar_gate'),
            'use_apriltag_fix': flag('use_apriltag_fix'),
            'use_tag_triggers': flag('use_tag_triggers'),
            'tag_map_file': L('tag_map_file'),
            'tag_survey_file': L('tag_survey_file'),
            'wait_for_green_light': flag('wait_for_green_light'),
            'laps': ParameterValue(L('laps'), value_type=int),
        }])

    wf_share = get_package_share_directory('mapless_wall_follower')
    args = [
        DeclareLaunchArgument('course_file', default_value=os.path.join(share, 'config', 'obstacle_course.yaml')),
        DeclareLaunchArgument('supervisor_params_file',
                              default_value=os.path.join(share, 'config', 'course_supervisor.yaml')),
        # ackermann-drive subscribes to /cmd_vel_smoothed (driveStack ackermannDrive.yaml).
        DeclareLaunchArgument('drive_cmd_vel_topic', default_value='/cmd_vel_smoothed'),
        DeclareLaunchArgument('smoother_output_topic', default_value='/cmd_vel_smoother_out'),
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('nav2_params_file', default_value=os.path.join(bringup, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('nav2_override_params_file', default_value=os.path.join(
            bringup, 'config', 'nav2_params_3d_obstacle_course_mppi_ackermann.yaml')),
        DeclareLaunchArgument('use_speed_filter', default_value='false'),
        DeclareLaunchArgument('speed_filter_params_file', default_value=os.path.join(
            bringup, 'config', 'nav2_params_3d_obstacle_course_mppi_speed_filter_ackermann.yaml')),
        DeclareLaunchArgument('mask_yaml_file', default_value=os.path.join(
            bringup, 'maps', 'obstacle_course_speed_mask.yaml')),
        DeclareLaunchArgument('use_wall_follower', default_value='true'),
        DeclareLaunchArgument('wall_follower_params_file', default_value=os.path.join(
            wf_share, 'config', 'mapless_wall_follower_ackermann.yaml')),
        DeclareLaunchArgument('wall_follower_override_file', default_value=os.path.join(
            share, 'config', 'mapless_wall_follower_obstacle_course.yaml')),
        DeclareLaunchArgument('use_path_tracker', default_value='true'),
        DeclareLaunchArgument('drive_lidar_gate', default_value='false'),
        DeclareLaunchArgument('use_apriltag_fix', default_value='false'),
        DeclareLaunchArgument('use_tag_triggers', default_value='false'),
        DeclareLaunchArgument('start_apriltag', default_value='false'),
        # Printed tag edge length (black square), metres. Must match the printed tags.
        DeclareLaunchArgument('tag_size', default_value='0.16'),
        DeclareLaunchArgument('tag_map_file', default_value=os.path.join(
            share, 'config', 'tag_map_obstacle_course.yaml')),
        DeclareLaunchArgument('tag_survey_file', default_value=''),
        DeclareLaunchArgument('wait_for_green_light', default_value='true'),
        # Laps of the course before stopping (the run is 2 laps).
        DeclareLaunchArgument('laps', default_value='2'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]
    return LaunchDescription(args + [
        controller, controller_speed_filter, controller_lifecycle,
        smoother, smoother_lifecycle,
        mask_server, filter_info_server, filter_lifecycle,
        wall_follower, apriltag, supervisor,
    ])
