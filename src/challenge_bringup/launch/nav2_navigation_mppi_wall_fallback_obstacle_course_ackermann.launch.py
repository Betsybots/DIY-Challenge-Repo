import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('challenge_bringup')
    wall_follower_dir = get_package_share_directory('mapless_wall_follower')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    override_params_file = LaunchConfiguration('override_params_file')
    recovery_params_file = LaunchConfiguration('recovery_params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    wall_follower = Node(
        package='mapless_wall_follower',
        executable='mapless_wall_follower_node',
        name='mapless_wall_follower',
        output='screen',
        parameters=[
            os.path.join(
                wall_follower_dir,
                'config',
                'mapless_wall_follower_ackermann.yaml',
            ),
            {
                # This private command reaches the vehicle only while Nav2's
                # AssistedTeleop wall-fallback recovery is active.
                'start_enabled': True,
                'cmd_vel_topic': '/cmd_vel_wall_follower',
            },
        ],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir,
                'launch',
                'nav2_navigation_mppi_obstacle_course_ackermann.launch.py',
            )
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'override_params_file': override_params_file,
            'recovery_params_file': recovery_params_file,
            'use_composition': use_composition,
            'container_name': container_name,
            'use_respawn': use_respawn,
            'log_level': log_level,
            # MPPI and wall recovery are mutually exclusive and share the
            # smoother input; the smoother alone publishes /cmd_vel_smoothed.
            'behavior_cmd_vel_topic': 'cmd_vel_mppi_raw',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                bringup_dir,
                'config',
                'nav2_params.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'override_params_file',
            default_value=os.path.join(
                bringup_dir,
                'config',
                'nav2_params_3d_obstacle_course_mppi_ackermann.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'recovery_params_file',
            default_value=os.path.join(
                bringup_dir,
                'config',
                'nav2_params_3d_obstacle_course_mppi_wall_fallback_ackermann.yaml',
            ),
        ),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
        wall_follower,
        navigation,
    ])
