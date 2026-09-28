import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('challenge_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start the Nav2 lifecycle nodes',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                bringup_dir,
                'config',
                'nav2_params_3d_speed_course_ackermann.yaml',
            ),
            description='Full path to the Ackermann speed-course Nav2 parameters',
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Use composed Nav2 bringup if true',
        ),
        DeclareLaunchArgument(
            'container_name',
            default_value='nav2_container',
            description='Nav2 component container name',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='Respawn Nav2 nodes after a crash',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Nav2 log level',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    bringup_dir,
                    'launch',
                    'nav2_navigation_launch.py',
                )
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_composition': use_composition,
                'container_name': container_name,
                'use_respawn': use_respawn,
                'log_level': log_level,
                # Route wait/recovery commands through velocity_smoother before
                # ackermann-drive receives them on /cmd_vel_smoothed.
                'behavior_cmd_vel_topic': 'cmd_vel',
            }.items(),
        ),
    ])
