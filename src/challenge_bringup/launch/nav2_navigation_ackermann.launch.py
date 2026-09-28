import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('challenge_bringup')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                bringup_dir,
                'config',
                'nav2_params_3d_ackermann.yaml',
            ),
            description='Full path to the Ackermann Nav2 parameter file',
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
                'params_file': params_file,
                'behavior_cmd_vel_topic': 'cmd_vel',
            }.items(),
        ),
    ])
