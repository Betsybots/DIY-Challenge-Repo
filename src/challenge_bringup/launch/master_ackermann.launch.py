import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('challenge_bringup')
    startup_delay = LaunchConfiguration('startup_delay')

    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'master.launch.py')
        ),
        launch_arguments={
            'autonomous': 'true',
            'startup_delay': startup_delay,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir,
                'launch',
                'nav2_navigation_ackermann.launch.py',
            )
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'startup_delay',
            default_value='5.0',
            description='Seconds to wait before starting Ackermann Nav2',
        ),
        base_bringup,
        TimerAction(period=startup_delay, actions=[nav2]),
    ])
