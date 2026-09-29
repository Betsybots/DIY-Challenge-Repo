from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('challenge_bringup')
    use_rviz = LaunchConfiguration('use_rviz')
    startup_delay = LaunchConfiguration('startup_delay')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument(
            'startup_delay',
            default_value='5.0',
            description='Sensor startup delay passed to the shared bringup',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [bringup_dir, '/launch/master.launch.py']
            ),
            launch_arguments={
                'autonomous': 'true',
                'use_rviz': use_rviz,
                'startup_delay': startup_delay,
                'nav2_launch_file':
                    'nav2_navigation_mppi_obstacle_course_ackermann.launch.py',
            }.items(),
        ),
    ])
