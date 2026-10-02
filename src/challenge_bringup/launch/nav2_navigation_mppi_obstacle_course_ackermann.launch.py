import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('challenge_bringup')

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
    behavior_cmd_vel_topic = LaunchConfiguration('behavior_cmd_vel_topic')
    drive_cmd_vel_topic = LaunchConfiguration('drive_cmd_vel_topic')

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        namespace=namespace,
        output='screen',
        parameters=[
            params_file,
            override_params_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel_mppi_raw'),
            # ackermann-drive (driveStack) subscribes to /cmd_vel_smoothed.
            ('cmd_vel_smoothed', drive_cmd_vel_topic),
        ],
    )

    smoother_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mppi_smoother',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['velocity_smoother'],
        }],
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
            description='Base 3D Ackermann Nav2 parameters',
        ),
        DeclareLaunchArgument(
            'override_params_file',
            default_value=os.path.join(
                bringup_dir,
                'config',
                'nav2_params_3d_obstacle_course_mppi_ackermann.yaml',
            ),
            description='Ackermann MPPI obstacle-course parameter overrides',
        ),
        DeclareLaunchArgument(
            'recovery_params_file',
            default_value=os.path.join(
                bringup_dir,
                'config',
                'nav2_params_no_overrides.yaml',
            ),
            description='Optional recovery policy loaded after MPPI parameters',
        ),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument(
            'container_name',
            default_value='nav2_container',
        ),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'drive_cmd_vel_topic',
            default_value='/cmd_vel_smoothed',
            description='Smoothed command for the Ackermann drive (ackermann-drive cmd_vel_topic)',
        ),
        DeclareLaunchArgument(
            'behavior_cmd_vel_topic',
            # Recoveries (BackUp reverses) bypass the forward-only smoother and
            # go straight to the drive, unless a wrapper overrides this.
            default_value=drive_cmd_vel_topic,
            description='Recovery behavior output topic',
        ),
        velocity_smoother,
        smoother_lifecycle_manager,
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
                'override_params_file': override_params_file,
                'recovery_params_file': recovery_params_file,
                'use_composition': use_composition,
                'container_name': container_name,
                'use_respawn': use_respawn,
                'log_level': log_level,
                'controller_cmd_vel_topic': 'cmd_vel_mppi_raw',
                'behavior_cmd_vel_topic': behavior_cmd_vel_topic,
            }.items(),
        ),
    ])
