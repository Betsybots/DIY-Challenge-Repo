from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='diy_world.sdf')
    robot_entity = 'betsybot'  # entity name spawned in Gazebo

    robot_pkg_share = get_package_share_directory('diy_robot_description')
    sim_pkg_share = get_package_share_directory('diy_sim')
    urdf_file = PathJoinSubstitution([robot_pkg_share, 'urdf', 'robot.urdf.xacro'])
    world_file = PathJoinSubstitution([sim_pkg_share, 'worlds', world])

    # 1) Start Gazebo Classic (server + GUI)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 2) robot_state_publisher publishes /robot_description
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                FindExecutable(name='xacro'), ' ', urdf_file
            ])
        }]
    )

    # 3) Spawn the model into Gazebo from /robot_description
    spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', robot_entity,
                   '-x', '0', '-y', '0', '-z', '0.05'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='empty.world'),
        gazebo,
        robot_state_pub,
        spawner
    ])

