"""
sim_competition.launch.py
--------------------------
Launch Ignition Gazebo (Fortress/Garden via ros_gz_sim) with the
competition arena world and spawn the DIY robot.

Usage:
  ros2 launch diy_sim sim_competition.launch.py
  ros2 launch diy_sim sim_competition.launch.py world:=diy_world.sdf
  ros2 launch diy_sim sim_competition.launch.py gui:=false   # headless
"""

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             SetEnvironmentVariable, TimerAction)
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                   PathJoinSubstitution, TextSubstitution)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world        = LaunchConfiguration('world',        default='competition_arena.sdf')
    gui          = LaunchConfiguration('gui',          default='true')
    robot_x      = LaunchConfiguration('robot_x',     default='0.0')
    robot_y      = LaunchConfiguration('robot_y',     default='0.0')
    robot_z      = LaunchConfiguration('robot_z',     default='0.05')
    robot_yaw    = LaunchConfiguration('robot_yaw',   default='0.0')

    robot_pkg_share  = get_package_share_directory('diy_robot_description')
    sim_pkg_share    = get_package_share_directory('diy_sim')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_file  = PathJoinSubstitution([robot_pkg_share, 'urdf', 'robot.urdf.xacro'])
    world_file = PathJoinSubstitution([sim_pkg_share, 'worlds', world])

    # Expose diy_sim/models so Ignition can resolve model:// URIs.
    # Also expose diy_robot_description/share so model://diy_robot_description/... resolves.
    # Set os.environ NOW so gz_sim.launch.py's OpaqueFunction picks it up
    # (it reads os.environ directly, not the ROS 2 launch environment).
    models_dir      = os.path.join(sim_pkg_share, 'models')
    robot_share_dir = os.path.dirname(robot_pkg_share)   # …/share  (parent of diy_robot_description)
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH',
                       os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''))
    new_gz_path = ':'.join(filter(None, [models_dir, robot_share_dir, existing_gz_path]))
    os.environ['GZ_SIM_RESOURCE_PATH']     = new_gz_path
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = new_gz_path

    # IGN_IP=127.0.0.1 forces Ignition transport to use loopback, bypassing
    # any VPN interfaces (e.g. cscotun0) that break UDP multicast discovery.
    os.environ.setdefault('IGN_IP', '127.0.0.1')

    # 1) Ignition Gazebo (server + optional GUI)
    #    gz_args: path to world + ' -r' (run immediately), passed as list so
    #    PathJoinSubstitution is resolved before the string is concatenated.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ros_gz_sim_share, '/launch/gz_sim.launch.py']
        ),
        launch_arguments={
            'gz_args': [world_file, TextSubstitution(text=' -r')],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # 2) robot_state_publisher — publishes TF and /robot_description
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command([FindExecutable(name='xacro'), ' ', urdf_file, ' sim:=true']),
                value_type=str
            )
        }]
    )

    # 3) Spawn the robot into Ignition from /robot_description topic
    spawner = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'diy_robot',
            '-x', robot_x,
            '-y', robot_y,
            '-z', robot_z,
            '-Y', robot_yaw,
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 4) Bridge essential topics between Ignition and ROS 2
    #    /clock        — sim time
    #    /cmd_vel      — drive commands (ROS2 → IGN)
    #    /odom         — wheel odometry (IGN → ROS2)
    #    /joint_states — wheel joint states (IGN → ROS2)
    #    /tf           — TF transforms (IGN → ROS2)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ]
    )

    # Static map → odom identity TF
    # SIM only: since there is no localization stack (FAST-LIO2 / EKF) in sim,
    # Nav2 needs a map→odom transform to exist.  Publishing identity (0,0,0)
    # means Nav2 treats odom frame == map frame — fine for testing the nav
    # pipeline without localization error.
    # NOTE: ros_gz_bridge forwards /tf with BARE frame names (no diy_robot/ prefix).
    # The odom frame published by Ignition's diff-drive/odometry plugins is simply "odom".
    # So the static TF must bridge map → odom (not map → diy_robot/odom).
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        # Set Ignition resource path + loopback IP for all child processes
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',      new_gz_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',  new_gz_path),
        SetEnvironmentVariable('IGN_IP', '127.0.0.1'),

        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('world', default_value='competition_arena.sdf',
                              description='SDF world file in diy_sim/worlds/'),
        DeclareLaunchArgument('gui',   default_value='true',
                              description='Launch Gazebo GUI (false = headless)'),
        DeclareLaunchArgument('robot_x',   default_value='0.0'),
        DeclareLaunchArgument('robot_y',   default_value='0.0'),
        DeclareLaunchArgument('robot_z',   default_value='0.05'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),

        robot_state_pub,
        gazebo,
        map_to_odom_tf,

        # Delay spawner + bridge by 8 s to let Ignition load the world
        TimerAction(period=8.0, actions=[spawner, bridge]),
    ])
