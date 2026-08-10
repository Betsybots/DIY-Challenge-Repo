"""
sim_simple_loop.launch.py
--------------------------
All-in-one launch: Ignition Gazebo (simple_loop.sdf) + Nav2 + circuit_runner.

This is the single command teammates need to replicate the nav pipeline
regression test on the simple_loop track.

Usage:
  # Build first (once):
  cd ~/DIY-Challenge-Repo && colcon build --symlink-install
  source install/setup.bash

  # Launch everything:
  ros2 launch diy_sim sim_simple_loop.launch.py

  # In a NEW terminal — fire the start signal:
  source ~/DIY-Challenge-Repo/install/setup.bash
  ros2 topic pub --once /green_light std_msgs/msg/Bool "data: true"

  # In another terminal — live telemetry:
  python3 src/diy_sim/scripts/sim_telemetry.py

  # Record trajectory to CSV + plot:
  python3 scripts/record_trajectory.py --out /tmp/run1

Optional args:
  gui:=false              — headless Gazebo (CI / SSH)
  use_rviz:=true          — open RViz2 for costmap / path visualisation
  use_circuit_runner:=false — skip auto-nav, use 2D Nav Goal in RViz manually
  world:=competition_arena.sdf — swap to the full competition world

Timing (automatic):
  t=0    — Ignition Gazebo starts
  t=8s   — Robot spawned + ROS-IGN bridge starts
  t=18s  — Nav2 + circuit_runner start (waits for bridge)
  t=?    — /green_light=true  → robot begins lap
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                   PathJoinSubstitution, TextSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# ── Timing constants ────────────────────────────────────────────────────────
SPAWN_DELAY_S  = 8.0   # seconds until Ignition has loaded the world
NAV2_DELAY_S   = 18.0  # seconds until Nav2 stack starts (after bridge is up)


def _nav2_stack(context, use_circuit_runner_lc, use_zone_nav_lc,
                use_rviz_lc, circuit_waypoints_lc):
    """OpaqueFunction: resolve paths at runtime and return nav2 nodes."""
    bringup_pkg  = get_package_share_directory('challenge_bringup')
    zone_nav_pkg = get_package_share_directory('diy_zone_nav')

    nav2_params       = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')
    keepout_mask_yaml = os.path.join(bringup_pkg, 'maps', 'keepout_mask.yaml')
    zone_nav_launch   = os.path.join(zone_nav_pkg, 'launch', 'zone_nav.launch.py')

    circuit_wp_default = os.path.join(
        zone_nav_pkg, 'config', 'sim_simple_loop_waypoints.yaml')
    circuit_wp = context.perform_substitution(circuit_waypoints_lc).strip()
    if not circuit_wp:
        circuit_wp = circuit_wp_default

    use_circuit = context.perform_substitution(use_circuit_runner_lc).lower()
    use_zone    = context.perform_substitution(use_zone_nav_lc).lower()
    use_rviz    = context.perform_substitution(use_rviz_lc).lower()

    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py',
            )
        ),
        launch_arguments={
            'params_file':  nav2_params,
            'use_sim_time': 'true',
            'autostart':    'True',
        }.items(),
    )

    # Keepout filter mask server (blocks inner-corner inflation pockets)
    filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': keepout_mask_yaml,
            'topic_name': '/keepout_filter_mask',
            'frame_id': 'map',
        }],
    )
    costmap_filter_info = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'type': 0,
            'filter_info_topic': '/costmap_filter_info',
            'mask_topic': '/keepout_filter_mask',
            'base': 0.0,
            'multiplier': 1.0,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
    ) if use_rviz == 'true' else None

    actions = [nav2_include, filter_mask_server, costmap_filter_info]
    if rviz_node:
        actions.append(rviz_node)

    if use_zone == 'true':
        zone_nav = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zone_nav_launch),
            launch_arguments={
                'launch_gate':             'false',
                'use_circuit_runner':      use_circuit,
                'odom_topic':              '/odom',
                'mux_param_service':       '/nonexistent/dummy',
                'circuit_waypoints_file':  circuit_wp,
            }.items(),
        )
        actions.append(zone_nav)

    return actions


def generate_launch_description():

    sim_pkg_share   = get_package_share_directory('diy_sim')
    robot_pkg_share = get_package_share_directory('diy_robot_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world', default='simple_loop.sdf')
    use_sim_time        = LaunchConfiguration('use_sim_time',        default='true')
    use_circuit_runner  = LaunchConfiguration('use_circuit_runner',  default='true')
    use_zone_nav        = LaunchConfiguration('use_zone_nav',        default='true')
    use_rviz            = LaunchConfiguration('use_rviz',            default='false')
    circuit_waypoints   = LaunchConfiguration('circuit_waypoints_file', default='')
    robot_x             = LaunchConfiguration('robot_x',   default='0.0')
    robot_y             = LaunchConfiguration('robot_y',   default='0.0')
    robot_z             = LaunchConfiguration('robot_z',   default='0.05')
    robot_yaw           = LaunchConfiguration('robot_yaw', default='0.0')

    urdf_file  = PathJoinSubstitution([robot_pkg_share, 'urdf', 'robot.urdf.xacro'])
    world_file = PathJoinSubstitution([sim_pkg_share, 'worlds', world])

    # Expose models so Ignition can resolve model:// URIs
    models_dir       = os.path.join(sim_pkg_share, 'models')
    robot_share_dir  = os.path.dirname(robot_pkg_share)
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH',
                       os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''))
    new_gz_path = ':'.join(filter(None, [models_dir, robot_share_dir, existing_gz_path]))
    os.environ['GZ_SIM_RESOURCE_PATH']     = new_gz_path
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = new_gz_path
    # Loopback IP prevents VPN interfaces from breaking Ignition transport
    os.environ.setdefault('IGN_IP', '127.0.0.1')

    # ── 1. Ignition Gazebo ──────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ros_gz_sim_share, '/launch/gz_sim.launch.py']
        ),
        launch_arguments={
            'gz_args':        [world_file, TextSubstitution(text=' -r')],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # ── 2. Robot state publisher ────────────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command([FindExecutable(name='xacro'), ' ', urdf_file]),
                value_type=str,
            ),
        }],
    )

    # ── 3. Robot spawner (delayed — world must load first) ──────────────────
    spawner = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'diy_robot',
            '-x', robot_x, '-y', robot_y,
            '-z', robot_z, '-Y', robot_yaw,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 4. ROS-IGN topic bridge (delayed with spawner) ──────────────────────
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
        ],
    )

    # Lidar bridge — scan used by Nav2 local costmap
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_lidar_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        ],
    )

    # ── 5. Static map → odom TF (identity — no localization in sim) ─────────
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        # Propagate env vars to child processes
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',     new_gz_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', new_gz_path),
        SetEnvironmentVariable('IGN_IP', '127.0.0.1'),

        # ── Declare args ──────────────────────────────────────────────────
        DeclareLaunchArgument('world',    default_value='simple_loop.sdf',
                              description='World file in diy_sim/worlds/ '
                                          '(e.g. simple_loop.sdf, competition_arena.sdf)'),
        DeclareLaunchArgument('use_sim_time',       default_value='true'),
        DeclareLaunchArgument('use_circuit_runner', default_value='true',
                              description='Auto-navigate using circuit_runner'),
        DeclareLaunchArgument('use_zone_nav',       default_value='true',
                              description='Include zone_nav state machine'),
        DeclareLaunchArgument('use_rviz',           default_value='false',
                              description='Open RViz2 for costmap visualisation'),
        DeclareLaunchArgument('circuit_waypoints_file', default_value='',
                              description='Override waypoints YAML path'),
        DeclareLaunchArgument('robot_x',   default_value='0.0'),
        DeclareLaunchArgument('robot_y',   default_value='0.0'),
        DeclareLaunchArgument('robot_z',   default_value='0.05'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0'),

        # ── Immediate (t=0) ───────────────────────────────────────────────
        gazebo,
        robot_state_pub,
        map_to_odom_tf,

        # ── Delayed: spawn robot + bridge (t=8s) ─────────────────────────
        TimerAction(period=SPAWN_DELAY_S, actions=[spawner, bridge, lidar_bridge]),

        # ── Delayed: Nav2 + circuit_runner (t=18s) ───────────────────────
        TimerAction(
            period=NAV2_DELAY_S,
            actions=[
                OpaqueFunction(
                    function=_nav2_stack,
                    args=[use_circuit_runner, use_zone_nav,
                          use_rviz, circuit_waypoints],
                )
            ],
        ),
    ])
