#!/usr/bin/env python3

"""
Static-map navigation stack for the custom A* and PD controllers.

Pipeline:
    saved map YAML/PGM
        -> map_server
        -> /map

    /map + /goal_pose
        -> A* planner
        -> /a_star/path

    /a_star/path
        -> PD motion planner
        -> cmd_vel_topic

Simulation:
    cmd_vel_topic := /cmd_vel
    base_frame    := base_footprint
    use_sim_time  := true

Hardware:
    cmd_vel_topic := /cmd_vel_nav
    base_frame    := base_link
    use_sim_time  := false

Example goal:
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ============================================================
    # Launch configurations
    # ============================================================

    map_yaml = LaunchConfiguration('map_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')

    robot_clearance = LaunchConfiguration('robot_clearance')

    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    kp = LaunchConfiguration('kp')
    kd = LaunchConfiguration('kd')
    step_size = LaunchConfiguration('step_size')

    max_linear_velocity = LaunchConfiguration(
        'max_linear_velocity'
    )

    max_angular_velocity = LaunchConfiguration(
        'max_angular_velocity'
    )

    goal_tolerance = LaunchConfiguration(
        'goal_tolerance'
    )

    # ============================================================
    # Launch description
    # ============================================================

    return LaunchDescription([

        # --------------------------------------------------------
        # Map
        # --------------------------------------------------------

        DeclareLaunchArgument(
            'map_yaml',
            description='Absolute path to the saved map YAML file.'
        ),

        # --------------------------------------------------------
        # Time
        # --------------------------------------------------------

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use ROS simulation clock.'
        ),

        # --------------------------------------------------------
        # TF frames
        # --------------------------------------------------------

        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description=(
                'Robot base frame. '
                'Use base_footprint in simulation '
                'and base_link on hardware.'
            )
        ),

        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Local odometry frame.'
        ),

        # --------------------------------------------------------
        # A* planner parameters
        # --------------------------------------------------------

        DeclareLaunchArgument(
            'robot_clearance',
            default_value='0.25',
            description=(
                'Minimum clearance from static obstacles '
                'used by A* in meters.'
            )
        ),

        # --------------------------------------------------------
        # PD controller output
        # --------------------------------------------------------

        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description=(
                'PD velocity output topic. '
                'Use /cmd_vel in simulation and '
                '/cmd_vel_nav on hardware.'
            )
        ),

        # --------------------------------------------------------
        # PD gains
        # --------------------------------------------------------

        DeclareLaunchArgument(
            'kp',
            default_value='2.0',
            description='PD proportional gain.'
        ),

        DeclareLaunchArgument(
            'kd',
            default_value='0.1',
            description='PD derivative gain.'
        ),

        DeclareLaunchArgument(
            'step_size',
            default_value='0.2',
            description='PD path look-ahead distance in meters.'
        ),

        DeclareLaunchArgument(
            'max_linear_velocity',
            default_value='0.3',
            description='Maximum commanded linear velocity in m/s.'
        ),

        DeclareLaunchArgument(
            'max_angular_velocity',
            default_value='1.0',
            description='Maximum commanded angular velocity in rad/s.'
        ),

        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='0.15',
            description='Final goal stopping tolerance in meters.'
        ),

        # ========================================================
        # Map server
        # ========================================================

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',

            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
            }],
        ),

        # ========================================================
        # Map lifecycle manager
        # ========================================================

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='map_lifecycle_manager',
            output='screen',

            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'map_server'
                ],
            }],
        ),

        # ========================================================
        # Custom A* planner
        # ========================================================

        Node(
            package='diy_planning',
            executable='a_star_planner_node',
            name='a_star_planner_node',
            output='screen',

            parameters=[{
                'use_sim_time': use_sim_time,

                'base_frame': base_frame,

                'robot_clearance': robot_clearance,
            }],
        ),

        # ========================================================
        # Custom PD controller
        # ========================================================

        Node(
            package='diy_motion_planner',
            executable='pd_motion_planner_node',
            name='pd_motion_planner_node',
            output='screen',

            parameters=[{
                'use_sim_time': use_sim_time,

                'odom_frame': odom_frame,
                'base_frame': base_frame,

                'cmd_vel_topic': cmd_vel_topic,

                'kp': kp,
                'kd': kd,

                'step_size': step_size,

                'max_linear_velocity':
                    max_linear_velocity,

                'max_angular_velocity':
                    max_angular_velocity,

                'goal_tolerance':
                    goal_tolerance,
            }],
        ),
    ])