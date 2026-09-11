"""
description.launch.py — diy_robot_description
═══════════════════════════════════════════════════════════════════════════════
PURPOSE
  Starts robot_state_publisher with the robot URDF. This establishes the
  full static TF tree (base_link → lidar_link, imu_link, camera_link, etc.)
  before any other node is launched.

WHY THIS MUST BE THE FIRST THING LAUNCHED
  Almost every other node in the stack depends on the TF tree:
    • FAST-LIO2 needs lidar_link → base_link to project points into the
      robot body frame for extrinsic compensation.
    • Nav2 needs base_link → lidar_link to build the costmap from lidar points.
    • RViz cannot display the robot model or sensor data correctly without it.
  Launching this first ensures TF is available before sensors start streaming.

WHAT IT PUBLISHES
  /robot_description  (std_msgs/String)  — raw URDF XML (used by RViz, Nav2)
  /tf_static          (tf2_msgs/TFMessage) — all fixed joints in the URDF

HOW THE XACRO IS PROCESSED
  The URDF is authored as a Xacro template (robot.urdf.xacro). The `Command`
  substitution runs `xacro <path>` at launch time, converting the Xacro XML
  into a plain URDF string that is passed as the robot_description parameter.
  This means you can edit the Xacro and relaunch without rebuilding.

CALIBRATION REMINDERS
  All joint transforms in robot.urdf.xacro are currently placeholder identity
  transforms marked with "CALIB:". After running calibrate_extrinsics.sh,
  update those joint xyz/rpy values with the measured transforms.
═══════════════════════════════════════════════════════════════════════════════
"""

import os
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # ── Resolve the installed package path ───────────────────────────────────
    # get_package_share_directory returns the installed share/ path so this
    # works regardless of whether the package is built in-place or installed.
    pkg_dir  = get_package_share_directory('diy_robot_description')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    raw_robot_desc = Command(['xacro', ' ', urdf_path, ' sim:=false'])
    robot_desc_param = ParameterValue(raw_robot_desc, value_type=str)

    # ── Launch argument ───────────────────────────────────────────────────────
    # use_sim_time allows this launch to be used in Gazebo/ROS bag replay
    # without modification — just pass use_sim_time:=true from the parent.
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Declare the argument with a safe default of 'false' (real hardware)
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock (set true for Gazebo/bag replay)'),

        # ── robot_state_publisher ────────────────────────────────────────────
        # Reads robot_description (URDF) and publishes:
        #   • /tf_static  — all fixed joints (base_link→lidar_link, etc.)
        #   • /robot_description — raw URDF string for RViz / Nav2
        #
        # Command(['xacro ', urdf_path]) runs as a shell command at launch time.
        # The output (URDF XML string) is stored in the robot_description param.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc_param,
                'use_sim_time':      use_sim_time,
            }],
        ),

        # ── joint_state_publisher ────────────────────────────────────────────
        # Continuous/revolute joints (wheels) need /joint_states before
        # robot_state_publisher can broadcast their TF — without this node
        # those links never appear in the tf tree (e.g. view_frames output).
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])