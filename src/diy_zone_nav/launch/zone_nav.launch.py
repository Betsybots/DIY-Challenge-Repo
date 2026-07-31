"""
zone_nav.launch.py — Launch the zone-aware navigation state machine.

Usage:
    ros2 launch diy_zone_nav zone_nav.launch.py

Optional args:
    waypoints_file:=<path>   — override zone_waypoints.yaml location
    verbose:=false           — suppress zone entry/exit logs
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = get_package_share_directory("diy_zone_nav")

    # Default config files shipped with the package
    default_params  = os.path.join(pkg_share, "config", "zone_nav_manager.yaml")
    default_wpoints = os.path.join(pkg_share, "config", "zone_waypoints.yaml")

    # ── Launch arguments ──────────────────────────────────────────────────
    declare_waypoints = DeclareLaunchArgument(
        "waypoints_file",
        default_value=default_wpoints,
        description="Path to zone_waypoints.yaml"
    )
    declare_verbose = DeclareLaunchArgument(
        "verbose",
        default_value="true",
        description="Log zone transitions (true/false)"
    )
    declare_launch_gate = DeclareLaunchArgument(
        "launch_gate",
        default_value="true",
        description=(
            "Launch lidar_odom_gate_node alongside the state machine. "
            "Set to false when challenge_master.launch.py has already started it "
            "under use_localization to avoid duplicate gate nodes."
        ),
    )

    # ── Lidar odometry gate node ──────────────────────────────────────────
    # Blocks /lidar_odometry → EKF2 during BLIND_DRIVE so FAST-LIO2 garbage
    # in the tunnel doesn't corrupt the EKF2 pose estimate.
    # Subscribes to /nav_mode and /lidar_odometry.
    # Republishes on /lidar_odometry_gated (EKF2's odom1_topic).
    #
    # launch_gate:=false when challenge_master already started the gate node
    # under its use_localization block, to avoid two competing gate nodes.
    lidar_gate_node = Node(
        package="diy_zone_nav",
        executable="lidar_odom_gate_node",
        name="lidar_odom_gate",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_gate")),
    )

    # ── Zone nav manager node ─────────────────────────────────────────────
    zone_nav_node = Node(
        package="diy_zone_nav",
        executable="zone_nav_manager_node",
        name="zone_nav_manager",
        output="screen",
        parameters=[
            # Load YAML params first, then override specific fields
            default_params,
            {
                # Pass the resolved waypoints file path as a ROS parameter.
                # This overrides the empty default in zone_nav_manager.yaml.
                "zone_waypoints_file": LaunchConfiguration("waypoints_file"),
                # A bare LaunchConfiguration evaluates to a string, which the
                # node rejects because `verbose` is declared as a bool.
                "verbose": ParameterValue(
                    LaunchConfiguration("verbose"), value_type=bool),
            },
        ],
        # Remap cmd_vel if your motor driver uses a different topic name
        # remappings=[("/cmd_vel", "/base/cmd_vel")],
    )

    return LaunchDescription([
        declare_waypoints,
        declare_verbose,
        declare_launch_gate,
        lidar_gate_node,
        zone_nav_node,
    ])
