"""
zone_nav.launch.py — Launch the zone-aware navigation state machine.

Usage:
    ros2 launch diy_zone_nav zone_nav.launch.py

Optional args:
    waypoints_file:=<path>          — override zone_waypoints.yaml location
    circuit_waypoints_file:=<path>  — override sim_circuit_waypoints.yaml location
    use_circuit_runner:=true        — start the closed-loop circuit orchestrator
    verbose:=false                  — suppress zone entry/exit logs
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory("diy_zone_nav")

    # Default config files shipped with the package
    default_params    = os.path.join(pkg_share, "config", "zone_nav_manager.yaml")
    default_wpoints   = os.path.join(pkg_share, "config", "zone_waypoints.yaml")
    default_circuit   = os.path.join(pkg_share, "config", "sim_circuit_waypoints.yaml")

    # ── Launch arguments ──────────────────────────────────────────────────
    declare_waypoints = DeclareLaunchArgument(
        "waypoints_file",
        default_value=default_wpoints,
        description="Path to zone_waypoints.yaml"
    )
    declare_circuit_waypoints = DeclareLaunchArgument(
        "circuit_waypoints_file",
        default_value=default_circuit,
        description="Path to sim_circuit_waypoints.yaml (used by circuit_runner_node)"
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
    declare_use_circuit_runner = DeclareLaunchArgument(
        "use_circuit_runner",
        default_value="false",
        description=(
            "Launch circuit_runner_node to autonomously drive the closed-loop circuit. "
            "Requires Nav2 (use_nav2:=true) and a /green_light signal to start."
        ),
    )
    # Sim override args — allow caller (sim_nav.launch.py) to remap topics
    # without modifying zone_nav_manager.yaml (which is the real-robot config).
    # REAL robot: defaults match zone_nav_manager.yaml exactly → no effect.
    # SIM:        sim_nav.launch.py passes /odom and a dummy mux service.
    declare_odom_topic = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odometry/filtered",  # REAL default
        description="Odometry topic. SIM: /odom  |  REAL: /odometry/filtered",
    )
    declare_mux_service = DeclareLaunchArgument(
        "mux_param_service",
        default_value="/cmd_vel_mux_node/set_parameters",  # REAL default
        description="cmd_vel_mux SetParameters service. SIM: /nonexistent/dummy to suppress warns",
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
            default_params,
            {
                "zone_waypoints_file": LaunchConfiguration("waypoints_file"),
                "verbose":             LaunchConfiguration("verbose"),
                # Sim/real switchable topics — see declare_odom_topic / declare_mux_service above
                "odom_topic":          LaunchConfiguration("odom_topic"),
                "mux_param_service":   LaunchConfiguration("mux_param_service"),
            },
        ],
    )

    # ── Circuit runner node ───────────────────────────────────────────────
    # Submits the closed-loop circuit waypoints to Nav2 FollowWaypoints and
    # re-submits after each lap until zone_nav_manager publishes DONE.
    # Only started when use_circuit_runner:=true.
    circuit_runner_node = Node(
        package="diy_zone_nav",
        executable="circuit_runner_node",
        name="circuit_runner",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_circuit_runner")),
        parameters=[{
            "circuit_waypoints_file": LaunchConfiguration("circuit_waypoints_file"),
            "retry_delay_s": 3.0,
            "action_server_timeout_s": 10.0,
        }],
    )

    return LaunchDescription([
        declare_waypoints,
        declare_circuit_waypoints,
        declare_verbose,
        declare_launch_gate,
        declare_use_circuit_runner,
        declare_odom_topic,
        declare_mux_service,
        lidar_gate_node,
        zone_nav_node,
        circuit_runner_node,
    ])
