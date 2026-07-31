#!/usr/bin/env python3
"""
diy_localization — localization.launch.py
══════════════════════════════════════════
Brings up the full localisation stack for DIY Robot Challenge 2026.

DATA FLOW (runtime):
──────────────────────────────────────────────────────────────────────────────
  /wheel_cmd_vel ──╮
  /imu/data      ──╰─ EKF1 (ekf_wimu.yaml, 100 Hz)  ──→  /wimu_odom
                                                            │
  /hesai/points  ──╮                                        │
  /imu/data      ──╰─ FAST-LIO2  ──→  /lidar_odometry ─────╯
                                                            │
                      EKF2 (ekf_local.yaml, 50 Hz) ←───────╯
                      →  /odometry/filtered    [odom → base_link TF for Nav2]

  /hesai/points  ──── NDT-OMP (ndt_localizer_node) ──→  map → odom TF
                      (matches scan against GlobalMap.pcd)
                      →  /ndt_pose             [for optional EKF3 / monitoring]

WHY THIS REPLACES GPS:
──────────────────────
  GPS (navsat_transform) was used to anchor the map→odom TF.
  On our outdoor course GPS is unreliable (multipath, no RTK fix guarantee).
  NDT-OMP gives us cm-accurate map→odom from our own LIO-SAM point cloud map —
  no GPS dependency, works through the entire course including near metal structures.

WHY OpaqueFunction:
───────────────────
  ROS 2 launch substitutions are lazy. OpaqueFunction lets us call .perform(context)
  to get resolved string values for Python if/else branching.

Arguments:
──────────
  mode           runtime | mapping   (default: runtime)
  config_file    FAST-LIO2 YAML name (default: fast_lio_hesai_qt64.yaml)
  use_rviz       true | false        (default: false)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


# ─────────────────────────────────────────────────────────────────────────────
# OpaqueFunction callback — all runtime branching logic lives here.
# context.perform() resolves lazy LaunchConfiguration values into plain strings.
# ─────────────────────────────────────────────────────────────────────────────
def launch_setup(context, *args, **kwargs):
    pkg_loc    = get_package_share_directory("diy_localization")
    config_dir = os.path.join(pkg_loc, "config")

    mode        = LaunchConfiguration("mode").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    use_rviz    = LaunchConfiguration("use_rviz").perform(context)

    nodes = []

    # ─────────────────────────────────────────────────────────────────────────
    # BLOCK 1 — FAST-LIO2 lidar-inertial odometry  (runtime mode only)
    # ─────────────────────────────────────────────────────────────────────────
    # FAST-LIO2 does NOT use the standard ROS 2 parameter file mechanism.
    # Its internal config loader reads two node parameters at startup:
    #   config_path — absolute directory path containing the YAML (trailing / required)
    #   config_file — filename only, e.g. "fast_lio_hesai_qt64.yaml"
    #
    # Remaps hard-coded /Odometry → /lidar_odometry for consistent topic naming.
    if mode == "runtime":
        nodes.append(Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_mapping",
            output="screen",
            parameters=[{
                "config_path": config_dir + "/",  # trailing slash required by FAST-LIO2
                "config_file": config_file,
            }],
            remappings=[("/Odometry", "/lidar_odometry")],
        ))

    # ─────────────────────────────────────────────────────────────────────────
    # BLOCK 2 — EKF1: Wheel + IMU  (100 Hz, high-rate gap filler)
    # ─────────────────────────────────────────────────────────────────────────
    # Fuses /wheel_cmd_vel + /imu/data into /wimu_odom at 100 Hz.
    # This fills the 100ms gaps between FAST-LIO2 scans so EKF2 always has
    # a fresh input. EKF1 does NOT publish TF — EKF2 owns odom→base_link.
    if mode == "runtime":
        nodes.append(Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_wimu",
            output="screen",
            parameters=[os.path.join(config_dir, "ekf_wimu.yaml")],
            remappings=[("odometry/filtered", "/wimu_odom")],
        ))

    # ─────────────────────────────────────────────────────────────────────────
    # BLOCK 3 — EKF2: EKF1 output + FAST-LIO2  (50 Hz, smooth + drift-corrected)
    # ─────────────────────────────────────────────────────────────────────────
    # Fuses /wimu_odom (fast, from EKF1) + /lidar_odometry (accurate, from FAST-LIO2).
    # Publishes /odometry/filtered at 50 Hz — consumed by Nav2 MPPI controller.
    # Also publishes odom → base_link TF.
    if mode == "runtime":
        nodes.append(Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            parameters=[os.path.join(config_dir, "ekf_local.yaml")],
            remappings=[("odometry/filtered", "/odometry/filtered")],
        ))

    # ─────────────────────────────────────────────────────────────────────────
    # BLOCK 4 — NDT-OMP localization  (map → odom TF)
    # ─────────────────────────────────────────────────────────────────────────
    # Replaces the old GPS/navsat_transform/EKF2 approach.
    # Loads GlobalMap.pcd and matches each Hesai scan to produce map→odom TF.
    # This is the global position anchor — without it Nav2 cannot plan globally.
    #
    # Included as a separate launch file to keep concerns separated.
    # The NDT node's config (map path, resolution, etc.) lives in
    # diy_ndt_localization/config/ndt_localizer.yaml.
    if mode == "runtime":
        ndt_launch_path = os.path.join(
            get_package_share_directory("diy_ndt_localization"),
            "launch",
            "ndt_localization.launch.py",
        )
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ndt_launch_path),
            launch_arguments={"use_rviz": "false"}.items(),
        ))

    # ─────────────────────────────────────────────────────────────────────────
    # BLOCK 5 — Optional RViz
    # ─────────────────────────────────────────────────────────────────────────
    if use_rviz == "true":
        rviz_config = os.path.join(
            get_package_share_directory("challenge_bringup"), "rviz", "localization.rviz"
        )
        if os.path.exists(rviz_config):
            nodes.append(Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ))

    return nodes


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="runtime",
            description="runtime | mapping",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="fast_lio_hesai_qt64.yaml",
            description="FAST-LIO2 config YAML filename inside diy_localization/config/",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz with localization displays",
        ),
        # All node construction deferred to launch_setup() for Python-level branching
        OpaqueFunction(function=launch_setup),
    ]
    )
