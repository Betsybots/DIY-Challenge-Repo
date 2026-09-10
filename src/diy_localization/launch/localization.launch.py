#!/usr/bin/env python3
"""
diy_localization — localization.launch.py
══════════════════════════════════════════
Brings up the full localisation stack for DIY Robot Challenge 2026.

DATA FLOW (runtime):
──────────────────────────────────────────────────────────────────────────────
  /hesai/points  ──╮
  /imu/data      ──╰─ FAST-LIO2  ──→  /lidar_odometry ──╮
                                                          │
  /wheel_odom    ──────────────────────────────────────╮ │
  /imu/data (angular rate only) ────────────────────────╮│ │
                                                         EKF (ekf_odom.yaml, 50 Hz)
                                                          →  /odometry/filtered
                                                             [odom → base_link TF for Nav2]

  Single EKF (not the old EKF1+EKF2 cascade — see ekf_odom.yaml's header for
  why): FAST-LIO2 already fuses /imu/data internally, so a second, separate
  robot_localization pre-fusion stage of the same IMU (the old EKF1) just
  double-counted it without adding real information. IMU is still fused
  directly here too, but angular-rate only, specifically so this EKF's own
  predict step stays accurate on wheel-slip terrain (gravel/pothole/bumps
  zones) between FAST-LIO2's ~10 Hz lidar corrections.

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
    # fast_lio_ros2 (the Hesai QT64 fork we vendor at src/fast_lio_ros2) uses
    # the STANDARD ROS 2 parameter mechanism: it declares each parameter with
    # rclcpp's declare_parameter() and expects the actual values via a normal
    # ROS 2 params YAML passed in the node's `parameters=` list (see its
    # own launch/lio_localizer.launch.py, which does exactly this). It does
    # NOT read a "config_path"/"config_file" pair of node parameters — that
    # was a leftover assumption from the old generic third_party_ws/FAST_LIO
    # package this repo used before switching to fast_lio_ros2. Passing
    # config_path/config_file (as this launch file previously did) silently
    # did nothing — fast_lio_ros2 never declares those parameter names, so it
    # just ran on 100% hardcoded defaults (wrong lidar/imu topics, wrong
    # extrinsics, etc.) regardless of what fast_lio_hesai_qt64.yaml said.
    #
    # Remaps hard-coded /Odometry → /lidar_odometry for consistent topic naming.
    if mode == "runtime":
        nodes.append(Node(
            package="fast_lio_ros2",
            executable="fastlio_mapping",
            name="fastlio_mapping",
            output="screen",
            parameters=[os.path.join(config_dir, config_file)],
            remappings=[("/Odometry", "/lidar_odometry")],
        ))

    # ─────────────────────────────────────────────────────────────────────────
    # BLOCK 2 — EKF: wheel + IMU (angular rate) + FAST-LIO2  (50 Hz)
    # ─────────────────────────────────────────────────────────────────────────
    # Fuses /wheel_odom + /imu/data (angular rate only) + /lidar_odometry_gated
    # into /odometry/filtered at 50 Hz. Publishes odom→base_link TF.
    #
    # This replaces the old two-stage EKF1(ekf_wimu.yaml)->EKF2(ekf_local.yaml)
    # cascade — see ekf_odom.yaml's header comment for why: FAST-LIO2 already
    # fuses /imu/data internally, so a separate upstream wheel+IMU EKF stage
    # just double-counted the same physical IMU without adding information.
    if mode == "runtime":
        nodes.append(Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            parameters=[os.path.join(config_dir, "ekf_odom.yaml")],
            remappings=[("odometry/filtered", "/odometry/filtered")],
        ))

    # ─────────────────────────────────────────────────────────────────────────
    # BLOCK 3 — NDT-OMP localization  (map → odom TF)
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
    # BLOCK 4 — Optional RViz
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
