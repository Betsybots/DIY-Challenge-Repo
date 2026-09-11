#!/usr/bin/env python3
"""
diy_localization — localization.launch.py
══════════════════════════════════════════
Brings up the full localisation stack for DIY Robot Challenge 2026.

DATA FLOW (runtime):
──────────────────────────────────────────────────────────────────────────────
  /hesai/points  ──╮
  /imu/data      ──╰─ FAST-LIO2  ──→  /lidar_odometry ──╮
                                       /cloud_registered_body │
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

  /lidar_odometry ──╮
  /cloud_registered_body ──╯── map_localizer (VGICP) ──→  map → odom TF
                       (matches scan against a saved map, e.g. maps/refined_map.pcd;
                        map must be loaded via trigger_map_relocalize.py's
                        one-shot /relocalize call at startup — no auto-load)

  Replaces the old NDT-OMP (diy_ndt_localization) approach — see
  diy_localization/config/map_localizer.yaml's header for the full
  rationale (NDT-OMP had a real TF-composition bug: it broadcast the raw
  scan-matched map→base_link pose directly as "map→odom", without ever
  composing against the actual odom→base_link transform).

WHY THIS REPLACES GPS:
──────────────────────
  GPS (navsat_transform) was used to anchor the map→odom TF.
  On our outdoor course GPS is unreliable (multipath, no RTK fix guarantee).
  map_localizer gives us cm-accurate map→odom from our own LIO-SAM/map_hba
  point cloud map — no GPS dependency, works through the entire course
  including near metal structures.

WHY OpaqueFunction:
───────────────────
  ROS 2 launch substitutions are lazy. OpaqueFunction lets us call .perform(context)
  to get resolved string values for Python if/else branching.

Arguments:
──────────
  mode           runtime | mapping   (default: runtime)
  config_file    FAST-LIO2 YAML name (default: fast_lio_hesai_qt64.yaml)
  use_rviz       true | false        (default: false)
  map_pcd_path   Absolute path to the saved map .pcd for map_localizer
                 (default: $DIY_ROS_WS/src/DIY-Challenge-Repo/maps/refined_map.pcd)
  initial_x/y/z/yaw/pitch/roll   Initial pose guess for map_localizer's
                 first relocalize call (default: 0.0, i.e. map origin)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
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
    # BLOCK 3 — map_localizer  (map → odom TF via VGICP against a saved map)
    # ─────────────────────────────────────────────────────────────────────────
    # Replaces the old NDT-OMP (diy_ndt_localization) approach — see
    # diy_localization/config/map_localizer.yaml's header for the full
    # rationale. Short version: NDT-OMP's publishTF() had a real TF-
    # composition bug (broadcast raw map→base_link as if it were map→odom,
    # with no odom→base_link lookup/composition at all — verified by
    # reading the code, no tf2_ros::Buffer/TransformListener existed in
    # that class despite the headers being included). map_localizer's own
    # TF math was checked term-by-term and is correct: it composes
    # map→odom = (map→body, from VGICP against the saved map) ×
    # inverse(odom→body, from FAST-LIO2's own /Odometry) — the same pattern
    # AMCL/every real localization node uses. diy_ndt_localization is kept
    # in the repo (unused by this launch file) in case of rollback — see
    # its own package header for the deprecation note.
    #
    # map_localizer does NOT auto-load a map at startup: it only loads a PCD
    # (and sets the initial pose guess) in response to a /relocalize service
    # call — trigger_map_relocalize.py makes that call once, waiting for the
    # service to come up first. Without it, map_localizer runs but silently
    # never produces any output (no error, no crash — just permanently idle).
    if mode == "runtime":
        map_pcd_path = LaunchConfiguration("map_pcd_path").perform(context)

        nodes.append(Node(
            package="map_localizer",
            executable="map_localizer_node",
            name="map_localizer_node",
            output="screen",
            parameters=[{"config_path": os.path.join(config_dir, "map_localizer.yaml")}],
        ))

        nodes.append(Node(
            package="diy_localization",
            executable="trigger_map_relocalize.py",
            name="trigger_map_relocalize",
            output="screen",
            parameters=[{
                "pcd_path": map_pcd_path,
                # Robot assumed to start at map origin (start line) — same
                # assumption diy_ndt_localization documented; override via
                # this launch file's initial_x/y/z/yaw/pitch/roll args if
                # the robot is ever started elsewhere on the map.
                # NOTE: LaunchConfiguration.perform() always returns a str —
                # must cast to float here, since trigger_map_relocalize.py
                # declares these as double parameters. Passing the raw
                # string raises rclpy.exceptions.InvalidParameterTypeException
                # at node startup (confirmed by actually launching this).
                "initial_x": float(LaunchConfiguration("initial_x").perform(context)),
                "initial_y": float(LaunchConfiguration("initial_y").perform(context)),
                "initial_z": float(LaunchConfiguration("initial_z").perform(context)),
                "initial_yaw": float(LaunchConfiguration("initial_yaw").perform(context)),
                "initial_pitch": float(LaunchConfiguration("initial_pitch").perform(context)),
                "initial_roll": float(LaunchConfiguration("initial_roll").perform(context)),
            }],
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
        DeclareLaunchArgument(
            "map_pcd_path",
            default_value=(
                os.path.join(
                    os.environ.get("DIY_ROS_WS", ""),
                    "src", "DIY-Challenge-Repo", "maps", "refined_map.pcd",
                )
                if os.environ.get("DIY_ROS_WS") else ""
            ),
            description=(
                "Absolute path to the saved map .pcd for map_localizer "
                "(produced by LIO_Localization's map_hba offline refinement). "
                "Defaults to $DIY_ROS_WS/src/DIY-Challenge-Repo/maps/refined_map.pcd "
                "when DIY_ROS_WS is set (see profiles/*.env) — override "
                "explicitly if that doesn't match your checkout layout."
            ),
        ),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_z", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        DeclareLaunchArgument("initial_pitch", default_value="0.0"),
        DeclareLaunchArgument("initial_roll", default_value="0.0"),
        # All node construction deferred to launch_setup() for Python-level branching
        OpaqueFunction(function=launch_setup),
    ]
    )
