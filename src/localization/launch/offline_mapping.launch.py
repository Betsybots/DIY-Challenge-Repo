#!/usr/bin/env python3
"""
diy_localization — offline_mapping.launch.py
═════════════════════════════════════════════
Wrapper for LIO-SAM prior-map generation sessions.

WHAT LIO-SAM DOES
─────────────────
LIO-SAM (Lidar Inertial Odometry via Smoothing And Mapping) is a
tightly-coupled lidar-inertial SLAM system.  IMU pre-integration
motion-compensates every lidar scan; scan matching builds odometry;
a factor graph with loop closure produces the globally consistent
3-D point-cloud map used for competition Nav2 localisation.

FOUR-NODE PIPELINE
──────────────────
  /hesai/points ──╮
  /imu/data     ──╰
       ↓
  imuPreintegration   — IMU pre-integration; pose prediction between scans
       ↓
  imageProjection     — range image + motion-distortion correction
       ↓
  featureExtraction   — edge (walls) + planar (floor) feature sets
       ↓
  mapOptimization     — factor-graph SLAM + loop closure → saves PCD + G2O

WORKFLOW
────────
  1. Source all overlays  (source scripts/env.sh [profile])
  2. Start mapping:        ros2 launch diy_localization offline_mapping.launch.py
  3. Drive the full mapping area at moderate speed
  4. Save the map:         ros2 service call /lio_sam/save_map ...
  5. Copy output:          cp -r ~/Documents/lio_sam_directory/  \
                                 src/challenge_bringup/maps/
  6. Update static_map.yaml to reference the new PCD files

Arguments
─────────
  lio_sam_config    path to LIO-SAM params YAML (default: auto-detect from lio_sam pkg)
  use_rviz          true | false  (default: true — mapping needs live visual feedback)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ─────────────────────────────────────────────────────────────────────────────
# OpaqueFunction callback — resolve config path and assemble the node list
# ─────────────────────────────────────────────────────────────────────────────
def launch_setup(context, *args, **kwargs):
    use_rviz       = LaunchConfiguration("use_rviz").perform(context)
    lio_sam_config = LaunchConfiguration("lio_sam_config").perform(context)

    # ─────────────────────────────────────────────────────────────────────────
    # BLOCK 1 — Resolve LIO-SAM config YAML path
    # ─────────────────────────────────────────────────────────────────────────
    # When the user provides an explicit path we use it as-is.
    # Otherwise we auto-detect the bundled params.yaml from the installed
    # lio_sam package — works after sourcing third_party_ws/install/setup.bash
    # (scripts/env.sh handles this automatically for the configured profile).
    if not lio_sam_config:
        try:
            pkg = get_package_share_directory("lio_sam")
            lio_sam_config = os.path.join(pkg, "config", "params.yaml")
        except Exception:
            raise RuntimeError(
                "lio_sam package not found. Source third_party_ws/install/setup.bash first."
            )

    nodes = [
        # ─────────────────────────────────────────────────────────────────────
        # BLOCK 2 — LIO-SAM four-node pipeline
        # ─────────────────────────────────────────────────────────────────────
        # Node 1: IMU pre-integration
        # Integrates raw IMU data between lidar scan timestamps to produce a
        # high-frequency initial pose prediction.  Motion-compensates each scan
        # so matching works correctly even during rapid starts / stops / turns.
        Node(
            package="lio_sam",
            executable="lio_sam_imuPreintegration",
            name="lio_sam_imuPreintegration",
            parameters=[lio_sam_config],
            output="screen",
        ),

        # Node 2: Image projection  (range image + motion-distortion correction)
        # Converts the raw 3-D PointCloud2 into a structured 2-D range image
        # and applies the IMU-derived motion correction to remove lidar distortion.
        # Also segments the ground plane and filters out-of-range points.
        Node(
            package="lio_sam",
            executable="lio_sam_imageProjection",
            name="lio_sam_imageProjection",
            parameters=[lio_sam_config],
            output="screen",
        ),

        # Node 3: Feature extraction
        # Computes per-point smoothness scores from the range image.
        # High smoothness → planar feature  (floor, ceiling, large flat surfaces).
        # Low smoothness  → edge feature    (walls, corners, poles, posts).
        # This sparse, two-class set is what scan matching operates on —
        # far fewer points than full-cloud ICP, enabling real-time performance.
        Node(
            package="lio_sam",
            executable="lio_sam_featureExtraction",
            name="lio_sam_featureExtraction",
            parameters=[lio_sam_config],
            output="screen",
        ),

        # Node 4: Map optimisation  (the SLAM back-end)
        # Builds a GTSAM factor graph combining:
        #   • Lidar odometry factors    (scan-to-submap ICP matching)
        #   • IMU pre-integration factors
        #   • GPS position factors      (when GPS is available in the config)
        #   • Loop closure factors      (radius-search + ICP verification)
        # On  ros2 service call /lio_sam/save_map  writes:
        #   ~/Documents/lio_sam_directory/CornerMap.pcd   (edge features)
        #   ~/Documents/lio_sam_directory/SurfaceMap.pcd  (planar features)
        #   ~/Documents/lio_sam_directory/trajectory.pcd  (robot path)
        Node(
            package="lio_sam",
            executable="lio_sam_mapOptimization",
            name="lio_sam_mapOptimization",
            parameters=[lio_sam_config],
            output="screen",
        ),
    ]

    # ─────────────────────────────────────────────────────────────────────────
    # BLOCK 3 — Optional RViz  (highly recommended during mapping)
    # ─────────────────────────────────────────────────────────────────────────
    # Default is true so the operator can see real-time loop-closure events
    # and verify map quality before saving.  Uses the lio_sam package's own
    # bundled rviz2.rviz config if available; otherwise launches blank.
    if use_rviz == "true":
        try:
            pkg = get_package_share_directory("lio_sam")
            rviz_cfg = os.path.join(pkg, "config", "rviz2.rviz")
        except Exception:
            rviz_cfg = ""

        # Only pass -d if the config file actually exists; guards against
        # a missing file causing rviz2 to crash at startup.
        rviz_args = ["-d", rviz_cfg] if rviz_cfg and os.path.exists(rviz_cfg) else []
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_mapping",
                arguments=rviz_args,
                output="screen",
            )
        )

    return nodes


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lio_sam_config",
                default_value="",
                description="Path to LIO-SAM params.yaml (empty = auto-detect)",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz for live map visualisation",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
