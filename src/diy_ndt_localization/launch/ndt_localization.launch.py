#!/usr/bin/env python3
"""
ndt_localization.launch.py
──────────────────────────────────────────────────────────────────────────────
Launches the NDT-OMP map-matching localization node.

What this starts:
  ndt_localizer_node  — loads GlobalMap.pcd, matches each Hesai scan,
                        publishes map->odom TF and /ndt_pose

This launch file is intentionally minimal — one node, one config file.
It is meant to be included from challenge_master.launch.py or run standalone
during localization testing.

Arguments:
  map_path    — override path to GlobalMap.pcd (default from config YAML)
  config_file — path to ndt_localizer.yaml (default: package config dir)
  use_rviz    — launch RViz with NDT visualisation (default: false)

Usage (standalone test):
  ros2 launch diy_ndt_localization ndt_localization.launch.py
  ros2 launch diy_ndt_localization ndt_localization.launch.py use_rviz:=true
──────────────────────────────────────────────────────────────────────────────
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("diy_ndt_localization")

    # Resolve launch arguments to plain Python strings
    config_file = LaunchConfiguration("config_file").perform(context)
    use_rviz    = LaunchConfiguration("use_rviz").perform(context)

    # Default config path is inside the installed package share directory
    if not config_file:
        config_file = os.path.join(pkg_dir, "config", "ndt_localizer.yaml")

    nodes = []

    # ── NDT localizer node ────────────────────────────────────────────────────
    # Loads GlobalMap.pcd, runs NDT-OMP per scan, publishes map->odom TF
    nodes.append(Node(
        package="diy_ndt_localization",
        executable="ndt_localizer_node",
        name="ndt_localizer_node",
        output="screen",
        parameters=[config_file],
        # Remap if Hesai driver uses a different topic name on your setup
        remappings=[
            ("/hesai/points", "/hesai/points"),
        ],
    ))

    # ── Optional RViz ─────────────────────────────────────────────────────────
    # Useful during first-time testing to visually verify map alignment
    if use_rviz == "true":
        rviz_config = os.path.join(pkg_dir, "config", "ndt_localizer.rviz")
        if os.path.exists(rviz_config):
            nodes.append(Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ))
        else:
            # Launch RViz without a preset config — add displays manually
            nodes.append(Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
            ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Path to ndt_localizer.yaml. Defaults to package config dir.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz for visual verification of map alignment.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
