#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# run_ndt_localization.sh
# Runs the NDT-OMP map-matching localization node.
#
# WHAT THIS DOES:
#   Starts ndt_localizer_node which:
#     1. Loads GlobalMap.pcd from the path set in ndt_localizer.yaml
#     2. Subscribes to /hesai/points (Hesai QT64 lidar at 10 Hz)
#     3. Runs NDT-OMP alignment per scan
#     4. Publishes:
#          map->odom TF       — consumed by Nav2 for global planning
#          /ndt_pose          — fed into EKF2 as map-frame correction
#          /ndt_fitness_score — monitor this in RViz; high = bad match
#
# PRE-REQUISITES:
#   1. Robot workspace must be built: colcon build in ~/robot_ws
#   2. GlobalMap.pcd must exist at the path set in ndt_localizer.yaml
#      (Run offline_mapping first if you don't have a map yet)
#   3. Hesai driver must be running: /hesai/points must be publishing
#   4. FAST-LIO2 and EKF1 should be running (see run_localization.sh)
#
# USAGE:
#   ./run_ndt_localization.sh              # normal run
#   ./run_ndt_localization.sh --rviz       # with RViz for visual verification
#   ./run_ndt_localization.sh --map PATH   # override map path
#
# MONITORING (in a separate terminal):
#   ros2 topic echo /ndt_fitness_score     # watch match quality (target < 1.0)
#   ros2 topic echo /ndt_pose              # watch pose estimates
#   ros2 run tf2_tools view_frames         # verify map->odom->base_link chain
# ─────────────────────────────────────────────────────────────────────────────

set -e  # exit immediately if any command fails

# ── Source ROS 2 and robot workspace ──────────────────────────────────────────
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash

# ── Parse arguments ───────────────────────────────────────────────────────────
USE_RVIZ="false"
MAP_PATH=""

while [[ "$#" -gt 0 ]]; do
  case $1 in
    --rviz)   USE_RVIZ="true" ;;
    --map)    MAP_PATH="$2"; shift ;;
    *)        echo "Unknown argument: $1"; exit 1 ;;
  esac
  shift
done

# ── Build launch command ───────────────────────────────────────────────────────
LAUNCH_CMD="ros2 launch diy_ndt_localization ndt_localization.launch.py use_rviz:=${USE_RVIZ}"

# Append map override if provided
if [[ -n "$MAP_PATH" ]]; then
  echo "[NDT] Using map: $MAP_PATH"
  LAUNCH_CMD="${LAUNCH_CMD} map_path:=${MAP_PATH}"
fi

echo "────────────────────────────────────────────────────────────"
echo "  Team Juggernauts — NDT-OMP Localization"
echo "  Map  : ${MAP_PATH:-see ndt_localizer.yaml}"
echo "  RViz : ${USE_RVIZ}"
echo "────────────────────────────────────────────────────────────"
echo ""
echo "  Monitor fitness score in another terminal:"
echo "    ros2 topic echo /ndt_fitness_score"
echo "  Good match = score < 1.0"
echo "  Bad match  = score > 1.0 (TF held from last good pose)"
echo ""

exec $LAUNCH_CMD
