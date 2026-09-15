#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/view_global_costmap.sh — Visualize the global costmap for the
# current course map (course_traced_smooth.yaml, derived from refined_map.pcd)
# ══════════════════════════════════════════════════════════════════════════════
# Standalone sanity-check tool: stands up nav2_map_server + a real Nav2
# global costmap (static_layer + inflation_layer) against the static course
# map, and opens RViz to view it. NOT part of the active custom nav stack --
# a_star_planner_node subscribes directly to the raw /map OccupancyGrid and
# does its own internal binary-radius inflation; it never reads this
# costmap. No robot, localizer, or lidar required.
#
# Usage:
#   ./scripts/view_global_costmap.sh [profile] [--map-yaml PATH] [--no-rviz]
#
# Verify:
#   ros2 topic echo /costmap/costmap --field info   — confirm it matches the map
#   rviz2 (opened automatically)                     — Map + Costmap displays
# ══════════════════════════════════════════════════════════════════════════════
set -eo pipefail
# NOTE: deliberately NOT using "set -u" here -- env.sh's own header
# explicitly warns against it, since it re-sources /opt/ros/humble/setup.bash
# internally, which is not nounset-safe (confirmed empirically, 2026-09-11:
# "set -u" + sourcing env.sh reliably crashes with "AMENT_TRACE_SETUP_FILES:
# unbound variable" in a shell where ROS hasn't already been sourced once).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

MAP_YAML=""
USE_RVIZ="true"
PROFILE_ARG=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --map-yaml)  MAP_YAML="$2"; shift 2 ;;
        --no-rviz)   USE_RVIZ="false"; shift ;;
        *)           PROFILE_ARG="$1"; shift ;;
    esac
done

source "${SCRIPT_DIR}/env.sh" "${PROFILE_ARG}"

MAP_YAML="${MAP_YAML:-${DIY_MAP_YAML:-${REPO_ROOT}/maps/course_traced_smooth.yaml}}"

if [[ ! -f "${MAP_YAML}" ]]; then
    echo "[ERROR] Map YAML not found: ${MAP_YAML}"
    exit 1
fi

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║  Global Costmap Viewer                    ║"
echo "╚══════════════════════════════════════════╝"
echo "  map_yaml : ${MAP_YAML}"
echo "  use_rviz : ${USE_RVIZ}"
echo ""
echo "  Press Ctrl-C to stop all nodes."
echo ""

ros2 launch challenge_bringup view_global_costmap.launch.py \
    map_yaml:="${MAP_YAML}" \
    use_rviz:="${USE_RVIZ}"
