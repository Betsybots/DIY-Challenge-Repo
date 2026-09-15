#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/trigger_goal_reached.sh — Manually fire /pd/goal_reached
# ══════════════════════════════════════════════════════════════════════════════
# Fakes the "goal reached" signal that pure_pursuit_motion_planner_node (or
# pd_motion_planner_node) normally publishes once the robot gets within
# goal_tolerance of the current /a_star/path's final pose.
#
# Use this to test diy_waypoint_sequencer's advance logic IN ISOLATION --
# no localization, no A* planner, no controller, no real robot movement
# required. Run waypoint_sequencer_node + this script only, and confirm:
#   - each call advances to the next waypoint (watch its log output, or
#     `ros2 topic echo /goal_pose`)
#   - the index wraps back to waypoint 1 after the last one, IF the
#     waypoints file has loop: true (or --loop was passed at launch)
#   - it correctly STOPS publishing (no more /goal_pose) after the last
#     waypoint if loop: false
#
# Usage:
#   ./scripts/trigger_goal_reached.sh          # fire once
#   ./scripts/trigger_goal_reached.sh --loop N [--delay SECONDS]
#       fire N times in a row (e.g. to walk through an entire course
#       instantly) with an optional delay between each (default 1.0s)
# ══════════════════════════════════════════════════════════════════════════════
set -eo pipefail

COUNT=1
DELAY="1.0"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --loop)   COUNT="$2"; shift 2 ;;
        --delay)  DELAY="$2"; shift 2 ;;
        *)        echo "Unknown argument: $1" >&2; exit 1 ;;
    esac
done

for ((i = 1; i <= COUNT; i++)); do
    echo "[trigger_goal_reached] Firing /pd/goal_reached (data: true) — ${i}/${COUNT}"
    ros2 topic pub --once /pd/goal_reached std_msgs/msg/Bool "data: true"
    if [[ "${i}" -lt "${COUNT}" ]]; then
        sleep "${DELAY}"
    fi
done

echo "[trigger_goal_reached] Done. Check waypoint_sequencer_node's log output"
echo "  (or 'ros2 topic echo /goal_pose') to confirm it advanced correctly."
