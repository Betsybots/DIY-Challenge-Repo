#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# run_zone_nav.sh
#
# Convenience wrapper for:
#   ros2 launch diy_zone_nav zone_nav.launch.py
#
# USAGE:
#   ./scripts/run_zone_nav.sh [OPTIONS]
#
# OPTIONS:
#   --waypoints <path>    Path to zone_waypoints.yaml
#                         Default: <package_share>/config/zone_waypoints.yaml
#   --quiet               Suppress zone entry/exit log messages
#   -h, --help            Show this help
#
# PREREQUISITES:
#   • ROS 2 Humble sourced (source /opt/ros/humble/setup.bash)
#   • Workspace built and sourced (source install/setup.bash)
#   • /odometry/filtered topic must be publishing (EKF2 running)
#   • /ndt_fitness_score topic must be publishing (NDT-OMP running)
#
# TYPICAL WORKFLOW:
#   Terminal 1: ./scripts/run_localization.sh          # NDT-OMP + dual EKF
#   Terminal 2: ros2 launch nav2_bringup bringup_launch.py ...
#   Terminal 3: ./scripts/run_zone_nav.sh              # this script
#   Terminal 4: ros2 topic pub /green_light std_msgs/Bool "data: true" --once
#
# TOPICS PUBLISHED:
#   /nav_mode   std_msgs/String — current state (INIT / NORMAL_NAV / SLOW_NAV /
#                                                BLIND_DRIVE / NAVIGATE_AROUND /
#                                                PUSH_THROUGH / DONE)
#
# TOPICS SUBSCRIBED:
#   /odometry/filtered     — robot position (EKF2)
#   /ndt_fitness_score     — NDT match quality (for tunnel exit detection)
#   /green_light           — std_msgs/Bool, True = green light, starts race
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

# ── Defaults ─────────────────────────────────────────────────────────────────
WAYPOINTS_FILE=""
VERBOSE="true"

# ── Parse arguments ───────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --waypoints)
      WAYPOINTS_FILE="$2"
      shift 2
      ;;
    --quiet)
      VERBOSE="false"
      shift
      ;;
    -h|--help)
      sed -n '/^# USAGE/,/^# ──.*─$/p' "$0" | head -n -1 | sed 's/^# //'
      exit 0
      ;;
    *)
      echo "[run_zone_nav] Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

# ── ROS 2 environment check ────────────────────────────────────────────────
if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck source=/dev/null
    source /opt/ros/humble/setup.bash
  else
    echo "[run_zone_nav] ERROR: ROS 2 Humble not found. Source it first." >&2
    exit 1
  fi
fi

# Source workspace install if available
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_INSTALL="$(realpath "${SCRIPT_DIR}/../../../../install/setup.bash" 2>/dev/null || true)"
if [[ -f "${WS_INSTALL}" ]]; then
  # shellcheck source=/dev/null
  source "${WS_INSTALL}"
fi

# ── Build launch arguments ─────────────────────────────────────────────────
LAUNCH_ARGS=()

if [[ -n "${WAYPOINTS_FILE}" ]]; then
  if [[ ! -f "${WAYPOINTS_FILE}" ]]; then
    echo "[run_zone_nav] ERROR: waypoints file not found: ${WAYPOINTS_FILE}" >&2
    exit 1
  fi
  LAUNCH_ARGS+=("waypoints_file:=${WAYPOINTS_FILE}")
fi

LAUNCH_ARGS+=("verbose:=${VERBOSE}")

# ── Print effective config ─────────────────────────────────────────────────
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Zone-Aware Navigation State Machine"
echo "  Team Juggernauts — DIY Robot Challenge 2026"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Waypoints: ${WAYPOINTS_FILE:-<package default>}"
echo "  Verbose:   ${VERBOSE}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "  Start race: ros2 topic pub /green_light std_msgs/Bool 'data: true' --once"
echo "  Monitor:    ros2 topic echo /nav_mode"
echo ""

# ── Run ───────────────────────────────────────────────────────────────────
exec ros2 launch diy_zone_nav zone_nav.launch.py "${LAUNCH_ARGS[@]}"
