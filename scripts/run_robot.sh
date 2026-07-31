#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/run_robot.sh — Competition launch
# ══════════════════════════════════════════════════════════════════════════════
# Launches the full competition stack for the selected profile.
# Wires: Hesai lidar → FAST-LIO2 → EKF → Nav2 → cmd_vel_mux → motor driver
#
# Usage:
#   ./scripts/run_robot.sh [profile]   (profile defaults to DIY_ROBOT_PROFILE)
#
# Examples:
#   ./scripts/run_robot.sh jetson      # full hardware stack on Jetson
#   ./scripts/run_robot.sh laptop      # replayed bag on laptop (set DIY_BAG_FILE)
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Source environment (sets DIY_* vars and all ROS overlays)
# shellcheck source=scripts/env.sh
source "${SCRIPT_DIR}/env.sh" "${1:-}"

echo ""
echo "╔══════════════════════════════════╗"
echo "║  DIY Challenge Robot — LAUNCH    ║"
echo "╚══════════════════════════════════╝"
echo "  Profile  : ${DIY_ROBOT_PROFILE}"
echo "  Nav2     : ${DIY_USE_NAV2}"
echo "  Zone nav : ${DIY_USE_ZONE_NAV:-false}"
echo "  Lidar    : ${DIY_USE_HESAI}"
echo "  SLAM     : ${DIY_USE_LOCALIZATION}"
echo "  Mux mode : ${DIY_MUX_MODE}"
echo ""

# Build the launch command from profile vars
ros2 launch challenge_bringup challenge_master.launch.py \
    use_joystick:="${DIY_USE_JOYSTICK:-false}" \
    use_nav2:="${DIY_USE_NAV2}" \
    use_zone_nav:="${DIY_USE_ZONE_NAV:-false}" \
    use_realsense:="${DIY_USE_REALSENSE}" \
    use_motor_driver:="${DIY_USE_MOTOR_DRIVER}" \
    use_hesai:="${DIY_USE_HESAI}" \
    use_micro_ros:="${DIY_USE_MICRO_ROS}" \
    use_localization:="${DIY_USE_LOCALIZATION}" \
    mux_mode:="${DIY_MUX_MODE}" \
    fastlio_config:="${DIY_FASTLIO_CONFIG:-fast_lio_hesai_qt64.yaml}"
