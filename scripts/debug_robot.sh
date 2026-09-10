#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/debug_robot.sh — Debug / developer launch
# ══════════════════════════════════════════════════════════════════════════════
# Same as run_robot.sh but additionally:
#   • Launches RViz2 with the challenge visualisation config
#   • Sets log level to DEBUG
#   • Optionally isolates a single node for targeted debugging
#
# Usage:
#   ./scripts/debug_robot.sh [profile] [--node <package/executable>]
#
# Examples:
#   ./scripts/debug_robot.sh
#   ./scripts/debug_robot.sh laptop
#   ./scripts/debug_robot.sh jetson --node diy_cmd_vel_mux/cmd_vel_mux_node
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Parse arguments: first non-flag arg is profile, --node takes next arg
PROFILE=""
ISOLATE_NODE=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --node) ISOLATE_NODE="$2"; shift 2 ;;
        --*)    echo "[debug_robot] Unknown flag: $1" >&2; exit 1 ;;
        *)      PROFILE="$1"; shift ;;
    esac
done

# Source environment
# shellcheck source=scripts/env.sh
source "${SCRIPT_DIR}/env.sh" "${PROFILE}"

# Force debug log level
export DIY_LOG_LEVEL=DEBUG
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

echo ""
echo "╔══════════════════════════════════╗"
echo "║  DIY Challenge Robot — DEBUG     ║"
echo "╚══════════════════════════════════╝"
echo "  Profile      : ${DIY_ROBOT_PROFILE}"
echo "  Log level     : DEBUG"
[[ -n "${ISOLATE_NODE}" ]] && echo "  Isolated node : ${ISOLATE_NODE}"
echo ""

if [[ -n "${ISOLATE_NODE}" ]]; then
    # Run a single node directly for breakpoint debugging
    IFS='/' read -r _pkg _exe <<< "${ISOLATE_NODE}"
    ros2 run "${_pkg}" "${_exe}" --ros-args \
        --log-level DEBUG \
        -p use_sim_time:=false
else
    ros2 launch challenge_bringup challenge_master.launch.py \
        use_joystick:="${DIY_USE_JOYSTICK:-false}" \
        use_nav2:="${DIY_USE_NAV2}" \
        use_zed:="${DIY_USE_ZED}" \
        use_motor_driver:="${DIY_USE_MOTOR_DRIVER}" \
        use_hesai:="${DIY_USE_HESAI}" \
        use_micro_ros:="${DIY_USE_MICRO_ROS}" \
        use_localization:="${DIY_USE_LOCALIZATION}" \
        use_cmd_vel_mux:="${DIY_USE_CMD_VEL_MUX:-true}" \
        mux_mode:="${DIY_MUX_MODE}" \
        fastlio_config:="${DIY_FASTLIO_CONFIG:-fast_lio_hesai_qt64.yaml}" \
        use_rviz:=true
fi
