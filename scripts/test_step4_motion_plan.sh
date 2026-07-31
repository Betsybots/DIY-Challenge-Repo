#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/test_step4_motion_plan.sh — Step 4: Autonomous motion plan demo
# ══════════════════════════════════════════════════════════════════════════════
# Runs the motion plan executor against either FAST-LIO2 or fused odometry.
# Demonstrates the Aug 14 requirement: forward → 90° turn → forward.
#
# Usage:
#   ./scripts/test_step4_motion_plan.sh [odom_source] [profile]
#
#   odom_source:
#     fastlio  — use /lidar_odometry  (FAST-LIO2 only, no EKF)  [default]
#     fused    — use /odometry/filtered (FAST-LIO2 + IMU EKF)
#
# Examples:
#   ./scripts/test_step4_motion_plan.sh fastlio jetson
#   ./scripts/test_step4_motion_plan.sh fused   jetson
#
# To trigger autonomous run:
#   ros2 param set /cmd_vel_mux_node mode AUTONOMOUS
#
# Verify:
#   ros2 topic echo /mux_mode           — should show AUTONOMOUS
#   ros2 topic echo /cmd_vel_safe       — motion commands flowing
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${SCRIPT_DIR}/_viz_helper.sh"
viz_parse_args "$@"

ODOM_SOURCE="${REMAINING_ARGS[0]:-fastlio}"
PROFILE="${REMAINING_ARGS[1]:-}"

source "${SCRIPT_DIR}/env.sh" "${PROFILE}"

# Resolve pose topic and plan file based on odom source
DEMO_PLAN="${REPO_ROOT}/src/challenge_bringup/config/motion_plan_demo.yaml"
RVIZ_CONFIG="${REPO_ROOT}/src/challenge_bringup/rviz/step4_motion_plan.rviz"

case "${ODOM_SOURCE}" in
    fastlio)
        POSE_TOPIC="/lidar_odometry"
        ;;
    fused)
        POSE_TOPIC="/odometry/filtered"
        ;;
    *)
        echo "[test_step4] ERROR: Unknown odom_source '${ODOM_SOURCE}'. Use: fastlio | fused" >&2
        exit 1
        ;;
esac

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║  Step 4 — Autonomous Motion Plan         ║"
echo "╚══════════════════════════════════════════╝"
echo "  Odom source : ${ODOM_SOURCE}  (${POSE_TOPIC})"
echo "  Plan file   : ${DEMO_PLAN}"
echo "  Sequence    : STRAIGHT 2m → TURN 90° → STRAIGHT 2m"
echo "  Visualization : ${VIZ_MODE}"
echo ""
echo "  ⚡ Mux starts in JOYSTICK mode."
echo "  ⚡ Switch to autonomous when ready:"
echo "     ros2 param set /cmd_vel_mux_node mode AUTONOMOUS"
echo ""
echo "  Press Ctrl-C to stop all nodes."
echo ""

cleanup() {
    echo ""
    echo "[test_step4] Stopping — publishing zero velocity..."
    ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist '{}' 2>/dev/null || true
    kill 0
}
trap cleanup INT TERM

# Node 1: FAST-LIO2 (always needed for lidar odometry)
ros2 launch diy_localization localization.launch.py \
    mode:=runtime \
    use_gps:=false \
    use_rviz:=false &
PID_FASTLIO=$!

# Node 2: EKF (only needed for fused mode)
if [[ "${ODOM_SOURCE}" == "fused" ]]; then
    EKF_CONFIG="${REPO_ROOT}/src/diy_localization/config/ekf_local.yaml"
    ros2 run robot_localization ekf_node \
        --ros-args \
        --params-file "${EKF_CONFIG}" &
    PID_EKF=$!
    echo "[test_step4] EKF started (PID=${PID_EKF})"
fi

# Node 3: Motor controller
ros2 run diy_motor_control_legacy diff-drive-main &
PID_MOTOR=$!

# Node 4: cmd_vel mux (starts in JOYSTICK — operator must switch to AUTONOMOUS)
ros2 run diy_cmd_vel_mux cmd_vel_mux_node &
PID_MUX=$!

# Node 5: Motion plan executor
ros2 run plan_b motion_plan_executor_node \
    --ros-args \
    -p plan_file:="${DEMO_PLAN}" \
    -p pose_topic:="${POSE_TOPIC}" \
    -p cmd_topic:=/cmd_vel_nav &
PID_PLAN=$!

echo "[test_step4] All nodes started."

viz_start "${RVIZ_CONFIG}"

echo ""
echo "  When ready to run autonomously:"
echo "    ros2 param set /cmd_vel_mux_node mode AUTONOMOUS"
echo ""
echo "  To abort and return to joystick:"
echo "    ros2 param set /cmd_vel_mux_node mode JOYSTICK"
echo ""

wait
