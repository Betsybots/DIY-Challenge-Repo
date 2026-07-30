#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/test_step1_wheel_odom.sh — Step 1: Wheel odometry + joystick
# ══════════════════════════════════════════════════════════════════════════════
# Brings up the motor controller and joystick so you can drive the robot
# manually and verify wheel odometry is publishing correctly.
#
# Usage:
#   ./scripts/test_step1_wheel_odom.sh [profile]
#
# Verify:
#   ros2 topic echo /wheel_cmd_vel     — twist values change as you drive
#   ros2 topic echo /joint_states      — wheel encoder positions
#   ros2 topic hz /wheel_cmd_vel       — should be ~50 Hz
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ── Parse --viz flag before sourcing env ──────────────────────────────────────
source "${SCRIPT_DIR}/_viz_helper.sh"
viz_parse_args "$@"

source "${SCRIPT_DIR}/env.sh" "${REMAINING_ARGS[0]:-}"

RVIZ_CONFIG="${REPO_ROOT}/src/challenge_bringup/rviz/step1_wheel_odom.rviz"

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║  Step 1 — Wheel Odometry + Joystick      ║"
echo "╚══════════════════════════════════════════╝"
echo "  Topics to verify:"
echo "    /wheel_cmd_vel   (nav_msgs/Odometry)"
echo "    /joint_states    (sensor_msgs/JointState)"
echo "    /cmd_vel_safe    (geometry_msgs/Twist)"
echo "  Visualization : ${VIZ_MODE}"
echo ""
echo "  Mux starts in JOYSTICK mode — use gamepad to drive."
echo "  Press Ctrl-C to stop all nodes."
echo ""

# Launch nodes in parallel using ros2 run; trap Ctrl-C to kill all
cleanup() {
    echo ""
    echo "[test_step1] Stopping all nodes..."
    kill 0
}
trap cleanup INT TERM

# Node 1: Motor controller — publishes /wheel_cmd_vel and /joint_states
ros2 run diy_motor_control_legacy diff-drive-main &
PID_MOTOR=$!

# Node 2: cmd_vel mux — starts in JOYSTICK mode (default)
ros2 run diy_cmd_vel_mux cmd_vel_mux_node &
PID_MUX=$!

# Node 3: Joystick driver + teleop
ros2 launch challenge_bringup joystick_drive.launch.py &
PID_JOY=$!

echo "[test_step1] All nodes started. PIDs: motor=${PID_MOTOR} mux=${PID_MUX} joy=${PID_JOY}"

viz_start "${RVIZ_CONFIG}"

echo "[test_step1] Run in another terminal to verify:"
echo "  ros2 topic echo /wheel_cmd_vel"
echo ""

wait
