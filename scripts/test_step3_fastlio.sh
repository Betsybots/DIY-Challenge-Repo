#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/test_step3_fastlio.sh — Step 3: FAST-LIO2 odometry + joystick
# ══════════════════════════════════════════════════════════════════════════════
# Brings up the Hesai lidar driver + FAST-LIO2 + joystick so you can drive
# and verify lidar-inertial odometry is publishing correctly.
#
# Usage:
#   ./scripts/test_step3_fastlio.sh [profile]
#
# Verify:
#   ros2 topic echo /lidar_odometry       — FAST-LIO2 pose output
#   ros2 topic hz /lidar_odometry         — should be ~10 Hz (lidar scan rate)
#   ros2 topic echo /cloud_registered     — registered point cloud (optional)
#   rviz2                                 — add Odometry display on /lidar_odometry
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${SCRIPT_DIR}/_viz_helper.sh"
viz_parse_args "$@"

source "${SCRIPT_DIR}/env.sh" "${REMAINING_ARGS[0]:-}"

RVIZ_CONFIG="${REPO_ROOT}/src/challenge_bringup/rviz/step3_fastlio.rviz"

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║  Step 3 — FAST-LIO2 + Joystick           ║"
echo "╚══════════════════════════════════════════╝"
echo "  Lidar     : /hesai/points → FAST-LIO2"
echo "  Output    : /lidar_odometry  (nav_msgs/Odometry)"
echo "  TF        : odom → base_link"
echo "  Visualization : ${VIZ_MODE}"
echo ""
echo "  Mux starts in JOYSTICK mode — use gamepad to drive."
echo "  Press Ctrl-C to stop all nodes."
echo ""

cleanup() {
    echo ""
    echo "[test_step3] Stopping all nodes..."
    kill 0
}
trap cleanup INT TERM

# Node 1: FAST-LIO2 (localization launch in runtime mode, no EKF, no GPS)
ros2 launch diy_localization localization.launch.py \
    mode:=runtime \
    use_gps:=false \
    use_rviz:=false &
PID_FASTLIO=$!

# Node 2: cmd_vel mux
ros2 run diy_cmd_vel_mux cmd_vel_mux_node &
PID_MUX=$!

# Node 3: Motor controller
ros2 run diy_motor_control_legacy diff-drive-main &
PID_MOTOR=$!

# Node 4: Joystick
ros2 launch challenge_bringup joystick_drive.launch.py &
PID_JOY=$!

echo "[test_step3] All nodes started."

viz_start "${RVIZ_CONFIG}"

echo "[test_step3] Verify in another terminal:"
echo "  ros2 topic echo /lidar_odometry"
echo "  ros2 topic hz /lidar_odometry"
echo ""

wait
