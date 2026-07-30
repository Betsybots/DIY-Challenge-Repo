#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/test_step2_fused_odom.sh — Step 2: Wheel odom + IMU fused via EKF
# ══════════════════════════════════════════════════════════════════════════════
# Brings up motor controller + EKF (fusing /wheel_cmd_vel + /imu/data) +
# joystick so you can drive and verify the fused odometry output.
#
# Uses the main ekf_local.yaml but remaps odom0 input from /lidar_odometry
# to /wheel_cmd_vel so FAST-LIO2 is NOT required for this step.
#
# Usage:
#   ./scripts/test_step2_fused_odom.sh [profile]
#
# Verify:
#   ros2 topic echo /odometry/filtered    — fused EKF output
#   ros2 run tf2_tools view_frames        — odom→base_link TF must exist
#   ros2 topic hz /odometry/filtered      — should be ~50 Hz
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${SCRIPT_DIR}/_viz_helper.sh"
viz_parse_args "$@"

source "${SCRIPT_DIR}/env.sh" "${REMAINING_ARGS[0]:-}"

EKF_CONFIG="${REPO_ROOT}/src/diy_localization/config/ekf_local.yaml"
RVIZ_CONFIG="${REPO_ROOT}/src/challenge_bringup/rviz/step2_fused_odom.rviz"

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║  Step 2 — Wheel Odom + IMU → EKF Fusion  ║"
echo "╚══════════════════════════════════════════╝"
echo "  EKF config : ${EKF_CONFIG}"
echo "  odom0      : /wheel_cmd_vel  (remapped from /lidar_odometry)"
echo "  imu0       : /imu/data"
echo "  Output     : /odometry/filtered"
echo "  Visualization : ${VIZ_MODE}"
echo ""
echo "  Mux starts in JOYSTICK mode — use gamepad to drive."
echo "  Press Ctrl-C to stop all nodes."
echo ""

cleanup() {
    echo ""
    echo "[test_step2] Stopping all nodes..."
    kill 0
}
trap cleanup INT TERM

# Node 1: Motor controller — publishes /wheel_cmd_vel
ros2 run diy_motor_control_legacy diff-drive-main &
PID_MOTOR=$!

# Node 2: EKF — fuse wheel odom + IMU
# Remap /lidar_odometry → /wheel_cmd_vel so ekf_local.yaml works without FAST-LIO2
ros2 run robot_localization ekf_node \
    --ros-args \
    --params-file "${EKF_CONFIG}" \
    -r /lidar_odometry:=/wheel_cmd_vel &
PID_EKF=$!

# Node 3: cmd_vel mux
ros2 run diy_cmd_vel_mux cmd_vel_mux_node &
PID_MUX=$!

# Node 4: Joystick
ros2 launch challenge_bringup joystick_drive.launch.py &
PID_JOY=$!

echo "[test_step2] All nodes started."

viz_start "${RVIZ_CONFIG}"

echo "[test_step2] Verify in another terminal:"
echo "  ros2 topic echo /odometry/filtered"
echo "  ros2 run tf2_tools view_frames"
echo ""

wait
