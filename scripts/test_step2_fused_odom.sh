#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/test_step2_fused_odom.sh — Step 2: Wheel odom + IMU fused via EKF
# ══════════════════════════════════════════════════════════════════════════════
# Brings up motor controller + EKF (fusing /wheel_cmd_vel + /imu/data) +
# joystick so you can drive and verify the fused odometry output.
#
# Uses the single merged ekf_odom.yaml (see its header for why the old
# EKF1(ekf_wimu.yaml)->EKF2(ekf_local.yaml) cascade was collapsed). Its
# odom0 key (/wheel_odom) is remapped to /wheel_cmd_vel because this bench
# test uses the OLD diy_motor_control_legacy node — not driveStack's tested
# differential-drive node, which is what actually publishes /wheel_odom.
# odom1 (/lidar_odometry_gated) simply has no publisher here (no FAST-LIO2
# in this bench test) — the EKF fuses wheel+IMU only, which is this step's
# purpose. Requires the ACEINNA IMU driver (imu_can_interface, launched
# independently on the RPi — not part of this repo) to already be running
# for imu0 (/imu/data) to actually be fused (not started by this script).
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

EKF_CONFIG="${REPO_ROOT}/src/diy_localization/config/ekf_odom.yaml"
RVIZ_CONFIG="${REPO_ROOT}/src/challenge_bringup/rviz/step2_fused_odom.rviz"

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║  Step 2 — Wheel Odom + IMU → EKF Fusion  ║"
echo "╚══════════════════════════════════════════╝"
echo "  EKF config : ${EKF_CONFIG}"
echo "  odom0      : /wheel_cmd_vel  (remapped from /wheel_odom)"
echo "  imu0       : /imu/data  (requires the independent RPi IMU driver already running)"
echo "  odom1      : /lidar_odometry_gated  (no publisher in this bench test — OK)"
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

# Node 2: EKF — fuse wheel odom + IMU + (lidar, if FAST-LIO2 also running)
# ekf_odom.yaml expects odom0=/wheel_odom (driveStack's tested topic), but
# this bench test uses the OLD diy_motor_control_legacy node, which
# publishes /wheel_cmd_vel instead — remap it in. imu0=/imu/data needs a
# live IMU driver (imu_can_interface) running for this test to actually see
# IMU fusion; odom1=/lidar_odometry_gated simply won't publish here (no
# FAST-LIO2 in this bench test) and the EKF runs on wheel+IMU alone — fine
# for this step's purpose.
# Node MUST be named ekf_filter_node_odom (via __node remap) to match the
# top-level key in ekf_odom.yaml — without this remap, ekf_node runs under
# its default name ekf_filter_node, the params file's keys never match, and
# it silently falls back to all-default params (no odom0/imu0 configured at
# all, i.e. no fusion happens even though nothing errors or crashes).
ros2 run robot_localization ekf_node \
    --ros-args \
    -r __node:=ekf_filter_node_odom \
    --params-file "${EKF_CONFIG}" \
    -r /wheel_odom:=/wheel_cmd_vel &
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
