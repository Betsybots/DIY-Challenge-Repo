#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/calibrate_extrinsics.sh — Lidar↔IMU extrinsic calibration
# ══════════════════════════════════════════════════════════════════════════════
# Records a lidar_imu_calib motion session (figure-8 excitation), then runs
# the calibration solver to produce the lidar→IMU transform.
#
# Result: extrinsic_T (translation) + extrinsic_R (rotation matrix)
#         Copy into fast_lio_hesai_qt64.yaml (marked CALIB: extrinsic_T / R)
#         and into robot.urdf.xacro (lidar_joint xyz / rpy).
#
# Prerequisites:
#   • Robot must be able to move freely (figure-8 manoeuvre ~2 m diameter)
#   • Hesai QT64 publishing on /lidar_points
#   • IMU publishing on /imu/data
#   • third_party_ws/install sourced (lidar_imu_calib package)
#
# Usage:
#   ./scripts/calibrate_extrinsics.sh [profile] [--duration <seconds>]
#   Default duration: 120 (2 min of excitation motion)
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PROFILE=""
DURATION=120

while [[ $# -gt 0 ]]; do
    case "$1" in
        --duration) DURATION="$2"; shift 2 ;;
        --*)        echo "[calibrate_extrinsics] Unknown flag: $1" >&2; exit 1 ;;
        *)          PROFILE="$1"; shift ;;
    esac
done

# shellcheck source=scripts/env.sh
source "${SCRIPT_DIR}/env.sh" "${PROFILE}"

CALIB_DIR="${REPO_ROOT}/calibration"
mkdir -p "${CALIB_DIR}"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${CALIB_DIR}/extrinsics_${TIMESTAMP}"
RESULT_FILE="${CALIB_DIR}/lidar_imu_extrinsics_${TIMESTAMP}.txt"

echo ""
echo "╔══════════════════════════════════╗"
echo "║  Lidar↔IMU Extrinsic Calibration ║"
echo "╚══════════════════════════════════╝"
echo "  Duration : ${DURATION} s"
echo "  Bag output: ${BAG_PATH}"
echo ""
echo "  Motion protocol:"
echo "    • Drive the robot in a figure-8 pattern (~2 m diameter circles)"
echo "    • Include turns in BOTH directions"
echo "    • Vary speed: slow → fast → slow"
echo "    • Avoid abrupt stops that could tip the robot"
echo ""
echo "  Press Enter when robot is in position and you are ready to record."
read -r

# ── Step 1: Record excitation bag ─────────────────────────────────────────────
echo "[calibrate_extrinsics] Recording for ${DURATION} s — DRIVE A FIGURE-8 NOW..."
timeout "${DURATION}" ros2 bag record \
    --output "${BAG_PATH}" \
    /lidar_points \
    /imu/data \
    /imu/data_raw || true

echo "[calibrate_extrinsics] Recording complete: ${BAG_PATH}"

# ── Step 2: Run lidar_imu_calib solver ────────────────────────────────────────
echo "[calibrate_extrinsics] Running lidar_imu_calib solver..."

ros2 bag play --clock "${BAG_PATH}" &
BAG_PID=$!

ros2 launch lidar_imu_calib calib_lidar_imu.launch.py \
    lidar_topic:=/lidar_points \
    imu_topic:=/imu/data \
    result_path:="${CALIB_DIR}/" || true

kill "${BAG_PID}" 2>/dev/null || true

# ── Step 3: Guidance ──────────────────────────────────────────────────────────
echo ""
echo "[calibrate_extrinsics] ✓ Calibration complete."
echo "[calibrate_extrinsics] Results directory: ${CALIB_DIR}/"
echo ""
echo "Next steps:"
echo "  1. Find the calibration result YAML in ${CALIB_DIR}/"
echo "     Look for extrinsic_T: [tx, ty, tz] and extrinsic_R: [r00, r01, ...]"
echo ""
echo "  2. Update src/diy_localization/config/fast_lio_hesai_qt64.yaml:"
echo "       extrinsic_T: [tx, ty, tz]"
echo "       extrinsic_R: [r00, r01, r02, r10, r11, r12, r20, r21, r22]"
echo "       extrinsic_est_en: false   # disable online estimation after calibration"
echo ""
echo "  3. Update src/diy_robot_description/urdf/robot.urdf.xacro"
echo "     lidar_joint xyz / rpy with the measured transform."
echo "     (Convert rotation matrix → rpy with scipy or tf_transformations)"
