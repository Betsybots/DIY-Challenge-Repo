#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/calibrate_imu.sh — IMU Allan Variance calibration
# ══════════════════════════════════════════════════════════════════════════════
# Runs a 2–3 hour static IMU recording session, then launches imu_utils to
# compute Allan Variance noise parameters (gyro/accel noise density + random walk).
#
# Output YAML: calibration/imu_calibration_<timestamp>.yaml
#              Copy values into fast_lio_hesai_qt64.yaml (gyr_cov, acc_cov, etc.)
#
# Prerequisites:
#   • Robot MUST be stationary on a level surface during recording
#   • STM32/IMU publishing on /imu/data at ≥ 200 Hz
#   • third_party_ws/install sourced (imu_utils package)
#
# Usage:
#   ./scripts/calibrate_imu.sh [profile] [--duration <seconds>]
#   Default duration: 7200 (2 hours — minimum for reliable Allan Variance)
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PROFILE=""
DURATION=7200

while [[ $# -gt 0 ]]; do
    case "$1" in
        --duration) DURATION="$2"; shift 2 ;;
        --*)        echo "[calibrate_imu] Unknown flag: $1" >&2; exit 1 ;;
        *)          PROFILE="$1"; shift ;;
    esac
done

# shellcheck source=scripts/env.sh
source "${SCRIPT_DIR}/env.sh" "${PROFILE}"

CALIB_DIR="${REPO_ROOT}/calibration"
mkdir -p "${CALIB_DIR}"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${CALIB_DIR}/imu_calib_${TIMESTAMP}"
RESULT_YAML="${CALIB_DIR}/imu_calibration_${TIMESTAMP}.yaml"

echo ""
echo "╔══════════════════════════════════╗"
echo "║  IMU Allan Variance Calibration  ║"
echo "╚══════════════════════════════════╝"
echo "  Duration : ${DURATION} s ($(( DURATION / 60 )) min)"
echo "  Bag output: ${BAG_PATH}"
echo ""
echo "  ⚠  IMPORTANT: Keep the robot COMPLETELY STATIONARY for the entire duration."
echo "  Press Enter to start, Ctrl+C to abort."
read -r

# ── Step 1: Record IMU data ────────────────────────────────────────────────────
echo "[calibrate_imu] Recording /imu/data for ${DURATION} seconds..."
timeout "${DURATION}" ros2 bag record \
    --output "${BAG_PATH}" \
    /imu/data \
    /imu/data_raw || true   # timeout exits non-zero; that is expected

echo "[calibrate_imu] Recording complete: ${BAG_PATH}"

# ── Step 2: Run imu_utils analysis ───────────────────────────────────────────
echo "[calibrate_imu] Launching imu_utils analysis..."

# imu_utils expects the bag to be played back while it subscribes to the IMU topic
ros2 bag play --clock "${BAG_PATH}" &
BAG_PID=$!

ros2 launch imu_utils imu_utils.launch.py \
    imu_topic:=/imu/data \
    imu_name:=diy_imu \
    data_save_path:="${CALIB_DIR}/" \
    max_time_min:=$(( DURATION / 60 ))

kill "${BAG_PID}" 2>/dev/null || true

# ── Step 3: Summary ───────────────────────────────────────────────────────────
echo ""
echo "[calibrate_imu] ✓ Analysis complete."
echo "[calibrate_imu] Results written to: ${CALIB_DIR}/"
echo ""
echo "Next steps:"
echo "  1. Open the generated YAML in ${CALIB_DIR}/"
echo "  2. Copy gyr_n, gyr_w, acc_n, acc_w into:"
echo "     src/localization/config/fast_lio_hesai_qt64.yaml"
echo "     (search for CALIB: gyr_cov / acc_cov / b_gyr_cov / b_acc_cov)"
