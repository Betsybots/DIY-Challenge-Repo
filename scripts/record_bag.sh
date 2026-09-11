#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/record_bag.sh — Record a competition / calibration session
# ══════════════════════════════════════════════════════════════════════════════
# Records all SAD-required topics to a timestamped bag directory.
# Bag is split every DIY_BAG_SPLIT_DURATION seconds (default: 300s = 5 min).
#
# Usage:
#   ./scripts/record_bag.sh [profile] [--label <tag>] [--topics extra topics...]
#
# Output:
#   ${DIY_BAG_OUTPUT_DIR}/<PROFILE>_<LABEL>_<TIMESTAMP>/
#
# Examples:
#   ./scripts/record_bag.sh jetson
#   ./scripts/record_bag.sh jetson --label course_run1
#   ./scripts/record_bag.sh laptop --label calibration --topics /imu/data_raw
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Parse arguments ───────────────────────────────────────────────────────────
PROFILE=""
LABEL="run"
EXTRA_TOPICS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --label)  LABEL="$2"; shift 2 ;;
        --topics) shift; while [[ $# -gt 0 && "$1" != --* ]]; do EXTRA_TOPICS+=("$1"); shift; done ;;
        --*)      echo "[record_bag] Unknown flag: $1" >&2; exit 1 ;;
        *)        PROFILE="$1"; shift ;;
    esac
done

# shellcheck source=scripts/env.sh
source "${SCRIPT_DIR}/env.sh" "${PROFILE}"

# ── Resolve output directory ──────────────────────────────────────────────────
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${DIY_BAG_OUTPUT_DIR:-${HOME}/bags}/${DIY_ROBOT_PROFILE}_${LABEL}_${TIMESTAMP}"

mkdir -p "${OUTPUT_DIR}"
echo "[record_bag] Bag output: ${OUTPUT_DIR}"

# ── SAD-required topic list ───────────────────────────────────────────────────
# Section 5.2 Data Recording Requirements
REQUIRED_TOPICS=(
    /lidar_points                  # 3-D lidar (raw scans for SLAM replay)
    /imu/data                      # IMU after calibration / FAST-LIO2 input
    /gps/fix                       # RTK GPS fix
    /zed/zed_node/rgb/image_rect_color   # ZED2i colour (obstacle classification)
    /zed/zed_node/left/image_rect_color  # Left rectified (VSLAM input)
    /zed/zed_node/right/image_rect_color # Right rectified (VSLAM input)
    /zed/zed_node/imu/data               # ZED2i IMU
    /tf                            # Full TF tree
    /tf_static                     # Static TF tree
    /odometry/filtered             # EKF1 output (replayed via rosbag_to_csv)
    /lidar_odometry                # FAST-LIO2 raw output
    /obstacle_class                # Detection classification (when available)
    /cmd_vel_safe                  # Final commanded velocity (audit)
    /mux_mode                      # Mux arbitration state
    /estop_active                  # E-stop state
    /stm32/heartbeat               # STM32 watchdog heartbeat
    /stm32/estop_active            # STM32 hardware estop
    /diagnostics                   # System health diagnostics
)

ALL_TOPICS=("${REQUIRED_TOPICS[@]}" "${EXTRA_TOPICS[@]}")

echo "[record_bag] Recording ${#ALL_TOPICS[@]} topics..."
echo "[record_bag] Split every ${DIY_BAG_SPLIT_DURATION:-300} seconds"
echo "[record_bag] Press Ctrl+C to stop recording."
echo ""

ros2 bag record \
    --output "${OUTPUT_DIR}" \
    --max-bag-duration "${DIY_BAG_SPLIT_DURATION:-300}" \
    --compression-mode file \
    --compression-format zstd \
    "${ALL_TOPICS[@]}"
