#!/usr/bin/env bash
# record_zone_nav.sh — Lightweight zone nav "flight data recorder"
# Records all zone-nav state topics at low bandwidth (no lidar, no camera).
# Use this alongside or instead of record_bag.sh for zone nav debugging.
#
# Usage:
#   ./scripts/record_zone_nav.sh [--label <tag>]
#   ./scripts/record_zone_nav.sh --label test_run1
#
# Output: ~/bags/zone_nav_<LABEL>_<TIMESTAMP>/
#
# After recording, analyse with:
#   ros2 bag play ~/bags/zone_nav_*/  --clock
#   plotjuggler --bag ~/bags/zone_nav_*/   (if installed)

set -euo pipefail

LABEL="run"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --label) LABEL="$2"; shift 2 ;;
        *) echo "Usage: $0 [--label <name>]" >&2; exit 1 ;;
    esac
done

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${HOME}/bags/zone_nav_${LABEL}_${TIMESTAMP}"
mkdir -p "${OUTPUT_DIR}"

echo "════════════════════════════════════════════════"
echo "  Zone Nav Flight Recorder — Team Juggernauts"
echo "  Output: ${OUTPUT_DIR}"
echo "  Press Ctrl+C to stop."
echo "════════════════════════════════════════════════"
echo ""

TOPICS=(
    # ── Zone nav state machine ────────────────────────────────
    /nav_mode                   # INIT/NORMAL_NAV/SLOW_NAV/BLIND_DRIVE/...
    /green_light                # Race start trigger

    # ── Velocity arbitration ──────────────────────────────────
    /mux_mode                   # JOYSTICK/AUTONOMOUS/BLIND_DRIVE/ESTOP_LOCK
    /cmd_vel_zone_nav           # Zone nav's commanded velocity
    /cmd_vel_nav                # Nav2's commanded velocity
    /cmd_vel_joy                # Joystick commanded velocity
    /cmd_vel_safe               # Final output to motors
    /speed_limit                # Active speed cap

    # ── Safety / estop ────────────────────────────────────────
    /estop_active               # Hardware estop state

    # ── Localization state ────────────────────────────────────
    /ndt_fitness_score          # NDT match quality (goes >0.8 in tunnel)
    /odometry/filtered          # Robot position (EKF2 output)
    /lidar_odometry             # FAST-LIO2 raw odometry
    /lidar_odometry_gated       # Gated lidar odom (off during BLIND_DRIVE)

    # ── TF (lightweight, useful for replaying in rviz2) ───────
    /tf
    /tf_static

    # ── Diagnostics ───────────────────────────────────────────
    /diagnostics
    /rosout
)

echo "[record_zone_nav] Recording ${#TOPICS[@]} topics..."
ros2 bag record \
    --output "${OUTPUT_DIR}" \
    --compression-mode file \
    --compression-format zstd \
    "${TOPICS[@]}"
