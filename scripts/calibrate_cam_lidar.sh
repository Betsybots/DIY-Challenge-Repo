#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/calibrate_cam_lidar.sh — Camera ↔ LiDAR extrinsic calibration
# ══════════════════════════════════════════════════════════════════════════════
# NEEDS UPDATE FOR ZED2i: this script still hardcodes RealSense-specific
# topics/launch calls below (/camera/color/image_raw, `ros2 launch
# realsense2_camera rs_launch.py`) — not yet updated to this robot's actual
# camera (ZED2i, package zed_wrapper). Not fixed in this pass.
#
# Records a ROS 2 bag containing both camera image frames and lidar point clouds
# for offline camera-to-lidar extrinsic calibration with Kalibr or similar tools.
#
# This script handles the DATA COLLECTION step only.
# Processing the bag to extract the extrinsic transform is done offline —
# see docs/Camera_Calibration_Guide.pdf Section 3.3 for Kalibr instructions.
#
# After calibration:
#   1. Update base_to_camera joint in src/diy_robot_description/urdf/robot.urdf.xacro
#   2. Rebuild: colcon build --packages-select diy_robot_description
#   3. Verify with RViz2 (see guide Section 4.3)
#
# Prerequisites:
#   • ZED2i publishing /zed/zed_node/rgb/image_rect_color
#   • Hesai QT64 publishing /lidar_points
#   • A calibration target visible to both sensors simultaneously:
#       - Recommended: flat board with AprilTags + reflective/matte checkerboard
#       - Minimum: matte checkerboard on flat rigid board (no shiny surfaces)
#
# Usage:
#   ./scripts/calibrate_cam_lidar.sh [--duration <seconds>] [--start-sensors]
#   Default duration: 180 (3 minutes — collect 15-20 target poses)
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DURATION=180
START_SENSORS=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --duration) DURATION="$2"; shift 2 ;;
        --start-sensors) START_SENSORS=true; shift ;;
        --help|-h)
            echo "Usage: $0 [--duration <seconds>] [--start-sensors]"
            exit 0 ;;
        *) echo "[calibrate_cam_lidar] Unknown arg: $1" >&2; exit 1 ;;
    esac
done

CALIB_DIR="${REPO_ROOT}/calibration"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${CALIB_DIR}/cam_lidar_${TIMESTAMP}"
mkdir -p "${CALIB_DIR}"

SENSOR_PIDS=()

# ── Banner ───────────────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║  Camera ↔ LiDAR Extrinsic Calibration        ║"
echo "║  Data Collection Step                         ║"
echo "╚══════════════════════════════════════════════╝"
echo "  Duration  : ${DURATION} s"
echo "  Bag output: ${BAG_PATH}"
echo ""
echo "  ┌────────────────────────────────────────────────────────────────────┐"
echo "  │  CALIBRATION TARGET REQUIREMENTS                                   │"
echo "  │                                                                     │"
echo "  │  The target must be visible to BOTH the camera and the lidar.      │"
echo "  │  • Use a flat rigid board (~50×40 cm minimum)                      │"
echo "  │  • Attach a MATTE (non-reflective) checkerboard pattern on top     │"
echo "  │  • The lidar detects the board EDGE — keep edges clean and flat    │"
echo "  │  • For best results: tape retro-reflective strips around the edge  │"
echo "  │    of the board to make it strongly visible in the lidar scan      │"
echo "  │                                                                     │"
echo "  │  SETUP:                                                             │"
echo "  │  • Keep the ROBOT STATIONARY throughout the session                │"
echo "  │  • Move the target board to 15–20 different positions/angles:      │"
echo "  │      - Different distances: 0.5 m, 1.0 m, 1.5 m, 2.0 m           │"
echo "  │      - Different lateral offsets: left, centre, right              │"
echo "  │      - Different tilts: face-on, tilted ±30° left/right/up/down   │"
echo "  │  • Hold the board STILL for 3–5 seconds at each pose               │"
echo "  │                                                                     │"
echo "  │  WHY: The solver needs many non-collinear board poses to           │"
echo "  │       constrain the 6-DOF extrinsic transform uniquely.            │"
echo "  └────────────────────────────────────────────────────────────────────┘"
echo ""

# ── Optionally start sensors ──────────────────────────────────────────────────
if [[ "$START_SENSORS" == "true" ]]; then
    echo "[INFO] Starting RealSense driver..."
    ros2 launch realsense2_camera rs_launch.py enable_color:=true &
    SENSOR_PIDS+=($!)

    echo "[INFO] Starting Hesai driver (assumed from challenge_master)..."
    echo "[WARN] If the Hesai driver is already running, ignore this step."
    sleep 5
fi

# ── Check topics are publishing ───────────────────────────────────────────────
echo "[INFO] Verifying sensor topics..."
MISSING=0

if ! timeout 5 ros2 topic hz /camera/color/image_raw 2>/dev/null | grep -q "average rate"; then
    echo "[ERROR] /camera/color/image_raw is NOT publishing."
    echo "  Start the RealSense driver:  ros2 launch realsense2_camera rs_launch.py"
    MISSING=1
fi

if ! timeout 5 ros2 topic hz /lidar_points 2>/dev/null | grep -q "average rate"; then
    echo "[ERROR] /lidar_points is NOT publishing."
    echo "  Start the Hesai driver or the full bringup."
    MISSING=1
fi

if [[ "$MISSING" -eq 1 ]]; then
    echo ""
    echo "[ERROR] Required topics not available. Exiting."
    for pid in "${SENSOR_PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
    exit 1
fi

echo "[INFO] Both sensors confirmed publishing."
echo ""
echo "[INFO] Current URDF placeholder (will be replaced after calibration):"
grep -A1 "base_to_camera" "${REPO_ROOT}/src/diy_robot_description/urdf/robot.urdf.xacro" \
    | grep origin || echo "  (could not parse URDF automatically)"
echo ""

# ── Quick physical measurement prompt ────────────────────────────────────────
echo "  ─────────────────────────────────────────────────────────────────────"
echo "  BEFORE recording: take a physical measurement with a ruler."
echo "  This gives you a starting estimate even if Kalibr is not processed."
echo ""
echo "  Measure the 3D offset from Hesai QT64 rotation centre to D435i lens:"
echo "    x = forward distance   (camera in front of lidar: positive)"
echo "    y = lateral distance   (camera to left of lidar: positive)"
echo "    z = vertical distance  (camera above lidar: positive)"
echo ""
read -rp "  Enter measured x offset (m, e.g. 0.12):  " MEAS_X
read -rp "  Enter measured y offset (m, e.g. 0.00):  " MEAS_Y
read -rp "  Enter measured z offset (m, e.g. -0.05): " MEAS_Z
read -rp "  Camera tilted? Enter rpy in radians (e.g. 0 0 0): " MEAS_RPY

echo ""
echo "  ── Physical measurement recorded ────────────────────────────────────"
echo "    xyz=${MEAS_X} ${MEAS_Y} ${MEAS_Z}   rpy=${MEAS_RPY}"
echo ""
echo "  This will be saved to: ${BAG_PATH}/physical_measurement.txt"
mkdir -p "${BAG_PATH}"
cat > "${BAG_PATH}/physical_measurement.txt" << MEAS_EOF
# Camera-to-lidar physical measurement
# Date: ${TIMESTAMP}
# Tool: calibrate_cam_lidar.sh
#
# base_to_camera joint values for robot.urdf.xacro:
#   <origin xyz="${MEAS_X} ${MEAS_Y} ${MEAS_Z}" rpy="${MEAS_RPY}"/>
#
x=${MEAS_X}
y=${MEAS_Y}
z=${MEAS_Z}
rpy=${MEAS_RPY}
#
# To apply immediately (before Kalibr processing):
#   Edit src/diy_robot_description/urdf/robot.urdf.xacro
#   Replace the base_to_camera <origin> line with the values above.
#   Then: colcon build --packages-select diy_robot_description
MEAS_EOF
echo "  Saved."
echo "  ─────────────────────────────────────────────────────────────────────"
echo ""

# ── Start recording ───────────────────────────────────────────────────────────
echo "[INFO] Starting bag recording for ${DURATION} seconds..."
echo "       >>> MOVE THE TARGET BOARD to 15–20 different positions NOW <<<"
echo ""

ros2 bag record \
    /camera/color/image_raw \
    /camera/color/camera_info \
    /lidar_points \
    --output "${BAG_PATH}/bag" \
    --max-bag-duration "${DURATION}" &
BAG_PID=$!

# Progress countdown
for ((i=DURATION; i>0; i--)); do
    printf "\r  Recording: %3d s remaining  " "$i"
    sleep 1
done
printf "\r  Recording: done                    \n"

wait "$BAG_PID" 2>/dev/null || true

echo ""
echo "  ┌────────────────────────────────────────────────────────────────────┐"
echo "  │  RECORDING COMPLETE                                                │"
echo "  │  Bag saved: ${BAG_PATH}/bag"
printf  "  │  %-66s │\n" ""
echo "  │  NEXT STEPS:                                                       │"
echo "  │                                                                     │"
echo "  │  Option A — Apply physical measurement immediately (fast):         │"
echo "  │    Edit src/diy_robot_description/urdf/robot.urdf.xacro           │"
echo "  │    Set base_to_camera origin to values in physical_measurement.txt │"
echo "  │    colcon build --packages-select diy_robot_description            │"
echo "  │    Verify with RViz2 (guide Section 4.3)                          │"
echo "  │                                                                     │"
echo "  │  Option B — Precision Kalibr calibration (offline, ~2 hours):    │"
echo "  │    See docs/Camera_Calibration_Guide.pdf Section 3.3 for steps.   │"
echo "  │    Input bag: ${BAG_PATH}/bag"
printf  "  │    %-66s │\n" ""
echo "  │  Result of either option: update URDF, rebuild, verify.            │"
echo "  └────────────────────────────────────────────────────────────────────┘"
echo ""

for pid in "${SENSOR_PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
