#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/calibrate_camera_intrinsics.sh — RealSense D435i intrinsic calibration
# ══════════════════════════════════════════════════════════════════════════════
# NEEDS UPDATE FOR ZED2i: this script still targets an Intel RealSense D435i
# (hardcoded topics, `ros2 launch realsense2_camera rs_launch.py`). This
# robot's actual camera is a ZED2i, which normally uses Stereolabs' own ZED
# SDK calibration tools rather than this checkerboard/camera_calibration
# flow — needs a team decision on approach, not just a topic-name swap. Not
# fixed in this pass.
#
# Guides you through a checkerboard-based intrinsic calibration of the D435i
# colour or IR camera using the ROS 2 camera_calibration package.
#
# NOTE: The D435i ships with factory-calibrated intrinsics stored in EEPROM.
#       Only run this script if the factory intrinsics appear corrupted or wrong
#       (see docs/Camera_Calibration_Guide.pdf Section 1.3 for how to check).
#
# Output: calibration/camera_intrinsics_<timestamp>/ost.yaml
#         Copy to src/challenge_bringup/config/ and add camera_info_url param
#         to challenge_master.launch.py (see guide Section 2.3).
#
# Prerequisites:
#   • ros-humble-camera-calibration installed
#   • RealSense D435i plugged in and driver running (or use --start-driver)
#   • 9×6 inner corner checkerboard, 25 mm squares, printed flat on rigid board
#
# Usage:
#   ./scripts/calibrate_camera_intrinsics.sh [--stream color|infra1|infra2]
#                                             [--start-driver]
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

STREAM="color"
START_DRIVER=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --stream)       STREAM="$2"; shift 2 ;;
        --start-driver) START_DRIVER=true; shift ;;
        --help|-h)
            echo "Usage: $0 [--stream color|infra1|infra2] [--start-driver]"
            exit 0 ;;
        *) echo "[calibrate_camera_intrinsics] Unknown arg: $1" >&2; exit 1 ;;
    esac
done

# ── Validate stream ──────────────────────────────────────────────────────────
if [[ "$STREAM" != "color" && "$STREAM" != "infra1" && "$STREAM" != "infra2" ]]; then
    echo "[ERROR] --stream must be one of: color, infra1, infra2" >&2
    exit 1
fi

# ── Resolve topic names ──────────────────────────────────────────────────────
case "$STREAM" in
    color)
        IMAGE_TOPIC="/camera/color/image_raw"
        CAMERA_TOPIC="/camera/color"
        WIDTH=640 HEIGHT=480
        ;;
    infra1)
        IMAGE_TOPIC="/camera/infra1/image_rect_raw"
        CAMERA_TOPIC="/camera/infra1"
        WIDTH=848 HEIGHT=480
        ;;
    infra2)
        IMAGE_TOPIC="/camera/infra2/image_rect_raw"
        CAMERA_TOPIC="/camera/infra2"
        WIDTH=848 HEIGHT=480
        ;;
esac

CALIB_DIR="${REPO_ROOT}/calibration"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${CALIB_DIR}/camera_intrinsics_${STREAM}_${TIMESTAMP}"
mkdir -p "${OUTPUT_DIR}"

# ── Banner ───────────────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║  RealSense D435i Intrinsic Calibration        ║"
echo "╚══════════════════════════════════════════════╝"
echo "  Stream      : ${STREAM}"
echo "  Image topic : ${IMAGE_TOPIC}"
echo "  Output dir  : ${OUTPUT_DIR}"
echo ""
echo "  ┌─────────────────────────────────────────────────────────────┐"
echo "  │  CHECKERBOARD REQUIREMENTS                                  │"
echo "  │  • 9×6 inner corners (10×7 squares)                        │"
echo "  │  • 25 mm square size                                        │"
echo "  │  • Printed flat — must not be curled or bent                │"
echo "  │  • Mounted on rigid board (foam board or wood)              │"
echo "  └─────────────────────────────────────────────────────────────┘"
echo ""
echo "  MOTION PROTOCOL:"
echo "    Move the checkerboard (not the robot) to cover all these:"
echo "    • Full left / right / up / down positions in the frame"
echo "    • Close distance (0.4 m) and far distance (2.0 m)"
echo "    • Tilt ±30° in roll, pitch, and yaw"
echo "    Goal: all four progress bars (X, Y, Size, Skew) go green"
echo "    Target: ~60–80 accepted calibration frames"
echo ""

# ── Check camera_calibration is installed ────────────────────────────────────
if ! ros2 pkg prefix camera_calibration &>/dev/null; then
    echo "[INFO] camera_calibration not found. Installing..."
    sudo apt-get install -y ros-humble-camera-calibration
fi

# ── Optionally start the RealSense driver ────────────────────────────────────
DRIVER_PID=""
if [[ "$START_DRIVER" == "true" ]]; then
    echo "[INFO] Starting RealSense driver..."
    if [[ "$STREAM" == "infra1" || "$STREAM" == "infra2" ]]; then
        EXTRA_ARGS="enable_infra1:=true enable_infra2:=true enable_infra_emitter:=false"
    else
        EXTRA_ARGS="enable_color:=true"
    fi
    # shellcheck disable=SC2086
    ros2 launch realsense2_camera rs_launch.py $EXTRA_ARGS &
    DRIVER_PID=$!
    echo "[INFO] Driver PID: ${DRIVER_PID}. Waiting 4 s for startup..."
    sleep 4
fi

# ── Confirm topic is active ───────────────────────────────────────────────────
echo "[INFO] Checking topic ${IMAGE_TOPIC}..."
if ! timeout 5 ros2 topic hz "${IMAGE_TOPIC}" 2>/dev/null | grep -q "average rate"; then
    echo ""
    echo "[ERROR] Topic ${IMAGE_TOPIC} is not publishing."
    echo "  Make sure the RealSense driver is running:"
    echo "    ros2 launch realsense2_camera rs_launch.py"
    [[ -n "$DRIVER_PID" ]] && kill "$DRIVER_PID" 2>/dev/null || true
    exit 1
fi
echo "[INFO] Topic OK — camera is publishing."
echo ""

# ── IR emitter warning ─────────────────────────────────────────────────────
if [[ "$STREAM" == "infra1" || "$STREAM" == "infra2" ]]; then
    echo "  ⚠  IMPORTANT: Disable the IR emitter before calibrating IR cameras."
    echo "     Run in another terminal:"
    echo "     ros2 param set /realsense2_camera2 enable_infra_emitter false"
    echo ""
    read -rp "  Press ENTER when the IR emitter is disabled..."
fi

echo "[INFO] Starting camera_calibration GUI..."
echo "  Click CALIBRATE when all bars are green, then click SAVE."
echo "  Results will be extracted to: ${OUTPUT_DIR}"
echo ""

# ── Run calibration ───────────────────────────────────────────────────────────
ros2 run camera_calibration cameracalibrator \
    --size 9x6 \
    --square 0.025 \
    --ros-args \
    --remap "image:=${IMAGE_TOPIC}" \
    --remap "camera:=${CAMERA_TOPIC}"

# ── Extract results ───────────────────────────────────────────────────────────
if [[ -f /tmp/calibrationdata.tar.gz ]]; then
    echo ""
    echo "[INFO] Extracting calibration results..."
    cd "${OUTPUT_DIR}"
    tar -xzf /tmp/calibrationdata.tar.gz
    echo "[INFO] Calibration YAML: ${OUTPUT_DIR}/ost.yaml"
    echo ""
    echo "  ┌──────────────────────────────────────────────────────────────────┐"
    echo "  │  NEXT STEPS                                                       │"
    echo "  │  1. Inspect ${OUTPUT_DIR}/ost.yaml                               │"
    echo "  │     Verify fx/fy are within expected range (see guide Section 1.4)│"
    echo "  │  2. If values look correct, copy to config/:                      │"
    echo "  │     cp ${OUTPUT_DIR}/ost.yaml \\                                  │"
    echo "  │        src/challenge_bringup/config/${STREAM}_camera_info.yaml   │"
    echo "  │  3. Add to challenge_master.launch.py:                            │"
    echo "  │     ${STREAM}_camera_info_url:=file:///path/to/ost.yaml          │"
    echo "  │  See docs/Camera_Calibration_Guide.pdf Section 2.3 for detail.   │"
    echo "  └──────────────────────────────────────────────────────────────────┘"
else
    echo "[WARNING] /tmp/calibrationdata.tar.gz not found."
    echo "  Did you click SAVE in the calibration GUI?"
fi

[[ -n "$DRIVER_PID" ]] && kill "$DRIVER_PID" 2>/dev/null || true
