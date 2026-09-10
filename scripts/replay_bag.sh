#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/replay_bag.sh — Replay a recorded bag through the stack
# ══════════════════════════════════════════════════════════════════════════════
# Starts the localisation + Nav2 stack in replay mode, then plays the bag.
# Sensor driver nodes are skipped; bag data is substituted instead.
#
# Usage:
#   ./scripts/replay_bag.sh <bag_path> [profile] [--rate <speed>] [--no-nav2]
#
# Examples:
#   ./scripts/replay_bag.sh ~/bags/jetson_run1_20260516 laptop
#   ./scripts/replay_bag.sh ~/bags/session laptop --rate 0.5
#   ./scripts/replay_bag.sh ~/bags/session laptop --no-nav2
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Parse arguments ───────────────────────────────────────────────────────────
BAG_PATH=""
PROFILE=""
RATE=1.0
USE_NAV2=true

while [[ $# -gt 0 ]]; do
    case "$1" in
        --rate)    RATE="$2"; shift 2 ;;
        --no-nav2) USE_NAV2=false; shift ;;
        --*)       echo "[replay_bag] Unknown flag: $1" >&2; exit 1 ;;
        *)
            if [[ -z "${BAG_PATH}" ]]; then
                BAG_PATH="$1"
            else
                PROFILE="$1"
            fi
            shift
            ;;
    esac
done

if [[ -z "${BAG_PATH}" ]]; then
    echo "Usage: $0 <bag_path> [profile] [--rate <speed>] [--no-nav2]" >&2
    exit 1
fi

if [[ ! -d "${BAG_PATH}" ]]; then
    echo "[replay_bag] ERROR: Bag path does not exist: ${BAG_PATH}" >&2
    exit 1
fi

# shellcheck source=scripts/env.sh
source "${SCRIPT_DIR}/env.sh" "${PROFILE}"

echo ""
echo "╔══════════════════════════════════╗"
echo "║  DIY Challenge Robot — REPLAY    ║"
echo "╚══════════════════════════════════╝"
echo "  Bag    : ${BAG_PATH}"
echo "  Profile: ${DIY_ROBOT_PROFILE}"
echo "  Rate   : ${RATE}x"
echo "  Nav2   : ${USE_NAV2}"
echo ""

# ── Launch the software stack without hardware drivers ────────────────────────
echo "[replay_bag] Starting software stack (no hardware drivers)..."
ros2 launch challenge_bringup challenge_master.launch.py \
    use_joystick:=false \
    use_nav2:="${USE_NAV2}" \
    use_zed:=false \
    use_motor_driver:=false \
    use_hesai:=false \
    use_micro_ros:=false \
    use_localization:="${DIY_USE_LOCALIZATION}" \
    use_cmd_vel_mux:=true \
    mux_mode:="${DIY_MUX_MODE}" \
    fastlio_config:="${DIY_FASTLIO_CONFIG:-fast_lio_hesai_qt64.yaml}" \
    use_rviz:=true &

STACK_PID=$!
echo "[replay_bag] Stack PID: ${STACK_PID}"

# Give the stack time to initialise
echo "[replay_bag] Waiting 5 s for stack to initialise..."
sleep 5

# ── Play the bag ──────────────────────────────────────────────────────────────
echo "[replay_bag] Playing bag at ${RATE}x speed..."
ros2 bag play \
    --rate "${RATE}" \
    --clock \
    "${BAG_PATH}"

echo "[replay_bag] Bag finished. Stopping stack..."
kill "${STACK_PID}" 2>/dev/null || true
wait "${STACK_PID}" 2>/dev/null || true
echo "[replay_bag] Done."
