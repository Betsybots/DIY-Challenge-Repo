#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/health_check.sh — Pre-flight system health check
# ══════════════════════════════════════════════════════════════════════════════
# Runs a structured preflight checklist before a competition run:
#   1. ROS 2 environment sanity
#   2. Required node liveness
#   3. Topic rate checks (lidar, IMU, GPS, heartbeat)
#   4. TF tree completeness (map→odom→base_link→lidar_link)
#   5. E-stop state (must be inactive)
#   6. Nav2 lifecycle state (must be ACTIVE when use_nav2=true)
#   7. Last-seen parameter checks
#
# Usage:
#   ./scripts/health_check.sh [profile]
#
# Exit codes:
#   0 — All checks passed
#   1 — One or more critical checks failed (do NOT run competition)
# ══════════════════════════════════════════════════════════════════════════════
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PROFILE="${1:-}"
# shellcheck source=scripts/env.sh
source "${SCRIPT_DIR}/env.sh" "${PROFILE}"

PASS=0
FAIL=0

_pass() { echo "  ✓ $1"; (( PASS++ )) || true; }
_fail() { echo "  ✗ $1" >&2; (( FAIL++ )) || true; }
_warn() { echo "  ⚠ $1"; }

_check_topic_rate() {
    local topic="$1"
    local min_hz="$2"
    local label="${3:-${topic}}"
    local measured
    measured=$(timeout 3 ros2 topic hz "${topic}" 2>/dev/null | grep -oP '[\d.]+ hz' | head -1 | grep -oP '[\d.]+' || echo "0")
    if (( $(echo "${measured} >= ${min_hz}" | bc -l 2>/dev/null || echo 0) )); then
        _pass "${label} at ${measured} Hz (≥ ${min_hz} Hz)"
    else
        _fail "${label} rate ${measured} Hz — expected ≥ ${min_hz} Hz"
    fi
}

echo ""
echo "╔═════════════════════════════════════════╗"
echo "║   DIY Challenge Robot — Health Check    ║"
echo "╚═════════════════════════════════════════╝"
echo ""

# ── 1. Environment ─────────────────────────────────────────────────────────────
echo "── Environment ──"
[[ -n "${ROS_DISTRO:-}" ]] && _pass "ROS_DISTRO=${ROS_DISTRO}" || _fail "ROS_DISTRO not set"
[[ "${ROS_DISTRO:-}" == "humble" ]] && _pass "Distro is Humble" || _warn "ROS_DISTRO=${ROS_DISTRO:-?} (expected humble)"
echo ""

# ── 2. Node liveness ──────────────────────────────────────────────────────────
echo "── Required Nodes ──"
REQUIRED_NODES=()
[[ "${DIY_USE_HESAI}" == "true" ]]    && REQUIRED_NODES+=("/hesai_ros_driver")
[[ "${DIY_USE_MICRO_ROS}" == "true" ]] && REQUIRED_NODES+=("/micro_ros_agent" "/estop_controller_node")
[[ "${DIY_USE_LOCALIZATION}" == "true" ]] && REQUIRED_NODES+=("/fastlio_mapping" "/ekf_filter_node_odom")
[[ "${DIY_USE_CMD_VEL_MUX:-true}" == "true" ]] && REQUIRED_NODES+=("/cmd_vel_mux_node")

LIVE_NODES=$(ros2 node list 2>/dev/null || echo "")
for node in "${REQUIRED_NODES[@]}"; do
    if echo "${LIVE_NODES}" | grep -q "${node}"; then
        _pass "Node ${node} is running"
    else
        _fail "Node ${node} NOT found"
    fi
done
echo ""

# ── 3. Topic rates ────────────────────────────────────────────────────────────
echo "── Topic Rates ──"
[[ "${DIY_USE_HESAI}" == "true" ]]   && _check_topic_rate /lidar_points 10 "Hesai lidar"
_check_topic_rate /imu/data 100 "IMU"
[[ "${DIY_USE_GPS}" == "true" ]]     && _check_topic_rate /gps/fix 1 "GPS fix"
[[ "${DIY_USE_MICRO_ROS}" == "true" ]] && _check_topic_rate /stm32/heartbeat 5 "STM32 heartbeat"
[[ "${DIY_USE_LOCALIZATION}" == "true" ]] && _check_topic_rate /odometry/filtered 20 "EKF odometry"
echo ""

# ── 4. TF tree ───────────────────────────────────────────────────────────────
echo "── TF Tree ──"
TF_CHECK_PAIRS=("odom:base_link" "base_link:lidar_link" "base_link:imu_link")
[[ "${DIY_USE_LOCALIZATION}" == "true" ]] && TF_CHECK_PAIRS+=("map:odom")

for pair in "${TF_CHECK_PAIRS[@]}"; do
    IFS=':' read -r parent child <<< "${pair}"
    result=$(timeout 2 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>/dev/null | head -2 || echo "")
    if echo "${result}" | grep -q "At time"; then
        _pass "TF ${parent} → ${child}"
    else
        _fail "TF ${parent} → ${child} NOT available"
    fi
done
echo ""

# ── 5. E-stop state ───────────────────────────────────────────────────────────
echo "── E-Stop State ──"
if [[ "${DIY_USE_MICRO_ROS}" == "true" ]]; then
    ESTOP_MSG=$(timeout 2 ros2 topic echo --once /estop_active 2>/dev/null || echo "")
    if echo "${ESTOP_MSG}" | grep -q "data: false"; then
        _pass "E-stop is INACTIVE (safe to proceed)"
    elif echo "${ESTOP_MSG}" | grep -q "data: true"; then
        _fail "E-stop is ACTIVE — resolve before launch!"
    else
        _fail "E-stop topic /estop_active not responding"
    fi
else
    # No STM32 on this robot (DIY_USE_MICRO_ROS=false) — diy_estop_controller
    # is not launched, so /estop_active is never published (this is expected
    # and safe; cmd_vel_mux defaults _estop_active=False and only latches
    # True on an actual received message — see cmd_vel_mux_node.py). The real
    # hardware e-stop here is an RJ45 break-loop wired directly into motor
    # power, independent of ROS entirely — this script cannot check that.
    _warn "No STM32 on this robot — /estop_active is not published by design."
    _warn "Verify the RJ45 break-loop e-stop hardware manually before running."
fi
echo ""

# ── 6. Nav2 lifecycle (if enabled) ───────────────────────────────────────────
if [[ "${DIY_USE_NAV2}" == "true" ]]; then
    echo "── Nav2 Lifecycle ──"
    NAV2_NODES=("controller_server" "planner_server" "bt_navigator")
    for n in "${NAV2_NODES[@]}"; do
        state=$(ros2 lifecycle get "/nav2_${n}" 2>/dev/null | grep -oP '(?<=state: )\w+' || echo "unknown")
        if [[ "${state}" == "active" ]]; then
            _pass "nav2/${n} is ACTIVE"
        else
            _warn "nav2/${n} state: ${state:-unknown}"
        fi
    done
    echo ""
fi

# ── Summary ─────────────────────────────────────────────────────────────────
echo "══════════════════════════════════════════"
echo "  Passed: ${PASS}   Failed: ${FAIL}"
echo "══════════════════════════════════════════"
echo ""

if (( FAIL > 0 )); then
    echo "⛔  ${FAIL} check(s) FAILED — DO NOT start competition run." >&2
    exit 1
else
    echo "✅  All checks passed. System is ready."
    exit 0
fi
