#!/usr/bin/env bash
# health_check_zone_nav.sh
# Pre-run sanity check for the zone navigation stack.
# Checks topics, services, and current mode values.
# Exits 0 if all checks pass, 1 if any fail.
# Usage: ./scripts/health_check_zone_nav.sh

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
RESET='\033[0m'

PASS=0
FAIL=0

pass() { echo -e "  ${GREEN}[PASS]${RESET} $1"; (( PASS++ )) || true; }
fail() { echo -e "  ${RED}[FAIL]${RESET} $1"; (( FAIL++ )) || true; }
info() { echo -e "  ${CYAN}[INFO]${RESET} $1"; }
warn() { echo -e "  ${YELLOW}[WARN]${RESET} $1"; }

check_topic_hz() {
    local topic="$1"
    local hz_out
    hz_out=$(timeout 4s ros2 topic hz "${topic}" --window 3 2>&1 || true)
    if echo "${hz_out}" | grep -q "average rate"; then
        pass "${topic} is publishing"
        return 0
    else
        fail "${topic} is NOT publishing (timeout or no data)"
        return 1
    fi
}

check_service_exists() {
    local svc="$1"
    local svc_list
    svc_list=$(ros2 service list 2>/dev/null || true)
    if echo "${svc_list}" | grep -qF "${svc}"; then
        pass "Service ${svc} is available"
        return 0
    else
        fail "Service ${svc} NOT found"
        return 1
    fi
}

echo ""
echo -e "${BOLD}═══════════════════════════════════════════════════${RESET}"
echo -e "${BOLD}  Zone Nav Health Check — Team Juggernauts 2026${RESET}"
echo -e "${BOLD}  $(date '+%Y-%m-%d %H:%M:%S')${RESET}"
echo -e "${BOLD}═══════════════════════════════════════════════════${RESET}"
echo ""

# ── 1. Topic publishing checks ──────────────────────────────────────────────
echo -e "${BOLD}── Topic Publishing Checks ──────────────────────${RESET}"

check_topic_hz "/nav_mode"
check_topic_hz "/mux_mode"
check_topic_hz "/odometry/filtered"
check_topic_hz "/ndt_fitness_score"
check_topic_hz "/cmd_vel_safe"

echo ""

# ── 2. Service availability checks ──────────────────────────────────────────
echo -e "${BOLD}── Service Availability Checks ──────────────────${RESET}"

check_service_exists "/global_costmap/global_costmap/set_parameters"
check_service_exists "/local_costmap/local_costmap/set_parameters"
check_service_exists "/cmd_vel_mux_node/set_parameters"

echo ""

# ── 3. Current value checks ──────────────────────────────────────────────────
echo -e "${BOLD}── Current Value Checks ──────────────────────────${RESET}"

# /nav_mode current value
nav_mode_val=$(ros2 topic echo --once --timeout 3 /nav_mode std_msgs/msg/String 2>/dev/null \
    | grep "data:" | awk '{print $2}' | tr -d "'\"" || true)
if [[ -n "${nav_mode_val}" ]]; then
    info "/nav_mode = ${nav_mode_val}"
    pass "/nav_mode value readable"
else
    fail "/nav_mode value unreadable (timeout)"
fi

# /mux_mode current value
mux_mode_val=$(ros2 topic echo --once --timeout 3 /mux_mode std_msgs/msg/String 2>/dev/null \
    | grep "data:" | awk '{print $2}' | tr -d "'\"" || true)
if [[ -n "${mux_mode_val}" ]]; then
    info "/mux_mode = ${mux_mode_val}"
    pass "/mux_mode value readable"
else
    fail "/mux_mode value unreadable (timeout)"
fi

# /estop_active — must be False
estop_val=$(ros2 topic echo --once --timeout 3 /estop_active std_msgs/msg/Bool 2>/dev/null \
    | grep "data:" | awk '{print $2}' | tr -d "'\"" || true)
if [[ -z "${estop_val}" ]]; then
    warn "/estop_active not publishing (timeout) — robot may be safe or topic unavailable"
elif [[ "${estop_val,,}" == "false" ]]; then
    info "/estop_active = ${estop_val}"
    pass "/estop_active is False (safe to run)"
else
    info "/estop_active = ${estop_val}"
    fail "/estop_active is TRUE — robot will not move until estop is released"
fi

echo ""

# ── Summary ──────────────────────────────────────────────────────────────────
echo -e "${BOLD}═══════════════════════════════════════════════════${RESET}"
echo -e "${BOLD}  Summary${RESET}"
echo -e "  ${GREEN}Passed: ${PASS}${RESET}   ${RED}Failed: ${FAIL}${RESET}"
echo -e "${BOLD}═══════════════════════════════════════════════════${RESET}"
echo ""

if (( FAIL > 0 )); then
    echo -e "${RED}Health check FAILED — fix the above issues before running.${RESET}"
    echo ""
    exit 1
else
    echo -e "${GREEN}Health check PASSED — stack looks ready.${RESET}"
    echo ""
    exit 0
fi
