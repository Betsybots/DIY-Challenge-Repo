#!/usr/bin/env bash
# inspect_zone_nav.sh
# Live dashboard for the zone navigation stack.
# Updates every 1 second. Press Ctrl-C to exit.
# Usage: ./scripts/inspect_zone_nav.sh

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
RESET='\033[0m'

TIMEOUT_LABEL="${RED}TIMEOUT${RESET}"

get_string_field() {
    # Usage: get_string_field TOPIC TYPE
    local topic="$1"
    local type="$2"
    local val
    val=$(ros2 topic echo --once --timeout 1 "${topic}" "${type}" 2>/dev/null \
        | grep "data:" | head -1 | awk '{print $2}' | tr -d "'\"" || true)
    echo "${val}"
}

get_float_field() {
    # Usage: get_float_field TOPIC TYPE
    local topic="$1"
    local type="$2"
    local val
    val=$(ros2 topic echo --once --timeout 1 "${topic}" "${type}" 2>/dev/null \
        | grep "data:" | head -1 | awk '{print $2}' || true)
    echo "${val}"
}

get_bool_field() {
    local topic="$1"
    local type="$2"
    local val
    val=$(ros2 topic echo --once --timeout 1 "${topic}" "${type}" 2>/dev/null \
        | grep "data:" | head -1 | awk '{print $2}' | tr -d "'\"" || true)
    echo "${val}"
}

get_odom_x() {
    ros2 topic echo --once --timeout 1 /odometry/filtered nav_msgs/msg/Odometry 2>/dev/null \
        | grep -A3 "position:" | grep "x:" | head -1 | awk '{print $2}' || true
}

get_odom_y() {
    ros2 topic echo --once --timeout 1 /odometry/filtered nav_msgs/msg/Odometry 2>/dev/null \
        | grep -A3 "position:" | grep "y:" | head -1 | awk '{print $2}' || true
}

get_cmd_vel_out() {
    local raw
    raw=$(ros2 topic echo --once --timeout 1 /cmd_vel_safe geometry_msgs/msg/Twist 2>/dev/null || true)
    local lx az
    lx=$(echo "${raw}" | grep -A1 "linear:" | grep "x:" | awk '{print $2}' || true)
    az=$(echo "${raw}" | grep -A3 "angular:" | grep "z:" | awk '{print $2}' || true)
    if [[ -n "${lx}" ]]; then
        printf "linear.x=%-6s  angular.z=%-6s" "${lx}" "${az}"
    else
        echo ""
    fi
}

fmt_val() {
    # Colour-code a value; empty = TIMEOUT
    local val="$1"
    if [[ -z "${val}" ]]; then
        echo -e "${TIMEOUT_LABEL}"
    else
        echo "${val}"
    fi
}

trap 'echo -e "\n${RESET}Exiting inspector."; exit 0' INT TERM

while true; do
    # Collect all values
    nav_mode=$(get_string_field /nav_mode  std_msgs/msg/String)
    mux_mode=$(get_string_field /mux_mode  std_msgs/msg/String)
    estop=$(get_bool_field     /estop_active std_msgs/msg/Bool)
    ndt=$(get_float_field      /ndt_fitness_score std_msgs/msg/Float32)
    speed_limit=$(get_float_field /speed_limit std_msgs/msg/Float32)
    robot_x=$(get_odom_x)
    robot_y=$(get_odom_y)
    cmd_vel_out=$(get_cmd_vel_out)

    # Format timestamp
    ts=$(date '+%Y-%m-%d %H:%M:%S')

    # Format estop with colour
    if [[ "${estop,,}" == "true" ]]; then
        estop_fmt="${RED}${estop}${RESET}"
    elif [[ "${estop,,}" == "false" ]]; then
        estop_fmt="${GREEN}${estop}${RESET}"
    elif [[ -z "${estop}" ]]; then
        estop_fmt="${TIMEOUT_LABEL}"
    else
        estop_fmt="${estop}"
    fi

    # Format nav_mode with colour
    case "${nav_mode}" in
        NORMAL_NAV)     nm_fmt="${GREEN}${nav_mode}${RESET}" ;;
        SLOW_NAV)       nm_fmt="${CYAN}${nav_mode}${RESET}" ;;
        BLIND_DRIVE)    nm_fmt="${RED}${nav_mode}${RESET}" ;;
        NAVIGATE_AROUND) nm_fmt="${YELLOW}${nav_mode}${RESET}" ;;
        PUSH_THROUGH)   nm_fmt="${YELLOW}${nav_mode}${RESET}" ;;
        DONE)           nm_fmt="${GREEN}${nav_mode}${RESET}" ;;
        INIT)           nm_fmt="${CYAN}${nav_mode}${RESET}" ;;
        "")             nm_fmt="${TIMEOUT_LABEL}" ;;
        *)              nm_fmt="${nav_mode}" ;;
    esac

    # Format cmd_vel output
    if [[ -z "${cmd_vel_out}" ]]; then
        cv_fmt="${TIMEOUT_LABEL}"
    else
        cv_fmt="${cmd_vel_out}"
    fi

    # Clear screen and print dashboard
    clear
    echo -e "${BOLD}═══════════════════════════════════════════════════${RESET}"
    echo -e "${BOLD}  Zone Nav Inspector — Team Juggernauts 2026${RESET}"
    echo -e "  ${ts}   (Ctrl-C to exit)"
    echo -e "${BOLD}═══════════════════════════════════════════════════${RESET}"
    printf "  %-16s : " "State Machine"
    echo -e "${nm_fmt}"
    printf "  %-16s : " "Mux Mode"
    echo -e "$(fmt_val "${mux_mode}")"
    printf "  %-16s : " "E-Stop Active"
    echo -e "${estop_fmt}"
    printf "  %-16s : " "Lap"
    echo "(not tracked via topic — see /zone_nav_manager params)"
    printf "  %-16s : " "NDT Fitness"
    echo -e "$(fmt_val "${ndt}")"
    printf "  %-16s : " "Robot X"
    echo -e "$(fmt_val "${robot_x}")"
    printf "  %-16s : " "Robot Y"
    echo -e "$(fmt_val "${robot_y}")"
    printf "  %-16s : " "Speed Limit"
    if [[ -z "${speed_limit}" ]]; then
        echo -e "${TIMEOUT_LABEL}"
    else
        echo "${speed_limit} m/s"
    fi
    printf "  %-16s : " "Cmd Vel Out"
    echo -e "${cv_fmt}"
    echo -e "${BOLD}═══════════════════════════════════════════════════${RESET}"

    sleep 1
done
