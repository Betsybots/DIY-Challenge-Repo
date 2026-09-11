#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/test_step5_full_autonomy.sh — Step 5: Full autonomy stack
#   (localizer + controller + waypoint sequencer, one terminal)
# ══════════════════════════════════════════════════════════════════════════════
# Brings up the WHOLE Jetson-side autonomy chain in one terminal, backgrounding
# each node the same way test_step3_fastlio.sh / test_step4_motion_plan.sh do:
#   1. diy_localization localization.launch.py   (FAST-LIO2 + EKF + map_localizer)
#   2. diy_motion_planner pd_navigation.launch.py (nav2_map_server + A* + PD)
#   3. diy_cmd_vel_mux cmd_vel_mux_node            (set to AUTONOMOUS automatically)
#   4. diy_waypoint_sequencer waypoint_sequencer.launch.py
#
# Without this script you would need 5 separate terminals just on the Jetson
# (one per node above, plus one free for verification/trigger commands) — see
# docs/reuse_plan_step1.md Step 18 for the full "how many terminals" discussion
# this replaces.
#
# initial_x/initial_y/initial_yaw for map_localizer's relocalization are
# AUTO-DERIVED from wp1 in --waypoints-file (see print_initial_pose_hint() in
# scripts/pick_waypoints.py — same numbers, just read directly from the file
# here instead of requiring you to copy them by hand). This assumes you
# physically place the robot at wp1's real-world position before running this
# script — if that is not true, override with --initial-x/--initial-y/
# --initial-yaw explicitly.
#
# PREREQUISITE (outside this repo, not checked by this script): the Zenoh
# bridge must already be up on BOTH the Jetson and the RPi before running
# this — without it, /imu/data and /wheel_odom never reach the EKF here,
# and /cmd_vel_nav never reaches driveStack's motor driver on the RPi. See
# docs/jetson_bringup_guide.md's prerequisites table.
#
# This script does NOT run driveStack — that stays on the RPi, in its own
# terminal, independent of everything here (per the Zenoh-bridge cmd_vel_safe
# design). It also does NOT auto-fire the green light — that is a deliberate,
# separate action you take once you've verified localization actually
# converged (see the printed verification commands below), so a bad
# convergence can't silently start the robot moving on its own.
#
# Usage:
#   scripts/test_step5_full_autonomy.sh [profile] [options]
#
# Options:
#   --map-pcd-path PATH     3D map for map_localizer (default: maps/refined_map.pcd)
#   --map-yaml PATH         2D map for nav2_map_server/A* (default: $DIY_MAP_YAML
#                           or maps/course_traced_smooth.yaml)
#   --waypoints-file PATH   waypoints.yaml for the sequencer (default: maps/test_waypoints.yaml)
#   --initial-x/--initial-y/--initial-yaw   override the auto-derived wp1 pose
#   --viz rviz|foxglove|none
#
# Verify (in a separate terminal, once this is running):
#   ros2 run tf2_ros tf2_echo map base_link     — confirm localization converged
#   ros2 topic echo /goal_pose                  — confirm goals advance wp1->wp2->...
#   ros2 topic echo /cmd_vel_nav                — confirm nonzero commands while moving
# Then, once you trust the above:
#   scripts/trigger_green_light.sh
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${SCRIPT_DIR}/_viz_helper.sh"
viz_parse_args "$@"
set -- "${REMAINING_ARGS[@]}"

# ── Argument parsing ──────────────────────────────────────────────────────────
PROFILE=""
MAP_PCD_PATH=""
MAP_YAML=""
WAYPOINTS_FILE=""
INITIAL_X=""
INITIAL_Y=""
INITIAL_YAW=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --map-pcd-path)   MAP_PCD_PATH="$2";   shift 2 ;;
        --map-yaml)       MAP_YAML="$2";       shift 2 ;;
        --waypoints-file) WAYPOINTS_FILE="$2"; shift 2 ;;
        --initial-x)      INITIAL_X="$2";      shift 2 ;;
        --initial-y)      INITIAL_Y="$2";      shift 2 ;;
        --initial-yaw)    INITIAL_YAW="$2";    shift 2 ;;
        *)
            if [[ -z "${PROFILE}" ]]; then
                PROFILE="$1"
            else
                echo "[test_step5] ERROR: Unexpected argument '$1'" >&2
                exit 1
            fi
            shift
            ;;
    esac
done

source "${SCRIPT_DIR}/env.sh" "${PROFILE}"

MAP_PCD_PATH="${MAP_PCD_PATH:-${REPO_ROOT}/maps/refined_map.pcd}"
MAP_YAML="${MAP_YAML:-${DIY_MAP_YAML:-${REPO_ROOT}/maps/course_traced_smooth.yaml}}"
WAYPOINTS_FILE="${WAYPOINTS_FILE:-${REPO_ROOT}/maps/test_waypoints.yaml}"

# ── Pre-flight file checks (fail fast, before starting anything) ────────────
for f in "${MAP_PCD_PATH}" "${MAP_YAML}" "${WAYPOINTS_FILE}"; do
    if [[ ! -f "${f}" ]]; then
        echo "[test_step5] ERROR: File not found: ${f}" >&2
        exit 1
    fi
done

# ── Auto-derive initial_x/y/yaw from wp1, unless explicitly overridden ──────
if [[ -z "${INITIAL_X}" || -z "${INITIAL_Y}" || -z "${INITIAL_YAW}" ]]; then
    WP1_POSE="$(python3 - "${WAYPOINTS_FILE}" <<'EOF'
import sys
import yaml

with open(sys.argv[1]) as f:
    data = yaml.safe_load(f) or {}
waypoints = data.get("waypoints", [])
if not waypoints:
    print("[test_step5] ERROR: no waypoints found in the waypoints file", file=sys.stderr)
    sys.exit(1)
wp1 = waypoints[0]
print(f"{wp1['x']} {wp1['y']} {wp1.get('yaw', 0.0)}")
EOF
)"
    read -r WP1_X WP1_Y WP1_YAW <<< "${WP1_POSE}"
    INITIAL_X="${INITIAL_X:-${WP1_X}}"
    INITIAL_Y="${INITIAL_Y:-${WP1_Y}}"
    INITIAL_YAW="${INITIAL_YAW:-${WP1_YAW}}"
fi

RVIZ_CONFIG="${REPO_ROOT}/src/challenge_bringup/rviz/localization.rviz"

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║  Step 5 — Full Autonomy (localizer + controller + wp seq) ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo "  map_pcd_path   : ${MAP_PCD_PATH}"
echo "  map_yaml       : ${MAP_YAML}"
echo "  waypoints_file : ${WAYPOINTS_FILE}"
echo "  initial pose   : x=${INITIAL_X} y=${INITIAL_Y} yaw=${INITIAL_YAW}  (from wp1, unless overridden)"
echo "  Visualization  : ${VIZ_MODE}"
echo ""
echo "  ASSUMES the robot is physically placed at wp1's real-world"
echo "  position right now. If it is not, stop and re-run with"
echo "  --initial-x/--initial-y/--initial-yaw matching where it really is."
echo ""
echo "  Green light is NOT fired automatically — verify localization"
echo "  converged first (see the header comment for commands), then run:"
echo "    scripts/trigger_green_light.sh"
echo ""
echo "  Press Ctrl-C to stop all nodes."
echo ""

cleanup() {
    echo ""
    echo "[test_step5] Stopping all nodes..."
    kill 0
}
trap cleanup INT TERM

# Node 1: localization (FAST-LIO2 + EKF + map_localizer + relocalize trigger)
ros2 launch diy_localization localization.launch.py \
    mode:=runtime \
    map_pcd_path:="${MAP_PCD_PATH}" \
    initial_x:="${INITIAL_X}" initial_y:="${INITIAL_Y}" initial_yaw:="${INITIAL_YAW}" &
PID_LOCALIZATION=$!

# Node 2: controller (nav2_map_server + A* planner + PD motion planner)
ros2 launch diy_motion_planner pd_navigation.launch.py \
    map_yaml:="${MAP_YAML}" &
PID_CONTROLLER=$!

# Node 3: cmd_vel mux — /cmd_vel_nav only reaches the motors once this is up
# AND in AUTONOMOUS mode (set automatically below, after it has time to start).
ros2 run diy_cmd_vel_mux cmd_vel_mux_node &
PID_MUX=$!

# Node 4: waypoint sequencer (waits for /green_light — see header comment)
ros2 launch diy_waypoint_sequencer waypoint_sequencer.launch.py \
    waypoints_file:="${WAYPOINTS_FILE}" &
PID_WAYPOINT_SEQ=$!

viz_start "${RVIZ_CONFIG}"

echo "[test_step5] All nodes started. Waiting for cmd_vel_mux_node to come up..."
sleep 5
ros2 param set /cmd_vel_mux_node mode AUTONOMOUS \
    || echo "[test_step5] WARNING: could not set mux mode yet — retry manually with scripts/set_mux_mode.sh AUTONOMOUS"

echo ""
echo "[test_step5] Verify in another terminal:"
echo "  ros2 run tf2_ros tf2_echo map base_link"
echo "  ros2 topic echo /goal_pose"
echo "  ros2 topic echo /cmd_vel_nav"
echo ""
echo "[test_step5] When ready: scripts/trigger_green_light.sh"
echo ""

wait
