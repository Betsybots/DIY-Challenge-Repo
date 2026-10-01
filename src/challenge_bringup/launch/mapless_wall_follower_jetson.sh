#!/usr/bin/env bash
# Jetson bringup for the mapless wall follower (see src/mapless_wall_follower/README.md).
# Starts, in order: robot_description -> Hesai LiDAR -> FAST-LIO -> wall follower.
# The IMU (Raspberry Pi) and the Ackermann drive stack must be started separately.
#
# Usage:
#   src/challenge_bringup/launch/mapless_wall_follower_jetson.sh [--terminals] [--green-light] \
#     [wall follower launch args...]
#
#   --terminals    Open each launch in its own terminal window (gnome-terminal or xterm)
#                  instead of logging to files. Ctrl-C here stops all of them; windows
#                  stay open afterwards so the output can be inspected.
#   --green-light  Start diy_zed_color_detection (with the ZED wrapper) and launch the
#                  wall follower with wait_for_green_light:=true, so GREEN enables it.
#                  Passing wait_for_green_light:=true directly does the same.
#
# Examples:
#   # Stationary test (default output /cmd_vel_nav)
#   ./mapless_wall_follower_jetson.sh
#   # Direct moving test straight to the Ackermann driver
#   ./mapless_wall_follower_jetson.sh cmd_vel_topic:=/cmd_vel_smoothed
#   # Same, one terminal window per launch
#   ./mapless_wall_follower_jetson.sh --terminals cmd_vel_topic:=/cmd_vel_smoothed
#   # Race start on the green light
#   ./mapless_wall_follower_jetson.sh --terminals --green-light cmd_vel_topic:=/cmd_vel_smoothed
#
# Env overrides: DIY_WS (workspace root), TOPIC_TIMEOUT (s), CAMERA_TIMEOUT (s), LOG_DIR.
# The controller starts DISABLED. Enable with:
#   ros2 topic pub --once /mapless_wall_follower/enable std_msgs/msg/Bool "{data: true}"
set -eo pipefail

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
DIY_WS="${DIY_WS:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
TOPIC_TIMEOUT="${TOPIC_TIMEOUT:-30}"
# ZED wrapper startup (depth model load) is much slower than the LiDAR.
CAMERA_TIMEOUT="${CAMERA_TIMEOUT:-90}"
LOG_DIR="${LOG_DIR:-/tmp/mapless_wall_follower_$(date +%Y%m%d_%H%M%S)}"

USE_TERMINALS=false
GREEN_LIGHT=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --terminals) USE_TERMINALS=true; shift ;;
    --green-light) GREEN_LIGHT=true; shift ;;
    *) break ;;
  esac
done

WF_ARGS=()
for arg in "$@"; do
  case "${arg,,}" in
    wait_for_green_light:=true | wait_for_green_light:=1) GREEN_LIGHT=true ;;
    wait_for_green_light:=*) ;;
    *) WF_ARGS+=("${arg}") ;;
  esac
done
if [[ "${GREEN_LIGHT}" == true ]]; then
  WF_ARGS+=("wait_for_green_light:=true")
fi
set -- "${WF_ARGS[@]}"

TERMINAL=""
if [[ "${USE_TERMINALS}" == true ]]; then
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "ERROR: --terminals needs a graphical session (DISPLAY is not set)." >&2
    exit 1
  fi
  for candidate in gnome-terminal xterm; do
    if command -v "${candidate}" >/dev/null; then
      TERMINAL="${candidate}"
      break
    fi
  done
  if [[ -z "${TERMINAL}" ]]; then
    echo "ERROR: --terminals needs gnome-terminal or xterm." >&2
    exit 1
  fi
fi

if [[ ! -f "${DIY_WS}/install/setup.bash" ]]; then
  echo "ERROR: ${DIY_WS}/install/setup.bash not found. Build the workspace or set DIY_WS." >&2
  exit 1
fi
# shellcheck disable=SC1091
source "${DIY_WS}/install/setup.bash"
set -u

# Job control: background launches get their own process group and do not ignore SIGINT.
set -m
mkdir -p "${LOG_DIR}"
PIDS=()
# PID files of the shell inside each terminal window (--terminals mode).
TERMINAL_PID_FILES=()

cleanup() {
  trap - EXIT INT TERM
  echo
  echo "[bringup] Shutting down..."
  for ((i = ${#TERMINAL_PID_FILES[@]} - 1; i >= 0; i--)); do
    if [[ -f "${TERMINAL_PID_FILES[i]}" ]]; then
      pkill -INT -P "$(cat "${TERMINAL_PID_FILES[i]}")" 2>/dev/null || true
    fi
  done
  for ((i = ${#PIDS[@]} - 1; i >= 0; i--)); do
    kill -INT -- "-${PIDS[i]}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  echo "[bringup] Done. Logs: ${LOG_DIR}"
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

start_in_terminal() {
  local name="$1"
  shift
  local pid_file="${LOG_DIR}/${name}.pid"
  local env_exports=""
  local var
  # gnome-terminal windows are spawned by its server and may not inherit this shell's env.
  for var in ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI \
    FASTRTPS_DEFAULT_PROFILES_FILE; do
    if [[ -n "${!var:-}" ]]; then
      env_exports+="export ${var}=$(printf '%q' "${!var}"); "
    fi
  done
  local inner_cmd
  inner_cmd="echo \$\$ > $(printf '%q' "${pid_file}"); ${env_exports}"
  inner_cmd+="source $(printf '%q' "${DIY_WS}/install/setup.bash"); "
  inner_cmd+="ros2 launch $(printf '%q ' "$@"); "
  inner_cmd+="echo; read -rp '[${name}] exited. Press Enter to close.'"

  echo "[bringup] Starting ${name} in a new ${TERMINAL} window"
  rm -f "${pid_file}"
  if [[ "${TERMINAL}" == gnome-terminal ]]; then
    gnome-terminal --title="${name}" -- bash -c "${inner_cmd}"
  else
    xterm -T "${name}" -e bash -c "${inner_cmd}" &
  fi
  TERMINAL_PID_FILES+=("${pid_file}")
}

start_bg() {
  local name="$1"
  shift
  if [[ "${USE_TERMINALS}" == true ]]; then
    start_in_terminal "${name}" "$@"
    return
  fi
  echo "[bringup] Starting ${name} (log: ${LOG_DIR}/${name}.log)"
  ros2 launch "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

wait_for_topic() {
  local topic="$1"
  local wait_s="${2:-${TOPIC_TIMEOUT}}"
  echo "[bringup] Waiting for ${topic} (timeout ${wait_s}s)..."
  if ! timeout "${wait_s}" ros2 topic echo --once "${topic}" >/dev/null 2>&1; then
    if [[ "${USE_TERMINALS}" == true ]]; then
      echo "ERROR: no message on ${topic} within ${wait_s}s. Check the launch windows." >&2
    else
      echo "ERROR: no message on ${topic} within ${wait_s}s. Check ${LOG_DIR}." >&2
    fi
    return 1
  fi
  echo "[bringup] ${topic} OK"
}

# 1. IMU runs on the Raspberry Pi; FAST-LIO needs it.
wait_for_topic /imu/data || exit 1

# 2. Robot description
start_bg robot_description robot_description description.launch.py

# Optional: ZED + colour detector, started early so the camera boots in parallel.
if [[ "${GREEN_LIGHT}" == true ]]; then
  start_bg color_detector diy_zed_color_detection color_detector.launch.py
fi

# 3. Hesai LiDAR
start_bg hesai_lidar hesai_ros_driver start.py
wait_for_topic /lidar_points || exit 1

# 4. FAST-LIO
start_bg fast_lio fast_lio_ros2 lio_localizer.launch.py
wait_for_topic /cloud_registered_body || exit 1

# Not fatal: the trigger stays armed and fires if the detector comes up later.
if [[ "${GREEN_LIGHT}" == true ]] &&
  ! wait_for_topic /color_detector/green_light "${CAMERA_TIMEOUT}"; then
  echo "WARNING: colour detector not ready; starting the wall follower anyway (DISABLED)." >&2
  echo "         Enable manually or fake GREEN:" >&2
  echo "         ros2 topic pub -r 10 /color_detector/green_light std_msgs/msg/Bool \"{data: true}\"" >&2
fi

# 5. Wall follower
if [[ "${USE_TERMINALS}" == true ]]; then
  start_bg wall_follower mapless_wall_follower mapless_wall_follower_ackermann.launch.py "$@"
  echo "[bringup] All launches started. Press Ctrl-C here to stop all of them."
  # Background + wait so Ctrl-C reaches this script's trap (set -m isolates foreground jobs).
  sleep infinity &
  PIDS+=("$!")
  wait "${PIDS[-1]}"
  exit 0
fi

# Foreground output for CONTROL DEBUG
echo "[bringup] Starting mapless wall follower $*"
ros2 launch mapless_wall_follower mapless_wall_follower_ackermann.launch.py "$@" &
PIDS+=("$!")
wait "${PIDS[-1]}"
