#!/usr/bin/env bash
# Jetson bringup for the mapless wall follower: opens one terminal per launch.
# Start the IMU (Raspberry Pi) and the Ackermann drive stack first.
# Close a launch with Ctrl-C in its window.
#
# Usage (extra args go to the wall follower launch):
#   src/challenge_bringup/launch/mapless_wall_follower_jetson.sh
#   src/challenge_bringup/launch/mapless_wall_follower_jetson.sh cmd_vel_topic:=/cmd_vel_smoothed
#   src/challenge_bringup/launch/mapless_wall_follower_jetson.sh cmd_vel_topic:=/cmd_vel_smoothed wait_for_green_light:=true

WS="$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)"

open_terminal() {
  # bash -i loads ~/.bashrc like a normal terminal; the window stays open after the launch exits.
  gnome-terminal --title="$1" -- bash -ic "source ${WS}/install/setup.bash; $2; exec bash"
}

open_terminal robot_description "ros2 launch robot_description description.launch.py"
sleep 3

open_terminal hesai_lidar "cd ~ && ./hesai.sh"
sleep 5

open_terminal fast_lio "ros2 launch fast_lio_ros2 lio_localizer.launch.py"
sleep 5

if [[ " $* " == *" wait_for_green_light:=true "* ]]; then
  open_terminal color_detector "ros2 launch diy_zed_color_detection color_detector.launch.py"
  sleep 10
fi

open_terminal wall_follower "ros2 launch mapless_wall_follower mapless_wall_follower_ackermann.launch.py $*"
