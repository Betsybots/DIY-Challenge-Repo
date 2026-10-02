#!/usr/bin/env bash
# Jetson bringup for the MPPI obstacle course: opens one terminal per launch.
# Start the IMU (Raspberry Pi) and the Ackermann drive stack first.
# Close a launch with Ctrl-C in its window.
#
# TF: odom -> base_footprint is owned ONLY by lio_ekf (FAST-LIO /Odometry + /wheel_odom
# + /imu/data -> /odom); FAST-LIO runs with its TF off, mcl_3dl owns map -> odom.
#
# Usage (extra args go to the Nav2 launch):
#   src/challenge_bringup/launch/obstacle_course_jetson.sh
#   src/challenge_bringup/launch/obstacle_course_jetson.sh log_level:=debug

WS="$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)"

open_terminal() {
  # bash -i loads ~/.bashrc like a normal terminal; the window stays open after the launch exits.
  gnome-terminal --title="$1" -- bash -ic "source ${WS}/install/setup.bash; $2; exec bash"
}

open_terminal robot_description "ros2 launch robot_description description.launch.py"
sleep 3

open_terminal hesai_lidar "cd ~ && ./hesai_launch.sh"
sleep 10

open_terminal fast_lio "ros2 launch fast_lio_ros2 lio_localizer.launch.py publish_tf:=false"
sleep 10

open_terminal lio_ekf "ros2 launch fast_lio_ros2 lio_ekf.launch.py output_topic:=/odom"
sleep 5

open_terminal mcl_3dl "ros2 launch mcl_3dl mcl_localizer.launch.py \
  cloud_topic:=/cloud_deskewed imu_topic:=/imu/data odom_topic:=/odom"
sleep 10

open_terminal nav2 "ros2 launch challenge_bringup nav2_navigation_mppi_obstacle_course_ackermann.launch.py $*"
