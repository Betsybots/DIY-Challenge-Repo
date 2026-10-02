#!/usr/bin/env bash
# Jetson bringup for the obstacle course WITH course_supervisor: same terminals as
# obstacle_course_jetson.sh (which stays the fallback), but navigation runs through
# course_supervisor (section-by-section Nav2 path following / wall follower /
# push-through tracker) and the supervisor drives the mcl_3dl lidar gate.
# Start the IMU (Raspberry Pi) and the Ackermann drive stack first.
#
# Usage (extra args go to course_supervisor.launch.py), e.g.:
#   src/challenge_bringup/launch/obstacle_course_supervisor_jetson.sh
#   src/challenge_bringup/launch/obstacle_course_supervisor_jetson.sh use_wall_follower:=false
#   src/challenge_bringup/launch/obstacle_course_supervisor_jetson.sh start_apriltag:=true use_apriltag_fix:=true

WS="$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)"

open_terminal() {
  gnome-terminal --title="$1" -- bash -ic "source ${WS}/install/setup.bash; $2; exec bash"
}

open_terminal zed_camera "ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i"
sleep 10

open_terminal robot_description "ros2 launch robot_description description.launch.py"
sleep 3

open_terminal zed_base_odom_relay "ros2 run diy_state_estimate zed_base_odom_relay --ros-args \
  -p zed_odom_topic:=/zed/zed_node/odom -p output_topic:=/odom \
  -p odom_frame:=odom -p base_frame:=base_footprint -p publish_tf:=true"
sleep 10

open_terminal hesai_lidar "cd ~ && ./hesai_launch.sh"
sleep 10

open_terminal fast_lio "ros2 launch fast_lio_ros2 lio_localizer.launch.py publish_tf:=false output_topic:=/Odometry_fastlio"
sleep 10

open_terminal mcl_3dl "ros2 launch mcl_3dl mcl_localizer.launch.py \
  cloud_topic:=/cloud_registered_body imu_topic:=/imu/data odom_topic:=/odom \
  use_lidar_gate:=true start_lidar_gate_zones:=false"
sleep 10

open_terminal course_supervisor "ros2 launch course_supervisor course_supervisor.launch.py drive_lidar_gate:=true $*"
