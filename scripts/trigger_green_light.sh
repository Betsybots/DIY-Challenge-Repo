#!/usr/bin/env bash
# Publish a single green-light message — useful for bench testing without the physical signal.
# Usage:  ./trigger_green_light.sh
echo "Publishing green light signal..."
ros2 topic pub --once /green_light std_msgs/msg/Bool "data: true"
echo "Done. Zone nav should transition INIT → NORMAL_NAV."
