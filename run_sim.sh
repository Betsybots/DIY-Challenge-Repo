#!/bin/bash
# Wrapper script to launch sim without strict unbound variable checking
set +u
cd "$(dirname "${BASH_SOURCE[0]}")" || exit
source install/setup.bash
set -u
ros2 launch challenge_bringup sim_launch.py "$@"
