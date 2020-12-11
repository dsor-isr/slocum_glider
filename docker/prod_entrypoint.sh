#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/slocum_glider_ws/install/setup.bash"

exec "$@"
