#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/slocum_glider_workspace/install/setup.bash"

exec "$@"
