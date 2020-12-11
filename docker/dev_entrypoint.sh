#!/bin/bash
set -e

eval $( fixuid )

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$HOME/slocum_glider_ws/devel/setup.bash"

exec "$@"
