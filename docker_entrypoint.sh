#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/slocum_glider_overlay/setup.bash"
exec "$@"
