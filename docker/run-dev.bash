#!/usr/bin/env bash
set -Eeuo pipefail

# Runs a docker container with the image created by build.bash

# Change into the parent directory of this script, no matter how it is invoked.
cd "$(dirname "$(readlink -f "$BASH_SOURCE")")/.."

RUN_ARGS=()

while [[ $# -gt 0 ]]; do
  case $1 in
    --)
      break
      ;;

    *)    # argument to run
      RUN_ARGS+=("$1")
      shift
      ;;
  esac
done

USERID=$(id -u)
GROUPID=$(id -g)

docker run -it \
       -v "$(pwd):/home/ros/slocum_glider_ws/src/slocum_glider" \
       --rm \
       -u "$USERID:$GROUPID" \
       "${RUN_ARGS[@]}" \
       slocum_glider_dev \
       "$@"

# Local Variables:
# sh-basic-offset: 2
# End:
