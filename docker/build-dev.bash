#!/usr/bin/env bash
set -Eeuo pipefail

# Change into the parent directory of this script, no matter how it is invoked.
cd "$(dirname "$(readlink -f "$BASH_SOURCE")")/.."

exec docker build -t slocum_glider_dev -f docker/Dockerfile.dev "$@" .

# Local Variables:
# sh-basic-offset: 2
# End:
