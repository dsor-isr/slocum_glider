#!/usr/bin/env bash

containerid=$(docker ps -aqf "ancestor=slocum_glider_dev")
docker exec -it "${containerid}" bash

# Local Variables:
# sh-basic-offset: 2
# End:
