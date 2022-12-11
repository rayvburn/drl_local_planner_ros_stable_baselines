#!/bin/bash

docker exec -it drl_local_planner \
    /bin/bash -c "source catkin_ws/devel/setup.bash; /bin/bash"
