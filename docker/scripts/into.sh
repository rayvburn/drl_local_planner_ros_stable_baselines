#!/bin/bash

docker exec -it drl_local_planner \
    /bin/bash -c "cd ..; source devel/setup.bash; cd src; /bin/bash"
