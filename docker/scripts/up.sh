#!/bin/bash

cd "$(dirname "$0")"
cd ../../..
export PRODUCTION_PATH=$PWD
export ARCH=`uname -m`
export NUM_THREADS=`nproc`

docker-compose --env-file $PRODUCTION_PATH/drl_local_planner_ros_stable_baselines/docker/up.env \
    -f $PRODUCTION_PATH/drl_local_planner_ros_stable_baselines/docker/up.yml \
    up $@
