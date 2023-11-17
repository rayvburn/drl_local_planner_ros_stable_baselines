#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR

tmp_dir=".docker_train_scratch"
mkdir -p $tmp_dir
echo "**" > $tmp_dir/.gitignore

# Running from scratch works flawlessly
docker run --rm \
    -v $(realpath $tmp_dir):/data \
    -v "$(realpath ../start_scripts)/training_params":"/usr/catkin_ws/src/drl_local_planner_ros_stable_baselines/start_scripts/training_params" \
    -e AGENT_NAME=ppo2_1_raw_data_disc_0 \
    -e NUM_SIM_ENVS=1 \
    ros-drl_local_planner

exit 0
