#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR

# Running a pretrained
docker run --rm \
    -v "$(realpath ../example_agents)":"/data/agents" \
    -v "$(realpath ../start_scripts)/training_params":"/usr/catkin_ws/src/drl_local_planner_ros_stable_baselines/start_scripts/training_params" \
    -e AGENT_NAME=ppo2_1_raw_data_disc_0_pretrained \
    -e NUM_SIM_ENVS=1 \
    --net=host \
    ros-drl_local_planner

exit 0
