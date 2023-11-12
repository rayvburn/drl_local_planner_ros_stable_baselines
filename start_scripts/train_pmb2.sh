#!/usr/bin/env bash
agent_id="${1:-pmb2_ppo2_1_raw_data_disc_0}"
num_sims="${2:-1}"

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $SCRIPT_DIR

./entrypoint_ppo2.sh \
    $agent_id \
    $num_sims \
    "./training_params/ppo2_params_pmb2.csv" \
    "./training_params/training_maps.csv" \
    "../flatland_setup/robot/pmb2_train.model.yaml" \
    "../rl_bringup/config/rl_params_common_pmb2.yaml"
