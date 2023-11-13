#! /bin/bash

path_to_venv=$(awk -F "=" '/path_to_venv/ {print $2}' ../rl_bringup/config/path_config.ini)
path_to_tensorboard_log=$(awk -F "=" '/path_to_tensorboard_log/ {print $2}' ../rl_bringup/config/path_config.ini)

logdir="${1:-pmb2_ppo2_1_raw_data_disc_0_1}"

source $path_to_venv/bin/activate
tensorboard --port 6004 --logdir $path_to_tensorboard_log/$logdir
deactivate
