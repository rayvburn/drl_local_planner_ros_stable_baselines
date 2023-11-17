#!/usr/bin/env bash
agent_id="ppo2_1_raw_data_disc_0_pretrained"
num_sims="1"

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $SCRIPT_DIR

## Section below adapts the PMB2 robot and training configuration to allow training on a pretrained policy

# 1. Adjust the dimensions of the robot in the policy params
policy_params_orig_file="./training_params/ppo2_params.csv"
policy_params_mod_file="./training_params/ppo2_params_pmb2_pretrained.csv"
cp -rf $policy_params_orig_file $policy_params_mod_file
# Using sed to replace the specified phrase (robot radius and rew value to find only unique pairs)
sed -i 's/,0\.5,19/,0.275,19/g' $policy_params_mod_file
sed -i 's/,0\.46,19/,0.275,19/g' $policy_params_mod_file

# 2. Change the map that will be used first for simulation
training_maps_orig_file="./training_params/training_maps.csv"
training_maps_mod_file="./training_params/training_maps_pmb2_pretrained.csv"
cp -rf $training_maps_orig_file $training_maps_mod_file
# Using sed to insert 'complex_map_4' at the beginning of the file
sed -i '1i\complex_map_4' $training_maps_mod_file

# 3. Change the configuration of laser sensors
robot_model_orig_file="../flatland_setup/robot/pmb2_train.model.yaml"
robot_model_mod_file="../flatland_setup/robot/pmb2_train_pretrained.model.yaml"
cp -rf $robot_model_orig_file $robot_model_mod_file
# Delete lines containing "# Scan metadata from the ROS topic"
sed -i '/# Scan metadata from the ROS topic/d' $robot_model_mod_file
# Delete lines containing "# angle: "
sed -i '/# angle: /d' $robot_model_mod_file
# Replace characters after "angle: " with "{min: -3.14159265359, max: 3.14159265359, increment: 0.06981317007}" maintaining indentation
sed -i -E 's/^([[:space:]]*)angle:.*/\1angle: {min: -3.14159265359, max: 3.14159265359, increment: 0.06981317007}/' $robot_model_mod_file

# 4. Change the laser representation to fit the policy given in agent_id
params_common_orig_file="../rl_bringup/config/rl_params_common_pmb2.yaml"
params_common_mod_file="../rl_bringup/config/rl_params_common_pmb2_pretrained.yaml"
cp -rf $params_common_orig_file $params_common_mod_file
# Using sed to replace the line with 'scan_size: '
sed -i '/scan_size:/c\scan_size: 90' $params_common_mod_file
# Define the YAML content to be appended
yaml_content="\
\n\
execution:\n\
  robot_radius: 0.275"
# Append YAML content to the file
sed -i -e "\$a\\$yaml_content" $params_common_mod_file

# Start training
./entrypoint_ppo2.sh \
    $agent_id \
    $num_sims \
    $policy_params_mod_file \
    $training_maps_mod_file \
    $robot_model_mod_file \
    $params_common_mod_file
