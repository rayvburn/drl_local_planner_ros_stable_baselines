#! /bin/sh

LOGPREFIX="[ros.sh]"

sim_name=$1
policy=$2
map=$3

# Set default values for optional arguments if not provided
robot_model_path="${4:-../flatland_setup/robot/robot1.model.yaml}"
robot_model_path=$(realpath $robot_model_path)
rl_params_path="${5:-../rl_bringup/config/rl_params_common.yaml}"
rl_params_path=$(realpath $rl_params_path)

#get config-params
path_to_catkin_ws=$(awk -F "=" '/path_to_catkin_ws/ {print $2}' ../rl_bringup/config/path_config.ini)
ros_ver=$(awk -F "=" '/ros_version/ {print $2}' ../rl_bringup/config/path_config.ini)

# source ros stuff
source /opt/ros/$ros_ver/setup.bash
source $path_to_catkin_ws/devel/setup.bash

map_path=$path_to_catkin_ws/src/drl_local_planner_ros_stable_baselines/flatland_setup/maps/$map

echo "$LOGPREFIX Starting with the following config:"
echo "$LOGPREFIX * sim_name             $sim_name"
echo "$LOGPREFIX * policy               $policy"
echo "$LOGPREFIX * map                  $map"
echo "$LOGPREFIX * map_path             $map_path"
echo "$LOGPREFIX * robot_model_path     $robot_model_path"
echo "$LOGPREFIX * rl_params_path       $rl_params_path"

# NOTE: spawn pose does not matter in train mode as subsequent poses are randomized anyway
if [ "$policy" = "CnnPolicy_multi_input_vel" ] || [ "$policy" = "CnnPolicy_multi_input_vel2" ] || [ "$policy" = "CnnPolicy" ];
then
    echo "$LOGPREFIX * state_representation image"
    echo "$LOGPREFIX Got a valid policy!"
    roslaunch rl_bringup setup_sim.launch \
        ns:="$sim_name" \
        mode:="train" \
        state_representation:="image" \
        map_path:="$map_path" \
        robot_model_path:="$robot_model_path" \
        params_file_common:="$rl_params_path"
fi

if [ "$policy" = "CNN1DPolicy" ] || [ "$policy" = "CNN1DPolicy_multi_input" ];
then
    echo "$LOGPREFIX * state_representation scan"
    echo "$LOGPREFIX Got a valid policy!"
    roslaunch rl_bringup setup_sim.launch \
        ns:="$sim_name" \
        mode:="train" \
        state_representation:="scan" \
        map_path:="$map_path" \
        robot_model_path:="$robot_model_path" \
        params_file_common:="$rl_params_path"
fi

