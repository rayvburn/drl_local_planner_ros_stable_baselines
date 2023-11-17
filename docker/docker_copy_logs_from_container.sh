#!/usr/bin/env bash
#
# Handy script to retrieve logs from a container that can be terminated in any time
#

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR

cp_dir=$(realpath "${1:-.docker_cp_temp}")

# Check if the directory exists
if [ -d "$cp_dir" ]; then
    # Ask for user confirmation
    read -p "Directory exists. Do you want to proceed? (y/n): " response

    # Check the user's response
    if [ "$response" = "y" ]; then
        echo "Proceeding..."
        # Add your logic here to perform actions if the user confirms
    else
        echo "Aborted by user."
        exit 0
    fi
fi

mkdir -p $cp_dir
echo "**" > $cp_dir/.gitignore

get_container_name() {
    docker ps --format "{{.Names}}" --filter ancestor=ros-drl_local_planner | head -n 1
}

container_name=$(get_container_name)
path_to_copy="/usr/catkin_ws/src/drl_local_planner_ros_stable_baselines/start_scripts/"

while [ -n "$container_name" ]
do
    # Perform your actions here
    docker cp ${container_name}:${path_to_copy} $cp_dir

    # Update container_name
    container_name=$(get_container_name)
done

echo "No running containers matching the patttern"
exit 0
