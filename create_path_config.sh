#!/usr/bin/env bash

# The function find_devel_directory takes a single argument, the starting directory, and searches for a directory named
# 'devel' in the parent directories.
# It loops through the parent directories using dirname until it finds a directory named 'devel'. If found, it prints
# the realpath of the 'devel' directory and returns. If not found, it prints an error message and exits with a non-zero
# status.
function find_devel_directory {
    local current_dir="$1"
    local devel_dir="devel"

    # Loop until we reach the root directory '/'
    while [ "$current_dir" != "/" ]; do
        # Check if 'devel' directory exists in the current directory
        if [ -d "$current_dir/$devel_dir" ]; then
            # # If 'devel' directory found, print its realpath and exit
            # realpath "$current_dir/$devel_dir"
            # If 'devel' directory found, print the realpath of the parent directory and exit
            realpath "$current_dir"
            return
        fi

        # Move one level up in the directory tree
        current_dir=$(dirname "$current_dir")
    done

    # If 'devel' directory not found, print an error message and exit with non-zero status
    echo "Error: 'devel' directory not found in the parent directories." >&2
    exit 1
}

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR

FILE_PATH=$(realpath "$SCRIPT_DIR/rl_bringup/config/path_config.ini")
DEVEL_DIR=$(realpath "$SCRIPT_DIR/rl_database")

mkdir -p $DEVEL_DIR
cat > "$FILE_PATH" <<EOF
[PATHES]
path_to_venv=$(realpath $SCRIPT_DIR/.venv)
path_to_train_data=$(realpath $DEVEL_DIR)
path_to_eval_data_train=$(realpath $DEVEL_DIR)/evaluation_data/train
path_to_eval_data_test=$(realpath $DEVEL_DIR)/evaluation_data/test
path_to_eval_sets=$(realpath $DEVEL_DIR)/evaluation_data/evaluation_sets
path_to_catkin_ws=$(find_devel_directory $(realpath $SCRIPT_DIR))
path_to_tensorboard_log=$(realpath $DEVEL_DIR)/tensorboard_log_ppo_10
path_to_models=$(realpath $DEVEL_DIR)/agents
ros_version=melodic
EOF

cat > "$DEVEL_DIR/.gitignore" <<EOF
# this is a devel directory created by the create_path_config.sh script
**
EOF
