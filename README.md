# What is this repository for?
* Setup to train a local planner with reinforcement learning approaches from [stable baselines](https://github.com/hill-a/stable-baselines) integrated ROS
* Training in a simulator fusion of [Flatland](https://github.com/avidbots/flatland) and [pedsim_ros](https://github.com/srl-freiburg/pedsim_ros)
* local planner has been trained on static and dynamic obstacles: [video](https://www.youtube.com/watch?v=nHvpO0hVnAg)
* Link to [IROS Paper](http://ras.papercept.net/images/temp/IROS/files/0122.pdf)
* Link to [Master Thesis](https://tams.informatik.uni-hamburg.de/publications/2019/MSc_Ronja_Gueldenring.pdf) for more in depth information.

# Prerequisites
- [docker](https://docs.docker.com/engine/install/ubuntu/)
- [docker-compose](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-compose-on-ubuntu-20-04)
- [nvidia-docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- tmux
- tmuxp

# Installation (Docker only)

Installation is performed via docker-compose. All corresponding files you can find in 'docker' folder.

For installation run the following commands in the terminal:
```
# In terminal 1:

cd <path_to_dir>/drl_local_planner_ros_stable_baselines
./docker/scripts/build.sh

# In terminal 2 (when the build in the 1st terminal is done and hanging):

./docker/scripts/save_build.sh

# Stop Terminal 1
```

# Example usage

1. Train agent

Run the command in the terminal:
```
cd <path_to_dir>/drl_local_planner_ros_stable_baselines
tmuxp load ./docker/scripts/tmuxp/train_dummy_example.yaml

```

2. Execute self-trained ppo-agent
```
cd <path_to_dir>/drl_local_planner_ros_stable_baselines
tmuxp load ./docker/scripts/tmuxp/run_dummy_example.yaml
```

# Training

1. From the scratch. Set up ./docker/scripts/train.yml. Run the command and wait for a very long time:
```
cd <path_to_dir>/drl_local_planner_ros_stable_baselines
./docker/scripts/train.sh
```

2. On pretrained models. It doesn't work for now but it can be fixed. Set up ./docker/scripts/train_pretrained.yml. Run the command and wait for a very long time:
```
cd <path_to_dir>/drl_local_planner_ros_stable_baselines
./docker/scripts/train_pretrained.sh
```

# Run trained models

If you want a good(real) visualization, change the param:

<path_to_dir>/drl_local_planner_ros_stable_baselines/rl_bringup/config/rl_common.yaml:
```
train_mode: 2
```

After running, you can send 2D Nav Goal in RVIZ to create the global path for the robot to follow.

1. 1 raw disc:
```
cd <path_to_dir>/drl_local_planner_ros_stable_baselines
tmuxp load ./docker/scripts/tmuxp/run_1_raw_disc.yaml
```

2. 3 raw disc:
```
cd <path_to_dir>/drl_local_planner_ros_stable_baselines
tmuxp load ./docker/scripts/tmuxp/run_3_raw_disc.yaml
```