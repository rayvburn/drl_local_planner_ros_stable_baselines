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
cd <path_to_dir>/drl_local_planner_ros_stable_baselines
./docker/scripts/build.sh
```

# Example usage

1. Train agent

Run the command in the terminal:
```
cd <path_to_dir>/drl_local_planner_ros_stable_baselines
tmuxp load ./docker/scripts/tmuxp/train.yaml
```

2. Execute self-trained ppo-agent
#### TODO
