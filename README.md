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
1. In start_scripts/training_params/ppo2_params, define the new agents training parameters. You can find examples of defining params for training from the scratch (pretrained_model_names field is empty) and for training on pretrained models.

    | Parameter               | Desctiption |
    |-------------------------|--------------|
    | agent_name              |  Number of timestamps the agent will be trained.             |
    | total_timesteps         | Number of timestamps the agent will be trained. |
    | policy |  see [PPO2 Doc](https://stable-baselines.readthedocs.io/en/master/modules/ppo2.html) |
    | gamma |  see [PPO2 Doc](https://stable-baselines.readthedocs.io/en/master/modules/ppo2.html) |
    | n_steps |  see [PPO2 Doc](https://stable-baselines.readthedocs.io/en/master/modules/ppo2.html) |
    | ent_coef |  see [PPO2 Doc](https://stable-baselines.readthedocs.io/en/master/modules/ppo2.html) |
    | learning_rate |  see [PPO2 Doc](https://stable-baselines.readthedocs.io/en/master/modules/ppo2.html) |
    | vf_coef |  see [PPO2 Doc](https://stable-baselines.readthedocs.io/en/master/modules/ppo2.html) |
    | max_grad_norm |  see [PPO2 Doc](https://stable-baselines.readthedocs.io/en/master/modules/ppo2.html) |
    | lam |  see [PPO2 Doc](https://stable-baselines.readthedocs.io/en/master/modules/ppo2.html) |
    | nminibatches |  see [PPO2 Doc](https://stable-baselines.readthedocs.io/en/master/modules/ppo2.html) |
    | noptepochs |  see [PPO2 Doc](https://stable-baselines.readthedocs.io/en/master/modules/ppo2.html) |
    | cliprange |  see [PPO2 Doc](https://stable-baselines.readthedocs.io/en/master/modules/ppo2.html) |
    | robot_radius | The radius if the robot footprint |
    | rew_func | The reward functions that should be used. They can be found and defined in rl_agent/src/rl_agent/env_utils/reward_container.py. |
    | num_stacks | State representation includes the current observation and (num_stacks - 1) previous observation. |
    | stack_offset | The number of timestamps between each stacked observation. |
    | disc_action_space | 0, if continuous action space. 1, if discrete action space. |
    | normalize | 0, if input should not be normalized. 1, if input should be normalized. |
    | stage | stage number of your training. It is supposed to be 0, if you train for the first time. If it is > 0, it loads the agent of the "pretrained_model_path" and continues training. |
    | pretrained_model_name | If stage > 0 this agent will be loaded and training can be continued. |
    | task_mode | - "ped" for training on pedestrians only; "static" for training on static objects only; "ped_static" for training on both, static |

2. In docker/train.yml add the desired agent name and the number of simulations in the row:
```
 ./entrypoint_ppo2.sh agent_name number_of_simulations
```

3. Run the command and wait for a very long time:
```
cd <path_to_dir>/drl_local_planner_ros_stable_baselines
./docker/scripts/train.sh
```

I have got some unexpected errors on that part, therefore, I have created the tmuxp conf in order to check if the model is really training:
```
cd <path_to_dir>/drl_local_planner_ros_stable_baselines
tmuxp load ./docker/scripts/tmuxp/train_from_csv.yaml
```
Just put there all parameters from the csv and the number of simulators should be equal to 1.


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


# Evaluate the model

In order to evaluate the trained agents, we need to create a set of testing episodes (evaluation set). You can do it by your own or just use prepared [set of episodes](https://drive.google.com/file/d/1y55HICHJX0UHzf1S93DATuSkeqNUDSwA/view?usp=share_link). It consists of 205 episodes, so if you will create different number of episodes, the changes in the code is required:
```
no_episodes = 205
``` 

The obtained file should be found in `data/evaluation_data/evaluation_sets` folder

1.  Launch the saving testing set of episodes (not neccesary)
```
cd <path_to_dir>/drl_local_planner_ros_stable_baselines
tmuxp load ./docker/scripts/tmuxp/save_episodes.yaml
```

2. Launch the evaluation script
```
cd <path_to_dir>/drl_local_planner_ros_stable_baselines
tmuxp load ./docker/scripts/tmuxp/evaluation.yaml
```
ps. check number of stack before launching, it should be correlated to the model trained. `num_stacks` in `rl_agent/scripts/evaluate_agent.py:38` 

3. Launch the analysis of the agent(s)
```
cd <path_to_dir>/drl_local_planner_ros_stable_baselines
tmuxp load ./docker/scripts/tmuxp/analysis_agents.yaml
```

In order to check the results of the agents performance:
1. open the browser
2. go to `http://localhost:6006/`
3. see the results of the trained agents by their names


### Additional comments

In order to evaluate new trained agent:
1. change the name of the agent in the list of agent_names in `rl_agent/scripts/evaluate_agent.py:37` and `rl_agent/scripts/analysis.py:95`:
```
agent_names = ["test"]
``` 
