#! /usr/bin/env python
'''
    @name:      analysis.py
    @brief:     Analysis of evaluation data
    @author:    Ronja Gueldenring
    @version:   3.5
    @date:      2019/04/05
'''
import os
home = os.path.expanduser("~")
import rospkg
from rl_agent.evaluation.Analysis_eval import Analysis
import pickle
import configparser
import tensorflow as tf


def analyse(complexity, evaluation_file_path, reward_file_path, save_path, logger):
    analysis = Analysis()
    results = analysis.load_results(evaluation_file_path)
    print("loaded data: %s"%evaluation_file_path)

    timesteps = analysis.get_timestep_list(results)
    # timesteps = analysis.reconstruct_timestep_array(timesteps)
    success, time_exceeded, collision = analysis.get_scores(results)

    print(f'LEN OF SUCCESS: {len(success)}')
    if (complexity == "train"):
        for i in range(len(success)):
            summary = tf.Summary(value=[tf.Summary.Value(tag='success', simple_value=success[i])])
            logger.add_summary(summary, i)
        [timesteps_tb, reward] = analysis.get_reward(reward_file_path)
    else:
        perc_success_drive = analysis.get_percentual_success_drive(results)

        path_ratio = analysis.get_path_length_ratio(results)

        speed = analysis.get_speed(results)

        min_ped_dist = analysis.get_min_ped_dist(results)

        avg_ped_dist = analysis.get_avg_ped_dist(results)

        cum_heading_changes = analysis.get_heading_changes(results, False)

        avg_heading_changes = analysis.get_heading_changes(results, False)

        for i in range(len(success)):
            summary = tf.Summary(value=[tf.Summary.Value(tag='success', simple_value=success[i])])
            logger.add_summary(summary, i)
            summary = tf.Summary(value=[tf.Summary.Value(tag='success_drive_percent', simple_value=perc_success_drive[i])])
            logger.add_summary(summary, i)
            summary = tf.Summary(value=[tf.Summary.Value(tag='path_ratio', simple_value=path_ratio[i])])
            logger.add_summary(summary, i)
            summary = tf.Summary(value=[tf.Summary.Value(tag='speed', simple_value=speed[i])])
            logger.add_summary(summary, i)
            summary = tf.Summary(value=[tf.Summary.Value(tag='min_pedestrian_distance', simple_value=min_ped_dist[i])])
            logger.add_summary(summary, i)
            summary = tf.Summary(value=[tf.Summary.Value(tag='average_pedestrian_distance', simple_value=avg_ped_dist[i])])
            logger.add_summary(summary, i)
            summary = tf.Summary(value=[tf.Summary.Value(tag='cummulative_heading_changes', simple_value=cum_heading_changes[i])])
            logger.add_summary(summary, i)
            summary = tf.Summary(value=[tf.Summary.Value(tag='average_heading_changes', simple_value=avg_heading_changes[i])])
            logger.add_summary(summary, i)

    print("saving results...")

    # Collecting all results in a dict
    analysis_results = {}
    analysis_results["timesteps"] = timesteps
    analysis_results["success"] = success
    analysis_results["time_exceeded"] = time_exceeded
    analysis_results["collision"] = collision

    # Saving results
    if complexity == "train":
        analysis_results["timesteps_tb"] = timesteps_tb
        analysis_results["reward"] = reward
    else:
        analysis_results["perc_success_drive"] = perc_success_drive
        analysis_results["path_ratio"] = path_ratio
        analysis_results["speed"] = speed
        analysis_results["min_ped_dist"] = min_ped_dist
        analysis_results["avg_ped_dist"] = avg_ped_dist
        analysis_results["cum_heading_changes"] = cum_heading_changes
        analysis_results["avg_heading_changes"] = avg_heading_changes

    with open('%s.pickle' % (save_path), 'wb') as handle:
        pickle.dump(analysis_results, handle, protocol=pickle.HIGHEST_PROTOCOL)

if __name__ == '__main__':
    complexity = "complex_map_1"    # train, simple, average, complex, follow_path,
    task_type = "ped"               # static or ped
    no_episodes = 101
    agent_names = ["test"]


    rospack = rospkg.RosPack()
    rl_bringup_path = rospack.get_path('rl_bringup')
    config = configparser.ConfigParser()
    config.read('%s/config/path_config.ini' % rl_bringup_path)
    path_to_eval_data_train = config['PATHES']['path_to_eval_data_train']
    path_to_eval_data_test = config['PATHES']['path_to_eval_data_test']
    path_to_tensorboard_log = config['PATHES']['path_to_tensorboard_log_eval']

    analysis_path = os.path.join(path_to_tensorboard_log, f'analysis_{agent_names[0]}')
    analysis_logger = tf.summary.FileWriter(analysis_path)



    for agent_name in agent_names:
        if complexity == "train":
            evaluation_file_path = "%s/%s_training"%(path_to_eval_data_train, agent_name)
        elif complexity == "follow_path":
            evaluation_file_path = "%s/%s_following_path"%(path_to_eval_data_test, agent_name)
        else:
            evaluation_file_path = "%s/%s_%s_eval_set_%s_%d" % (path_to_eval_data_test, agent_name, task_type, complexity, no_episodes)

        reward_file_path = "%s/run_%s_reward" % (path_to_eval_data_test, agent_name)

        # Saving results
        if complexity == "train":
            save_path = "%s/%s_analysis" % (path_to_eval_data_train, agent_name)
        elif complexity == "follow_path":
            save_path = "%s/%s_analysis_follow_path" % (path_to_eval_data_test, agent_name)
        else:
            save_path = "%s/%s_%s_analysis_eval_set_%s_%s" % (
                path_to_eval_data_test, agent_name, task_type, complexity, no_episodes)

        analyse(complexity, evaluation_file_path, reward_file_path, save_path, analysis_logger)

