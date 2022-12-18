/**
 * 
 * Ronja Gueldenring
 * his class is needed, because rospy and python3.5 is not compatible regarding 
 * the tf-package.
 * The class provides all transformation needed by rl_agent in python
 * 
**/

#ifndef TF_PYTHON_H
#define TF_PYTHON_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>
#include <cmath>

namespace rl_agent {
  /**
   * @class TFPython
   * @brief this class is needed, because rospy and python3.5 is not compatible regarding 
   * the tf-package.
   * The class provides all transformation needed by rl_agent in python
   *
   */
  class TFPython{
    public:
      TFPython(const ros::NodeHandle& node_handle);
      void robot_to_goal_transform();
      void peds_to_robot_transform();

    private:
      ros::NodeHandle nh_;

      geometry_msgs::PoseStamped goal_;
      ros::Subscriber goal_sub_;
      void goal_callback(const geometry_msgs::PoseStamped& goal);

      ros::Publisher transformed_goal_pub_;

      pedsim_msgs::AgentStates peds_;
      ros::Subscriber peds_sub_;
      void peds_callback(const pedsim_msgs::AgentStates& peds);

      ros::Publisher transformed_peds_pub_;
      tf::TransformListener listener_;

  };
};
#endif /* TF_PYTHON_H */
