/**
 * 
 * Ronja Gueldenring
 * This class is needed, because rospy and python3.5 is not compatible regarding 
 * the tf-package.
 * The class provides all transformation needed by rl_agent in python
 * 
**/
#include <rl_agent/tf_python.h>
#include <rl_agent/utils.h>

namespace rl_agent {

	TFPython::TFPython(const ros::NodeHandle& node_handle): nh_(node_handle){
        std::string goal_topic = "move_base_simple/goal";
        std::string transformed_goal_topic = "rl_agent/robot_to_goal";
        nh_.param("rl_agent/train/goal_topic", goal_topic, goal_topic);
        nh_.param("rl_agent/train/transformed_goal_topic", transformed_goal_topic, transformed_goal_topic);
        goal_topic = rl_agent::adjustTopicName(ros::this_node::getNamespace(), goal_topic);
        transformed_goal_topic = rl_agent::adjustTopicName(ros::this_node::getNamespace(), transformed_goal_topic);

        goal_sub_ = nh_.subscribe(goal_topic, 1, &TFPython::goal_callback, this);
        transformed_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(transformed_goal_topic, 1, false);

        nh_.getParam("rl_agent/robot_frame", robot_frame_);
    }

	/**
	 * 
	 * @brief Publishes transform from robot to goal for rl-agent
     * 
	 **/ 
    void TFPython::robot_to_goal_transform(){
        if ( goal_.header.frame_id == ""){
            return;
        }
        tf::StampedTransform rob_to_map;
		try{
		listener_.lookupTransform(robot_frame_, goal_.header.frame_id,
								ros::Time(0), rob_to_map);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
        }

        tf::Transform map_to_goal;
        tf::poseMsgToTF(goal_.pose, map_to_goal);

        tf::Transform rob_to_goal;
        rob_to_goal = rob_to_map * map_to_goal;

        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = robot_frame_;
        tf::poseTFToMsg(rob_to_goal, msg.pose);
        transformed_goal_pub_.publish(msg);
        return;
    }


    void TFPython::goal_callback(const geometry_msgs::PoseStamped& goal){
        goal_ = goal;
    }


}; // namespace rl_agent

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_python");
  ros::NodeHandle node;
  rl_agent::TFPython tf(node);
  ros::WallRate r(25);    
  while (ros::ok()) {
    tf.robot_to_goal_transform();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
};