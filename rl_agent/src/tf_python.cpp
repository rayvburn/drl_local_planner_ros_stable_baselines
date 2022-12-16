/**
 * 
 * Ronja Gueldenring
 * This class is needed, because rospy and python3.5 is not compatible regarding 
 * the tf-package.
 * The class provides all transformation needed by rl_agent in python
 * 
**/
#include <rl_agent/tf_python.h>


namespace rl_agent {

	TFPython::TFPython(const ros::NodeHandle& node_handle): nh_(node_handle){
        goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &TFPython::goal_callback, this);
        transformed_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("rl_agent/robot_to_goal", 1, false);
        peds_sub_ = nh_.subscribe("pedsim_simulator/simulated_agents", 1, &TFPython::peds_callback, this);
        transformed_peds_pub_ = nh_.advertise<pedsim_msgs::AgentStates>("rl_agent/peds_to_robot", 1, false);
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
		listener_.lookupTransform("base_footprint", goal_.header.frame_id,  
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
        msg.header.frame_id = "base_footprint";
        tf::poseTFToMsg(rob_to_goal, msg.pose);
        transformed_goal_pub_.publish(msg);
        return;
    }

    /**
	 * 
	 * @brief Publishes transform from agents to the robot for rl-agent
     * 
	 **/ 
    void TFPython::peds_to_robot_transform(){
        if ( peds_.header.frame_id == ""){
            return;
        }
        tf::StampedTransform rob_to_map;
		try{
		listener_.lookupTransform("base_footprint", peds_.header.frame_id,  
								ros::Time(0), rob_to_map);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
        }

        pedsim_msgs::AgentStates msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "base_footprint";
        for (const auto &ped : peds_.agent_states)
        {
            tf::Transform map_to_ped;
            tf::poseMsgToTF(ped.pose, map_to_ped);
            tf::Transform rob_to_ped;
            rob_to_ped = rob_to_map * map_to_ped;
            
            pedsim_msgs::AgentState sub_msg;
            sub_msg.header.stamp = msg.header.stamp;
            sub_msg.header.frame_id = "base_footprint";
            tf::poseTFToMsg(rob_to_ped.inverse(), sub_msg.pose);
            sub_msg.twist.linear.x = std::sqrt(std::pow((ped.twist.linear.x), 2) + pow((ped.twist.linear.y), 2));
            msg.agent_states.push_back(sub_msg);
        }
        transformed_peds_pub_.publish(msg);        
        return;
    }


    void TFPython::goal_callback(const geometry_msgs::PoseStamped& goal){
        goal_ = goal;
    }

    void TFPython::peds_callback(const pedsim_msgs::AgentStates& peds){
        peds_ = peds;
    }


}; // namespace rl_agent

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_python");
  ros::NodeHandle node;
  rl_agent::TFPython tf(node);
  ros::WallRate r(25);    
  while (ros::ok()) {
    tf.robot_to_goal_transform();
    tf.peds_to_robot_transform();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
};