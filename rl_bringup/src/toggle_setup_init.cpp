 /*
 * @name	 	  toggle_setup_init.cpp
 * @brief	 	  Simulation will be triggered for n_sec for initialize setup.
 * @details   Then, a customized move_base (https://github.com/rayvburn/navigation_flatland) is expected to orchestrate
 *            simulation steps.
 * @author  	Ronja Gueldenring
 * @date 		  2019/04/05
 **/
#include <ros/ros.h>
#include <flatland_msgs/Step.h>

// relative names will be modified to be placed within the node namespace
std::string adjustTopicName(const std::string& node_namespace, const std::string& name);

int main(int argc, char** argv){
    
  ros::init(argc, argv, "map_echo");
  ros::NodeHandle node;

  std::string train_mode_param = ros::this_node::getNamespace() + "/rl_agent/train_mode";
  int rl_mode;
  train_mode_param = adjustTopicName(ros::this_node::getNamespace(), train_mode_param);
  node.getParam(train_mode_param, rl_mode);

  bool keep_clock_running = false;
  if(rl_mode == 2){
    keep_clock_running = true;
  }
  
  float n_sec = 10.0;
  auto step_sim_srv_name = ros::this_node::getNamespace() + "/step";
  step_sim_srv_name = adjustTopicName(ros::this_node::getNamespace(), step_sim_srv_name);
  ros::ServiceClient step_simulation_ = node.serviceClient<flatland_msgs::Step>(step_sim_srv_name);

  flatland_msgs::Step msg;
  msg.request.step_time.data = 0.1;
  ros::WallTime begin = ros::WallTime::now();
  ros::WallRate r(30);
  while ((ros::WallTime::now() - begin).toSec() < n_sec || keep_clock_running) {
      if(!step_simulation_.call(msg)){
        ROS_ERROR("Failed to call step_simulation_ service");
      }
    r.sleep();
  }

  return 0;
};

std::string adjustTopicName(const std::string& node_namespace, const std::string& name) {
    if (name.length() && name.front() != '/') {
        auto name_full = "/" + node_namespace + "/" + name;
        // keep only 1 slash at the start
        std::string name_clean = "/";
        bool letter_encountered = false;
        for (const char& c : name_full) {
            if (c == '/') {
                // trim front slashes
                if (!letter_encountered) {
                    continue;
                }
                name_clean += c;
            } else {
                name_clean += c;
                letter_encountered = true;
            }
        }
        return name_clean;
    }
    return name;
};
