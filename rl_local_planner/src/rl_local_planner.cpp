/*
 * @name	 	rl_local_planner.cpp
 * @brief	 	Connector of move_base and rl_agent using RL library. Forwards action of rl_agent
 * 				to move_base each time step.
 * @author  	Ronja Gueldenring
 * @date 	  	2019/04/05
 **/
#include <pluginlib/class_list_macros.h>
#include <rl_local_planner/rl_local_planner.h>
#include <rl_local_planner/utils.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/MarkerArray.h>

PLUGINLIB_EXPORT_CLASS(rl_local_planner::RLLocalPlanner, nav_core::BaseLocalPlanner)
namespace rl_local_planner {

	RLLocalPlanner::RLLocalPlanner(): nh_(){

	}//RLLocalPlanner

	void RLLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
		costmap_2d::Costmap2DROS* costmap_ros){

		// params
		tf_ = tf;
		costmap_ = costmap_ros;

		// getting params from param server
		nh_.getParam("rl_agent/robot_frame", robot_frame_);
		nh_.getParam("rl_agent/train_mode", rl_mode_);
		nh_.getParam("rl_agent/goal_threshold", goal_threshold_);
		
		std::string action_topic = "rl_agent/action";
		std::string done_topic = "rl_agent/done";
		std::string trigger_agent_topic = "trigger_agent";
		std::string set_path_service_name = ros::this_node::getNamespace() + "/wp_generator/set_gobal_plan";
		std::string markers_topic = "rl_agent/planner_markers";
		nh_.param("rl_agent/action_topic", action_topic, action_topic);
		nh_.param("rl_agent/done_topic", done_topic, done_topic);
		nh_.param("rl_agent/trigger_agent_topic", trigger_agent_topic, trigger_agent_topic);
		nh_.param("rl_agent/wp_generator_path_srv", set_path_service_name, set_path_service_name);
		nh_.param("rl_agent/markers_topic", markers_topic, markers_topic);

		action_topic = adjustTopicName(ros::this_node::getNamespace(), action_topic);
		done_topic = adjustTopicName(ros::this_node::getNamespace(), done_topic);
		trigger_agent_topic = adjustTopicName(ros::this_node::getNamespace(), trigger_agent_topic);
		set_path_service_name = adjustTopicName(ros::this_node::getNamespace(), set_path_service_name);

		// initializing class variables
		if (rl_mode_ == 1)
			goal_threshold_ = 0.0;

		is_action_new_ = false;
		done_ = false;

		// Subscriber
		agent_action_sub_ = nh_.subscribe(action_topic, 1, &RLLocalPlanner::agent_action_callback_, this);
		done_sub_ = nh_.subscribe(done_topic, 1, &RLLocalPlanner::done_callback_, this);
		
		//Publisher
		trigger_agent_pub = nh_.advertise<std_msgs::Bool>(trigger_agent_topic, 1, false);
		marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(markers_topic, 1, false);
		
		// Services
		set_path_service_ = nh_.serviceClient<rl_msgs::SetPath>(set_path_service_name);
	} //initialize

   	bool RLLocalPlanner::isGoalReached(){

		geometry_msgs::TransformStamped transform;
		try {
			transform = tf_->lookupTransform(costmap_->getGlobalFrameID(), path_frame_, ros::Time(0));
		} catch (tf2::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

		// cannot transform tf2::Vector3 directly
		geometry_msgs::Vector3 original_goal_transformed_geom;
		tf2::doTransform(tf2::toMsg(original_goal_), original_goal_transformed_geom, transform);
		tf2::Vector3 original_goal_transformed;
		tf2::fromMsg(original_goal_transformed_geom, original_goal_transformed);

		geometry_msgs::PoseStamped robot_pose;
		if (!costmap_->getRobotPose(robot_pose)) {
			ROS_ERROR("Cannot check whether the goal is reached because could not get a robot pose from the costmap");
			return false;
		}

		tf2::Vector3 goal_global_frame_vector(
			original_goal_transformed.getX() - robot_pose.pose.position.x,
			original_goal_transformed.getY() - robot_pose.pose.position.y,
			original_goal_transformed.getZ() - robot_pose.pose.position.z
		);

		publishMarkers(robot_pose, original_goal_transformed);

		double goal_dist = metric_dist(goal_global_frame_vector.getX(), goal_global_frame_vector.getY());
		if (goal_dist < goal_threshold_) {
			ROS_INFO("RLLocalPlanner reached the goal!");
			return true;
		}

		if(done_){
			ROS_INFO("RLLocalPlanner received 'done' flag while reaching the goal!");
			done_ = false;
			return true;
		}
		
		return false;

  	} //isGoalReached

  	bool RLLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_plan)
  	{
		// goal_threshold_ = goal_threshold;
		if ( orig_plan.size() < 1 ){
			ROS_ERROR_STREAM("RLLocalPlanner: Got an empty plan");
			return false;
		}
		geometry_msgs::PoseStamped original_goal = orig_plan.back();
		original_goal_ = tf2::Vector3(original_goal.pose.position.x,
							original_goal.pose.position.y,
							0.);
		path_frame_ = original_goal.header.frame_id;
		done_ = false;
		rl_msgs::SetPath srv;
		srv.request.path.header.stamp = ros::Time::now();
		srv.request.path.header.frame_id = path_frame_;
		srv.request.path.poses = orig_plan;
		if (!set_path_service_.call(srv)){
			ROS_ERROR("Failed set path on waypoint generator.");
 			return false;
		}
		return true;
  	} //setPlan

	bool RLLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
		// ROS_WARN("Velocity command");
		// Trigger agent to compute next action
		std_msgs::Bool msg;
		msg.data = true;
		trigger_agent_pub.publish(msg);

		//Waiting for agents action
		ros::WallRate r(1000);
		//Periodic logging during the wait
		auto action_wait_begin = ros::WallTime::now();
		bool action_wait_logged = false;
		while(!is_action_new_ && !done_){
			r.sleep();
			// throttled logging do not work well here as in the sim only the WallTime is progressing while we wait
			if (!periodicEventWallTime(action_wait_begin, action_wait_logged, 5.0)) {
				continue;
			}
			ROS_WARN(
				"RLLocalPlanner is waiting for the action. In simulation it may be caused by the stopped ROS time."
			);
		}

		is_action_new_ = false;

		//assigning new action
		if(done_){
			action_.linear.x = 0.0;
			action_.angular.z = 0.0;
		}
		cmd_vel = action_;
		return true;

	}//computeVelocityCommands
	

	double RLLocalPlanner::metric_dist(double x, double y){
		double dist = sqrt(pow(x , 2) + pow(y , 2));
		return dist; 
	}//metric_dist

	void RLLocalPlanner::agent_action_callback_(const geometry_msgs::Twist& cmd_vel){
		action_ = cmd_vel;
		is_action_new_ = true;
	}//agent_action_callback_

	void RLLocalPlanner::done_callback_(const std_msgs::Bool& done){
		done_ = done.data; 
	}//done_callback_

	void RLLocalPlanner::publishMarkers(
		const geometry_msgs::PoseStamped& robot_pose,
		const tf2::Vector3& goal_transformed
	) {
		if (marker_pub_.getNumSubscribers() == 0) {
			return;
		}

		visualization_msgs::MarkerArray markers;

		// goal original
		visualization_msgs::Marker marker;
		marker.header.frame_id = path_frame_;
		marker.header.stamp = ros::Time::now();
		marker.action = visualization_msgs::Marker::ADD;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.ns = "goal original";
		marker.pose.position.x = original_goal_.getX();
		marker.pose.position.y = original_goal_.getY();
		marker.pose.orientation.w = 1.0;
		marker.color.r = 1.0;
		marker.color.a = 1.0;
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		markers.markers.push_back(marker);

		// goal transformed
		marker.ns = "goal transformed";
		marker.header.frame_id = costmap_->getGlobalFrameID();
		marker.pose.position.x = goal_transformed.getX();
		marker.pose.position.y = goal_transformed.getY();
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		markers.markers.push_back(marker);

		// robot pose
		marker.ns = "robot pose";
		marker.header.frame_id = costmap_->getGlobalFrameID();
		marker.pose.position.x = robot_pose.pose.position.x;
		marker.pose.position.y = robot_pose.pose.position.y;
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
		markers.markers.push_back(marker);

		marker_pub_.publish(markers);
	}

	bool RLLocalPlanner::periodicEventWallTime(const ros::WallTime& begin, bool& event_occurred_flag, double period) {
		int period_int = static_cast<int>(period);
		int elapsed_int = static_cast<int>((ros::WallTime::now() - begin).toSec());

		if (elapsed_int == 0) {
			return false;
		}
		if (!event_occurred_flag && elapsed_int % period_int == 0) {
			event_occurred_flag = true;
			return true;
		} else if (event_occurred_flag && elapsed_int % period_int != 0) {
			event_occurred_flag = false;
		}
		return false;
	}

}; // namespace rl_local_planner
