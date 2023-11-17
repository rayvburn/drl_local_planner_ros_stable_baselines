'''
    @name:      ros_env_cont_raw_scan_prep_wo.py
    @brief:     This class is a simulation environment wrapper for
                the Polar Representation
                with continuous action space.
    @author:    Ronja Gueldenring
    @version:   3.5
    @date:      2019/04/05
'''

import rospy
# python relevant
import numpy as np
from gym import spaces

# custom classes
from rl_agent.env_wrapper.ros_env_raw_scan_prep_wp import RosEnvRawScanPrepWp

# messages
from geometry_msgs.msg import Twist

# Parameters
GOAL_RADIUS_DEFAULT = 0.4
WAYPOINT_RADIUS_DEFAULT = 0.2

class RosEnvContRawScanPrepWp(RosEnvRawScanPrepWp):
    '''
    This class is a simulation environment wrapper for
    the Polar Representation
    with continuous action space.
    '''
    def __init__(self, ns, state_collector, stack_offset, stack_size, robot_radius = 0.46, reward_fnc=6, debug=False, execution_mode="train", task_mode="static"):
        state_size_t = rospy.get_param("%s/rl_agent/scan_size"% ns)
        state_size = (state_size_t,2, 1)
        observation_space = spaces.Box(low=0, high=6, shape=state_size, dtype=np.float)

        self.v_max_ = rospy.get_param("%s/rl_agent/max_vel_x" % ns, 0.5)
        self.w_max_ = rospy.get_param("%s/rl_agent/max_vel_th" % ns, 0.5)
        action_space = spaces.Box(
            low=np.array([0.0, -self.w_max_]),
            high=np.array([self.v_max_, self.w_max_]),
            dtype=np.float
        )

        goal_radius = rospy.get_param("%s/rl_agent/xy_goal_tolerance" % ns, GOAL_RADIUS_DEFAULT)
        super(RosEnvContRawScanPrepWp, self).__init__(ns, state_collector, execution_mode, task_mode, state_size, observation_space, stack_offset, [], action_space, debug, goal_radius, WAYPOINT_RADIUS_DEFAULT, robot_radius, reward_fnc)

    def get_cmd_vel_(self, action):
        vel_msg = Twist()
        vel_msg.linear.x = action[0]
        vel_msg.angular.z = action[1]
        return vel_msg

    def get_action_list(self):
        action_list = []
        return action_list
