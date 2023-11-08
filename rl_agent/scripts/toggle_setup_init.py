#!/usr/bin/env python
# 
# @name	 	  toggle_setup_init.cpp
# @brief	 	  Simulation will be triggered for n_sec for initialize setup.
# @details   Then, a customized move_base (https://github.com/rayvburn/navigation_flatland) is expected to orchestrate
#            simulation steps.
# @author  	Ronja Gueldenring
# @date 		  2019/04/05
#
# @rayvburn - added a Python version of `toggle_setup_init` node from `rl_bringup` (originally written in C++)
# Aims to avoid simulator dependencies during pkg compilation (to flawlessly compile on a real robot)

import rospy
import time

from flatland_msgs.srv import Step, StepRequest
from rl_agent.common_utils import adjust_topic_name

if __name__ == '__main__':
  rospy.init_node("toggle_setup_init", anonymous=True)
  train_mode_param = rospy.get_namespace() + "/rl_agent/train_mode"
  train_mode_param = adjust_topic_name(rospy.get_namespace(), train_mode_param)

  rl_mode = rospy.get_param(train_mode_param, 0)

  keep_clock_running = False
  if rl_mode == 2:
    keep_clock_running = True

  n_sec = 10.0
  step_sim_srv_name = rospy.get_namespace() + "/step"
  step_sim_srv_name = adjust_topic_name(rospy.get_namespace(), step_sim_srv_name)
  step_simulation_ = rospy.ServiceProxy(step_sim_srv_name, Step)
  step_simulation_.wait_for_service(30.0)

  msg = StepRequest()
  msg.step_time.data = 0.1
  begin = rospy.rostime.get_rostime()
  rate = rospy.Rate(10)

  # this small sleep is necessary to start rolling (otherwise rate sleeps forever)
  time.sleep(0.1)

  while (
    not rospy.is_shutdown()
    and ((rospy.rostime.get_rostime() - begin).to_sec() < n_sec or keep_clock_running)):
      try:
        if not step_simulation_.call(msg):
          rospy.logerr("Failed to call step_simulation_ service from the `toggle_setup_init` node")

        # rate.sleep()
        # There are some strange issues with rate.sleep(); when the service is called properly (returns true),
        # but the execution of the iteration consists only of the service call, the sleep blocks forever.
        # On the other hand, when there is some rospy.loginfo before the rate.sleep(), all works properly.
        # The call below acts as a workaround for the call above.
        time.sleep(0.03)
      except rospy.ROSInterruptException:
        print("Interrupt exception occurred in `toggle_setup_init.py` node")
