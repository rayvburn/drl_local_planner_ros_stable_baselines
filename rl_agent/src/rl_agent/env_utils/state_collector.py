'''
    @name:      state_collector.py
    @brief:     This class collects most recent relevant state data of the environment of the RL-agent.
    @author:    Ronja Gueldenring
    @version:   3.5
    @date:      2019/04/05
'''
# python relevant
import numpy as np
import copy

# ros-relevant
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from rl_msgs.msg import Waypoint
from rl_msgs.srv import StateImageGenerationSrv
from geometry_msgs.msg import TwistStamped, PoseStamped, Pose
from std_srvs.srv import SetBool
from rl_msgs.srv import MergeScans
import time

class StateCollector():
    """
    This class collects most recent relevant state data of the environment of the RL-agent.
    """
    def __init__(self, ns, train_mode):
        # Class variables
        self.__mode = train_mode                      # Mode of RL-agent (Training or Executrion ?)
        self.__ns = ns                          # namespace of simulation environment
        self.__is__new = False                  # True, if waypoint reached
        self.__static_scan = LaserScan()        # Laserscan only contains static objects
        self.__ped_scan = LaserScan()           # Laserscan only contains pedestrians
        self.__f_scan = LaserScan()
        self.__b_scan = LaserScan()

        self.__img = OccupancyGrid()            # input image
        self.__wps= Waypoint()                  # most recent Waypoints
        self.__twist = TwistStamped()           # most recent velocity
        self.__goal = Pose()                    # most recent goal position in robot frame
        self.__state_mode = 2                   # 0, if image as input state representation
                                                # 1, if stacked image representation in same frame
                                                # 2, if scan as input state representation

        self.__robot_base_frame = rospy.get_param("%s/rl_agent/robot_frame" % (self.__ns), "base_footprint")
        self.__f_scan.header.frame_id = self.__robot_base_frame
        self.__b_scan.header.frame_id = self.__robot_base_frame

        # prepare list of topics to subscribe (default values taken from the original implementation)
        waypoint_topic = rospy.get_param("%s/rl_agent/waypoint_topic" % (self.__ns), "%s/wp" % (self.__ns))
        waypoint_reached_topic = rospy.get_param("%s/rl_agent/waypoint_reached_topic" % (self.__ns), "%s/wp_reached" % (self.__ns))
        scan_static_obs_topic = rospy.get_param("%s/rl_agent/train/laserscan_static_obstacles_topic" % (self.__ns), "%s/static_laser" % (self.__ns))
        scan_pedestrians_topic = rospy.get_param("%s/rl_agent/train/laserscan_pedestrians_topic" % (self.__ns), "%s/ped_laser" % (self.__ns))
        twist_topic = rospy.get_param("%s/rl_agent/train/twist_topic" % (self.__ns), "%s/twist" % (self.__ns))
        robot_to_goal_topic = rospy.get_param("%s/rl_agent/transformed_goal_topic" % (self.__ns), "%s/rl_agent/robot_to_goal" % (self.__ns))
        scan_rear_topic = rospy.get_param("%s/rl_agent/execution/scan_rear_topic" % (self.__ns), "%s/b_scan" % (self.__ns))
        scan_front_topic = rospy.get_param("%s/rl_agent/execution/scan_front_topic" % (self.__ns), "%s/f_scan" % (self.__ns))

        # Subscriber
        self.wp_sub_ = rospy.Subscriber(waypoint_topic, Waypoint, self.__wp_callback, queue_size=1)

        if ["train", "eval"].__contains__(self.__mode):
            # Info only avaible during evaluation and training
            self.wp_sub_reached_ = rospy.Subscriber(waypoint_reached_topic, Waypoint, self.__wp_reached_callback, queue_size=1)

            self.static_scan_sub_ = rospy.Subscriber(scan_static_obs_topic, LaserScan, self.__static_scan_callback,
                                                     queue_size=1)

            self.ped_scan_sub_ = rospy.Subscriber(scan_pedestrians_topic, LaserScan, self.__ped_scan_callback,
                                                  queue_size=1)
            self.twist_sub_ = rospy.Subscriber(twist_topic, TwistStamped, self.__twist_callback, queue_size=1)
            self.goal_sub_ = rospy.Subscriber(robot_to_goal_topic, PoseStamped, self.__goal_callback, queue_size=1)
        else:
            self.static_rear_scan_sub_ = rospy.Subscriber(scan_rear_topic, LaserScan,
                                                     self.__b_scan_callback,
                                                     queue_size=1)
            self.static_front_scan_sub_ = rospy.Subscriber(scan_front_topic, LaserScan,
                                                     self.__f_scan_callback,
                                                     queue_size=1)

        # Service
        image_srv_name = rospy.get_param("%s/rl_agent/state_image_generation_srv_name" % (self.__ns), '%s/image_generator/get_image' % (self.__ns))
        merge_scans_srv_name = rospy.get_param("%s/rl_agent/merge_scans_srv_name" % (self.__ns), '%s/merge_scans' % (self.__ns))

        self.__img_srv = rospy.ServiceProxy(image_srv_name, StateImageGenerationSrv)
        self.__merge_scans = rospy.ServiceProxy(merge_scans_srv_name, MergeScans)

        # (legacy) See the "Fully synchronized" note in the get_state
        # self.__sim_in_step = rospy.ServiceProxy('%s/is_in_step' % (self.__ns), SetBool)

    def get_state(self):
        """
        Provides the most recent state with laserscan data, input image, waypoints, robots velocity, goal position
        :returns    laser scan of static objects, ([] in execution mode)
                    laserscan of pedestrians, ([] in execution mode)
                    state image,                ([] if raw data state mode)
                    next waypoints,
                    twist of the robot,
                    goal position in robot frame
        """
        if ["train", "eval"].__contains__(self.__mode):

            # Fully synchronized --> slows down simulation speed!
            # resp = self.__sim_in_step(True)
            # while(resp.success):
            #     time.sleep(0.0001)
            #     resp = self.__sim_in_step(True)
            # while self.__ped_scan.header.stamp.to_sec() + 0.11 < float(resp.message):
            #     time.sleep(0.005)
            # while self.__static_scan.header.stamp.to_sec() + 0.11 < float(resp.message):
            #     time.sleep(0.005)

            # start = time.time()
            static_scan_msg = self.__remove_nans_from_scan(self.__static_scan)
            ped_scan_msg = self.__remove_nans_from_scan(self.__ped_scan)
            # print("__remove_nans_from_scan: %f" % (time.time() - start))

            # start = time.time()
            merged_scan = LaserScan()
            merged_scan.header.frame_id = self.__robot_base_frame
            merged_scan.header.stamp = static_scan_msg.header.stamp
            merged_scan.ranges = np.minimum(static_scan_msg.ranges, ped_scan_msg.ranges)
            merged_scan.range_max = static_scan_msg.range_max
            merged_scan.range_min = static_scan_msg.range_min
            merged_scan.angle_increment = static_scan_msg.angle_increment
            merged_scan.angle_min = static_scan_msg.angle_min
            merged_scan.angle_max = static_scan_msg.angle_max
            # print("merge_scan: %f" % (time.time() - start))
            # start = time.time()
            wp_cp = copy.deepcopy(self.__wps)
            self.__wps.is_new.data = False
            # print("deep_copy: %f" % (time.time() - start))

            start = time.time()
            if (self.__state_mode == 0):
                # ToDo: Service call takes very long. Find more efficient solution!
                resp = self.__img_srv(merged_scan, wp_cp)
                self.__img = resp.img
                # print("img service call: %f" % (time.time() - start))
            else:
                self.__img = []
            if not len(merged_scan.ranges):
                rospy.logerr(
                    "State collector is in the '"
                    + str(self.__mode)
                    + "' mode and will return an ill-formed merged scan with "
                    + str(len(merged_scan.ranges))
                    + " elements. It is created from a 'static' scan ('"
                    + str(self.static_scan_sub_.resolved_name)
                    + "' topic) with "
                    + str(len(static_scan_msg.ranges))
                    + " elements and a 'ped' scan ('"
                    + str(self.ped_scan_sub_.resolved_name)
                    + "' topic) with "
                    + str(len(ped_scan_msg.ranges))
                    + " elements"
                )

            if not len(self.__wps.points):
                rospy.logerr(
                    "State collector is in the '"
                    + str(self.__mode)
                    + "' mode and will return an ill-formed waypoints list with "
                    + str(len(self.__wps.points))
                    + " elements received from '"
                    + str(self.wp_sub_.resolved_name)
                    + "' and '"
                    + str(+ str(self.wp_sub_reached_.resolved_name))
                    + "' topics"
                )
            return static_scan_msg, ped_scan_msg, merged_scan, self.__img, wp_cp, self.__twist, self.__goal
        else:
            scans = []
            scans.append(self.__f_scan)
            scans.append(self.__b_scan)
            resp = self.__merge_scans(scans)
            merged_scan = resp.merged_scan
            if not len(merged_scan.ranges):
                rospy.logerr(
                    "State collector is in the '"
                    + str(self.__mode)
                    + "' mode and will return an ill-formed merged scan with "
                    + str(len(merged_scan.ranges))
                    + " elements. It is created from a 'front' scan ('"
                    + str(self.static_front_scan_sub_.resolved_name)
                    + "' topic) with "
                    + str(len(self.__f_scan.ranges))
                    + " elements and a 'rear' scan ('"
                    + str(self.static_rear_scan_sub_.resolved_name)
                    + "' topic) with "
                    + str(len(self.__b_scan.ranges))
                    + " elements"
                )
            wp_cp = copy.deepcopy(self.__wps)
            self.__wps.is_new.data = False
            if (self.__state_mode == 0):
                resp = self.__img_srv(merged_scan, wp_cp)
                self.__img = resp.img
            else:
                self.__img = []

            if not len(self.__wps.points):
                rospy.logerr(
                    "State collector is in the '"
                    + str(self.__mode)
                    + "' mode and will return an ill-formed waypoints list with "
                    + str(len(self.__wps.points))
                    + " elements received from '"
                    + str(self.wp_sub_.resolved_name)
                    + "' topic"
                )
            return [], [], merged_scan, self.__img, wp_cp, self.__twist, self.__goal

    def get_static_scan(self):
        """
        Provides the most recent laser scan of static objects
        :return laser scan
        """
        scan_msg = self.__remove_nans_from_scan(self.__static_scan)
        return scan_msg

    def set_state_mode(self, state_mode):
        self.__state_mode = state_mode

    def __remove_nans_from_scan(self, scan_msg):
        """
        Replaces nan-values with maximum distance of scanner.
        :param scan_msg scan message, where nan-values need to be removed.
        """
        scan = np.array(scan_msg.ranges)
        scan[np.isnan(scan)] = scan_msg.range_max
        scan_msg.ranges = scan
        return scan_msg

    def __static_scan_callback(self, data):
        self.__static_scan = data

    def __ped_scan_callback(self, data):
        self.__ped_scan = data

    def __f_scan_callback(self, data):
        self.__f_scan = data

    def __b_scan_callback(self, data):
        self.__b_scan = data

    def __wp_callback(self, data):
        if not self.__wps.is_new.data:
            self.__wps = data

    def __twist_callback(self, data):
        self.__twist = data

    def __goal_callback(self, data):
        self.__goal = data.pose

    def __is_wp_reached_callback(self, data):
        self.__is__new = data.data

    def __wp_reached_callback(self, data):
        self.__wps = data