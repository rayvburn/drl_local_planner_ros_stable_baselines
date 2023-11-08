import rospy

# greedy search as rospy.search_param looks only in neighbouring namespaces
def find_ros_param(pattern):
    param_path = None
    params = rospy.get_param_names()
    for param in params:
        if pattern in param:
            param_path = param
            break
    return param_path

# relative names will be modified to be placed within the node namespace
def adjust_topic_name(node_namespace, topic_name):
    if topic_name.startswith("/"):
        # If it starts with "/", do not modify
        return topic_name
    # If it doesn't start with "/", compose the new string
    topic_name_mod = "/" + node_namespace + "/" + topic_name
    # Remove (possible) extra consecutive "/" characters
    return '/' + '/'.join(filter(None, topic_name_mod.split('/')))

def get_ros_param_pattern(pattern, default):
    param_key = find_ros_param(pattern)
    if param_key == None:
        return default
    return rospy.get_param(param_key, default)

def get_ros_param_patterns(param_keys, default, selfunc="max"):
    param_values = []
    for key in param_keys:
        # skip None
        if not key:
            continue
        param_values.append(rospy.get_param(key, default))
    if selfunc == "max":
        param_value = max(param_values)
    else:
        param_value = min(param_values)
    # possibly None
    if not param_value:
        return default
    # valid value
    return param_value

def get_ros_param_footprint_circumradius(default):
    param_keys = [
        find_ros_param("robot_radius"), # costmap_2d's param for circular robots
        find_ros_param("robot_circumradius")
    ]
    return get_ros_param_patterns(param_keys, default)
