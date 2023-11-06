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
