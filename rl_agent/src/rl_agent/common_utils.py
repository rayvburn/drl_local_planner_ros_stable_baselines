# relative names will be modified to be placed within the node namespace
def adjust_topic_name(node_namespace, topic_name):
    if topic_name.startswith("/"):
        # If it starts with "/", do not modify
        return topic_name
    # If it doesn't start with "/", compose the new string
    topic_name_mod = "/" + node_namespace + "/" + topic_name
    # Remove (possible) extra consecutive "/" characters
    return '/' + '/'.join(filter(None, topic_name_mod.split('/')))
