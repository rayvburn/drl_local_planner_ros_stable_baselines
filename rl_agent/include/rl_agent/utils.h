#pragma once

#include <string>

namespace rl_agent {

// relative names will be modified to be placed within the node namespace
std::string adjustTopicName(const std::string& node_namespace, const std::string& name);

}
