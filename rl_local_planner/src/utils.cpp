#include <rl_local_planner/utils.h>

namespace rl_local_planner {

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

};
