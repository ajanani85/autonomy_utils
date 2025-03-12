#include <autonomy_utils/lib/ros2_utils.hpp>

namespace ros2
{
    /**
     * @brief Get the names of all the nodes running under ros2.
     * this function cannot differentiate between a lifecycle managed and a normal node
     *
     * @return std::vector<std::string> of all the running nodes
     */
    std::vector<std::string> getNodeNames()
    {
        return ros2::Node::node->get_node_names();
    }

    /**
     * @brief A function that checks if a given node name is lifecycle managed or not.
     * This function queries the available services of the specified node
     * and checks if it provides lifecycle management services such as
     * "lifecycle_msgs/srv/ChangeState".
     *
     * @param node_name name of the node to check
     * @return true If the node is lifecycle-managed
     * @return false If the node is a regular node
     */
    bool isManagedNode(const std::string &node_name)
    {
        std::map<std::string, std::vector<std::string>> services = ros2::Node::node->get_service_names_and_types_by_node(node_name, "");

        for (std::pair<const std::string, std::vector<std::string>> &service : services)
        {
            for (const std::string &type : service.second)
            {
                if (type.find("lifecycle_msgs/srv/ChangeState") != std::string::npos)
                {
                    return true;
                }
            }
        }

        return false;
    }

}