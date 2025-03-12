#ifndef AUTONOMY_UTILS_INCLUDE_LIBS_ROS2_UTILS_H_
#define AUTONOMY_UTILS_INCLUDE_LIBS_ROS2_UTILS_H_

#include <autonomy_utils/lib/ros2_nodes.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>

namespace ros2
{
    /**
     * @brief Get the list of all node names. This function cannot differentiate between a normal node and
     * a managed node
     *
     * @return std::vector<std::string> list of node names
     */
    std::vector<std::string> getNodeNames();

    /**
     * @brief A function that checks if a node is a managed node by looking at the presence of a particular service
     *
     * @param node_name the name of the node
     * @return true if the node exists and it is a managed node
     * @return false if otherwise
     */
    bool isManagedNode(const std::string &node_name);

    /**
     * @brief A template function for fast printing and debug
     *
     * @tparam T ros2 message type
     * @param prefix a message to be printed before the actual message
     * @param msg the message to be printed
     */
    template <typename T>
    void print(const std::string &prefix, const T &msg)
    {
        std::string res = prefix;

        if constexpr (std::is_same_v<T, std_msgs::msg::String>)
        {
            msg += msg.data;
        }
        else if constexpr (std::is_same_v<T, nav_msgs::msg::Odometry>)
        {
            // todo: create a proper string
        }
        // todo: complete for other important messages
    }

}

#endif /*AUTONOMY_UTILS_INCLUDE_LIBS_ROS2_UTILS_H_*/