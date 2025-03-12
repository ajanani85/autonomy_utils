#ifndef AUTONOMY_UTILS_INCLUDE_LIBS_ROS2_NODES_H_
#define AUTONOMY_UTILS_INCLUDE_LIBS_ROS2_NODES_H_


#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace ros2
{

    /*
    Configure	ros2 lifecycle set my_lifecycle_node configure	Moves from unconfigured → inactive
    Activate	ros2 lifecycle set my_lifecycle_node activate	Moves from inactive → active
    Deactivate	ros2 lifecycle set my_lifecycle_node deactivate	Moves from active → inactive
    Cleanup	ros2 lifecycle set my_lifecycle_node cleanup	Moves from inactive → unconfigured
    Shutdown	ros2 lifecycle set my_lifecycle_node shutdown	Moves to final shutdown state
    error


    */

    class LifeCycle : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        explicit LifeCycle(const std::string &node_name);
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State &state) override;

        // Register external classes' on_configure() callbacks
        void register_on_configure_callback(std::function<void()> callback);

        // Register external classes' on_activate() callbacks
        void register_on_activate_callback(std::function<void()> callback);

        // Register external classes' on_deactivate() callbacks
        void register_on_deactivate_callback(std::function<void()> callback);

        // Register external classes' on_cleanup() callbacks
        void register_on_cleanup_callback(std::function<void()> callback);

        // Register external classes' on_shutdown() callbacks
        void register_on_shutdown_callback(std::function<void()> callback);

        // Register external classes' on_error() callbacks
        void register_on_error_callback(std::function<void()> callback);

    private:
        std::vector<std::function<void()>> configure_callbacks_;
        std::vector<std::function<void()>> activate_callbacks_;
        std::vector<std::function<void()>> deactivate_callbacks_;
        std::vector<std::function<void()>> cleanup_callbacks_;
        std::vector<std::function<void()>> shutdown_callbacks_;
        std::vector<std::function<void()>> error_callbacks_;
    };

    class Node
    {
    public:
        Node()
        {
        }

        virtual ~Node()
        {
        }

        static std::shared_ptr<ros2::LifeCycle> node;
    };

    rclcpp::QoS getSensorQoS(int q_size = 5);

    // toDuration() can be used whenever a float or double has to be converted to std::chrono::duration datatype
    // This function is needed when you are create a timer
    template <typename T>
    std::chrono::duration<T, std::ratio<1>> toDuration(T seconds)
    {
        return std::chrono::duration<T, std::ratio<1>>(seconds);
    }

}

// inline rclcpp::Time ClockNow() {return ros2::Node::node->now();};
inline rclcpp::Time ClockNow() { return ros2::Node::node->get_clock()->now(); };
inline rclcpp::Time clockNow() { return ros2::Node::node->get_clock()->now(); };
inline rclcpp::Clock::SharedPtr getClock() { return ros2::Node::node->get_clock(); };

// Defines to convert ROS1 outputs to ROS2
#define ROS_INFO(...) RCLCPP_INFO(ros2::Node::node->get_logger(), __VA_ARGS__)
#define ROS_INFO_THROTTLE(duration, ...) RCLCPP_INFO_THROTTLE(ros2::Node::node->get_logger(), *ros2::Node::node->get_clock(), (long)((duration) * 1000.0), __VA_ARGS__)
#define ROS_INFO_STREAM(...) RCLCPP_INFO_STREAM(ros2::Node::node->get_logger(), __VA_ARGS__)
#define ROS_INFO_STREAM_THROTTLE(duration, ...) RCLCPP_INFO_STREAM_THROTTLE(ros2::Node::node->get_logger(), *ros2::Node::node->get_clock(), (long)((duration) * 1000.0), __VA_ARGS__)

#define ROS_INFO_COND(...) RCLCPP_INFO_EXPRESSION(ros2::Node::node->get_logger(), __VA_ARGS__)
#define ROS_INFO_STREAM_COND(...) RCLCPP_INFO_STREAM_EXPRESSION(ros2::Node::node->get_logger(), __VA_ARGS__)

#define ROS_WARN(...) RCLCPP_WARN(ros2::Node::node->get_logger(), __VA_ARGS__)
#define ROS_WARN_THROTTLE(duration, ...) RCLCPP_WARN_THROTTLE(ros2::Node::node->get_logger(), *ros2::Node::node->get_clock(), (long)((duration) * 1000.0), __VA_ARGS__)
#define ROS_WARN_STREAM(...) RCLCPP_WARN_STREAM(ros2::Node::node->get_logger(), __VA_ARGS__)
#define ROS_WARN_STREAM_THROTTLE(duration, ...) RCLCPP_WARN_STREAM_THROTTLE(ros2::Node::node->get_logger(), *ros2::Node::node->get_clock(), (long)((duration) * 1000.0), __VA_ARGS__)

#define ROS_ERROR(...) RCLCPP_ERROR(ros2::Node::node->get_logger(), __VA_ARGS__)
#define ROS_ERROR_STREAM(...) RCLCPP_ERROR_STREAM(ros2::Node::node->get_logger(), __VA_ARGS__)
#define ROS_ERROR_THROTTLE(duration, ...) RCLCPP_ERROR_THROTTLE(ros2::Node::node->get_logger(), *ros2::Node::node->get_clock(), (long)((duration) * 1000.0), __VA_ARGS__)
#define ROS_ERROR_STREAM_THROTTLE(duration, ...) RCLCPP_ERROR_STREAM_THROTTLE(ros2::Node::node->get_logger(), *ros2::Node::node->get_clock(), (long)((duration) * 1000.0), __VA_ARGS__)

#define ROS_DEBUG(...) RCLCPP_DEBUG(ros2::Node::node->get_logger(), __VA_ARGS__)
#define ROS_DEBUG_STREAM(...) RCLCPP_DEBUG_STREAM(ros2::Node::node->get_logger(), __VA_ARGS__)
#define ROS_DEBUG_THROTTLE(duration, ...) RCLCPP_DEBUG_THROTTLE(ros2::Node::node->get_logger(), *ros2::Node::node->get_clock(), (long)((duration) * 1000.0), __VA_ARGS__)
#define ROS_DEBUG_STREAM_THROTTLE(duration, ...) RCLCPP_DEBUG_STREAM_THROTTLE(ros2::Node::node->get_logger(), *ros2::Node::node->get_clock(), (long)((duration) * 1000.0), __VA_ARGS__)


#endif /*AUTONOMY_UTILS_INCLUDE_LIBS_ROS2_NODES_H_*/