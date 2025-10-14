#include <autonomy_utils/lib/ros2_nodes.h>

namespace ros2
{
    LifeCycle::LifeCycle(const std::string &node_name, rclcpp::NodeOptions node_options)
        : rclcpp_lifecycle::LifecycleNode(node_name, node_options, true) {}

    /*
        ON CONFIGURE CALLBACK

    */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifeCycle::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring...");

        for (const auto &callback : configure_callbacks_)
        {
            callback();
        }

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void LifeCycle::register_on_configure_callback(std::function<void()> callback)
    {
        configure_callbacks_.push_back(callback);
    }

    /*
        ON ACTIVATE CALLBACK

    */

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifeCycle::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating...");

        for (const auto &callback : activate_callbacks_)
        {
            callback();
        }

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void LifeCycle::register_on_activate_callback(std::function<void()> callback)
    {
        activate_callbacks_.push_back(callback);
    }

    /*
        ON DEACTIVATE CALLBACK

    */

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifeCycle::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating...");
        for (const auto &callback : deactivate_callbacks_)
        {
            callback();
        }

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void LifeCycle::register_on_deactivate_callback(std::function<void()> callback)
    {
        deactivate_callbacks_.push_back(callback);
    }

    /*
        ON CLEANUP CALLBACK

    */

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifeCycle::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up...");
        for (const auto &callback : cleanup_callbacks_)
        {
            callback();
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void LifeCycle::register_on_cleanup_callback(std::function<void()> callback)
    {
        cleanup_callbacks_.push_back(callback);
    }

    /*
        ON SHUTDOWN CALLBACK

    */

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifeCycle::on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Shutting down...");
        for (const auto &callback : shutdown_callbacks_)
        {
            callback();
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void LifeCycle::register_on_shutdown_callback(std::function<void()> callback)
    {
        shutdown_callbacks_.push_back(callback);
    }

    /*
        ON ERROR CALLBACK

    */

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifeCycle::on_error(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Error down...");
        for (const auto &callback : error_callbacks_)
        {
            callback();
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void LifeCycle::register_on_error_callback(std::function<void()> callback)
    {
        error_callbacks_.push_back(callback);
    }

    /**
     * @brief Get the Sensor QoS object
     * 
     * @param q_size The size of the queue
     * @return rclcpp::QoS
     * @details This function returns the sensor QoS with the given queue size.
     * The default queue size is 5.
     */
    rclcpp::QoS getSensorQoS(int q_size)
    {
        rclcpp::QoS qos = rclcpp::SensorDataQoS();
        qos.get_rmw_qos_profile().depth = q_size;


        return qos;
    }

}