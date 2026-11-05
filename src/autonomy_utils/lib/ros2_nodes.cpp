#include <autonomy_utils/lib/ros2_nodes.h>
#include <unistd.h>
#include <cstdio>
#include <sstream>

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

    rclcpp::QoS getQoS(const std::string &name, int q_size)
    {

        rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)).reliable();
        qos.history(rclcpp::HistoryPolicy::KeepLast).keep_last(q_size);
        if (name == "RELIABLE")
        {
            return qos;
        }
        else if (name == "BEST_EFFORT")
        {
            return getSensorQoS(q_size);
        }
        return qos;
    }

    int Node::get_node_pid(const std::string &node_name)
    {
        // Split "/base/ublox_dgnss" into namespace "/base" and name "ublox_dgnss"; they appear on the
        // command line as "__ns:=/base" and "__node:=ublox_dgnss"
        size_t slash = node_name.rfind('/');
        std::string name = node_name.substr(slash == std::string::npos ? 0 : slash + 1);
        std::string ns = (slash == std::string::npos || slash == 0) ? "" : node_name.substr(0, slash);
        if (!ns.empty() && ns.front() != '/')
        {
            ns = "/" + ns;
        }
        if (name.empty())
        {
            return -1;
        }

        // The name goes into a shell command, so only allow valid ROS name characters
        for (char c : name)
        {
            if (!isalnum(static_cast<unsigned char>(c)) && c != '_')
            {
                return -1;
            }
        }

        // -w so "ublox_dgnss_base" does not also match "ublox_dgnss_base_extra"
        std::string command = "ps -aux | grep -w " + name + " | grep -v grep";
        FILE *ps = popen(command.c_str(), "r");
        if (!ps)
        {
            return -1;
        }

        const int self_pid = getpid();
        int found_pid = -1;
        char buf[4096];
        while (fgets(buf, sizeof(buf), ps) != nullptr)
        {
            // ps -aux columns: USER PID %CPU ...
            std::istringstream iss(buf);
            std::string user;
            int pid = -1;
            if (!(iss >> user >> pid) || pid <= 0 || pid == self_pid)
            {
                continue;
            }

            std::string line(buf);
            // skip the "ros2 run" python wrapper; its child is the actual node
            if (line.find("ros2 run") != std::string::npos)
            {
                continue;
            }

            // same node name can run in several namespaces (/base/ublox_dgnss, /rover/ublox_dgnss)
            if (!ns.empty())
            {
                std::string ns_arg = "__ns:=" + ns;
                size_t pos = line.find(ns_arg);
                size_t after = pos + ns_arg.size();
                if (pos == std::string::npos || (after < line.size() && !isspace(static_cast<unsigned char>(line[after]))))
                {
                    continue;
                }
            }

            found_pid = pid;
            break;
        }
        pclose(ps);

        return found_pid;
    }

}