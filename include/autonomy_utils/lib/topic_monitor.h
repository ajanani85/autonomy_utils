#ifndef AUTONOMY_UTILS_LIB_TOPIC_MONITOR_H
#define AUTONOMY_UTILS_LIB_TOPIC_MONITOR_H

#include <autonomy_utils/lib/ros2_core.h>
#include <deque>

namespace ros2
{

    template <typename T>
    class TopicMonitor
    {
    public:
        TopicMonitor(std::string &topic) : topic_(topic)
        {
            configure();
        }

        ~TopicMonitor() {}

        double getFrequency() const { return frequency_; }

        double getMaxFrequency() const { return max_frequency_; }
        
        double getMinFrequency() const { return min_frequency_; }

        void reset()
        {
            frequency_ = 0.0;
            max_frequency_ = std::numeric_limits<double>::lowest();
            min_frequency_ = std::numeric_limits<double>::max();
        }
        

    private:
        const std::string topic_;

        rclcpp::TimerBase::SharedPtr timer_;

        typename rclcpp::Subscription<T>::SharedPtr sub_;

        rclcpp::Publisher<autonomy_msgs::msg::TopicStats>::SharedPtr stats_pub_;

        void configure()
        {
            stats_pub_ = ros2::Node::node->create_publisher<autonomy_msgs::msg::TopicStats>("/topic_stats",1);

            sub_ = ros2::Node::node->create_subscription<T>(topic_, 1, std::bind(&TopicMonitor::callback, this, std::placeholders::_1));
            
            timer_ = ros2::Node::node->create_wall_timer(ros2::toDuration(0.01), std::bind(&TopicMonitor::timerCB, this));
        }

        void callback(const typename T::SharedPtr msg)
        {
            timestamps_.push_back(clockNow());
        }

        void timerCB()
        {
            const rclcpp::Time now = clockNow();

            // Drop timestamps 1 second old or older, so timestamps_ only ever
            // holds messages received within the trailing 1-second window.
            while (!timestamps_.empty() && (now - timestamps_.front()).seconds() >= 1.0)
            {
                timestamps_.pop_front();
            }

            // Message count within that trailing 1-second window is the frequency in Hz.
            frequency_ = static_cast<double>(timestamps_.size());

            max_frequency_ = std::max(max_frequency_, frequency_);

            min_frequency_ = std::min(min_frequency_, frequency_);
        }

        double frequency_ = 0.0;

        double max_frequency_ = std::numeric_limits<double>::lowest();
        
        double min_frequency_ = std::numeric_limits<double>::max();

        std::deque<rclcpp::Time> timestamps_;
    };
}

#endif /* AUTONOMY_UTILS_LIB_TOPIC_MONITOR_H */