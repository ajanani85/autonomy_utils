#ifndef AUTONOMY_UTILS_LIB_KEYBOARD_CLASS_H_
#define AUTONOMY_UTILS_LIB_KEYBOARD_CLASS_H_

#include <autonomy_utils/lib/ros2_core.h>
#include <std_msgs/msg/string.hpp>

namespace ros2
{
class KeyboardClass
{
public:
    KeyboardClass();

    ~KeyboardClass();
private: 

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr keyboard_pub_;

    std::thread key_thread_;

    std::atomic<bool> running_;

    void configure();

    void activate();

    void keyLoop();

    char getKey();

    
};
}

#endif /*AUTONOMY_UTILS_LIB_KEYBOARD_CLASS_H_*/