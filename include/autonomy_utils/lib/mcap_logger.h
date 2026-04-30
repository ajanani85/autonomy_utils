#ifndef AUTONOMY_UTILS_LIB_MCAP_LOGGER_H_
#define AUTONOMY_UTILS_LIB_MCAP_LOGGER_H_

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <set>
#include <mutex>
#include <memory>
#include <cstdarg>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <autonomy_utils/lib/ros2_core.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <rosidl_generator_traits/message_traits.hpp>



#define LOG_MSG(topic, msg, level) \
{ \
    LOG_MSG_STAMP(topic, msg, clockNow(), level); \
}

#define LOG_MSG_STAMP(topic, msg, t, level) \
{ \
    if (McapLogger::instance()->isLogging() && \
        (level) <= McapLogger::instance()->getLogLevel()) { \
        McapLogger::instance()->LOG((msg), (topic), (t), (level)); \
    } \
}


class McapLogger : public Logger
{
    static McapLogger *s_instance_;


private:
};

#endif  // AUTONOMY_UTILS_LIB_MCAP_LOGGER_H_