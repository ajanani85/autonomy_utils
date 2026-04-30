#ifndef AUTONOMY_UTILS_LIB_LOGGER_H_
#define AUTONOMY_UTILS_LIB_LOGGER_H_

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <filesystem>

#include <autonomy_utils/lib/ros2_core.h>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <set>
#include <mutex>
#include <memory>
#include <cstdarg>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#define DEFAULT_BAG_DIR "/var/log/terabase/"
#define DEFAULT_MEDIA_DIR "/media/terabase"

class Logger
{
public:
    Logger();

    virtual ~Logger() = 0;

    void add_topic(const std::string &topic, const std::string &message_in_str);

    void dumpParams(std::string prefix);

    void startLogging(std::string prefix, unsigned log_level);

    void stopLogging();

    unsigned getLogLevel() const;

    bool isLogging() const;

    template <class MessageT>
    static std::string messageTypeAsString()
    {
        return rosidl_generator_traits::name<MessageT>();
    }

    enum BagLoggerLevel : unsigned int
    {
        OFF = 0,
        NORM = 1,
        FINE = 2,
        EXTRA = 3,
        ALL = 4
    };

protected:
    std::unique_ptr<rosbag2_cpp::Writer> writer_;

    unsigned log_level_{0};

    std::string prefix_{""};

    std::string file_name_;

    static std::string getLogFileDir();

    static std::string getSequence(std::string dir, std::string prefix);

    static std::string getLogFileName(std::string prefix, std::string suffix = "");

    // General purpose logging routines that can be used by any process supplying their own file descriptors
    // These can be used like a printf and are mainly used to log printable text
    void open_log(std::string prefix, int log_level);

    static void open_log(std::string prefix, FILE **debug_file, int log_level);

    static void close_log(FILE **debug_file);

    void close_log();

    void debug_log(const char* format, ...);

    static void debug_log(FILE **debug_file, const char *format, ...);

    void setLogHandle(FILE** handle);

    FILE **file_handle_{NULL};

    FILE *debug_file_{NULL};

private:
    bool is_logging_{false};

    void close_bagfile();
};

#endif /* AUTONOMY_UTILS_LIB_LOGGER_H_ */