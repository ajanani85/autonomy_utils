#ifndef AUTONOMY_UTILS_LIB_BAG_LOGGER_H_
#define AUTONOMY_UTILS_LIB_BAG_LOGGER_H_

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <autonomy_utils/lib/ros2_core.h>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_storage/storage_options.hpp>


#define DEFAULT_BAG_DIR "/var/log/terabase/"
#define DEFAULT_MEDIA_DIR "/media/terabase"


//in ros2 you cannot do ros::Time::now(). You have to send the time over
#define LOG_MSG(topic, msg, level) { LOG_MSG_STAMP(topic, msg, clockNow(), level) }

#define LOG_MSG_STAMP(topic, msg, t, level) { if (BagLogger::instance()->isLogging() && \
                                                 (level) <= BagLogger::instance()->getLogLevel()) { \
											BagLogger::instance()->LOG((msg), (topic), (t), (level)); } }

// #define LOG_MSG2(msg, topic, msg_str, t, level) { LOG_MSG2_STAMP(msg, topic, msg_str, t, level) }

// #define LOG_MSG2_STAMP(msg, topic, msg_str, t, level) { if (BagLogger::instance()->isLogging() && \
//                                                  (level) <= BagLogger::instance()->getLogLevel()) { \
// 											BagLogger::instance()->LOG2((msg), (topic), (msg_str), (t), (level)); } }

class BagLogger {
    static BagLogger *s_instance_;

    unsigned log_level_ {0};

    bool is_logging_ {false};

    std::string prefix_ {""};

    std::string file_name_;

public:
    BagLogger()
    {
    	writer_ = std::make_unique<rosbag2_cpp::Writer>();
    }

    ~BagLogger()
    {
    	writer_.release();
    }

    void add_topic(const std::string &topic, const std::string &message_in_str);

    // void LOG(std::shared_ptr<rclcpp::SerializedMessage> message,
    // 		rclcpp::Time &time, const std::string &topic, const std::string &message_string, int level);

    template <class MessageT>
    void LOG(const MessageT &msg, const std::string &topic, const rclcpp::Time &time, int level)
    {
    	if(is_logging_ && level <= log_level_)
    	{
    		writer_->write(msg, topic, time);
    	}
    }

    // template <class MessageT>
    // void LOG2(const MessageT &msg, const std::string &topic, const std::string &msg_str, const rclcpp::Time &time, int level)
    // {
    // 	if(is_logging_ && level <= log_level_)
    // 	{
    // 		writer_->write(msg, topic, time);
    // 	}
    // }

    template<class MessageT>
    static std::string messageTypeAsString()
    {
    	return rosidl_generator_traits::name<MessageT>();
    }

    template<class MessageT>
    void serializeMessage(const MessageT &m, std::shared_ptr<rclcpp::SerializedMessage> message)
    {
    	rclcpp::SerializedMessage serialized_msg;
    	rclcpp::Serialization<MessageT> serialization;
    	serialization.serialize_message(&message, serialized_msg);
    }


    std::unique_ptr<rosbag2_cpp::Writer> writer_;


   static BagLogger *instance() {
        if (!s_instance_)
        {
            s_instance_ = new BagLogger;
        }
        return s_instance_;
    }


    bool isLogging() { return is_logging_; }

    static std::string getLogFileDir();

    static std::string getSequence(std::string dir, std::string prefix);

    static std::string getLogFileName(std::string prefix, std::string suffix="");

    void dumpParams(std::string prefix);

    void startLogging(std::string prefix, unsigned log_level);

    void startLogging(const std::string &prefix, unsigned log_level, rosbag2_storage::StorageOptions &options);

    void stopLogging();

    unsigned getLogLevel() const {
        return log_level_;
    }

    bool isLogging() const {
        return is_logging_;
    }

    enum BagLoggerLevel: unsigned int {OFF = 0, NORM = 1, FINE = 2, EXTRA = 3, ALL = 4};

    // General purpose logging routines that can be used by any process supplying their own file descriptors
    // These can be used like a printf and are mainly used to log printable text
    static void open_log(std::string prefix, FILE **debug_file, int log_level);
    static void close_log(FILE **debug_file);
    static void debug_log(FILE **debug_file, const char* format, ...);

    FILE **file_handle_ {NULL};
    FILE *debug_file_ {NULL};

    void setLogHandle(FILE** handle) {
        if (file_handle_ != NULL)
        {
            perror("LOGGER:setLogHandle - file-handle already set and not NULL\n");
            return;
        }
        file_handle_ = handle;
    }
    void open_log(std::string prefix, int log_level) {
      return;
      if (file_handle_ == NULL)
        {
    	  	perror("LOGGER:open_log - file handle NULL and needs to be set\n");
            return;
        }
        open_log(prefix, file_handle_, log_level);
    }
    void close_log() {
      return;
        if (file_handle_ == NULL)
        {
        	perror("LOGGER:close_log - file handle NULL and needs to be set\n");
            return;
        }
        close_log(file_handle_);
    }
    void debug_log(const char* format, ...) {
      return;
        if (file_handle_ == NULL)
        {
        	perror("LOGGER:debug_log - file handle NULL and needs to be set\n");
            return;
        }
        va_list args;
        va_start( args, format );
        debug_log(file_handle_, format, args);
        va_end( args );
    }

    void close_bagfile()
    {
    	writer_->get_implementation_handle().close();
    }

private:

};

#endif /* AUTONOMY_UTILS_LIB_BAG_LOGGER_H_ */