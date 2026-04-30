#ifndef AUTONOMY_UTILS_LIB_BAG_LOGGER_H_
#define AUTONOMY_UTILS_LIB_BAG_LOGGER_H_

#include <autonomy_utils/lib/logger.h>

#define DEFAULT_BAG_DIR "/var/log/terabase/"
#define DEFAULT_MEDIA_DIR "/media/terabase"

// in ros2 you cannot do ros::Time::now(). You have to send the time over
#define LOG_MSG(topic, msg, level) {LOG_MSG_STAMP(topic, msg, clockNow(), level)}

#define LOG_MSG_STAMP(topic, msg, t, level)                           \
    {                                                                 \
        if (BagLogger::instance()->isLogging() &&                     \
            (level) <= BagLogger::instance()->getLogLevel())          \
        {                                                             \
            BagLogger::instance()->LOG((msg), (topic), (t), (level)); \
        }                                                             \
    }

class BagLogger : public Logger
{
    static BagLogger *s_instance_;

public:
    BagLogger();

    ~BagLogger();

    template <class MessageT>
    void LOG(const MessageT &msg, const std::string &topic, const rclcpp::Time &time, int level)
    {
        if (isLogging() && level <= getLogLevel())
        {
            writer_->write(msg, topic, time);
        }
    }

    template <class MessageT>
    void serializeMessage(const MessageT &m, std::shared_ptr<rclcpp::SerializedMessage> message)
    {
        rclcpp::SerializedMessage serialized_msg;
        rclcpp::Serialization<MessageT> serialization;
        serialization.serialize_message(&message, serialized_msg);
    }

    static BagLogger *instance()
    {
        if (!s_instance_)
        {
            s_instance_ = new BagLogger;
        }
        return s_instance_;
    }
    

private:
};

#endif /* AUTONOMY_UTILS_LIB_BAG_LOGGER_H_ */