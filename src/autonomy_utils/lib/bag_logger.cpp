#include <autonomy_utils/lib/bag_logger.h>
#include <filesystem>

BagLogger* BagLogger::s_instance_ = 0;

std::string BagLogger::getLogFileDir()
{
  using namespace std;
  namespace fs = std::filesystem; // Alias for std::filesystem

  fs::path p(DEFAULT_MEDIA_DIR);
  try
  {
    for (const auto& entry : fs::directory_iterator(p))
    {
      if (fs::is_directory(entry))
      {
        std::size_t found = entry.path().filename().string().find("LOG");
        if (found != std::string::npos)
        {
          return entry.path().string() + "/";
        }
      }
    }
  }
  catch (const fs::filesystem_error& e)
  {
    if (e.code() == std::errc::permission_denied)
    {
      ROS_ERROR_STREAM("Search permission is denied for one of the directories "
                       << "in the path prefix of " << p << "\n");
    }
    else
    {
      ROS_ERROR_STREAM("is_directory(" << p << ") failed with " << e.code().message() << '\n');
    }
  }

  ROS_WARN("LG: NO LOG_XXX SD CARD FOUND\n");

  return std::string(DEFAULT_BAG_DIR);
}

void BagLogger::add_topic(const std::string& topic, const std::string& message_in_str)
{
  rosbag2_storage::TopicMetadata tm;
  tm.name = topic;
  tm.type = message_in_str;
  tm.serialization_format = rmw_get_serialization_format();
  tm.offered_qos_profiles = "";

  writer_->create_topic(tm);
}

// void BagLogger::LOG(std::shared_ptr<rclcpp::SerializedMessage> message, rclcpp::Time& time, const std::string& topic,
//                     const std::string& message_string, int level)
// {
//   if (is_logging_ && level <= log_level_)
//   {
//     // writer_->write(*message, topic, message_string, time);
//   }
// }

std::string BagLogger::getSequence(std::string dir, std::string prefix)
{
  using namespace std;
  namespace fs = std::filesystem; // Alias for std::filesystem

  unsigned int seq_num = 0;

  fs::path p(dir);
  try
  {
    for (const auto& entry : fs::directory_iterator(p))
    {
      if (fs::is_directory(entry)) // Eliminate non-directory entries
      {
        std::vector<std::string> fields;
        std::string filename = entry.path().filename().string();

        // Split the filename by '-' to extract fields
        size_t pos = 0;
        while ((pos = filename.find('-')) != std::string::npos)
        {
          fields.push_back(filename.substr(0, pos));
          filename.erase(0, pos + 1);
        }
        fields.push_back(filename); // Add the last field

        if (fields.size() >= 2 && fields[0] == prefix)
        {
          unsigned long tmp = std::stoul(fields[1]);
          if (tmp < 9999 && tmp > seq_num)
          {
            seq_num = tmp;
          }
        }
      }
    }
  }
  catch (const fs::filesystem_error& e)
  {
    if (e.code() == std::errc::permission_denied)
    {
      ROS_ERROR_STREAM("Search permission is denied for one of the directories "
                       << "in the path prefix of " << p << "\n");
    }
    else
    {
      ROS_ERROR_STREAM("is_directory(" << p << ") failed with " << e.code().message() << '\n');
    }
  }

  seq_num++;

  std::stringstream s;
  s << std::setfill('0') << std::setw(4) << seq_num;

  return s.str();
}

std::string BagLogger::getLogFileName(std::string prefix, std::string suffix)
{
  //    log_name_prefix_ = prefix;
  time_t rawtime;
  struct tm* timeinfo;
  char buffer[80];

  time(&rawtime);
  timeinfo = localtime(&rawtime);

  std::string dir_str = getLogFileDir();

  std::string seq_str = getSequence(dir_str, prefix);

  // std::string format_str = dir_str+prefix+"-"+seq_str+"-%Y-%m-%d-%H-%M-%S."+suffix;
  std::string format_str = dir_str + prefix + "-" + seq_str + "-%Y-%m-%d-%H-%M-%S";
  strftime(buffer, 80, format_str.c_str(), timeinfo);
  std::string str(buffer);

  return str;
}

void BagLogger::dumpParams(std::string prefix)
{
  std::string filename = getLogFileName(prefix);
  std::string cmd = "rosparam dump " + filename;

  printf("BAG_LOGGER: Dumping params to %s\n", filename.c_str());

  system(cmd.c_str());

  return;
}

void BagLogger::startLogging(std::string prefix, unsigned log_level)
{
  if (is_logging_)
  {
    if ((prefix == prefix_) && (log_level == log_level_))
    {
      printf("LG: Already logging at prefix[%s] and log level[%d]\n", prefix.c_str(), log_level_);
      return;
    }
    else
    {
      printf("LG: Closing bag file %s\n", file_name_.c_str());
      // bag.close();
      close_bagfile();
      log_level_ = 0;
      prefix_ = "";
      is_logging_ = false;
    }
  }

  if (log_level > 0)
  {
    prefix_ = prefix;
    file_name_ = getLogFileName(prefix);
    // bag.open(file_name_, rosbag::bagmode::Write);
    printf("prefix: %s, file_name_: %s\n", prefix.c_str(), file_name_.c_str());

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = /*std::string("sqlite:///") +*/ file_name_;
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(storage_options);
    printf("Opening bag file %s\n", file_name_.c_str());
    log_level_ = log_level;
    is_logging_ = true;
  }
}

void BagLogger::stopLogging()
{
  if (is_logging_)
  {
    log_level_ = 0;
    is_logging_ = false;
    printf("LG: Closing bag file %s\n", file_name_.c_str());
    prefix_ = "";
    close_bagfile();
  }
  return;
}

void BagLogger::open_log(std::string prefix, FILE** debug_file, int log_level)
{
  return;
  if (*debug_file != NULL)
  {
    printf("LOGGER: Tried to reopen test logger with prefix %s\n", prefix.c_str());
    return;
  }
  if (log_level > 0)
  {
    std::string log_name = BagLogger::getLogFileName(prefix, "txt");

    if (log_name != "")
    {
      printf("LOGGER: OPENING GENERAL TEXT LOG FILE %s\n", log_name.c_str());
      *debug_file = fopen(log_name.c_str(), "w");
    }
  }
}

void BagLogger::close_log(FILE** debug_file)
{
  return;
  if (*debug_file != NULL)
  {
    fclose(*debug_file);
    *debug_file = NULL;
  }
}

void BagLogger::debug_log(FILE** debug_file, const char* format, ...)
{
  va_list args;
  return;
  if (*debug_file == NULL)
  {
    return;
  }

  va_start(args, format);
  vfprintf(*debug_file, format, args);
  va_end(args);

  fflush(*debug_file);
}