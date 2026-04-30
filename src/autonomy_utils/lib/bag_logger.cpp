#include <autonomy_utils/lib/bag_logger.h>
#include <filesystem>

BagLogger *BagLogger::s_instance_ = 0;

BagLogger::BagLogger() : Logger()
{
}

BagLogger::~BagLogger()
{
    writer_.release();
}
