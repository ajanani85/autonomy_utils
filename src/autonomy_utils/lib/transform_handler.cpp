#include <autonomy_utils/lib/transform_handler.h>

namespace ros2
{
TransformHandler::TransformHandler(std::shared_ptr<ros2::Transformer> transformer, 
    const std::string &source, 
    const std::string &target): source_(source), target_(target), transformer_(transformer)
{
    if(!transformer_)
    {
        ROS_INFO(RED "Transformer is not setup properly" COLOR_RESET);
    }
}

TransformHandler::~TransformHandler()
{

}

bool TransformHandler::isSet()
{
    return is_set_;
}

geometry_msgs::msg::TransformStamped &TransformHandler::getTransform()
{
    if(!transformer_)
    {
        transformer_ = std::make_shared<ros2::Transformer>();
    }

    if(is_set_)
    {
       return tf_stamped_; 
    }
    
    //if it is not set
    if(transformer_->getTransform(source_, target_, tf_stamped_, 1.0, false))
    {
        is_set_ = true;
        return tf_stamped_;
    }
    else
    {
        ROS_INFO(RED "Cannot find a transform between src:%s and target:%s" COLOR_RESET, source_.c_str(), target_.c_str());
        return tf_stamped_;
    }
   
}

void TransformHandler::reset()
{
    is_set_ = false;
}

void TransformHandler::print(const std::string &msg)
{
    ROS_INFO(GREEN "%s src: %s, tgt: %s T[%f,%f,%f], R[%f,%f,%f,%f]" COLOR_RESET, msg.c_str(), 
        source_.c_str(), target_.c_str(),
        tf_stamped_.transform.translation.x, 
        tf_stamped_.transform.translation.y,
        tf_stamped_.transform.translation.z,
        tf_stamped_.transform.rotation.x,
        tf_stamped_.transform.rotation.y,
        tf_stamped_.transform.rotation.z,
        tf_stamped_.transform.rotation.w);
}
}