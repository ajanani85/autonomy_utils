/*
 * transformer.cpp
 *
 *  Created on: May 18, 2018
 *      Author: ubuntu
 */

 #include <autonomy_utils/lib/transformer.h>

 #include <tf2/transform_datatypes.h>
 
 namespace ros2
 {
 
 Transformer::Transformer()
 {
     // TODO Auto-generated constructor stub
 
 }
 
 Transformer::~Transformer() {
     // TODO Auto-generated destructor stub
 }
 
 tf2_ros::Buffer& Transformer::tfBuffer() {
     if (!tfBuffer_instance_)
     {
         tfBuffer_instance_ = new tf2_ros::Buffer(getClock());
         tfListener_instance_ = new tf2_ros::TransformListener(*tfBuffer_instance_);
     }
     return *tfBuffer_instance_;
 }
 
 bool Transformer::getTransform(const std::string target, const std::string source,
                   geometry_msgs::msg::TransformStamped& transform,  double wait, bool debug) {
     if (wait < 0.0)
     {
         wait = DEFAULT_TRANSFORM_WAIT;
     }
 
     try
     {
         transform = tfBuffer().lookupTransform(target, source, rclcpp::Time(0), rclcpp::Duration(wait, 0));
     }
     catch (tf2::TransformException &ex) {
         ROS_ERROR_STREAM("Transformer::getTransform TRANSFORM ERROR source[" << source << "], target[" << target << "]  ERROR: " << ex.what());
         // ROS_ERROR_STREAM("Transformer::getTransform TRANSFORM ERROR source[" << "source" << "], target[" << "target" << "]  ERROR: " << ex.what());
         return false;
     }
 
     if (debug)
     {
         //todo: figure out message printing
         //RCLCPP_INFO_STREAM(node_->get_logger(), "Transformer::getTransform TRANSFORM = \n" << transform);
     }
 
     return true;
 }
 
 bool Transformer::getTransform(bool& initialized_flag, const std::string target, const std::string source,
                                geometry_msgs::msg::TransformStamped& transform, double wait, bool debug) {
     if (!initialized_flag)
     {
         if (!getTransform(target, source, transform, wait, debug))
         {
             return false;
         }
         initialized_flag = true;
     }
 
     return true;
 }
 
 void Transformer::rotateOrientation(geometry_msgs::msg::Quaternion& orientation, geometry_msgs::msg::Quaternion& rotation,
                                     bool debug) {
     tf2::Quaternion q_orig;
     tf2::Quaternion q_rot;
     tf2::Quaternion q_new;
 
     tf2::fromMsg(orientation, q_orig);
     tf2::fromMsg(rotation, q_rot);
 
 
 
     q_new = q_rot*q_orig;  // Calculate the new orientation
     q_new.normalize();
     orientation = tf2::toMsg(q_new);
 
     if (debug)
     {
 
 
         double orig_yaw = ros2::Rotate::getYaw(q_orig) * 180.0 / 3.14159;
         double rot_yaw = ros2::Rotate::getYaw(q_rot) * 180.0 / 3.14159;
         double new_yaw =  ros2::Rotate::getYaw(q_new) * 180.0 / 3.14159;
         ROS_INFO("Transformer::rotateOrientation orientation[%0.3f] rotation[%0.3f] new[%0.3f]\n", orig_yaw, rot_yaw, new_yaw);
     }
 }
 
 bool Transformer::rotateVector(const std::string target_frame,
         const std::string source_frame, Eigen::Vector3d& target_vector,
         const Eigen::Vector3d& source_vector, bool debug) {
 
     geometry_msgs::msg::TransformStamped ts;
     if (!getTransform(target_frame, source_frame, ts, true))
     {
         ROS_ERROR("Kinematics::getLinearVelocity : No transform from[%s] to[%s]", source_frame.c_str(), target_frame.c_str());
         return false;
     }
 
     Eigen::Quaterniond rot;
     rot.x() = ts.transform.rotation.x;
     rot.y() = ts.transform.rotation.y;
     rot.z() = ts.transform.rotation.z;
     rot.w() = ts.transform.rotation.w;
 
     target_vector = rot * source_vector;
 
     return true;
 }
 
 } /* namespace am */