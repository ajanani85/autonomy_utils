/*
 * transformer.h
 *
 *  Created on: May 18, 2018
 *      Author: ubuntu
 */

 #ifndef AUTONOMY_UTILS_INCLUDE_TRANSFORMER_H_
 #define AUTONOMY_UTILS_INCLUDE_TRANSFORMER_H_
 
 #include <string>
 
 #include <autonomy_utils/lib/ros2_core.h>
 #include <tf2/LinearMath/Quaternion.h>
 #include <tf2/LinearMath/Transform.h>
 #include <tf2_ros/transform_broadcaster.h>
 #include <tf2_ros/transform_listener.h>
 #include <tf2_ros/buffer.h>
 #include <geometry_msgs/msg/quaternion.hpp>
 
 #include <eigen3/Eigen/Geometry>
 #include <eigen3/Eigen/StdVector>
 #include <autonomy_utils/lib/rotate.h>
 
 namespace ros2
 {
 #define DEFAULT_TRANSFORM_WAIT 0.07
 
 class Transformer
 {
 public:
     Transformer();
     virtual ~Transformer();
 
     tf2_ros::Buffer& tfBuffer();
 
     /// Used to fetch the latest transform between two frames
     /// Return true if it works and false otherwise
     bool getTransform(const std::string target, const std::string source,
                       geometry_msgs::msg::TransformStamped& transform,  double wait = 1.0, bool debug = false);
 
     /// Used to fetch the latest transform between two frames
     /// Checks the initialized flag and will not call the lookup if it is true.
     /// Otherwise it attempts to lookup the transform and set the initialized flag appropriately.
     /// Return true if the transform is valid and false otherwise
     bool getTransform(bool& initialized_flag, const std::string target, const std::string source,
                       geometry_msgs::msg::TransformStamped& transform,  double wait = 1.0, bool debug = false);
 
     /// Used to rotate the "orientation" coordinates by the "rotation" coordinates
     void rotateOrientation(geometry_msgs::msg:: Quaternion& orientation, geometry_msgs::msg::Quaternion& rotation, bool debug = false);
 
     /// Used to rotate a vector from source frame to target frame.
     /// Returns false if failed,
     bool rotateVector(const std::string target_frame, const std::string source_frame,
             Eigen::Vector3d& target_vector, const Eigen::Vector3d& source_vector, bool debug = false);
 
 private:
     // transform buffer management. Use factory creation method to auto create the tfBuffer
     // later make this a static to save space and make more efficient
     tf2_ros::Buffer *tfBuffer_instance_ {NULL};
     tf2_ros::TransformListener *tfListener_instance_;
 };
 
 } /* namespace am */
 
 #endif /* AUTONOMY_UTILS_INCLUDE_TRANSFORMER_H_ */