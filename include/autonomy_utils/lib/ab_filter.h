/*
 * ftp_requset.h
 *
 *  Created on: Mar 13, 2016
 *      Author: ubuntu
 */

 #ifndef AUTONOMY_UTILS_LIB_AB_FILTER_H_
 #define AUTONOMY_UTILS_LIB_AB_FILTER_H_
 
 #include <nav_msgs/msg/odometry.hpp>
 
 namespace ros2
 {
 
 class AlphaBetaFilter
 {
 public:
     AlphaBetaFilter();
 
     AlphaBetaFilter(double weight, double init_x = 0.0, double init_y = 0.0, double init_z = 0.0);
 
     AlphaBetaFilter(double x_weight, double y_weight, double z_weight, double init_x, double init_y, double init_z);
 
     void setWeight(double weight)
     {
         weight_ = weight;
     }
 
     double filter(double new_value);
 
     bool filter(double &x, double &y, double &z);
 
     bool filter(nav_msgs::msg::Odometry &odom);
 
     bool filter(geometry_msgs::msg::Pose &pose);
 
     void setInitialPose(double x, double y, double z);
 
     double getValue()
     {
         return old_value_;
     }
 
 private:
     double weight_ = {0.5};
     double x_weight_ = {0.5};
     double y_weight_ = {0.5};
     double z_weight_ = {0.5};
     double old_value_ = {0.0};
     bool data_is_set_ = {false};
     double initial_x_ = {0.0};
     double initial_y_ = {0.0};
     double initial_z_ = {0.0};
 };
 
 }
 #endif /* AUTONOMY_UTILS_LIB_AB_FILTER_H_ */