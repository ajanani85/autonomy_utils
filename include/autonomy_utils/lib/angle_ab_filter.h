
 #ifndef AUTONOMY_UTILS_LIB_ANGLE_AB_FILTER_H_
 #define AUTONOMY_UTILS_LIB_ANGLE_AB_FILTER_H_
 
 #include <geometry_msgs/msg/quaternion.hpp>
 #include <autonomy_utils/lib/rotate.h>
 
 namespace ros2
 {
 
 class AngleAlphaBetaFilter
 {
 public:
     AngleAlphaBetaFilter(double weight, double init_r = 0.0, double init_p = 0.0, double init_y = 0.0);
     ~AngleAlphaBetaFilter();
     void setWeight(double weight)
     {
         weight_ = weight;
     }
 
     void initialize(const geometry_msgs::msg::Quaternion &q);
 
     void initialize(double r, double p, double y);
 
     double filter(double new_angle);
 
     bool updateIfOk(double angle, double &filtered_angle);
 
     bool filter(geometry_msgs::msg::Quaternion &q);
 
     double getAngle()
     {
         return old_angle_;
     }
 
 private:
     double weight_ = 0.5;
     double old_angle_ = 0.0;
     bool data_is_set_ = false;
 
     double init_roll_ = {0.0};
     double init_pitch_ = {0.0};
     double init_yaw_ = {0.0};
 };
 
 }
 #endif /* AUTONOMY_UTILS_LIB_ANGLE_AB_FILTER_H_ */