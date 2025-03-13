/*
 * ftp_requset.h
 *
 *  Created on: Mar 13, 2016
 *      Author: ubuntu
 */

 #ifndef AUTONOMY_UTILS_LIB_ANGLES_H_
 #define AUTONOMY_UTILS_LIB_ANGLES_H_
 
 namespace ros2
 {
 
 class Angles
 {
 public:
     static int compare(double ang1, double ang2);
     static double getDiffSigned(double ang1, double ang2);
     static double getDiff(double ang1, double ang2);
         static double getOrderedDiff(double ang1, double ang2);
     static bool isBetween(double target, double left_angle, double right_angle, double & clamped);
 
 private:
 };
 
 }
 #endif /* AUTONOMY_UTILS_LIB_ANGLES_H_ */