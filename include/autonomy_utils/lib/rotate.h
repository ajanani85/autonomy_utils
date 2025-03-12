#ifndef AUTONOMY_UTILS_INCLUDE_LIBS_ROTATE_H_
#define AUTONOMY_UTILS_INCLUDE_LIBS_ROTATE_H_

#include <math.h>
#include <stdio.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <autonomy_utils/lib/point_types.h>

namespace ros2
{
  struct EULER
  {
    double angle;
    double s;
    double c;
  };

  class Rotate
  {
  private:
    EULER ROLL, PITCH, YAW;
    double R[9];
    double R_inv[9];
    double R_adj[9];
    double R_t[9];
    double R_det;

    void setRotationMatrixComponents();
    void processR();
  public:
    Rotate(double roll, double pitch, double yaw);
    Rotate(double x, double y, double z, double w);
    Rotate(geometry_msgs::msg::Quaternion &q);
    Rotate(tf2::Quaternion &q);
    Rotate();

    static EULER getEuler(double ang);
    void setRPY(double roll, double pitch, double yaw);
    void getRPY(double &roll, double &pitch, double &yaw);


    /*
      ROTATIOTION AROUND ALL THREE AXISES WITH PREDEFINED ANGLES.
      BEAR IN MIND THAT THE ROTATION MATRIX IS IN ORDER OF ZYX.
      THIS IS THE EULER ORDER AND IF YOU MULTIPLY THE MATRICES IN DIFFERENT ORDERS
      YOU WILL GET DIFFERENT RESULTS. THIS IS THE CORRECT ONE.
     */
    void rotate(float &x, float &y, float &z);
    /*
      ROTATIOTION AROUND ALL THREE AXISES WITH PREDEFINED ANGLES IN INVERTED FASHION.
      THE ROTATION MATRIX PASSED ORIGINALLY IS INVERTED.
    */
    void rotateInv(float &x, float &y, float &z);

    void rotateTranspose(float &x, float &y, float &z);

    //ROTATE AROUND ONLY ONE AXIS WITH NEW ANGLE.
    void rotate(float &x, float &y, float &z, double angle, char axis);

    //ROTATE AROUND ONLY ONE AXIS WITH PREDEFINED ANGLE -> IT IS SLIGHTLY FASTER THAN CONSIDERING ALL THREE.
    void rotate(float &x, float &y, float &z, char axis);

    //ROTATE AROUND ONLY ONE AXIS WITH PREDEFINED ANGLE -> IT IS SLIGHTLY FASTER THAN CONSIDERING ALL THREE.
    //GIVEN XY AS INT -> Use Case is in 2D Images
    //void rotate(int &x, int &y, char axis);

    //STATIC METHOD TO ROTATE AROUND AN AXIS
    static void rotate(double &x, double &y, double &z, float roll, float pitch, float yaw);

    static void rotate(double &x, double &y, double &z, geometry_msgs::msg::Quaternion &q_rot);

    static void rotate(double &x, double &y, double &z, const geometry_msgs::msg::Quaternion &q_rot);

    static void rotate(geometry_msgs::msg::Point32 &p, const geometry_msgs::msg::Quaternion &q_rot);

    static void rotate(pcl::PointXYZIR &p, const geometry_msgs::msg::Quaternion &q_rot);

    static void rotate(pcl::PointXYZ &p, const geometry_msgs::msg::Quaternion &q_rot);
    static void rotate(pcl::PointXYZRGB &p, const geometry_msgs::msg::Quaternion &q_rot);

    /*
     * @brief transforms a point with the passed transformed stamped
     */
    static void transform(geometry_msgs::msg::Point32 &p_in, const geometry_msgs::msg::TransformStamped &tranform);

    /*
     * @brief transforms a pose with another pose
     */
    static void transform(const geometry_msgs::msg::Pose &src, geometry_msgs::msg::Pose &des);

    /*
     * @brief inverse transform
     */
    static void reverse(geometry_msgs::msg::Quaternion &q);

    //Static method to convert quaternion by component to rpy by component
    static void getRPY(double x, double y, double z, double w, double &roll, double &pitch, double &yaw);

    //Static method to convert quaternion to rpy
    static void getRPY(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw);

    //Static method to convert quaternion to rpy
    static void getRPY(tf2::Quaternion &q, double &roll, double &pitch, double &yaw);

    //Static method to convert pose to rpy in geometry_msgs::Vector3
    static geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::Pose &pose);

    //Static method to get Yaw from geometry_msgs Quaternion
    static double getYaw(const geometry_msgs::msg::Quaternion &q);

    static double getYaw(tf2::Quaternion &q);

    //Static method to convert from rpy to quaternion
    static tf2::Quaternion toQuaternion(const std::array<double,3> rpy);

    //Static method to convert from rpy to quaternion message
    static geometry_msgs::msg::Quaternion toQuaternionMsg(const std::array<double,3> rpy);

    //Static method to convert from rpy to quaternion
    static tf2::Quaternion toQuaternion(double r, double p, double y);

    //Static method to convert from rpy to quaternion message
    static geometry_msgs::msg::Quaternion toQuaternionMsg(double r, double p, double y);

    static double toRadian(double ang);
    static double toDegree(double ang);
    static void toRadian(geometry_msgs::msg::Point &rpy);
    static void toDegree(geometry_msgs::msg::Point &rpy);

    /*
     * @brief assure that the angle is between -M_PI and M_PI
     */
    static double wrap_pi(double angle);

  };
}

#endif /*AUTONOMY_UTILS_INCLUDE_LIBS_ROTATE_H_*/
