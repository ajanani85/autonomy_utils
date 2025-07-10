#ifndef AUTONOMY_UTILS_LIB_IMU_CLASS_H_
#define AUTONOMY_UTILS_LIB_IMU_CLASS_H_

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autonomy_utils/lib/rotate.h>
#include <autonomy_utils/lib/ros2_core.h>
#include <pcl/point_types.h>
#include <math.h>
#include <stdio.h>

namespace ros2
{
  enum AXIS
  {
    ROLL,
    PITCH,
    YAW,
    ROLL_PITCH,
    ROLL_YAW,
    PITCH_YAW,
    RPY
  };
  class ImuClass
  {
  public:
    /*
     * @brief default constructor. the user needs to subscribe to imu topics
     */
    ImuClass();

    /**
     * @brief constructor
     * @param imu_pot_size size of the internal imu buffer
     */
    ImuClass(int imu_pot_size);

    /**
     * @brief constructor for customized imu source
     * @param node Shared pointer to the ROS2 node
     * @param imu_topic Topic name for the IMU data
     * @param imu_pot_size Size of the internal IMU buffer
     */
    ImuClass(rclcpp::Node::SharedPtr node, const std::string &imu_topic, int imu_pot_size = 400);

    /**
     * @brief destructor
     */
    ~ImuClass();

    /**
     * @brief add an IMU message to the internal buffer
     * @param imu IMU message to be added
     */
    void addImu(const sensor_msgs::msg::Imu &imu);

    /**
     * @brief get the latest IMU message
     * @param res Reference to store the latest IMU message
     * @return 0 on success, -1 if no IMU data is available
     */
    int getImu(std::vector<sensor_msgs::msg::Imu> &res, const rclcpp::Time &t, double time_difference);

    /**
     * @brief get the latest IMU message
     * @param res Reference to store the latest IMU message
     * @return 0 on success, -1 if no IMU data is available
     */
    int getImu(sensor_msgs::msg::Imu &res, const rclcpp::Time &t);

    /**
     * @brief get the latest IMU message without time filtering
     * @param res Reference to store the latest IMU message
     * @return 0 on success, -1 if no IMU data is available
     */
    int getImu(sensor_msgs::msg::Imu &res);

    /**
     * @brief transform a point in space using the latest IMU data
     * @param p Point to be transformed
     * @param t Time at which the transformation is applied
     * @param infuse_axis Axis to be infused in the transformation (default is ROLL_PITCH)
     * @return 0 on success, -1 if no IMU data is available
     */
    int transform(geometry_msgs::msg::Point32 &p, const rclcpp::Time &t, int infuse_axis = AXIS::ROLL_PITCH);

    /**
     * @brief transform a odometry message using the latest IMU data
     * @param odom Odometry message to be transformed
     * @param t Time at which the transformation is applied
     * @param rpy Reference to store the roll, pitch, and yaw angles
     * @param infuse_axis Axis to be infused in the transformation (default is RPY)
     * @return 0 on success, -1 if no IMU data is available
     */
    int transform(nav_msgs::msg::Odometry &odom, const rclcpp::Time &t, std::vector<double> &rpy, int infuse_axis = AXIS::RPY);

    /**
     * @brief transform a point in space using the latest IMU data
     * @param x X coordinate of the point
     * @param y Y coordinate of the point
     * @param z Z coordinate of the point
     * @param t Time at which the transformation is applied
     * @param infuse_axis Axis to be infused in the transformation (default is ROLL_PITCH)
     * @return 0 on success, -1 if no IMU data is available
     */
    int transform(float &x, float &y, float &z, const rclcpp::Time &t, int infuse_axis = AXIS::ROLL_PITCH);

    /**
     * @brief transform a point in space using the latest IMU data
     * @param x X coordinate of the point
     * @param y Y coordinate of the point
     * @param z Z coordinate of the point
     * @param t Time at which the transformation is applied
     * @param infuse_axis Axis to be infused in the transformation (default is ROLL_PITCH)
     * @return 0 on success, -1 if no IMU data is available
     */
    int transform(int &x, int &y, int &z, const rclcpp::Time &t, int infuse_axis = AXIS::ROLL_PITCH);

    /**
     * @brief transform a point cloud using the latest IMU data
     * @param pc Point cloud to be transformed
     * @param t Time at which the transformation is applied
     * @param time_difference Time difference between the point cloud and the IMU data
     */
    int transform(pcl::PointCloud<pcl::PointXYZIR> &pc, const rclcpp::Time &t, double time_difference, int infuse_axis);

    /**
     * @brief transform a point cloud using the latest IMU data
     * @param pc Point cloud to be transformed
     * @param t Time at which the transformation is applied
     * @param time_difference Time difference between the point cloud and the IMU data
     * @param infuse_axis Axis to be infused in the transformation (default is ROLL_PITCH)
     * @return 0 on success, -1 if no IMU data is
     */
    int transform(pcl::PointCloud<pcl::PointXYZ> &pc, const rclcpp::Time &t, double time_difference, int infuse_axis);

    /**
     * @brief setup the IMU class with a specific size for the internal buffer
     * @param imu_pot_size Size of the internal IMU buffer
     */
    void setup(int imu_pot_size);

    /**
     * @brief get the number of IMU messages in the internal buffer
     * @return Number of IMU messages in the internal buffer
     */
    int imuCount() const;

    /**
     * @brief normalize the IMU message
     * @param imu IMU message to be normalized
     * This function ensures that the orientation and angular velocity are normalized
     */
    static void normalize(sensor_msgs::msg::Imu &imu);

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    int imu_pot_size_;
    int imu_cnt_;
    std::vector<sensor_msgs::msg::Imu> imu_pot_;

    /**
     * @brief apply the axis transformation to the IMU message
     * @param src Source IMU message to be transformed
     * @param axis Axis to be applied (ROLL, PITCH, YAW, ROLL_PITCH, ROLL_YAW, PITCH_YAW, RPY)
     * @param roll Reference to store the roll angle
     * @param pitch Reference to store the pitch angle
     * @param yaw Reference to store the yaw angle
     */
    void applyAxis(sensor_msgs::msg::Imu &src, int axis, double &roll, double &pitch, double &yaw);

    /**
     * @brief callback function for the IMU subscription
     * @param imu_msg Shared pointer to the received IMU message
     * This function processes the incoming IMU message and adds it to the internal buffer.
     */
    void imuCB(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  };
}

#endif // AUTONOMY_UTILS_LIB_IMU_CLASS_H_