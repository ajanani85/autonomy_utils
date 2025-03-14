#ifndef AM_UTILS_INCLUDE_AM_UTILS_AM_PCL_UTILS_H_
#define AM_UTILS_INCLUDE_AM_UTILS_AM_PCL_UTILS_H_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autonomy_utils/lib/point_types.h>
#include <unordered_set>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>

namespace ros2
{

class PCLUtils
{

public:
      template<class T>
      static T & toPcl(const geometry_msgs::msg::Point &in, T &out)
      {
            out.x = in.x;
            out.y = in.y;
            out.z = in.z;

            return out;
      }

      template<class T>
      static T & toPcl(const geometry_msgs::msg::Point32 &in, T &out)
      {
            out.x = in.x;
            out.y = in.y;
            out.z = in.z; 
            return out;
      }

      static void doTransform(const pcl::PointXYZ & t_in, pcl::PointXYZ & t_out, const geometry_msgs::msg::TransformStamped & transform);

      static void doTransform(const pcl::PointXYZIR & t_in, pcl::PointXYZIR & t_out, const geometry_msgs::msg::TransformStamped & transform);


      static pcl::PointCloud<pcl::PointXYZIR>::ConstPtr convertToPointXYZIR(const sensor_msgs::msg::PointCloud2::SharedPtr pc2, pcl::PointCloud<pcl::PointXYZIR>::Ptr &pcl);
      static pcl::PointCloud<pcl::PointXYZ>::ConstPtr convertToPointXYZ(const sensor_msgs::msg::PointCloud2::SharedPtr pc2, pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl);
      static bool convertToPointXYZ(const sensor_msgs::msg::Image::ConstSharedPtr &image, const sensor_msgs::msg::CameraInfo::SharedPtr &cam_info,
            pcl::PointCloud<pcl::PointXYZ> &cloud, geometry_msgs::msg::TransformStamped &transform, std::string &error_msg,
            double downsample_factor = 1.0, double depth_factor = 1000.0, double min_distance = 0.5, double max_distance = 6.0);

      static void convertToPointXYZIR(const sensor_msgs::msg::PointCloud2::SharedPtr pc2, pcl::PointCloud<pcl::PointXYZIR> &pcl);
      static void convertToPointXYZIR(const sensor_msgs::msg::PointCloud2 &pc2, pcl::PointCloud<pcl::PointXYZIR> &pcl);
      static void convertToPointXYZ(const sensor_msgs::msg::PointCloud2::SharedPtr pc2, pcl::PointCloud<pcl::PointXYZ> &pcl);
      static void convertToPointXYZ(const sensor_msgs::msg::PointCloud2 &pc2, pcl::PointCloud<pcl::PointXYZ> &pcl);

      static void toPointCloud2(const pcl::PointCloud<pcl::PointXYZ> &cloud, sensor_msgs::msg::PointCloud2 &cloud_out);
      static void toPointCloud2(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, sensor_msgs::msg::PointCloud2 &cloud_out);
      static void toPointCloud2(const pcl::PointCloud<pcl::PointXYZIR> &cloud, sensor_msgs::msg::PointCloud2 &cloud_out);
      static void toPointCloud2(const pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud, const std::vector<int> &valid_indices, sensor_msgs::msg::PointCloud2 &cloud_out);
      static void toPointCloud2(const pcl::PointCloud<pcl::PointXYZIR> &cloud, const std::vector<int> &valid_indices, sensor_msgs::msg::PointCloud2 &cloud_out);


      static bool isSetToNaN(pcl::PointXYZIR &point);
      static bool isSetToNaN(pcl::PointXYZ &point);
      static bool isSetToNaN(pcl::PointXYZRGB &point);

      static void setNaN(pcl::PointXYZIR &point);
      static void setNaN(pcl::PointXYZ &point);
      static void setNaN(pcl::PointXYZRGB &point);
};

}

#endif /* AM_UTILS_INCLUDE_AM_UTILS_AM_PCL_UTILS_H_ */