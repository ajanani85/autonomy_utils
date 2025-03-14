#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <autonomy_utils/lib/ros2_core.h>
#include <autonomy_utils/lib/pcl_utils.h>


namespace ros2
{

pcl::PointCloud<pcl::PointXYZIR>::ConstPtr PCLUtils::convertToPointXYZIR(
    const sensor_msgs::msg::PointCloud2::SharedPtr pc2, pcl::PointCloud<pcl::PointXYZIR>::Ptr &pcl)
{
  pcl->header = pcl_conversions::toPCL(pc2->header);
  pcl->resize(pc2->height * pc2->width);
  pcl->height = pc2->height;
  pcl->width = pc2->width;
  int index = 0;
  for (sensor_msgs::PointCloud2ConstIterator<float> it(*pc2, "x"); it != it.end(); ++it) {
      float x = it[0];
      float y = it[1];
      float z = it[2];
      float intensity = it[4];
      pcl->points[index].x = x;
      pcl->points[index].y = y;
      pcl->points[index].z = z;
      pcl->points[index].intensity = intensity;
      pcl->points[index].range = std::sqrt(x * x + y * y + z * z);
      index++;
  }

  return pcl;
}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr PCLUtils::convertToPointXYZ(const sensor_msgs::msg::PointCloud2::SharedPtr pc2,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl)
{
	pcl->header = pcl_conversions::toPCL(pc2->header);
	pcl->resize(pc2->height * pc2->width);
	pcl->height = pc2->height;
	pcl->width = pc2->width;
	int index = 0;
	for (sensor_msgs::PointCloud2ConstIterator<float> it(*pc2, "x"); it != it.end(); ++it) {
		float x = it[0];
		float y = it[1];
		float z = it[2];
		float intensity = it[4];
		pcl->points[index].x = x;
		pcl->points[index].y = y;
		pcl->points[index].z = z;
		index++;
	}

	return pcl;
}

void PCLUtils::convertToPointXYZIR(const sensor_msgs::msg::PointCloud2::SharedPtr pc2,
        pcl::PointCloud<pcl::PointXYZIR> &pcl)
{
	pcl.header = pcl_conversions::toPCL(pc2->header);
	pcl.resize(pc2->height * pc2->width);
	pcl.height = pc2->height;
	pcl.width = pc2->width;
	int index = 0;
	for (sensor_msgs::PointCloud2ConstIterator<float> it(*pc2, "x"); it != it.end(); ++it) {
		float x = it[0];
		float y = it[1];
		float z = it[2];
		float intensity = it[4];
		pcl.points[index].x = x;
		pcl.points[index].y = y;
		pcl.points[index].z = z;
		pcl.points[index].intensity = intensity;
		pcl.points[index].range = std::sqrt(x * x + y * y + z * z);
		index++;
	}
}

void PCLUtils::convertToPointXYZIR(const sensor_msgs::msg::PointCloud2 &pc2,
          pcl::PointCloud<pcl::PointXYZIR> &pcl)
{
	pcl.header = pcl_conversions::toPCL(pc2.header);
	pcl.resize(pc2.height * pc2.width);
	pcl.height = pc2.height;
	pcl.width = pc2.width;
	int index = 0;
	for (sensor_msgs::PointCloud2ConstIterator<float> it(pc2, "x"); it != it.end(); ++it) {
		float x = it[0];
		float y = it[1];
		float z = it[2];
		float intensity = it[4];
		pcl.points[index].x = x;
		pcl.points[index].y = y;
		pcl.points[index].z = z;
		pcl.points[index].intensity = intensity;
		pcl.points[index].range = std::sqrt(x * x + y * y + z * z);
		index++;
	}

}
void PCLUtils::convertToPointXYZ(const sensor_msgs::msg::PointCloud2::SharedPtr pc2,
         pcl::PointCloud<pcl::PointXYZ> &pcl)
{
	pcl.header = pcl_conversions::toPCL(pc2->header);
	pcl.resize(pc2->height * pc2->width);
	pcl.height = pc2->height;
	pcl.width = pc2->width;
	int index = 0;
	for (sensor_msgs::PointCloud2ConstIterator<float> it(*pc2, "x"); it != it.end(); ++it) {
		float x = it[0];
		float y = it[1];
		float z = it[2];
		float intensity = it[4];
		pcl.points[index].x = x;
		pcl.points[index].y = y;
		pcl.points[index].z = z;
		index++;
	}
}

void PCLUtils::convertToPointXYZ(const sensor_msgs::msg::PointCloud2 &pc2,
           pcl::PointCloud<pcl::PointXYZ> &pcl)
{

	pcl.header = pcl_conversions::toPCL(pc2.header);
	pcl.resize(pc2.height * pc2.width);
	pcl.height = pc2.height;
	pcl.width = pc2.width;
	int index = 0;
	for (sensor_msgs::PointCloud2ConstIterator<float> it(pc2, "x"); it != it.end(); ++it) {
		float x = it[0];
		float y = it[1];
		float z = it[2];
		float intensity = it[4];
		pcl.points[index].x = x;
		pcl.points[index].y = y;
		pcl.points[index].z = z;
		index++;
	}
}


bool PCLUtils::convertToPointXYZ(const sensor_msgs::msg::Image::ConstSharedPtr &image, const sensor_msgs::msg::CameraInfo::SharedPtr &cam_info,
            pcl::PointCloud<pcl::PointXYZ> &cloud, geometry_msgs::msg::TransformStamped &transform, std::string &error_msg,
            double downsample_factor, double depth_factor, double min_distance, double max_distance)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch(cv_bridge::Exception& e)
	{
		error_msg = std::string(e.what());
		return false;
	}


	cv::Mat frame = cv_ptr->image;

	//get camera intrinsics
	float fx = cam_info->k[0];
	float fy = cam_info->k[4];
	float cx = cam_info->k[2];
	float cy = cam_info->k[5];

	pcl::PointXYZ bad_point;
	ros2::PCLUtils::setNaN(bad_point);

	//setup cloud
	cloud.width = image->width/downsample_factor; //frame.cols
	cloud.height = image->height/downsample_factor; //frame.rows

	//RCLCPP_INFO(node_->get_logger(), "cloud[%d, %d]", cloud.width, cloud.height);

	cloud.points.resize(cloud.width * cloud.height);
	int cnt = -1;
	for(int r = 0; r < frame.rows; r += downsample_factor)
	{
		for(int c = 0; c < frame.cols; c += downsample_factor)
		{

			cnt++;

			//RCLCPP_INFO(node_->get_logger(), "r:%d c:%d, cnt: %d, cloud.points: %d", r,c, cnt, (int)cloud.points.size());
			int idx = r * frame.cols + c;

			float z = frame.at<float>(r,c) / depth_factor;
			//ROS_INFO("z: %f vs raw_z: %f", z, frame.at<float>(r,c));

			if (z == 0 || std::isnan(z) || z < min_distance || z > max_distance)
			{
				cloud.points[cnt] = bad_point;
				continue;
			}

			pcl::PointXYZ p;
			p.z = z;
			p.x = (c - cx) * z / fx;
			p.y = (r - cy) * z / fy;

			//ROS_INFO("pre tf P[%f, %f, %f]", p.x, p.y, p.z);

			ros2::PCLUtils::doTransform(p, p,transform);
			cloud.points[cnt] = p;
		}
	}


	return true;
}

void PCLUtils::toPointCloud2(const pcl::PointCloud<pcl::PointXYZ> &cloud, sensor_msgs::msg::PointCloud2 &cloud_out)
{
	pcl::toROSMsg<pcl::PointXYZ>(cloud, cloud_out);
}

void PCLUtils::toPointCloud2(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, sensor_msgs::msg::PointCloud2 &cloud_out)
{
	pcl::toROSMsg<pcl::PointXYZRGB>(cloud, cloud_out);
}

void PCLUtils::toPointCloud2(const pcl::PointCloud<pcl::PointXYZIR> &cloud, sensor_msgs::msg::PointCloud2 &cloud_out)
{
	pcl::toROSMsg<pcl::PointXYZIR>(cloud, cloud_out);
}

void PCLUtils::toPointCloud2(const pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud,
		  const std::vector<int> &valid_indices, sensor_msgs::msg::PointCloud2 &cloud_out)
{
	pcl::PointCloud<pcl::PointXYZIR> cloud_in;
	cloud_in.points.resize(cloud->size());

	for(int idx : valid_indices)
	{
		cloud_in.points[idx] = cloud->points[idx];
	}
	pcl::toROSMsg<pcl::PointXYZIR>(cloud_in, cloud_out);
}

void PCLUtils::toPointCloud2(const pcl::PointCloud<pcl::PointXYZIR> &cloud,
		  const std::vector<int> &valid_indices, sensor_msgs::msg::PointCloud2 &cloud_out)
{
	pcl::PointCloud<pcl::PointXYZIR> cloud_in;
	cloud_in.points.resize(cloud.size());

	for(int idx : valid_indices)
	{
		cloud_in.points[idx] = cloud.points[idx];
	}
	pcl::toROSMsg<pcl::PointXYZIR>(cloud_in, cloud_out);
}

bool PCLUtils::isSetToNaN(pcl::PointXYZIR &point)
{
  if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
      std::isnan(point.intensity) || std::isnan(point.intensity))
  {
    setNaN(point);
    return true;
  }
  return false;
}

void PCLUtils::setNaN(pcl::PointXYZIR &point)
{
  point.x = std::nanf("");
  point.y = std::nanf("");
  point.z = std::nanf("");
  point.intensity = std::nanf("");
  point.range = std::nanf("");
}

bool PCLUtils::isSetToNaN(pcl::PointXYZ &point)
{
  if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
  {
    setNaN(point);
    return true;
  }
  return false;
}

bool PCLUtils::isSetToNaN(pcl::PointXYZRGB &point)
{
  if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
  {
    setNaN(point);
    return true;
  }
  return false;
}

void PCLUtils::setNaN(pcl::PointXYZ &point)
{
  point.x = std::nanf("");
  point.y = std::nanf("");
  point.z = std::nanf("");
}

void PCLUtils::doTransform(const pcl::PointXYZ & t_in, pcl::PointXYZ & t_out, const geometry_msgs::msg::TransformStamped & transform)
{
	KDL::Vector v_out = tf2::gmTransformToKDL(transform) * KDL::Vector(t_in.x, t_in.y, t_in.z);
	t_out.x = v_out[0];
	t_out.y = v_out[1];
	t_out.z = v_out[2];
}

void PCLUtils::doTransform(const pcl::PointXYZIR & t_in, pcl::PointXYZIR & t_out, const geometry_msgs::msg::TransformStamped & transform)
{
	KDL::Vector v_out = tf2::gmTransformToKDL(transform) * KDL::Vector(t_in.x, t_in.y, t_in.z);
	t_out.x = v_out[0];
	t_out.y = v_out[1];
	t_out.z = v_out[2];
}
void PCLUtils::setNaN(pcl::PointXYZRGB &point)
{
  point.x = std::nanf("");
  point.y = std::nanf("");
  point.z = std::nanf("");
}

} // namespace