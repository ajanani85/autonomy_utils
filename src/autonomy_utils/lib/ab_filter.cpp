#include <autonomy_utils/lib/ab_filter.h>

namespace ros2
{

AlphaBetaFilter::AlphaBetaFilter()
{
}

AlphaBetaFilter::AlphaBetaFilter(double weight, double init_x, double init_y, double init_z)
{
	weight_ = weight;
	initial_x_ = init_x;
	initial_y_ = init_y;
	initial_z_ = init_z;
}

AlphaBetaFilter::AlphaBetaFilter(double x_weight, double y_weight, double z_weight, double init_x, double init_y, double init_z)
{
	x_weight_ = x_weight;
	y_weight_ = y_weight;
	z_weight_ = z_weight;
	initial_x_ = init_x;
	initial_y_ = init_y;
	initial_z_ = init_z;
}

void AlphaBetaFilter::setInitialPose(double x, double y, double z)
{
	initial_x_ = x;
	initial_y_ = y;
	initial_z_ = z;
	data_is_set_ = false;
}

bool AlphaBetaFilter::filter(double &x, double &y, double &z)
{

	static double old_x = initial_x_;
	static double old_y = initial_y_;
	static double old_z = initial_z_;

	if(!data_is_set_)
	{
		data_is_set_ = true;
		old_x = x;
		return false;
	}
	old_x = x * x_weight_ + old_x * (1.0 - x_weight_);
	x = old_x;

	old_y = y * y_weight_ + old_y * (1.0 - y_weight_);
	y = old_y;

	old_z = z * z_weight_ + old_z * (1.0 - z_weight_);
	z = old_z;

	return true;
}


bool AlphaBetaFilter::filter(nav_msgs::msg::Odometry &odom)
{
	return filter(odom.pose.pose);
}

bool AlphaBetaFilter::filter(geometry_msgs::msg::Pose &pose)
{
	return filter(pose.position.x, pose.position.y, pose.position.z);
}

double AlphaBetaFilter::filter(double new_value)
{
	if(!data_is_set_)
	{
		data_is_set_ = true;
		old_value_ = new_value;
		return old_value_;
	}
	old_value_ = new_value * weight_ + old_value_ * (1.0 - weight_);
	return old_value_;
}

}