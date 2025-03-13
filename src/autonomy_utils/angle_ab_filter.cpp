#include <autonomy_utils/lib/angle_ab_filter.h>
#include <autonomy_utils/lib/geometry.h>
#include <autonomy_utils/lib/angles.h>

namespace ros2
{

AngleAlphaBetaFilter::AngleAlphaBetaFilter(double weight, double init_r, double init_p, double init_y)
{
	weight_ = weight;
	init_roll_ = init_r;
	init_pitch_ = init_p;
	init_yaw_ = init_y;
}

AngleAlphaBetaFilter::~AngleAlphaBetaFilter()
{

}

void AngleAlphaBetaFilter::initialize(const geometry_msgs::msg::Quaternion &q)
{
	ros2::Rotate::getRPY(q, init_roll_, init_pitch_, init_yaw_);
}

void AngleAlphaBetaFilter::initialize(double r, double p, double y)
{
	init_roll_ = r;
	init_pitch_ = p;
	init_yaw_ = y;
	data_is_set_ = false;
}

bool AngleAlphaBetaFilter::filter(geometry_msgs::msg::Quaternion &q)
{

	double yaw = 0.0, roll = 0.0, pitch = 0.0;

	ros2::Rotate::getRPY(q, roll, pitch, yaw);

	double new_yaw;
	bool result = updateIfOk(yaw, new_yaw);

	q = ros2::Rotate::toQuaternionMsg(roll, pitch, new_yaw);

	return result;
}

bool AngleAlphaBetaFilter::updateIfOk(double new_angle, double &filtered_angle)
{
        double na = ros2::wrap_pi(new_angle);
	if(!data_is_set_)
	{
		data_is_set_ = true;
		old_angle_ = na;
		filtered_angle = old_angle_;
		return false;
	}

	double angle_diff = ros2::Angles::getDiffSigned(old_angle_, new_angle);
	filtered_angle = ros2::wrap_pi(old_angle_ + angle_diff * weight_);
        old_angle_ = filtered_angle;
	return true;
}

double AngleAlphaBetaFilter::filter(double new_angle)
{
	double na = ros2::wrap_pi(new_angle);
	if(!data_is_set_)
	{
		data_is_set_ = true;
		old_angle_ = na;
		return old_angle_;
	}

        double angle_diff = ros2::Angles::getDiffSigned(old_angle_, new_angle);
        double filtered_angle = ros2::wrap_pi(old_angle_ + angle_diff * weight_);
        old_angle_ = filtered_angle;
	return filtered_angle;
}

}