#include <autonomy_utils/lib/ros2_params.hpp>

namespace ros2
{
	void getParam(const std::string &param_name, std::vector<int> &param, const std::vector<int> &initial_value)
	{
		ros2::declare_parameter<std::vector<int>>(param_name, initial_value);

		rclcpp::Parameter temp = ros2::Node::node->get_parameter(param_name);

		ros2::Node::node->get_parameter(param_name, temp);

		std::vector<int64_t> res = temp.as_integer_array();

		for (const int64_t &i : res)
		{
			param.push_back(static_cast<int>(i));
		}
	}

	bool getParam(const std::string &param_name, std::vector<int> &param)
	{
		rclcpp::Parameter temp;

		if (!ros2::Node::node->get_parameter(param_name, temp))
		{
			return false;
		}

		std::vector<int64_t> res = temp.as_integer_array();

		for (const int64_t &i : res)
		{
			param.push_back(static_cast<int>(i));
		}

		return true;
	}

	void getParam(const std::string &param_name, std::vector<uint8_t> &param, const std::vector<uint8_t> &initial_value)
	{
		ros2::declare_parameter<std::vector<uint8_t>>(param_name, initial_value);

		rclcpp::Parameter temp = ros2::Node::node->get_parameter(param_name);

		param = temp.as_byte_array();
	}

	bool getParam(const std::string &param_name, std::vector<uint8_t> &param)
	{
		rclcpp::Parameter temp;
		if (!ros2::Node::node->get_parameter(param_name, temp))
		{
			return false;
		}

		param = temp.as_byte_array();
		return true;
	}

	void getParam(const std::string &param_name, std::vector<bool> &param, const std::vector<bool> &initial_value)
	{
		ros2::declare_parameter<std::vector<bool>>(param_name, initial_value);

		rclcpp::Parameter temp = ros2::Node::node->get_parameter(param_name);

		param = temp.as_bool_array();
	}

	bool getParam(const std::string &param_name, std::vector<bool> &param)
	{
		rclcpp::Parameter temp;

		if (!ros2::Node::node->get_parameter(param_name, temp))
		{
			return false;
		}

		param = temp.as_bool_array();
		return true;
	}

	void getParam(const std::string &param_name, std::vector<double> &param, const std::vector<double> &initial_value)
	{
		ros2::declare_parameter<std::vector<double>>(param_name, initial_value);

		rclcpp::Parameter temp = ros2::Node::node->get_parameter(param_name);

		param = temp.as_double_array();
	}

	bool getParam(const std::string &param_name, std::vector<double> &param)
	{
		rclcpp::Parameter temp;
		if (!ros2::Node::node->get_parameter(param_name, temp))
		{
			return false;
		}

		param = temp.as_double_array();
		return true;
	}

	void getParam(const std::string &param_name, std::vector<std::string> &param, const std::vector<std::string> &initial_value)
	{
		ros2::declare_parameter<std::vector<std::string>>(param_name, initial_value);

		rclcpp::Parameter temp = ros2::Node::node->get_parameter(param_name);

		param = temp.as_string_array();
	}

	bool getParam(const std::string &param_name, std::vector<std::string> &param)
	{
		rclcpp::Parameter temp;
		if (!ros2::Node::node->get_parameter(param_name, temp))
		{
			return false;
		}

		param = temp.as_string_array();
		return true;
	}

}