#ifndef AUTONOMY_UTILS_INCLUDE_LIBS_ROS2_PARAMS_H_
#define AUTONOMY_UTILS_INCLUDE_LIBS_ROS2_PARAMS_H_

#include <autonomy_utils/lib/ros2_nodes.h>

namespace ros2
{
	/**
	 * @brief ROS 2 requires parameter declaration to:
	 * ✔ Prevent undeclared parameters from being used
	 * ✔ Enforce type safety
	 * ✔ Enable runtime parameter changes
	 * ✔ Improve debugging
	 * ✔ Enhance security
	 * declare_parameter function is a template function. this function is called by all instances of getParam
	 * 	and therefore, you do not need to call this function
	 * @param param_name a const string for param name
	 * @param param a const param value
	 * @return void
	 */
	template <class T>
	void declare_parameter(const std::string &param_name, const T &param)
	{
		try
		{
			ros2::Node::node->declare_parameter(param_name, param);
		}
		catch (rclcpp::exceptions::ParameterAlreadyDeclaredException &e)
		{
			;
		}
	}
	// get parameter for vector of integers
	void getParam(const std::string &param_name, std::vector<int> &param, const std::vector<int> &initial_value);
	bool getParam(const std::string &param_name, std::vector<int> &param);

	// get parameter for vector of bytes
	void getParam(const std::string &param_name, std::vector<uint8_t> &param, const std::vector<uint8_t> &initial_value);
	bool getParam(const std::string &param_name, std::vector<uint8_t> &param);

	// get parameter for vector of booleans
	void getParam(const std::string &param_name, std::vector<bool> &param, const std::vector<bool> &initial_value);
	bool getParam(const std::string &param_name, std::vector<bool> &param);

	// get parameter for vector of doubles
	void getParam(const std::string &param_name, std::vector<double> &param, const std::vector<double> &initial_value);
	bool getParam(const std::string &param_name, std::vector<double> &param);

	// get parameter for vector of strings
	void getParam(const std::string &param_name, std::vector<std::string> &param, const std::vector<std::string> &initial_value);
	bool getParam(const std::string &param_name, std::vector<std::string> &param);

	// get parameters for all basic datatypes except arrays
	template <class T>
	bool getParam(const std::string &param_name, T &param, const T &initial_value)
	{
		ros2::declare_parameter(param_name, initial_value);
		return ros2::Node::node->get_parameter_or(param_name, param, initial_value);
	}
	template <class T>
	bool getParam(const std::string &param_name, T &param)
	{
		return ros2::Node::node->get_parameter_or(param_name, param);
	}

	// Compatible with getParam with different method name
	template <class T>
	void getParameter(const std::string &param_name, T &param, const T &initial_value)
	{
		ros2::getParam(param_name, param, initial_value);
	}
	template <class T>
	bool getParameter(const std::string &param_name, T &param)
	{
		return ros2::getParam(param_name, param);
	}

}

#endif /*AUTONOMY_UTILS_INCLUDE_LIBS_ROS2_PARAMS_H_*/