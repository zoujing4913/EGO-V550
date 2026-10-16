#ifndef PLAN_ENV_PARAM_UTILS_HPP
#define PLAN_ENV_PARAM_UTILS_HPP

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <type_traits>
#include <vector>

namespace plan_env
{

/*
 * ROS 1 wrote `2` and `2.0` interchangeably, and a ROS 1 `getParam()` never
 * complained about the numeric type. In ROS 2 a parameter keeps the type of the
 * YAML value it came from, so declaring a `double` parameter and overriding it
 * with an integer (e.g. `ros2 launch ... map_size_x:=40`) or an integer parameter
 * with a float raises rclcpp::exceptions::InvalidParameterTypeException at
 * startup.
 *
 * getParam() avoids that: the parameter is declared with `dynamic_typing`, so the
 * override keeps whatever type it has, and the stored value is then coerced to the
 * type the C++ code expects. Missing parameters fall back to the default.
 */
template<typename T>
inline T coerceParameterValue(const rclcpp::Parameter & param, const T & default_value)
{
  if constexpr (std::is_same_v<T, double>) {
    switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        return param.as_double();
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return static_cast<double>(param.as_int());
      case rclcpp::ParameterType::PARAMETER_STRING:
        try {
          return std::stod(param.as_string());
        } catch (const std::exception &) {
          return default_value;
        }
      default:
        return default_value;
    }
  } else if constexpr (std::is_same_v<T, int>) {
    switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return static_cast<int>(param.as_int());
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        return static_cast<int>(param.as_double());
      case rclcpp::ParameterType::PARAMETER_STRING:
        try {
          return static_cast<int>(std::stod(param.as_string()));
        } catch (const std::exception &) {
          return default_value;
        }
      default:
        return default_value;
    }
  } else if constexpr (std::is_same_v<T, bool>) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      return param.as_bool();
    }
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      const std::string s = param.as_string();
      return s == "true" || s == "True" || s == "1";
    }
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return param.as_int() != 0;
    }
    return default_value;
  } else if constexpr (std::is_same_v<T, std::string>) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      return param.as_string();
    }
    return default_value;
  } else if constexpr (std::is_same_v<T, std::vector<double>>) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      return param.as_double_array();
    }
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      std::vector<double> out;
      for (const auto v : param.as_integer_array()) {
        out.push_back(static_cast<double>(v));
      }
      return out;
    }
    return default_value;
  } else if constexpr (std::is_same_v<T, std::vector<int>>) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      std::vector<int> out;
      for (const auto v : param.as_integer_array()) {
        out.push_back(static_cast<int>(v));
      }
      return out;
    }
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      std::vector<int> out;
      for (const auto v : param.as_double_array()) {
        out.push_back(static_cast<int>(v));
      }
      return out;
    }
    return default_value;
  } else {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      return default_value;
    }
    try {
      return param.get_value<T>();
    } catch (const std::exception &) {
      return default_value;
    }
  }
}

template<typename T>
inline T getParam(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & default_value)
{
  if (node->has_parameter(name)) {
    return coerceParameterValue<T>(node->get_parameter(name), default_value);
  }
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  rclcpp::ParameterValue value =
    node->declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor);
  return coerceParameterValue<T>(rclcpp::Parameter(name, value), default_value);
}

}  // namespace plan_env

#endif  // PLAN_ENV_PARAM_UTILS_HPP
