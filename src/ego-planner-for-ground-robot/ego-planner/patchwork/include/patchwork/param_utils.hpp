#ifndef PATCHWORK_PARAM_UTILS_HPP
#define PATCHWORK_PARAM_UTILS_HPP

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <type_traits>
#include <vector>

namespace patchwork
{

/*
 * ROS 1 parameter files mixed integer and floating point numbers freely (e.g.
 * `sensor_height: 0`). ROS 2 keeps the YAML type, so a `double` parameter that is
 * overridden by an integer would raise rclcpp::exceptions::
 * InvalidParameterTypeException. getParam() declares parameters with dynamic
 * typing and coerces the stored value instead.
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

}  // namespace patchwork

#endif  // PATCHWORK_PARAM_UTILS_HPP
