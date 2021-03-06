#ifndef PARAMS_H
#define PARAMS_H

#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fog_lib
{

  template <class T>
  bool parse_param(const std::string &param_name, T &param_dest, rclcpp::Node& node)
  {
#ifdef ROS_FOXY
    node.declare_parameter(param_name); // for Foxy
#else
    node.declare_parameter<T>(param_name); // for Galactic and newer
#endif
    if (!node.get_parameter(param_name, param_dest))
    {
      RCLCPP_ERROR(node.get_logger(), "Could not load param '%s'", param_name.c_str());
      return false;
    }
    else
    {
      RCLCPP_INFO_STREAM(node.get_logger(), "Loaded '" << param_name << "' = '" << param_dest << "'");
    }
    return true;
  }

  bool parse_param(const std::string& param_name, rclcpp::Duration& param_dest, rclcpp::Node& node)
  {
    using T = double;
#ifdef ROS_FOXY
    node.declare_parameter(param_name); // for Foxy
#else
    node.declare_parameter<T>(param_name); // for Galactic and newer
#endif
    T tmp;
    if (!node.get_parameter(param_name, tmp))
    {
      RCLCPP_ERROR(node.get_logger(), "Could not load param '%s'", param_name.c_str());
      return false;
    }
    else
    {
      param_dest = rclcpp::Duration::from_seconds(tmp);
      RCLCPP_INFO_STREAM(node.get_logger(), "Loaded '" << param_name << "' = '" << tmp << "s'");
    }
    return true;
  }

}

#endif // PARAMS_H
