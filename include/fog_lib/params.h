#ifndef PARAMS_H
#define PARAMS_H

#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fog_lib
{

  template <class T>
  bool parse_param(const std::string &param_name, T &param_dest, rclcpp::Node& node)
  {
    try
    {
#ifdef ROS_FOXY
      node.declare_parameter(param_name); // for Foxy
#else
      node.declare_parameter<T>(param_name); // for Galactic and newer
#endif
      node.get_parameter(param_name, param_dest);
      RCLCPP_INFO_STREAM(node.get_logger(), "Loaded '" << param_name << "' = '" << param_dest << "'");
    }
    catch (const rclcpp::ParameterTypeException& e)
    {
      RCLCPP_ERROR_STREAM(node.get_logger(), "Could not load param '" << param_name << "': " << e.what());
      return false;
    }
    return true;
  }

  template <class T>
  bool parse_param(const std::string& param_name, std::vector<T>& param_dest, rclcpp::Node& node)
  {
    try
    {
#ifdef ROS_FOXY
      node.declare_parameter(param_name); // for Foxy
#else
      node.declare_parameter<std::vector<T>>(param_name); // for Galactic and newer
#endif
      node.get_parameter(param_name, param_dest);
      RCLCPP_INFO_STREAM(node.get_logger(), "Loaded '" << param_name << "' = ");
      for (const auto& el : param_dest)
        std::cout << "\t" << el << "\n";
    }
    catch (const rclcpp::ParameterTypeException& e)
    {
      RCLCPP_ERROR_STREAM(node.get_logger(), "Could not load param '" << param_name << "': " << e.what());
      return false;
    }
    return true;
  }

  bool parse_param(const std::string& param_name, rclcpp::Duration& param_dest, rclcpp::Node& node)
  {
    try
    {
      using T = double;
#ifdef ROS_FOXY
      node.declare_parameter(param_name); // for Foxy
#else
      node.declare_parameter<T>(param_name); // for Galactic and newer
#endif
      T tmp;
      node.get_parameter(param_name, tmp);
      param_dest = rclcpp::Duration::from_seconds(tmp);
      RCLCPP_INFO_STREAM(node.get_logger(), "Loaded '" << param_name << "' = " << tmp << "s");
    }
    catch (const rclcpp::ParameterTypeException& e)
    {
      RCLCPP_ERROR_STREAM(node.get_logger(), "Could not load param '" << param_name << "': " << e.what());
      return false;
    }
    return true;
  }

  template <class T>
  T parse_param2(const std::string &param_name, bool& ok_out, rclcpp::Node& node)
  {
    T out;
    ok_out = parse_param(param_name, out, node);
    return out;
  }

  template <>
  rclcpp::Duration parse_param2(const std::string &param_name, bool& ok_out, rclcpp::Node& node)
  {
    rclcpp::Duration out = rclcpp::Duration::from_seconds(0);
    ok_out = parse_param(param_name, out, node);
    return out;
  }

}

#endif // PARAMS_H
