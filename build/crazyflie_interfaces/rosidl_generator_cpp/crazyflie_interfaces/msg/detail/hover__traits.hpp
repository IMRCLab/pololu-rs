// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crazyflie_interfaces:msg/Hover.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__HOVER__TRAITS_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__HOVER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crazyflie_interfaces/msg/detail/hover__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace crazyflie_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Hover & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: vx
  {
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << ", ";
  }

  // member: vy
  {
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << ", ";
  }

  // member: yaw_rate
  {
    out << "yaw_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rate, out);
    out << ", ";
  }

  // member: z_distance
  {
    out << "z_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.z_distance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Hover & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: vx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << "\n";
  }

  // member: vy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << "\n";
  }

  // member: yaw_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rate, out);
    out << "\n";
  }

  // member: z_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.z_distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Hover & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace crazyflie_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use crazyflie_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const crazyflie_interfaces::msg::Hover & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::msg::Hover & msg)
{
  return crazyflie_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::msg::Hover>()
{
  return "crazyflie_interfaces::msg::Hover";
}

template<>
inline const char * name<crazyflie_interfaces::msg::Hover>()
{
  return "crazyflie_interfaces/msg/Hover";
}

template<>
struct has_fixed_size<crazyflie_interfaces::msg::Hover>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<crazyflie_interfaces::msg::Hover>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<crazyflie_interfaces::msg::Hover>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__HOVER__TRAITS_HPP_
