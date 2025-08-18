// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crazyflie_interfaces:msg/FullState.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__FULL_STATE__TRAITS_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__FULL_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crazyflie_interfaces/msg/detail/full_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'acc'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace crazyflie_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const FullState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: twist
  {
    out << "twist: ";
    to_flow_style_yaml(msg.twist, out);
    out << ", ";
  }

  // member: acc
  {
    out << "acc: ";
    to_flow_style_yaml(msg.acc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FullState & msg,
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

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "twist:\n";
    to_block_style_yaml(msg.twist, out, indentation + 2);
  }

  // member: acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc:\n";
    to_block_style_yaml(msg.acc, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FullState & msg, bool use_flow_style = false)
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
  const crazyflie_interfaces::msg::FullState & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::msg::FullState & msg)
{
  return crazyflie_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::msg::FullState>()
{
  return "crazyflie_interfaces::msg::FullState";
}

template<>
inline const char * name<crazyflie_interfaces::msg::FullState>()
{
  return "crazyflie_interfaces/msg/FullState";
}

template<>
struct has_fixed_size<crazyflie_interfaces::msg::FullState>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<geometry_msgs::msg::Twist>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<crazyflie_interfaces::msg::FullState>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<geometry_msgs::msg::Twist>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<crazyflie_interfaces::msg::FullState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__FULL_STATE__TRAITS_HPP_
