// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motion_capture_tracking_interfaces:msg/NamedPose.idl
// generated code does not contain a copyright notice

#ifndef MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE__TRAITS_HPP_
#define MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "motion_capture_tracking_interfaces/msg/detail/named_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace motion_capture_tracking_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const NamedPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NamedPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NamedPose & msg, bool use_flow_style = false)
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

}  // namespace motion_capture_tracking_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use motion_capture_tracking_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const motion_capture_tracking_interfaces::msg::NamedPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  motion_capture_tracking_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motion_capture_tracking_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const motion_capture_tracking_interfaces::msg::NamedPose & msg)
{
  return motion_capture_tracking_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<motion_capture_tracking_interfaces::msg::NamedPose>()
{
  return "motion_capture_tracking_interfaces::msg::NamedPose";
}

template<>
inline const char * name<motion_capture_tracking_interfaces::msg::NamedPose>()
{
  return "motion_capture_tracking_interfaces/msg/NamedPose";
}

template<>
struct has_fixed_size<motion_capture_tracking_interfaces::msg::NamedPose>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<motion_capture_tracking_interfaces::msg::NamedPose>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<motion_capture_tracking_interfaces::msg::NamedPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE__TRAITS_HPP_
