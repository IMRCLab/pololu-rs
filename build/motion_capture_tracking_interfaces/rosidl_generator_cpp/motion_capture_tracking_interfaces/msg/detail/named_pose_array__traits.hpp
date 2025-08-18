// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motion_capture_tracking_interfaces:msg/NamedPoseArray.idl
// generated code does not contain a copyright notice

#ifndef MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE_ARRAY__TRAITS_HPP_
#define MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "motion_capture_tracking_interfaces/msg/detail/named_pose_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'poses'
#include "motion_capture_tracking_interfaces/msg/detail/named_pose__traits.hpp"

namespace motion_capture_tracking_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const NamedPoseArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: poses
  {
    if (msg.poses.size() == 0) {
      out << "poses: []";
    } else {
      out << "poses: [";
      size_t pending_items = msg.poses.size();
      for (auto item : msg.poses) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NamedPoseArray & msg,
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

  // member: poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.poses.size() == 0) {
      out << "poses: []\n";
    } else {
      out << "poses:\n";
      for (auto item : msg.poses) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NamedPoseArray & msg, bool use_flow_style = false)
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
  const motion_capture_tracking_interfaces::msg::NamedPoseArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  motion_capture_tracking_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motion_capture_tracking_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const motion_capture_tracking_interfaces::msg::NamedPoseArray & msg)
{
  return motion_capture_tracking_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<motion_capture_tracking_interfaces::msg::NamedPoseArray>()
{
  return "motion_capture_tracking_interfaces::msg::NamedPoseArray";
}

template<>
inline const char * name<motion_capture_tracking_interfaces::msg::NamedPoseArray>()
{
  return "motion_capture_tracking_interfaces/msg/NamedPoseArray";
}

template<>
struct has_fixed_size<motion_capture_tracking_interfaces::msg::NamedPoseArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<motion_capture_tracking_interfaces::msg::NamedPoseArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<motion_capture_tracking_interfaces::msg::NamedPoseArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE_ARRAY__TRAITS_HPP_
