// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crazyflie_interfaces:msg/LogBlock.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__LOG_BLOCK__TRAITS_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__LOG_BLOCK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crazyflie_interfaces/msg/detail/log_block__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace crazyflie_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const LogBlock & msg,
  std::ostream & out)
{
  out << "{";
  // member: topic_name
  {
    out << "topic_name: ";
    rosidl_generator_traits::value_to_yaml(msg.topic_name, out);
    out << ", ";
  }

  // member: frequency
  {
    out << "frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.frequency, out);
    out << ", ";
  }

  // member: variables
  {
    if (msg.variables.size() == 0) {
      out << "variables: []";
    } else {
      out << "variables: [";
      size_t pending_items = msg.variables.size();
      for (auto item : msg.variables) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const LogBlock & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: topic_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "topic_name: ";
    rosidl_generator_traits::value_to_yaml(msg.topic_name, out);
    out << "\n";
  }

  // member: frequency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.frequency, out);
    out << "\n";
  }

  // member: variables
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.variables.size() == 0) {
      out << "variables: []\n";
    } else {
      out << "variables:\n";
      for (auto item : msg.variables) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LogBlock & msg, bool use_flow_style = false)
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
  const crazyflie_interfaces::msg::LogBlock & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::msg::LogBlock & msg)
{
  return crazyflie_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::msg::LogBlock>()
{
  return "crazyflie_interfaces::msg::LogBlock";
}

template<>
inline const char * name<crazyflie_interfaces::msg::LogBlock>()
{
  return "crazyflie_interfaces/msg/LogBlock";
}

template<>
struct has_fixed_size<crazyflie_interfaces::msg::LogBlock>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<crazyflie_interfaces::msg::LogBlock>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<crazyflie_interfaces::msg::LogBlock>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__LOG_BLOCK__TRAITS_HPP_
