// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crazyflie_interfaces:srv/Land.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__LAND__TRAITS_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__LAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crazyflie_interfaces/srv/detail/land__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'duration'
#include "builtin_interfaces/msg/detail/duration__traits.hpp"

namespace crazyflie_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Land_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: group_mask
  {
    out << "group_mask: ";
    rosidl_generator_traits::value_to_yaml(msg.group_mask, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: duration
  {
    out << "duration: ";
    to_flow_style_yaml(msg.duration, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Land_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: group_mask
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "group_mask: ";
    rosidl_generator_traits::value_to_yaml(msg.group_mask, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duration:\n";
    to_block_style_yaml(msg.duration, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Land_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace crazyflie_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use crazyflie_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const crazyflie_interfaces::srv::Land_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::srv::Land_Request & msg)
{
  return crazyflie_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::srv::Land_Request>()
{
  return "crazyflie_interfaces::srv::Land_Request";
}

template<>
inline const char * name<crazyflie_interfaces::srv::Land_Request>()
{
  return "crazyflie_interfaces/srv/Land_Request";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::Land_Request>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Duration>::value> {};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::Land_Request>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Duration>::value> {};

template<>
struct is_message<crazyflie_interfaces::srv::Land_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace crazyflie_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Land_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Land_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Land_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace crazyflie_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use crazyflie_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const crazyflie_interfaces::srv::Land_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::srv::Land_Response & msg)
{
  return crazyflie_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::srv::Land_Response>()
{
  return "crazyflie_interfaces::srv::Land_Response";
}

template<>
inline const char * name<crazyflie_interfaces::srv::Land_Response>()
{
  return "crazyflie_interfaces/srv/Land_Response";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::Land_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::Land_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<crazyflie_interfaces::srv::Land_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<crazyflie_interfaces::srv::Land>()
{
  return "crazyflie_interfaces::srv::Land";
}

template<>
inline const char * name<crazyflie_interfaces::srv::Land>()
{
  return "crazyflie_interfaces/srv/Land";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::Land>
  : std::integral_constant<
    bool,
    has_fixed_size<crazyflie_interfaces::srv::Land_Request>::value &&
    has_fixed_size<crazyflie_interfaces::srv::Land_Response>::value
  >
{
};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::Land>
  : std::integral_constant<
    bool,
    has_bounded_size<crazyflie_interfaces::srv::Land_Request>::value &&
    has_bounded_size<crazyflie_interfaces::srv::Land_Response>::value
  >
{
};

template<>
struct is_service<crazyflie_interfaces::srv::Land>
  : std::true_type
{
};

template<>
struct is_service_request<crazyflie_interfaces::srv::Land_Request>
  : std::true_type
{
};

template<>
struct is_service_response<crazyflie_interfaces::srv::Land_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__LAND__TRAITS_HPP_
