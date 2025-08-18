// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crazyflie_interfaces:srv/StartTrajectory.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__START_TRAJECTORY__TRAITS_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__START_TRAJECTORY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crazyflie_interfaces/srv/detail/start_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace crazyflie_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const StartTrajectory_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: group_mask
  {
    out << "group_mask: ";
    rosidl_generator_traits::value_to_yaml(msg.group_mask, out);
    out << ", ";
  }

  // member: trajectory_id
  {
    out << "trajectory_id: ";
    rosidl_generator_traits::value_to_yaml(msg.trajectory_id, out);
    out << ", ";
  }

  // member: timescale
  {
    out << "timescale: ";
    rosidl_generator_traits::value_to_yaml(msg.timescale, out);
    out << ", ";
  }

  // member: reversed
  {
    out << "reversed: ";
    rosidl_generator_traits::value_to_yaml(msg.reversed, out);
    out << ", ";
  }

  // member: relative
  {
    out << "relative: ";
    rosidl_generator_traits::value_to_yaml(msg.relative, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StartTrajectory_Request & msg,
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

  // member: trajectory_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trajectory_id: ";
    rosidl_generator_traits::value_to_yaml(msg.trajectory_id, out);
    out << "\n";
  }

  // member: timescale
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timescale: ";
    rosidl_generator_traits::value_to_yaml(msg.timescale, out);
    out << "\n";
  }

  // member: reversed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reversed: ";
    rosidl_generator_traits::value_to_yaml(msg.reversed, out);
    out << "\n";
  }

  // member: relative
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "relative: ";
    rosidl_generator_traits::value_to_yaml(msg.relative, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StartTrajectory_Request & msg, bool use_flow_style = false)
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
  const crazyflie_interfaces::srv::StartTrajectory_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::srv::StartTrajectory_Request & msg)
{
  return crazyflie_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::srv::StartTrajectory_Request>()
{
  return "crazyflie_interfaces::srv::StartTrajectory_Request";
}

template<>
inline const char * name<crazyflie_interfaces::srv::StartTrajectory_Request>()
{
  return "crazyflie_interfaces/srv/StartTrajectory_Request";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::StartTrajectory_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::StartTrajectory_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<crazyflie_interfaces::srv::StartTrajectory_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace crazyflie_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const StartTrajectory_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StartTrajectory_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StartTrajectory_Response & msg, bool use_flow_style = false)
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
  const crazyflie_interfaces::srv::StartTrajectory_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::srv::StartTrajectory_Response & msg)
{
  return crazyflie_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::srv::StartTrajectory_Response>()
{
  return "crazyflie_interfaces::srv::StartTrajectory_Response";
}

template<>
inline const char * name<crazyflie_interfaces::srv::StartTrajectory_Response>()
{
  return "crazyflie_interfaces/srv/StartTrajectory_Response";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::StartTrajectory_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::StartTrajectory_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<crazyflie_interfaces::srv::StartTrajectory_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<crazyflie_interfaces::srv::StartTrajectory>()
{
  return "crazyflie_interfaces::srv::StartTrajectory";
}

template<>
inline const char * name<crazyflie_interfaces::srv::StartTrajectory>()
{
  return "crazyflie_interfaces/srv/StartTrajectory";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::StartTrajectory>
  : std::integral_constant<
    bool,
    has_fixed_size<crazyflie_interfaces::srv::StartTrajectory_Request>::value &&
    has_fixed_size<crazyflie_interfaces::srv::StartTrajectory_Response>::value
  >
{
};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::StartTrajectory>
  : std::integral_constant<
    bool,
    has_bounded_size<crazyflie_interfaces::srv::StartTrajectory_Request>::value &&
    has_bounded_size<crazyflie_interfaces::srv::StartTrajectory_Response>::value
  >
{
};

template<>
struct is_service<crazyflie_interfaces::srv::StartTrajectory>
  : std::true_type
{
};

template<>
struct is_service_request<crazyflie_interfaces::srv::StartTrajectory_Request>
  : std::true_type
{
};

template<>
struct is_service_response<crazyflie_interfaces::srv::StartTrajectory_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__START_TRAJECTORY__TRAITS_HPP_
