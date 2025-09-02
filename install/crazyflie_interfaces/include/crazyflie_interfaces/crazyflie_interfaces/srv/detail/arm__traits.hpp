// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crazyflie_interfaces:srv/Arm.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__ARM__TRAITS_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__ARM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crazyflie_interfaces/srv/detail/arm__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace crazyflie_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Arm_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: arm
  {
    out << "arm: ";
    rosidl_generator_traits::value_to_yaml(msg.arm, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Arm_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: arm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "arm: ";
    rosidl_generator_traits::value_to_yaml(msg.arm, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Arm_Request & msg, bool use_flow_style = false)
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
  const crazyflie_interfaces::srv::Arm_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::srv::Arm_Request & msg)
{
  return crazyflie_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::srv::Arm_Request>()
{
  return "crazyflie_interfaces::srv::Arm_Request";
}

template<>
inline const char * name<crazyflie_interfaces::srv::Arm_Request>()
{
  return "crazyflie_interfaces/srv/Arm_Request";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::Arm_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::Arm_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<crazyflie_interfaces::srv::Arm_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace crazyflie_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Arm_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Arm_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Arm_Response & msg, bool use_flow_style = false)
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
  const crazyflie_interfaces::srv::Arm_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::srv::Arm_Response & msg)
{
  return crazyflie_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::srv::Arm_Response>()
{
  return "crazyflie_interfaces::srv::Arm_Response";
}

template<>
inline const char * name<crazyflie_interfaces::srv::Arm_Response>()
{
  return "crazyflie_interfaces/srv/Arm_Response";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::Arm_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::Arm_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<crazyflie_interfaces::srv::Arm_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<crazyflie_interfaces::srv::Arm>()
{
  return "crazyflie_interfaces::srv::Arm";
}

template<>
inline const char * name<crazyflie_interfaces::srv::Arm>()
{
  return "crazyflie_interfaces/srv/Arm";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::Arm>
  : std::integral_constant<
    bool,
    has_fixed_size<crazyflie_interfaces::srv::Arm_Request>::value &&
    has_fixed_size<crazyflie_interfaces::srv::Arm_Response>::value
  >
{
};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::Arm>
  : std::integral_constant<
    bool,
    has_bounded_size<crazyflie_interfaces::srv::Arm_Request>::value &&
    has_bounded_size<crazyflie_interfaces::srv::Arm_Response>::value
  >
{
};

template<>
struct is_service<crazyflie_interfaces::srv::Arm>
  : std::true_type
{
};

template<>
struct is_service_request<crazyflie_interfaces::srv::Arm_Request>
  : std::true_type
{
};

template<>
struct is_service_response<crazyflie_interfaces::srv::Arm_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__ARM__TRAITS_HPP_
