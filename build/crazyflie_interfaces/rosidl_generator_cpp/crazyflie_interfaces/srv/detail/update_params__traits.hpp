// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crazyflie_interfaces:srv/UpdateParams.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__UPDATE_PARAMS__TRAITS_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__UPDATE_PARAMS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crazyflie_interfaces/srv/detail/update_params__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace crazyflie_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const UpdateParams_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: params
  {
    if (msg.params.size() == 0) {
      out << "params: []";
    } else {
      out << "params: [";
      size_t pending_items = msg.params.size();
      for (auto item : msg.params) {
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
  const UpdateParams_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: params
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.params.size() == 0) {
      out << "params: []\n";
    } else {
      out << "params:\n";
      for (auto item : msg.params) {
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

inline std::string to_yaml(const UpdateParams_Request & msg, bool use_flow_style = false)
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
  const crazyflie_interfaces::srv::UpdateParams_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::srv::UpdateParams_Request & msg)
{
  return crazyflie_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::srv::UpdateParams_Request>()
{
  return "crazyflie_interfaces::srv::UpdateParams_Request";
}

template<>
inline const char * name<crazyflie_interfaces::srv::UpdateParams_Request>()
{
  return "crazyflie_interfaces/srv/UpdateParams_Request";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::UpdateParams_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::UpdateParams_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<crazyflie_interfaces::srv::UpdateParams_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace crazyflie_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const UpdateParams_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UpdateParams_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UpdateParams_Response & msg, bool use_flow_style = false)
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
  const crazyflie_interfaces::srv::UpdateParams_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::srv::UpdateParams_Response & msg)
{
  return crazyflie_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::srv::UpdateParams_Response>()
{
  return "crazyflie_interfaces::srv::UpdateParams_Response";
}

template<>
inline const char * name<crazyflie_interfaces::srv::UpdateParams_Response>()
{
  return "crazyflie_interfaces/srv/UpdateParams_Response";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::UpdateParams_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::UpdateParams_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<crazyflie_interfaces::srv::UpdateParams_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<crazyflie_interfaces::srv::UpdateParams>()
{
  return "crazyflie_interfaces::srv::UpdateParams";
}

template<>
inline const char * name<crazyflie_interfaces::srv::UpdateParams>()
{
  return "crazyflie_interfaces/srv/UpdateParams";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::UpdateParams>
  : std::integral_constant<
    bool,
    has_fixed_size<crazyflie_interfaces::srv::UpdateParams_Request>::value &&
    has_fixed_size<crazyflie_interfaces::srv::UpdateParams_Response>::value
  >
{
};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::UpdateParams>
  : std::integral_constant<
    bool,
    has_bounded_size<crazyflie_interfaces::srv::UpdateParams_Request>::value &&
    has_bounded_size<crazyflie_interfaces::srv::UpdateParams_Response>::value
  >
{
};

template<>
struct is_service<crazyflie_interfaces::srv::UpdateParams>
  : std::true_type
{
};

template<>
struct is_service_request<crazyflie_interfaces::srv::UpdateParams_Request>
  : std::true_type
{
};

template<>
struct is_service_response<crazyflie_interfaces::srv::UpdateParams_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__UPDATE_PARAMS__TRAITS_HPP_
