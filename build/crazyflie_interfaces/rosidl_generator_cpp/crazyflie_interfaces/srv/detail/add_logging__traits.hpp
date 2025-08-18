// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crazyflie_interfaces:srv/AddLogging.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__ADD_LOGGING__TRAITS_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__ADD_LOGGING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crazyflie_interfaces/srv/detail/add_logging__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace crazyflie_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const AddLogging_Request & msg,
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

  // member: vars
  {
    if (msg.vars.size() == 0) {
      out << "vars: []";
    } else {
      out << "vars: [";
      size_t pending_items = msg.vars.size();
      for (auto item : msg.vars) {
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
  const AddLogging_Request & msg,
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

  // member: vars
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vars.size() == 0) {
      out << "vars: []\n";
    } else {
      out << "vars:\n";
      for (auto item : msg.vars) {
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

inline std::string to_yaml(const AddLogging_Request & msg, bool use_flow_style = false)
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
  const crazyflie_interfaces::srv::AddLogging_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::srv::AddLogging_Request & msg)
{
  return crazyflie_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::srv::AddLogging_Request>()
{
  return "crazyflie_interfaces::srv::AddLogging_Request";
}

template<>
inline const char * name<crazyflie_interfaces::srv::AddLogging_Request>()
{
  return "crazyflie_interfaces/srv/AddLogging_Request";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::AddLogging_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::AddLogging_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<crazyflie_interfaces::srv::AddLogging_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace crazyflie_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const AddLogging_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AddLogging_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AddLogging_Response & msg, bool use_flow_style = false)
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
  const crazyflie_interfaces::srv::AddLogging_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::srv::AddLogging_Response & msg)
{
  return crazyflie_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::srv::AddLogging_Response>()
{
  return "crazyflie_interfaces::srv::AddLogging_Response";
}

template<>
inline const char * name<crazyflie_interfaces::srv::AddLogging_Response>()
{
  return "crazyflie_interfaces/srv/AddLogging_Response";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::AddLogging_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::AddLogging_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<crazyflie_interfaces::srv::AddLogging_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<crazyflie_interfaces::srv::AddLogging>()
{
  return "crazyflie_interfaces::srv::AddLogging";
}

template<>
inline const char * name<crazyflie_interfaces::srv::AddLogging>()
{
  return "crazyflie_interfaces/srv/AddLogging";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::AddLogging>
  : std::integral_constant<
    bool,
    has_fixed_size<crazyflie_interfaces::srv::AddLogging_Request>::value &&
    has_fixed_size<crazyflie_interfaces::srv::AddLogging_Response>::value
  >
{
};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::AddLogging>
  : std::integral_constant<
    bool,
    has_bounded_size<crazyflie_interfaces::srv::AddLogging_Request>::value &&
    has_bounded_size<crazyflie_interfaces::srv::AddLogging_Response>::value
  >
{
};

template<>
struct is_service<crazyflie_interfaces::srv::AddLogging>
  : std::true_type
{
};

template<>
struct is_service_request<crazyflie_interfaces::srv::AddLogging_Request>
  : std::true_type
{
};

template<>
struct is_service_response<crazyflie_interfaces::srv::AddLogging_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__ADD_LOGGING__TRAITS_HPP_
