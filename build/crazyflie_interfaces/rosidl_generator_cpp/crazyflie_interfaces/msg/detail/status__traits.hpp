// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crazyflie_interfaces:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__STATUS__TRAITS_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crazyflie_interfaces/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace crazyflie_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Status & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: supervisor_info
  {
    out << "supervisor_info: ";
    rosidl_generator_traits::value_to_yaml(msg.supervisor_info, out);
    out << ", ";
  }

  // member: battery_voltage
  {
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << ", ";
  }

  // member: pm_state
  {
    out << "pm_state: ";
    rosidl_generator_traits::value_to_yaml(msg.pm_state, out);
    out << ", ";
  }

  // member: rssi
  {
    out << "rssi: ";
    rosidl_generator_traits::value_to_yaml(msg.rssi, out);
    out << ", ";
  }

  // member: num_rx_broadcast
  {
    out << "num_rx_broadcast: ";
    rosidl_generator_traits::value_to_yaml(msg.num_rx_broadcast, out);
    out << ", ";
  }

  // member: num_tx_broadcast
  {
    out << "num_tx_broadcast: ";
    rosidl_generator_traits::value_to_yaml(msg.num_tx_broadcast, out);
    out << ", ";
  }

  // member: num_rx_unicast
  {
    out << "num_rx_unicast: ";
    rosidl_generator_traits::value_to_yaml(msg.num_rx_unicast, out);
    out << ", ";
  }

  // member: num_tx_unicast
  {
    out << "num_tx_unicast: ";
    rosidl_generator_traits::value_to_yaml(msg.num_tx_unicast, out);
    out << ", ";
  }

  // member: latency_unicast
  {
    out << "latency_unicast: ";
    rosidl_generator_traits::value_to_yaml(msg.latency_unicast, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Status & msg,
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

  // member: supervisor_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "supervisor_info: ";
    rosidl_generator_traits::value_to_yaml(msg.supervisor_info, out);
    out << "\n";
  }

  // member: battery_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << "\n";
  }

  // member: pm_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pm_state: ";
    rosidl_generator_traits::value_to_yaml(msg.pm_state, out);
    out << "\n";
  }

  // member: rssi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rssi: ";
    rosidl_generator_traits::value_to_yaml(msg.rssi, out);
    out << "\n";
  }

  // member: num_rx_broadcast
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_rx_broadcast: ";
    rosidl_generator_traits::value_to_yaml(msg.num_rx_broadcast, out);
    out << "\n";
  }

  // member: num_tx_broadcast
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_tx_broadcast: ";
    rosidl_generator_traits::value_to_yaml(msg.num_tx_broadcast, out);
    out << "\n";
  }

  // member: num_rx_unicast
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_rx_unicast: ";
    rosidl_generator_traits::value_to_yaml(msg.num_rx_unicast, out);
    out << "\n";
  }

  // member: num_tx_unicast
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_tx_unicast: ";
    rosidl_generator_traits::value_to_yaml(msg.num_tx_unicast, out);
    out << "\n";
  }

  // member: latency_unicast
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latency_unicast: ";
    rosidl_generator_traits::value_to_yaml(msg.latency_unicast, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Status & msg, bool use_flow_style = false)
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
  const crazyflie_interfaces::msg::Status & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::msg::Status & msg)
{
  return crazyflie_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::msg::Status>()
{
  return "crazyflie_interfaces::msg::Status";
}

template<>
inline const char * name<crazyflie_interfaces::msg::Status>()
{
  return "crazyflie_interfaces/msg/Status";
}

template<>
struct has_fixed_size<crazyflie_interfaces::msg::Status>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<crazyflie_interfaces::msg::Status>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<crazyflie_interfaces::msg::Status>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__STATUS__TRAITS_HPP_
