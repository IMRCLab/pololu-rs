// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crazyflie_interfaces:msg/ConnectionStatistics.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS__TRAITS_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crazyflie_interfaces/msg/detail/connection_statistics__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace crazyflie_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ConnectionStatistics & msg,
  std::ostream & out)
{
  out << "{";
  // member: uri
  {
    out << "uri: ";
    rosidl_generator_traits::value_to_yaml(msg.uri, out);
    out << ", ";
  }

  // member: sent_count
  {
    out << "sent_count: ";
    rosidl_generator_traits::value_to_yaml(msg.sent_count, out);
    out << ", ";
  }

  // member: sent_ping_count
  {
    out << "sent_ping_count: ";
    rosidl_generator_traits::value_to_yaml(msg.sent_ping_count, out);
    out << ", ";
  }

  // member: receive_count
  {
    out << "receive_count: ";
    rosidl_generator_traits::value_to_yaml(msg.receive_count, out);
    out << ", ";
  }

  // member: enqueued_count
  {
    out << "enqueued_count: ";
    rosidl_generator_traits::value_to_yaml(msg.enqueued_count, out);
    out << ", ";
  }

  // member: ack_count
  {
    out << "ack_count: ";
    rosidl_generator_traits::value_to_yaml(msg.ack_count, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ConnectionStatistics & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: uri
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "uri: ";
    rosidl_generator_traits::value_to_yaml(msg.uri, out);
    out << "\n";
  }

  // member: sent_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sent_count: ";
    rosidl_generator_traits::value_to_yaml(msg.sent_count, out);
    out << "\n";
  }

  // member: sent_ping_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sent_ping_count: ";
    rosidl_generator_traits::value_to_yaml(msg.sent_ping_count, out);
    out << "\n";
  }

  // member: receive_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "receive_count: ";
    rosidl_generator_traits::value_to_yaml(msg.receive_count, out);
    out << "\n";
  }

  // member: enqueued_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enqueued_count: ";
    rosidl_generator_traits::value_to_yaml(msg.enqueued_count, out);
    out << "\n";
  }

  // member: ack_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ack_count: ";
    rosidl_generator_traits::value_to_yaml(msg.ack_count, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ConnectionStatistics & msg, bool use_flow_style = false)
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
  const crazyflie_interfaces::msg::ConnectionStatistics & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::msg::ConnectionStatistics & msg)
{
  return crazyflie_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::msg::ConnectionStatistics>()
{
  return "crazyflie_interfaces::msg::ConnectionStatistics";
}

template<>
inline const char * name<crazyflie_interfaces::msg::ConnectionStatistics>()
{
  return "crazyflie_interfaces/msg/ConnectionStatistics";
}

template<>
struct has_fixed_size<crazyflie_interfaces::msg::ConnectionStatistics>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<crazyflie_interfaces::msg::ConnectionStatistics>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<crazyflie_interfaces::msg::ConnectionStatistics>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS__TRAITS_HPP_
