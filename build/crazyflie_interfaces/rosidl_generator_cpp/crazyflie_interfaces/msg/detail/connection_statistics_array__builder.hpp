// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:msg/ConnectionStatisticsArray.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS_ARRAY__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/msg/detail/connection_statistics_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace msg
{

namespace builder
{

class Init_ConnectionStatisticsArray_stats
{
public:
  explicit Init_ConnectionStatisticsArray_stats(::crazyflie_interfaces::msg::ConnectionStatisticsArray & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::msg::ConnectionStatisticsArray stats(::crazyflie_interfaces::msg::ConnectionStatisticsArray::_stats_type arg)
  {
    msg_.stats = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::msg::ConnectionStatisticsArray msg_;
};

class Init_ConnectionStatisticsArray_header
{
public:
  Init_ConnectionStatisticsArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConnectionStatisticsArray_stats header(::crazyflie_interfaces::msg::ConnectionStatisticsArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ConnectionStatisticsArray_stats(msg_);
  }

private:
  ::crazyflie_interfaces::msg::ConnectionStatisticsArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::msg::ConnectionStatisticsArray>()
{
  return crazyflie_interfaces::msg::builder::Init_ConnectionStatisticsArray_header();
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS_ARRAY__BUILDER_HPP_
