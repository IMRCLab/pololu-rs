// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:msg/ConnectionStatistics.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/msg/detail/connection_statistics__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace msg
{

namespace builder
{

class Init_ConnectionStatistics_ack_count
{
public:
  explicit Init_ConnectionStatistics_ack_count(::crazyflie_interfaces::msg::ConnectionStatistics & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::msg::ConnectionStatistics ack_count(::crazyflie_interfaces::msg::ConnectionStatistics::_ack_count_type arg)
  {
    msg_.ack_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::msg::ConnectionStatistics msg_;
};

class Init_ConnectionStatistics_enqueued_count
{
public:
  explicit Init_ConnectionStatistics_enqueued_count(::crazyflie_interfaces::msg::ConnectionStatistics & msg)
  : msg_(msg)
  {}
  Init_ConnectionStatistics_ack_count enqueued_count(::crazyflie_interfaces::msg::ConnectionStatistics::_enqueued_count_type arg)
  {
    msg_.enqueued_count = std::move(arg);
    return Init_ConnectionStatistics_ack_count(msg_);
  }

private:
  ::crazyflie_interfaces::msg::ConnectionStatistics msg_;
};

class Init_ConnectionStatistics_receive_count
{
public:
  explicit Init_ConnectionStatistics_receive_count(::crazyflie_interfaces::msg::ConnectionStatistics & msg)
  : msg_(msg)
  {}
  Init_ConnectionStatistics_enqueued_count receive_count(::crazyflie_interfaces::msg::ConnectionStatistics::_receive_count_type arg)
  {
    msg_.receive_count = std::move(arg);
    return Init_ConnectionStatistics_enqueued_count(msg_);
  }

private:
  ::crazyflie_interfaces::msg::ConnectionStatistics msg_;
};

class Init_ConnectionStatistics_sent_ping_count
{
public:
  explicit Init_ConnectionStatistics_sent_ping_count(::crazyflie_interfaces::msg::ConnectionStatistics & msg)
  : msg_(msg)
  {}
  Init_ConnectionStatistics_receive_count sent_ping_count(::crazyflie_interfaces::msg::ConnectionStatistics::_sent_ping_count_type arg)
  {
    msg_.sent_ping_count = std::move(arg);
    return Init_ConnectionStatistics_receive_count(msg_);
  }

private:
  ::crazyflie_interfaces::msg::ConnectionStatistics msg_;
};

class Init_ConnectionStatistics_sent_count
{
public:
  explicit Init_ConnectionStatistics_sent_count(::crazyflie_interfaces::msg::ConnectionStatistics & msg)
  : msg_(msg)
  {}
  Init_ConnectionStatistics_sent_ping_count sent_count(::crazyflie_interfaces::msg::ConnectionStatistics::_sent_count_type arg)
  {
    msg_.sent_count = std::move(arg);
    return Init_ConnectionStatistics_sent_ping_count(msg_);
  }

private:
  ::crazyflie_interfaces::msg::ConnectionStatistics msg_;
};

class Init_ConnectionStatistics_uri
{
public:
  Init_ConnectionStatistics_uri()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConnectionStatistics_sent_count uri(::crazyflie_interfaces::msg::ConnectionStatistics::_uri_type arg)
  {
    msg_.uri = std::move(arg);
    return Init_ConnectionStatistics_sent_count(msg_);
  }

private:
  ::crazyflie_interfaces::msg::ConnectionStatistics msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::msg::ConnectionStatistics>()
{
  return crazyflie_interfaces::msg::builder::Init_ConnectionStatistics_uri();
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS__BUILDER_HPP_
