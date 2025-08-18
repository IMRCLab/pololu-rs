// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__STATUS__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace msg
{

namespace builder
{

class Init_Status_latency_unicast
{
public:
  explicit Init_Status_latency_unicast(::crazyflie_interfaces::msg::Status & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::msg::Status latency_unicast(::crazyflie_interfaces::msg::Status::_latency_unicast_type arg)
  {
    msg_.latency_unicast = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Status msg_;
};

class Init_Status_num_tx_unicast
{
public:
  explicit Init_Status_num_tx_unicast(::crazyflie_interfaces::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_latency_unicast num_tx_unicast(::crazyflie_interfaces::msg::Status::_num_tx_unicast_type arg)
  {
    msg_.num_tx_unicast = std::move(arg);
    return Init_Status_latency_unicast(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Status msg_;
};

class Init_Status_num_rx_unicast
{
public:
  explicit Init_Status_num_rx_unicast(::crazyflie_interfaces::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_num_tx_unicast num_rx_unicast(::crazyflie_interfaces::msg::Status::_num_rx_unicast_type arg)
  {
    msg_.num_rx_unicast = std::move(arg);
    return Init_Status_num_tx_unicast(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Status msg_;
};

class Init_Status_num_tx_broadcast
{
public:
  explicit Init_Status_num_tx_broadcast(::crazyflie_interfaces::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_num_rx_unicast num_tx_broadcast(::crazyflie_interfaces::msg::Status::_num_tx_broadcast_type arg)
  {
    msg_.num_tx_broadcast = std::move(arg);
    return Init_Status_num_rx_unicast(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Status msg_;
};

class Init_Status_num_rx_broadcast
{
public:
  explicit Init_Status_num_rx_broadcast(::crazyflie_interfaces::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_num_tx_broadcast num_rx_broadcast(::crazyflie_interfaces::msg::Status::_num_rx_broadcast_type arg)
  {
    msg_.num_rx_broadcast = std::move(arg);
    return Init_Status_num_tx_broadcast(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Status msg_;
};

class Init_Status_rssi
{
public:
  explicit Init_Status_rssi(::crazyflie_interfaces::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_num_rx_broadcast rssi(::crazyflie_interfaces::msg::Status::_rssi_type arg)
  {
    msg_.rssi = std::move(arg);
    return Init_Status_num_rx_broadcast(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Status msg_;
};

class Init_Status_pm_state
{
public:
  explicit Init_Status_pm_state(::crazyflie_interfaces::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_rssi pm_state(::crazyflie_interfaces::msg::Status::_pm_state_type arg)
  {
    msg_.pm_state = std::move(arg);
    return Init_Status_rssi(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Status msg_;
};

class Init_Status_battery_voltage
{
public:
  explicit Init_Status_battery_voltage(::crazyflie_interfaces::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_pm_state battery_voltage(::crazyflie_interfaces::msg::Status::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_Status_pm_state(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Status msg_;
};

class Init_Status_supervisor_info
{
public:
  explicit Init_Status_supervisor_info(::crazyflie_interfaces::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_battery_voltage supervisor_info(::crazyflie_interfaces::msg::Status::_supervisor_info_type arg)
  {
    msg_.supervisor_info = std::move(arg);
    return Init_Status_battery_voltage(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Status msg_;
};

class Init_Status_header
{
public:
  Init_Status_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Status_supervisor_info header(::crazyflie_interfaces::msg::Status::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Status_supervisor_info(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Status msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::msg::Status>()
{
  return crazyflie_interfaces::msg::builder::Init_Status_header();
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__STATUS__BUILDER_HPP_
