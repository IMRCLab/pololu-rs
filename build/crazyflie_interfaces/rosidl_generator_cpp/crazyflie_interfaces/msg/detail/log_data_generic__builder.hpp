// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:msg/LogDataGeneric.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__LOG_DATA_GENERIC__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__LOG_DATA_GENERIC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/msg/detail/log_data_generic__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace msg
{

namespace builder
{

class Init_LogDataGeneric_values
{
public:
  explicit Init_LogDataGeneric_values(::crazyflie_interfaces::msg::LogDataGeneric & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::msg::LogDataGeneric values(::crazyflie_interfaces::msg::LogDataGeneric::_values_type arg)
  {
    msg_.values = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::msg::LogDataGeneric msg_;
};

class Init_LogDataGeneric_timestamp
{
public:
  explicit Init_LogDataGeneric_timestamp(::crazyflie_interfaces::msg::LogDataGeneric & msg)
  : msg_(msg)
  {}
  Init_LogDataGeneric_values timestamp(::crazyflie_interfaces::msg::LogDataGeneric::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_LogDataGeneric_values(msg_);
  }

private:
  ::crazyflie_interfaces::msg::LogDataGeneric msg_;
};

class Init_LogDataGeneric_header
{
public:
  Init_LogDataGeneric_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LogDataGeneric_timestamp header(::crazyflie_interfaces::msg::LogDataGeneric::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LogDataGeneric_timestamp(msg_);
  }

private:
  ::crazyflie_interfaces::msg::LogDataGeneric msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::msg::LogDataGeneric>()
{
  return crazyflie_interfaces::msg::builder::Init_LogDataGeneric_header();
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__LOG_DATA_GENERIC__BUILDER_HPP_
