// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:msg/LogBlock.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__LOG_BLOCK__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__LOG_BLOCK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/msg/detail/log_block__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace msg
{

namespace builder
{

class Init_LogBlock_variables
{
public:
  explicit Init_LogBlock_variables(::crazyflie_interfaces::msg::LogBlock & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::msg::LogBlock variables(::crazyflie_interfaces::msg::LogBlock::_variables_type arg)
  {
    msg_.variables = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::msg::LogBlock msg_;
};

class Init_LogBlock_frequency
{
public:
  explicit Init_LogBlock_frequency(::crazyflie_interfaces::msg::LogBlock & msg)
  : msg_(msg)
  {}
  Init_LogBlock_variables frequency(::crazyflie_interfaces::msg::LogBlock::_frequency_type arg)
  {
    msg_.frequency = std::move(arg);
    return Init_LogBlock_variables(msg_);
  }

private:
  ::crazyflie_interfaces::msg::LogBlock msg_;
};

class Init_LogBlock_topic_name
{
public:
  Init_LogBlock_topic_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LogBlock_frequency topic_name(::crazyflie_interfaces::msg::LogBlock::_topic_name_type arg)
  {
    msg_.topic_name = std::move(arg);
    return Init_LogBlock_frequency(msg_);
  }

private:
  ::crazyflie_interfaces::msg::LogBlock msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::msg::LogBlock>()
{
  return crazyflie_interfaces::msg::builder::Init_LogBlock_topic_name();
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__LOG_BLOCK__BUILDER_HPP_
