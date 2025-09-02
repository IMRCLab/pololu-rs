// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:srv/AddLogging.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__ADD_LOGGING__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__ADD_LOGGING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/srv/detail/add_logging__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace srv
{

namespace builder
{

class Init_AddLogging_Request_vars
{
public:
  explicit Init_AddLogging_Request_vars(::crazyflie_interfaces::srv::AddLogging_Request & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::srv::AddLogging_Request vars(::crazyflie_interfaces::srv::AddLogging_Request::_vars_type arg)
  {
    msg_.vars = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::srv::AddLogging_Request msg_;
};

class Init_AddLogging_Request_frequency
{
public:
  explicit Init_AddLogging_Request_frequency(::crazyflie_interfaces::srv::AddLogging_Request & msg)
  : msg_(msg)
  {}
  Init_AddLogging_Request_vars frequency(::crazyflie_interfaces::srv::AddLogging_Request::_frequency_type arg)
  {
    msg_.frequency = std::move(arg);
    return Init_AddLogging_Request_vars(msg_);
  }

private:
  ::crazyflie_interfaces::srv::AddLogging_Request msg_;
};

class Init_AddLogging_Request_topic_name
{
public:
  Init_AddLogging_Request_topic_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AddLogging_Request_frequency topic_name(::crazyflie_interfaces::srv::AddLogging_Request::_topic_name_type arg)
  {
    msg_.topic_name = std::move(arg);
    return Init_AddLogging_Request_frequency(msg_);
  }

private:
  ::crazyflie_interfaces::srv::AddLogging_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::srv::AddLogging_Request>()
{
  return crazyflie_interfaces::srv::builder::Init_AddLogging_Request_topic_name();
}

}  // namespace crazyflie_interfaces


namespace crazyflie_interfaces
{

namespace srv
{

namespace builder
{

class Init_AddLogging_Response_success
{
public:
  Init_AddLogging_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::crazyflie_interfaces::srv::AddLogging_Response success(::crazyflie_interfaces::srv::AddLogging_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::srv::AddLogging_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::srv::AddLogging_Response>()
{
  return crazyflie_interfaces::srv::builder::Init_AddLogging_Response_success();
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__ADD_LOGGING__BUILDER_HPP_
